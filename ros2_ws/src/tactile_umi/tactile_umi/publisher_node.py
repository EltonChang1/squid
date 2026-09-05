"""``visuo_tactile_publisher``: hardware (or simulated) sensors -> ROS 2 topics.

Topics (see blueprint section 5):
    /tactile/left/image_raw   sensor_msgs/Image           120 Hz, bgr8, left_gel_frame
    /tactile/right/image_raw  sensor_msgs/Image           120 Hz, bgr8, right_gel_frame
    /wrist/image_raw          sensor_msgs/Image            60 Hz, bgr8, wrist_camera_frame
    /wrist/wrench             geometry_msgs/WrenchStamped 1000 Hz, ft_sensor_frame
    /gripper/state            sensor_msgs/JointState      1000 Hz, gripper_finger_joint
    /pose/end_effector        geometry_msgs/PoseStamped    200 Hz (sim only; real source is
                                                          an external MoCap/SLAM node)

Design notes vs the blueprint sketch:
* Each camera has its own capture thread (``camera_source``) and the Image is
  published from that thread, stamped at grab time, so a slow device never
  blocks the others and no high-rate ROS timer is needed.
* Serial frames are read by a background thread, stamped at arrival, and
  published from that thread in order. The rclpy executor only runs the
  5 s stats timer (sim pose is its own paced thread), which keeps CPU low
  enough for 1 kHz + 2x120 fps + 60 fps in pure Python.
* ``sim:=true`` replaces every device with a simulator, including a pty-backed
  firmware emulator, so the full stack runs inside Docker on a laptop.
"""

from __future__ import annotations

import array
import math
import os
import select
import sys
import threading
import time

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, WrenchStamped
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Header

from .calibration import EncoderCalibration, FiniteDifference
from .camera_source import CameraSource, SimCameraSource, V4L2CameraSource
from .fast_spin import spin_until_signal
from .firmware_emulator import FirmwareEmulator
from .qos import image_qos, signal_qos
from .serial_reader import SerialFrameReader


def _stamp_from_ns(ns: int) -> Time:
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def image_msg_from_bgr(img: np.ndarray, stamp_ns: int, frame_id: str) -> Image:
    """Build a bgr8 Image without cv_bridge.

    ``Image.data`` is assigned an ``array.array('B')``: rclpy accepts it without
    the per-element validation it runs on ``bytes``/``list`` (which costs
    ~100 ms for a 640x480x3 frame and made 120 fps impossible).
    """
    msg = Image()
    msg.header = Header(stamp=_stamp_from_ns(stamp_ns), frame_id=frame_id)
    msg.height, msg.width = int(img.shape[0]), int(img.shape[1])
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = array.array('B', np.ascontiguousarray(img, dtype=np.uint8).tobytes())
    return msg


class _NonBlockingFdStream:
    """Minimal read(n) wrapper over a raw fd (used for the emulator pty).

    Blocks in ``select()`` until bytes arrive so the reader thread wakes exactly
    when data lands (best arrival timestamps) instead of polling.
    """

    def __init__(self, path: str, timeout_s: float = 0.002):
        self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        self.timeout_s = timeout_s

    def read(self, n: int) -> bytes:
        ready, _, _ = select.select([self.fd], [], [], self.timeout_s)
        if not ready:
            return b''
        try:
            return os.read(self.fd, max(n, 4096))
        except BlockingIOError:
            return b''

    def close(self) -> None:
        os.close(self.fd)


class VisuoTactilePublisher(Node):

    def __init__(self) -> None:
        super().__init__('visuo_tactile_publisher')

        # ---- parameters ------------------------------------------------------
        self.declare_parameter('sim', False)
        self.declare_parameter('sim_camera_jitter_ms', 0.0)
        self.declare_parameter('sim_serial_stall_every', 0)
        self.declare_parameter('sim_serial_stall_ms', 0.0)
        self.declare_parameter('left_camera_device', 0)
        self.declare_parameter('right_camera_device', 2)
        self.declare_parameter('wrist_camera_device', 4)
        self.declare_parameter('enable_wrist_camera', True)
        self.declare_parameter('tactile_width', 640)
        self.declare_parameter('tactile_height', 480)
        self.declare_parameter('tactile_fps', 120.0)
        self.declare_parameter('wrist_width', 1920)
        self.declare_parameter('wrist_height', 1080)
        self.declare_parameter('wrist_fps', 60.0)
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baud', 115200)
        # Encoder -> width. Default: 8 mm pitch-radius pinion, closed at angle 0.
        self.declare_parameter('encoder_width_scale', 0.008)
        self.declare_parameter('encoder_width_offset', 0.0)
        self.declare_parameter('gripper_max_width', 0.085)
        self.declare_parameter('sim_pose_rate', 200.0)

        p = self.get_parameter
        self.sim = bool(p('sim').value)
        self.cal = EncoderCalibration(
            scale_m_per_rad=float(p('encoder_width_scale').value),
            offset_m=float(p('encoder_width_offset').value),
            max_width_m=float(p('gripper_max_width').value))
        self._vel = FiniteDifference(alpha=0.3)

        # ---- publishers ------------------------------------------------------
        self.pub_left = self.create_publisher(Image, '/tactile/left/image_raw', image_qos())
        self.pub_right = self.create_publisher(Image, '/tactile/right/image_raw', image_qos())
        self.pub_wrist = self.create_publisher(Image, '/wrist/image_raw', image_qos())
        self.pub_wrench = self.create_publisher(WrenchStamped, '/wrist/wrench', signal_qos())
        self.pub_gripper = self.create_publisher(JointState, '/gripper/state', signal_qos())
        self.pub_pose = None

        # ---- sources ---------------------------------------------------------
        clock_ns = self._ros_now_ns
        tw, th, tfps = int(p('tactile_width').value), int(p('tactile_height').value), \
            float(p('tactile_fps').value)
        ww, wh, wfps = int(p('wrist_width').value), int(p('wrist_height').value), \
            float(p('wrist_fps').value)
        self._emulator = None
        self.cam_wrist: CameraSource | None
        self._max_publish_s: dict = {}
        on_left = self._camera_callback(self.pub_left, 'left_gel_frame')
        on_right = self._camera_callback(self.pub_right, 'right_gel_frame')
        on_wrist = self._camera_callback(self.pub_wrist, 'wrist_camera_frame')
        if self.sim:
            jit = float(p('sim_camera_jitter_ms').value) / 1e3
            self.cam_left: CameraSource = SimCameraSource(
                'left', tw, th, tfps, clock_ns, jit, seed=1, on_frame=on_left)
            self.cam_right: CameraSource = SimCameraSource(
                'right', tw, th, tfps, clock_ns, jit, seed=2, on_frame=on_right)
            # Wrist sim at reduced resolution to keep CPU modest in CI.
            self.cam_wrist = SimCameraSource(
                'wrist', min(ww, 640), min(wh, 360), wfps, clock_ns, jit, seed=3,
                on_frame=on_wrist)
            self._emulator = FirmwareEmulator(
                stall_every=int(p('sim_serial_stall_every').value),
                stall_s=float(p('sim_serial_stall_ms').value) / 1e3)
            self._emulator.start()
            port = self._emulator.port
            self.serial = SerialFrameReader(
                port, clock_ns=clock_ns, stream=_NonBlockingFdStream(port),
                on_frames=self._publish_serial)
            self.pub_pose = self.create_publisher(
                PoseStamped, '/pose/end_effector', signal_qos(400))
            self._t0 = time.monotonic()
            # Paced thread rather than a 200 Hz rclpy timer: the executor costs
            # ~1 ms of Python per timer event, a thread with sleep() ~20 us.
            self._pose_stop = threading.Event()
            self._pose_thread = threading.Thread(
                target=self._sim_pose_loop, args=(float(p('sim_pose_rate').value),),
                name='sim_pose', daemon=True)
            self._pose_thread.start()
            self.get_logger().info(f'SIM mode: firmware emulator on {port}')
        else:
            self.cam_left = V4L2CameraSource(
                'left', p('left_camera_device').value, tw, th, tfps, clock_ns=clock_ns,
                on_frame=on_left)
            self.cam_right = V4L2CameraSource(
                'right', p('right_camera_device').value, tw, th, tfps, clock_ns=clock_ns,
                on_frame=on_right)
            self.cam_wrist = None
            if bool(p('enable_wrist_camera').value):
                self.cam_wrist = V4L2CameraSource(
                    'wrist', p('wrist_camera_device').value, ww, wh, wfps, clock_ns=clock_ns,
                    on_frame=on_wrist)
            self.serial = SerialFrameReader(
                str(p('serial_port').value), int(p('serial_baud').value), clock_ns=clock_ns,
                on_frames=self._publish_serial)

        for cam in (self.cam_left, self.cam_right, self.cam_wrist):
            if cam is not None:
                cam.start()
                if isinstance(cam, V4L2CameraSource):
                    self.get_logger().info(f'camera {cam.name}: {cam.actual_settings()}')
        self.serial.start()

        self.create_timer(5.0, self._log_stats)
        self.get_logger().info('visuo_tactile_publisher ready')

    # ---- helpers -------------------------------------------------------------
    def _ros_now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _camera_callback(self, pub, frame_id: str):
        # Runs on the camera's capture thread. rclpy publishers are thread-safe.
        # The max publish() duration is tracked: a reliable DDS writer with a
        # full history blocks (holding the GIL) when a subscriber falls behind,
        # which would show up as stalls on every other stream.
        def _on_frame(f) -> None:
            msg = image_msg_from_bgr(f.image, f.stamp_ns, frame_id)
            t0 = time.perf_counter()
            pub.publish(msg)
            dt = time.perf_counter() - t0
            if dt > self._max_publish_s[frame_id]:
                self._max_publish_s[frame_id] = dt
        self._max_publish_s[frame_id] = 0.0
        return _on_frame

    # ---- callbacks -----------------------------------------------------------
    def _publish_serial(self, frames) -> None:
        # Runs on the serial reader thread, in arrival order.
        for sf in frames:
            if not sf.frame.valid:
                continue
            stamp = _stamp_from_ns(sf.stamp_ns)

            w = WrenchStamped()
            w.header.stamp = stamp
            w.header.frame_id = 'ft_sensor_frame'
            w.wrench.force.x, w.wrench.force.y, w.wrench.force.z = sf.frame.force
            w.wrench.torque.x, w.wrench.torque.y, w.wrench.torque.z = sf.frame.torque
            self.pub_wrench.publish(w)

            width = self.cal.width(sf.frame.encoder)
            j = JointState()
            j.header.stamp = stamp
            j.header.frame_id = 'gripper_base'
            j.name = ['gripper_finger_joint']
            j.position = [width]
            j.velocity = [self._vel.update(width, sf.stamp_ns)]
            j.effort = [-sf.frame.force[2]]  # grasp force proxy: -Fz along jaw axis
            self.pub_gripper.publish(j)

    def _sim_pose_loop(self, rate_hz: float) -> None:
        period = 1.0 / rate_hz
        next_t = time.monotonic()
        while not self._pose_stop.is_set():
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()
            self._publish_sim_pose()

    def _publish_sim_pose(self) -> None:
        t = time.monotonic() - self._t0
        msg = PoseStamped()
        msg.header.stamp = _stamp_from_ns(self._ros_now_ns())
        msg.header.frame_id = 'world'
        msg.pose.position.x = 0.3 * math.cos(0.5 * t)
        msg.pose.position.y = 0.3 * math.sin(0.5 * t)
        msg.pose.position.z = 0.4 + 0.05 * math.sin(1.5 * t)
        yaw = 0.5 * t
        msg.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.orientation.w = math.cos(yaw / 2)
        self.pub_pose.publish(msg)

    def _log_stats(self) -> None:
        s = self.serial
        cams = [c for c in (self.cam_left, self.cam_right, self.cam_wrist) if c is not None]
        cam_txt = ' '.join(
            f'{c.name}:cap={c.frames_captured},err={c.errors}' for c in cams)
        pub_txt = ' '.join(f'{k.split("_")[0]}={v * 1e3:.1f}ms'
                           for k, v in self._max_publish_s.items())
        for k in self._max_publish_s:
            self._max_publish_s[k] = 0.0
        emu_txt = f' emu_dropped={self._emulator.frames_dropped}' if self._emulator else ''
        self.get_logger().info(
            f'serial frames={s.parser.frames} resyncs={s.parser.resyncs} '
            f'gaps={s.gaps} max_gap={s.max_gap_ns / 1e6:.2f}ms{emu_txt} | {cam_txt} | '
            f'max publish {pub_txt}')
        s.max_gap_ns = 0

    def destroy_node(self) -> bool:
        if self._emulator is not None:
            self._pose_stop.set()
            self._pose_thread.join(timeout=1.0)
        for cam in (self.cam_left, self.cam_right, self.cam_wrist):
            if cam is not None:
                cam.stop()
        self.serial.stop()
        if self._emulator is not None:
            self._emulator.stop()
        return super().destroy_node()


def main(args=None) -> None:
    # Five Python threads share the GIL (3 cameras, serial, pose). The default
    # 5 ms switch interval lets a rendering camera thread hold the GIL while a
    # serial frame waits to be stamped; 0.5 ms bounds that scheduling jitter.
    sys.setswitchinterval(0.0005)
    rclpy.init(args=args)
    node = VisuoTactilePublisher()
    spin_until_signal(node)


if __name__ == '__main__':
    main()
