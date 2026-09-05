"""``mcap_recorder``: subscribe to the sensor streams and write an MCAP bag.

Runs on ``RawSpinner`` (``fast_spin.py``) rather than the rclpy executor so
the two 1 kHz streams plus images are drained in batches without per-message
executor overhead, and so SIGINT/SIGTERM always close the bag cleanly.

Output: ``<session_root>/session_YYYYMMDD_HHMMSS/trajectory/trajectory_0.mcap``
via ``rosbag2_py.SequentialWriter`` with ``storage_id="mcap"`` (requires
``ros-humble-rosbag2-storage-mcap``; MCAP is not the default in Humble).

Alignment gate (blueprint "Data Ingestion Agent" prompt): each tactile image
stamp is compared to the nearest ``/wrist/wrench`` stamp; images whose
|offset| exceeds ``max_jitter_ms`` (default 2.0) are dropped and counted.
Gating is causal (``CausalGate``): an image is held until the wrench stream
has advanced past it, because DDS delivers the two topics independently and
the 1 kHz reference routinely arrives *after* the image it brackets.
Non-image streams are always recorded so nothing is lost for offline analysis.

Subscriptions are ``raw=True``: serialized CDR bytes are written straight to
the bag and the header stamp is read from the bytes (``cdr.py``), so a 921 KB
image is never deserialized in Python.
"""

from __future__ import annotations

import datetime as _dt
import os
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, WrenchStamped
import rclpy
from rclpy.node import Node
import rosbag2_py
from sensor_msgs.msg import Image, JointState

from .alignment import AlignmentGate, CausalGate
from .cdr import header_stamp_ns
from .fast_spin import RawSpinner, spin_raw
from .qos import qos_for_topic

TOPICS = {
    '/tactile/left/image_raw': (Image, 'sensor_msgs/msg/Image'),
    '/tactile/right/image_raw': (Image, 'sensor_msgs/msg/Image'),
    '/wrist/image_raw': (Image, 'sensor_msgs/msg/Image'),
    '/wrist/wrench': (WrenchStamped, 'geometry_msgs/msg/WrenchStamped'),
    '/gripper/state': (JointState, 'sensor_msgs/msg/JointState'),
    '/pose/end_effector': (PoseStamped, 'geometry_msgs/msg/PoseStamped'),
    '/system/sync_status': (DiagnosticArray, 'diagnostic_msgs/msg/DiagnosticArray'),
}
GATED_TOPICS = ('/tactile/left/image_raw', '/tactile/right/image_raw')
REFERENCE_TOPIC = '/wrist/wrench'


def make_session_dir(root: str, now: _dt.datetime | None = None) -> str:
    now = now or _dt.datetime.now()
    path = os.path.join(root, now.strftime('session_%Y%m%d_%H%M%S'))
    os.makedirs(root, exist_ok=True)
    return path


class McapRecorder(Node):

    def __init__(self) -> None:
        super().__init__('mcap_recorder')
        self.declare_parameter('session_root', '/data/sessions')
        self.declare_parameter('bag_name', 'trajectory')
        self.declare_parameter('max_jitter_ms', 2.0)
        self.declare_parameter('gate_hold_ms', 250.0)
        self.declare_parameter('drop_without_reference', True)
        self.declare_parameter('storage_preset', 'zstd_fast')  # '' for uncompressed
        self.declare_parameter('cache_mb', 256)
        self.declare_parameter('gate_images', True)
        self.declare_parameter('topics', list(TOPICS.keys()))

        root = str(self.get_parameter('session_root').value)
        self.session_dir = make_session_dir(root)
        bag_uri = os.path.join(self.session_dir, str(self.get_parameter('bag_name').value))

        storage = rosbag2_py.StorageOptions(uri=bag_uri, storage_id='mcap')
        preset = str(self.get_parameter('storage_preset').value)
        if preset:
            storage.storage_preset_profile = preset
        # Buffer writes in rosbag2's cache so chunk compression/flush happens on
        # its consumer thread instead of stalling the subscription loop.
        storage.max_cache_size = int(self.get_parameter('cache_mb').value) * 1024 * 1024
        converter = rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                                output_serialization_format='cdr')
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(storage, converter)
        self._lock = threading.Lock()
        self._closed = False

        gate = AlignmentGate(
            max_offset_ms=float(self.get_parameter('max_jitter_ms').value),
            drop_without_reference=bool(self.get_parameter('drop_without_reference').value))
        self.gate = CausalGate(gate, hold_ns=int(self.get_parameter('gate_hold_ms').value * 1e6))
        self.gate_images = bool(self.get_parameter('gate_images').value)
        self.written = dict.fromkeys(TOPICS, 0)
        self.dropped = dict.fromkeys(GATED_TOPICS, 0)

        selected = list(self.get_parameter('topics').value)
        self.spinner = RawSpinner(self)
        n_topics = 0
        for topic in selected:
            if topic not in TOPICS:
                self.get_logger().warn(f'unknown topic {topic}, skipping')
                continue
            msg_type, type_name = TOPICS[topic]
            self.writer.create_topic(rosbag2_py.TopicMetadata(
                name=topic, type=type_name, serialization_format='cdr'))
            self.spinner.add_raw_subscription(
                msg_type, topic, lambda m, t=topic: self._on_msg(t, m),
                qos_for_topic(topic, image_depth=120))
            n_topics += 1

        self.spinner.add_periodic(0.02, self._flush_expired)
        self.spinner.add_periodic(5.0, self._log_progress)
        self.get_logger().info(
            f'recording {n_topics} topics to {bag_uri} (mcap, {preset or "raw"})')

    # ---- ingest ----------------------------------------------------------------
    def _on_msg(self, topic: str, data: bytes) -> None:
        if self._closed:
            return
        try:
            stamp_ns = header_stamp_ns(data)
        except ValueError:
            stamp_ns = self.get_clock().now().nanoseconds
        wall = time.monotonic_ns()

        if topic == REFERENCE_TOPIC:
            self._write(topic, data, stamp_ns)
            self._settle(self.gate.add_reference(stamp_ns, wall))
        elif self.gate_images and topic in GATED_TOPICS:
            self._settle(self.gate.submit(stamp_ns, (topic, data), wall))
        else:
            self._write(topic, data, stamp_ns)

    def _settle(self, decided) -> None:
        for keep, _off, stamp_ns, (topic, data) in decided:
            if keep:
                self._write(topic, data, stamp_ns)
            else:
                self.dropped[topic] += 1

    def _flush_expired(self) -> None:
        if not self._closed:
            self._settle(self.gate.flush_expired(time.monotonic_ns()))

    def _write(self, topic: str, data: bytes, stamp_ns: int) -> None:
        with self._lock:
            if self._closed:
                return
            # Header stamp as log time so Foxglove/plots line up on sensor time.
            self.writer.write(topic, data, stamp_ns)
            self.written[topic] += 1

    # ---- housekeeping ----------------------------------------------------------
    def _log_progress(self) -> None:
        s = self.gate.gate.stats.summary()
        wr = ' '.join(f'{t.split("/")[-2] if t.endswith("image_raw") else t.split("/")[-1]}={n}'
                      for t, n in self.written.items() if n)
        sp = self.spinner
        self.get_logger().info(
            f'written: {wr} | gate: checked={s["checked"]} dropped={s["dropped"]} '
            f'no_ref={s["no_reference"]} pending={len(self.gate)} '
            f'(max {self.gate.max_pending}, {self.gate.pending_bytes / 1e6:.1f} MB) '
            f'p99={s["p99_ms"]:.3f}ms max={s["max_ms"]:.3f}ms | '
            f'spin: msgs={sp.messages} wakeups={sp.wakeups} max_batch={sp.max_batch} '
            f'max_busy={sp.max_busy_ns / 1e6:.1f}ms')
        sp.max_busy_ns = 0

    def close(self) -> None:
        if self._closed:
            return
        # Decide everything still parked with the references we have.
        self._settle(self.gate.flush_all())
        with self._lock:
            self._closed = True
            self.writer.close()  # writes the MCAP summary/footer
        s = self.gate.gate.stats.summary()
        self.get_logger().info(
            f'closed {self.session_dir}: written={sum(self.written.values())} '
            f'dropped_images={sum(self.dropped.values())} align_p99={s["p99_ms"]:.3f}ms')

    def destroy_node(self) -> bool:
        self.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = McapRecorder()
    spin_raw(node, node.spinner, on_exit=node.close)


if __name__ == '__main__':
    main()
