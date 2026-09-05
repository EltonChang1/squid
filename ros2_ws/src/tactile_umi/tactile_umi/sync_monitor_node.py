"""``sync_monitor``: publishes /system/sync_status (diagnostic_msgs/DiagnosticArray, 10 Hz).

For every stream: rate, inter-arrival jitter (p50/p99/max deviation from the
nominal period), max gap, staleness. For each tactile image stream: offset to
the nearest /wrist/wrench sample. Levels:

    OK     image<->wrench p99 offset < ok_offset_ms (default 1.0, blueprint target)
    WARN   p99 offset < warn_offset_ms (default 2.0) or rate < 90 % nominal
    ERROR  otherwise, or a stream is stale for > stale_ms
"""

from __future__ import annotations

import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, WrenchStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState

from .alignment import AlignmentGate, CausalGate, RateMonitor
from .cdr import header_stamp_ns
from .fast_spin import RawSpinner, spin_raw
from .qos import qos_for_topic

# DiagnosticStatus.level is a ROS `byte`, i.e. Python `bytes` of length 1.
OK, WARN, ERROR = DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR
LEVEL_NAME = {OK: 'OK', WARN: 'WARN', ERROR: 'ERROR'}

STREAMS = {
    # topic: (type, nominal Hz)
    '/tactile/left/image_raw': (Image, 120.0),
    '/tactile/right/image_raw': (Image, 120.0),
    '/wrist/image_raw': (Image, 60.0),
    '/wrist/wrench': (WrenchStamped, 1000.0),
    '/gripper/state': (JointState, 1000.0),
    '/pose/end_effector': (PoseStamped, 200.0),
}
TACTILE_TOPICS = ('/tactile/left/image_raw', '/tactile/right/image_raw')
REFERENCE_TOPIC = '/wrist/wrench'


class SyncMonitor(Node):

    def __init__(self) -> None:
        super().__init__('sync_monitor')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('ok_offset_ms', 1.0)
        self.declare_parameter('warn_offset_ms', 2.0)
        self.declare_parameter('stale_ms', 500.0)
        self.declare_parameter('optional_topics', ['/pose/end_effector', '/wrist/image_raw'])
        self.ok_ns = int(self.get_parameter('ok_offset_ms').value * 1e6)
        self.warn_ns = int(self.get_parameter('warn_offset_ms').value * 1e6)
        self.stale_ns = int(self.get_parameter('stale_ms').value * 1e6)
        self.optional = set(self.get_parameter('optional_topics').value)

        self.monitors = {t: RateMonitor(hz) for t, (_, hz) in STREAMS.items()}
        # Causal, like the recorder: an image's offset is measured only once the
        # wrench stream has passed it (or after a 250 ms hold if it stalled).
        self.gates = {
            t: CausalGate(AlignmentGate(max_offset_ms=self.get_parameter('warn_offset_ms').value,
                                        keep_history=5000, drop_without_reference=False),
                          hold_ns=250_000_000)
            for t in TACTILE_TOPICS}
        # Raw subscriptions on a RawSpinner: only the header stamp is needed,
        # so images are never deserialized and 2.5k msg/s costs little CPU.
        self.spinner = RawSpinner(self)
        for topic, (msg_type, _) in STREAMS.items():
            self.spinner.add_raw_subscription(
                msg_type, topic, lambda m, t=topic: self._on_msg(t, m),
                qos_for_topic(topic, image_depth=60, signal_depth=500))

        self.pub = self.create_publisher(DiagnosticArray, '/system/sync_status', 10)
        self.spinner.add_periodic(1.0 / float(self.get_parameter('publish_rate').value),
                                  self._publish)
        self.spinner.add_periodic(0.05, self._flush_gates)
        self.get_logger().info('sync_monitor ready')

    def _on_msg(self, topic: str, data: bytes) -> None:
        try:
            ns = header_stamp_ns(data)
        except ValueError:
            return
        self.monitors[topic].add(ns)
        wall = time.monotonic_ns()
        if topic == REFERENCE_TOPIC:
            for g in self.gates.values():
                g.add_reference(ns, wall)
        elif topic in self.gates:
            self.gates[topic].submit(ns, None, wall)

    def _flush_gates(self) -> None:
        wall = time.monotonic_ns()
        for g in self.gates.values():
            g.flush_expired(wall)

    def _publish(self) -> None:
        now = self.get_clock().now().nanoseconds
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        worst = OK

        for topic, mon in self.monitors.items():
            st = DiagnosticStatus(name=f'tactile_umi/stream{topic}', hardware_id='tactile_umi')
            nominal = STREAMS[topic][1]
            rate = mon.rate_hz()
            jit = mon.jitter_ms()
            stale = mon.stale_ns(now)
            kv = [
                KeyValue(key='nominal_hz', value=f'{nominal:.0f}'),
                KeyValue(key='rate_hz', value=f'{rate:.1f}'),
                KeyValue(key='count', value=str(mon.count)),
                KeyValue(key='jitter_p50_ms', value=f'{jit["p50"]:.3f}'),
                KeyValue(key='jitter_p99_ms', value=f'{jit["p99"]:.3f}'),
                KeyValue(key='jitter_max_ms', value=f'{jit["max"]:.3f}'),
                KeyValue(key='max_gap_ms', value=f'{mon.max_gap_ns / 1e6:.3f}'),
                KeyValue(key='stale_ms', value='nan' if stale is None else f'{stale / 1e6:.1f}'),
            ]
            if mon.count == 0 or (stale is not None and stale > self.stale_ns):
                if topic in self.optional:
                    st.level, st.message = OK, 'no data (optional stream)'
                else:
                    st.level, st.message = ERROR, 'stale / no data'
            elif rate < 0.9 * nominal:
                st.level = WARN
                st.message = f'rate {rate:.1f} Hz below nominal {nominal:.0f}'
            else:
                st.level, st.message = OK, 'streaming'
            st.values = kv
            arr.status.append(st)
            worst = max(worst, st.level)
            mon.reset_max()

        for topic, cgate in self.gates.items():
            stats = cgate.gate.stats
            s = stats.summary()
            st = DiagnosticStatus(name=f'tactile_umi/align{topic}', hardware_id='tactile_umi')
            st.values = [
                KeyValue(key='reference', value=REFERENCE_TOPIC),
                KeyValue(key='checked', value=str(s['checked'])),
                KeyValue(key='over_threshold', value=str(s['dropped'])),
                KeyValue(key='no_reference', value=str(s['no_reference'])),
                KeyValue(key='pending', value=str(len(cgate))),
                KeyValue(key='offset_p50_ms', value=f'{s["p50_ms"]:.3f}'),
                KeyValue(key='offset_p99_ms', value=f'{s["p99_ms"]:.3f}'),
                KeyValue(key='offset_max_ms', value=f'{s["max_ms"]:.3f}'),
            ]
            p99_ns = s['p99_ms'] * 1e6
            if not stats.offsets_ns:
                st.level, st.message = WARN, 'no images checked in this window'
            elif p99_ns < self.ok_ns:
                st.level, st.message = OK, f'p99 offset {s["p99_ms"]:.3f} ms'
            elif p99_ns < self.warn_ns:
                st.level, st.message = WARN, f'p99 offset {s["p99_ms"]:.3f} ms'
            else:
                st.level, st.message = ERROR, f'p99 offset {s["p99_ms"]:.3f} ms'
            arr.status.append(st)
            worst = max(worst, st.level)
            # Rolling window: keep percentile stats fresh per publish period.
            stats.offsets_ns.clear()

        summary = DiagnosticStatus(
            name='tactile_umi/summary', hardware_id='tactile_umi', level=worst,
            message=LEVEL_NAME.get(worst, '?'))
        arr.status.insert(0, summary)
        self.pub.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SyncMonitor()
    spin_raw(node, node.spinner)


if __name__ == '__main__':
    main()
