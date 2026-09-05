"""Low-overhead spin loop for high-rate raw subscriptions.

``rclpy``'s executor costs roughly 300-400 us of Python per delivered message
(wait-set rebuild, ``Task`` wrapping, one message per subscription per wake).
At the 2500 msg/s this package produces (two 1 kHz streams plus ~400 fps of
images) that alone saturates a core and lets the subscriber queues fall
behind. ``RawSpinner`` does what rclcpp does internally instead:

* one ``rcl`` wait set reused across iterations,
* on wake, every ready subscription is drained completely (all queued
  messages, not one),
* raw ``bytes`` delivered straight to a plain callback (no ``Task``),
* simple periodic callbacks driven from the same loop,
* Python-level SIGINT/SIGTERM handling so shutdown is graceful even when the
  process inherited an ignored SIGINT (background job of a non-interactive
  shell, e.g. under ``ros2 launch`` started from a script).

Only ``raw=True`` subscriptions are supported; that is all the recorder and
sync monitor need. ``rclpy`` internals used here (``_rclpy.WaitSet``,
``Subscription.handle.take_message``) are the same ones the executor uses in
Humble.
"""

from __future__ import annotations

import signal
import time
from typing import Callable, List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

RawCallback = Callable[[bytes], None]


class RawSpinner:

    def __init__(self, node: Node, wait_timeout_s: float = 0.005,
                 install_signal_handlers: bool = True):
        self.node = node
        self.wait_timeout_ns = int(wait_timeout_s * 1e9)
        self._subs: List[tuple] = []  # (Subscription, msg_type, callback)
        self._periodic: List[list] = []  # [period_ns, next_due_ns, callback]
        self._stop = False
        self.messages = 0
        self.wakeups = 0
        self.max_batch = 0
        self.max_busy_ns = 0  # longest time spent outside wait() in one iteration
        if install_signal_handlers:
            for sig in (signal.SIGINT, signal.SIGTERM):
                signal.signal(sig, self._on_signal)

    # ---- registration --------------------------------------------------------
    def add_raw_subscription(self, msg_type, topic: str, callback: RawCallback,
                             qos: QoSProfile):
        # The executor never sees this subscription; the callback passed to
        # rclpy is a placeholder.
        sub = self.node.create_subscription(msg_type, topic, lambda _m: None, qos, raw=True)
        self._subs.append((sub, msg_type, callback))
        return sub

    def add_periodic(self, period_s: float, callback: Callable[[], None]) -> None:
        period_ns = int(period_s * 1e9)
        self._periodic.append([period_ns, time.monotonic_ns() + period_ns, callback])

    # ---- control -------------------------------------------------------------
    def stop(self) -> None:
        self._stop = True

    def _on_signal(self, signum, _frame) -> None:
        self.node.get_logger().info(f'signal {signum}: shutting down')
        self._stop = True

    def spin(self) -> None:
        ws = _rclpy.WaitSet(len(self._subs), 0, 0, 0, 0, 0, self.node.context.handle)
        by_ptr = {sub.handle.pointer: (sub, msg_type, cb) for sub, msg_type, cb in self._subs}
        try:
            while not self._stop and rclpy.ok(context=self.node.context):
                ws.clear_entities()
                for sub, _, _ in self._subs:
                    ws.add_subscription(sub.handle)
                ws.wait(self.wait_timeout_ns)
                self.wakeups += 1
                t_wake = time.monotonic_ns()
                for ptr in ws.get_ready_entities('subscription'):
                    entry = by_ptr.get(ptr)
                    if entry is None:
                        continue
                    sub, msg_type, cb = entry
                    n = 0
                    while True:
                        with sub.handle:
                            taken = sub.handle.take_message(msg_type, True)
                        if taken is None:
                            break
                        cb(taken[0])
                        n += 1
                    self.messages += n
                    if n > self.max_batch:
                        self.max_batch = n
                now = time.monotonic_ns()
                for entry in self._periodic:
                    if now >= entry[1]:
                        entry[2]()
                        entry[1] = now + entry[0]
                busy = time.monotonic_ns() - t_wake
                if busy > self.max_busy_ns:
                    self.max_busy_ns = busy
        except _rclpy.RCLError:
            # Context shut down underneath the wait (external shutdown); exit quietly.
            pass


def spin_raw(node: Node, spinner: RawSpinner, on_exit: Optional[Callable[[], None]] = None):
    """Spin, then run ``on_exit`` (e.g. close a bag) and tear down the node."""
    try:
        spinner.spin()
    finally:
        if on_exit is not None:
            on_exit()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def spin_until_signal(node: Node, poll_s: float = 0.1) -> None:
    """Regular executor spin with Python-level SIGINT/SIGTERM handling.

    Replaces ``rclpy.spin`` + ``except KeyboardInterrupt``: with the default
    handler a second SIGINT (e.g. ``ros2 launch`` and a wrapping ``timeout``
    both signalling the process group) raises ``KeyboardInterrupt`` in the
    middle of ``destroy_node`` and the process dies with a traceback.
    """
    stop = {'flag': False}

    def _on_signal(signum, _frame):
        if not stop['flag']:
            node.get_logger().info(f'signal {signum}: shutting down')
        stop['flag'] = True

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, _on_signal)
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    try:
        while not stop['flag'] and rclpy.ok(context=node.context):
            executor.spin_once(timeout_sec=poll_s)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok(context=node.context):
            rclpy.shutdown(context=node.context)
