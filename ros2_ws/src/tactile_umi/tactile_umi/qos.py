"""QoS profiles shared by the publisher and every subscriber in the package.

Keeping them in one place guarantees publisher/subscriber compatibility (a
RELIABLE subscriber never matches a BEST_EFFORT publisher).

* Images are BEST_EFFORT. A RELIABLE writer with KEEP_LAST history must wait
  for the oldest sample to be acknowledged before overwriting it, and with
  Fast DDS that wait (up to ``max_blocking_time``, tens of ms) happens inside
  ``publish()`` while holding the Python GIL: every other thread in the
  publisher, including the 1 kHz serial reader, stalls and its arrival
  timestamps are corrupted. Measured: up to 42 ms per publish, 200+ ms serial
  gaps. Best effort over the local shared-memory transport loses frames only
  when a subscriber's own queue overflows, and the recorder/sync monitor count
  that. This matches ROS 2's ``sensor_data`` profile rationale.
* Low-rate/high-rate small signals (wrench, joint state, pose, diagnostics) are
  RELIABLE with a deep history (~2 s) so every 1 kHz sample reaches the bag
  even if the recorder pauses briefly.
"""

from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


def image_qos(depth: int = 30) -> QoSProfile:
    return QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      history=QoSHistoryPolicy.KEEP_LAST, depth=depth)


def signal_qos(depth: int = 2000) -> QoSProfile:
    return QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                      history=QoSHistoryPolicy.KEEP_LAST, depth=depth)


def qos_for_topic(topic: str, image_depth: int = 30, signal_depth: int = 2000) -> QoSProfile:
    if topic.endswith('image_raw'):
        return image_qos(image_depth)
    return signal_qos(signal_depth)
