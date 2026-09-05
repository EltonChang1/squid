import struct

import pytest

from tactile_umi.cdr import header_stamp_ns


def _cdr(sec, nsec, little=True):
    enc = b'\x00\x01\x00\x00' if little else b'\x00\x00\x00\x00'
    return enc + struct.pack('<iI' if little else '>iI', sec, nsec) + b'\x00' * 8


def test_little_endian():
    assert header_stamp_ns(_cdr(1_700_000_000, 123_456_789)) == 1_700_000_000_123_456_789


def test_big_endian():
    assert header_stamp_ns(_cdr(5, 7, little=False)) == 5_000_000_007


def test_too_short():
    with pytest.raises(ValueError):
        header_stamp_ns(b'\x00\x01\x00\x00\x01')


def test_matches_rclpy_serialization():
    rclpy = pytest.importorskip('rclpy')  # noqa: F841 - only run inside a ROS environment
    from geometry_msgs.msg import WrenchStamped
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Image

    w = WrenchStamped()
    w.header.stamp.sec, w.header.stamp.nanosec = 42, 99
    assert header_stamp_ns(serialize_message(w)) == 42_000_000_099
    im = Image()
    im.header.stamp.sec, im.header.stamp.nanosec = 1, 2
    im.header.frame_id = 'left_gel_frame'
    assert header_stamp_ns(serialize_message(im)) == 1_000_000_002
