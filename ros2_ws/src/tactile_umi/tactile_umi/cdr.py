"""Minimal CDR helpers for raw (serialized) ROS 2 messages.

Every stream in this package starts with ``std_msgs/Header``, whose first
field is ``builtin_interfaces/Time stamp {int32 sec, uint32 nanosec}``. In a
serialized message that is:

    bytes 0-3   CDR encapsulation header (0x00, 0x01 = CDR little-endian)
    bytes 4-7   header.stamp.sec      (int32)
    bytes 8-11  header.stamp.nanosec  (uint32)

Reading the stamp straight out of the bytes lets the recorder and sync monitor
use ``raw=True`` subscriptions and never deserialize a 921 KB image just to
learn its timestamp. (Deserializing images in rclpy costs tens of ms each,
which starved the pipeline at 2 x 120 fps.)
"""

from __future__ import annotations

import struct

_LE = struct.Struct('<iI')
_BE = struct.Struct('>iI')


def header_stamp_ns(raw: bytes) -> int:
    """Return header.stamp as nanoseconds from a serialized header-first message."""
    if len(raw) < 12:
        raise ValueError('serialized message too short to contain a Header')
    # Encapsulation: byte 1 -> 0x00 CDR_BE, 0x01 CDR_LE (PL_CDR variants 0x02/0x03).
    little = raw[1] in (0x01, 0x03)
    sec, nsec = (_LE if little else _BE).unpack_from(raw, 4)
    return sec * 1_000_000_000 + nsec
