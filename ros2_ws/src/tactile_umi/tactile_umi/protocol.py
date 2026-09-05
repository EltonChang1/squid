"""MCU -> host wire protocol (see docs/protocol.md).

Frame = 0xAA 0x55 + 28-byte little-endian payload ``<fffffff``:
    force_x, force_y, force_z [N], torque_x, torque_y, torque_z [N*m],
    encoder angle [rad, unwrapped].

``FrameParser`` is a byte-oriented state machine that tolerates arbitrary
chunk boundaries and resynchronises on the two-byte header after garbage.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
from typing import Iterable, List

SYNC0 = 0xAA
SYNC1 = 0x55
HEADER = bytes((SYNC0, SYNC1))
PAYLOAD_FORMAT = '<fffffff'
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FORMAT)  # 28
FRAME_SIZE = len(HEADER) + PAYLOAD_SIZE  # 30
FRAME_RATE_HZ = 1000

_payload_struct = struct.Struct(PAYLOAD_FORMAT)


@dataclass(frozen=True)
class Frame:
    """One decoded sensor frame."""

    force: tuple  # (fx, fy, fz) N
    torque: tuple  # (tx, ty, tz) N*m
    encoder: float  # rad
    valid: bool = True

    @property
    def as_tuple(self) -> tuple:
        return (*self.force, *self.torque, self.encoder)


def pack_frame(fx: float, fy: float, fz: float,
               tx: float, ty: float, tz: float,
               encoder: float) -> bytes:
    """Serialise one frame exactly as the firmware does."""
    return HEADER + _payload_struct.pack(fx, fy, fz, tx, ty, tz, encoder)


def unpack_payload(payload: bytes) -> Frame:
    """Decode a 28-byte payload into a ``Frame``."""
    if len(payload) != PAYLOAD_SIZE:
        raise ValueError(f'payload must be {PAYLOAD_SIZE} bytes, got {len(payload)}')
    vals = _payload_struct.unpack(payload)
    valid = all(math.isfinite(v) for v in vals)
    return Frame(force=vals[0:3], torque=vals[3:6], encoder=vals[6], valid=valid)


class FrameParser:
    """Incremental parser with header resynchronisation.

    Usage::

        parser = FrameParser()
        for chunk in serial_chunks:
            for frame in parser.feed(chunk):
                ...
    """

    _HUNT0 = 0
    _HUNT1 = 1
    _PAYLOAD = 2

    def __init__(self) -> None:
        self._state = self._HUNT0
        self._buf = bytearray()
        self._discarding = False
        self.frames = 0
        self.invalid_frames = 0
        self.resyncs = 0  # number of contiguous runs of discarded bytes
        self.resync_bytes = 0  # total bytes discarded while hunting for a header

    def reset(self) -> None:
        self._state = self._HUNT0
        self._buf.clear()
        self._discarding = False

    def _discard(self, n: int) -> None:
        self.resync_bytes += n
        if not self._discarding:
            self._discarding = True
            self.resyncs += 1

    def feed(self, data: bytes) -> List[Frame]:
        out: List[Frame] = []
        for b in data:
            if self._state == self._HUNT0:
                if b == SYNC0:
                    self._state = self._HUNT1
                else:
                    self._discard(1)
            elif self._state == self._HUNT1:
                if b == SYNC1:
                    self._state = self._PAYLOAD
                    self._buf.clear()
                    self._discarding = False
                elif b == SYNC0:
                    # "AA AA 55": previous AA was garbage, this one may be sync.
                    self._discard(1)
                else:
                    self._discard(2)
                    self._state = self._HUNT0
            else:  # _PAYLOAD
                self._buf.append(b)
                if len(self._buf) == PAYLOAD_SIZE:
                    frame = unpack_payload(bytes(self._buf))
                    self.frames += 1
                    if not frame.valid:
                        self.invalid_frames += 1
                    out.append(frame)
                    self._state = self._HUNT0
        return out

    def feed_all(self, chunks: Iterable[bytes]) -> List[Frame]:
        out: List[Frame] = []
        for c in chunks:
            out.extend(self.feed(c))
        return out
