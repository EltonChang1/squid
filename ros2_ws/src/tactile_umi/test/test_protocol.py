import math
import os
import random
import struct

import pytest

from tactile_umi.protocol import (
    FRAME_SIZE,
    FrameParser,
    HEADER,
    pack_frame,
    PAYLOAD_FORMAT,
    PAYLOAD_SIZE,
    unpack_payload,
)


def _frame(i: float) -> bytes:
    return pack_frame(i, i + 1, i + 2, i + 3, i + 4, i + 5, i + 6)


def test_constants_match_spec():
    assert HEADER == b'\xaa\x55'
    assert PAYLOAD_FORMAT == '<fffffff'
    assert PAYLOAD_SIZE == 28
    assert FRAME_SIZE == 30


def test_pack_unpack_roundtrip():
    raw = _frame(1.5)
    assert len(raw) == FRAME_SIZE
    assert raw[:2] == HEADER
    f = unpack_payload(raw[2:])
    assert f.force == pytest.approx((1.5, 2.5, 3.5))
    assert f.torque == pytest.approx((4.5, 5.5, 6.5))
    assert f.encoder == pytest.approx(7.5)
    assert f.valid


def test_clean_stream():
    p = FrameParser()
    stream = b''.join(_frame(float(i)) for i in range(100))
    frames = p.feed(stream)
    assert len(frames) == 100
    assert [f.force[0] for f in frames] == pytest.approx(list(range(100)))
    assert p.resyncs == 0
    assert p.resync_bytes == 0


def test_partial_reads_any_chunking():
    stream = b''.join(_frame(float(i)) for i in range(50))
    rng = random.Random(1234)
    for _ in range(20):
        p = FrameParser()
        got = []
        idx = 0
        while idx < len(stream):
            n = rng.randint(1, 45)
            got.extend(p.feed(stream[idx:idx + n]))
            idx += n
        assert len(got) == 50
        assert [f.encoder for f in got] == pytest.approx([i + 6 for i in range(50)])


def test_misaligned_start():
    stream = b''.join(_frame(float(i)) for i in range(5))
    p = FrameParser()
    frames = p.feed(stream[7:])  # start mid-frame
    assert len(frames) == 4
    assert frames[0].force[0] == pytest.approx(1.0)
    assert p.resync_bytes == FRAME_SIZE - 7
    assert p.resyncs == 1


def test_resyncs_count_discard_runs():
    p = FrameParser()
    stream = _frame(0.0) + b'\x01\x02\x03' + _frame(1.0) + _frame(2.0) + b'\x09' + _frame(3.0)
    frames = p.feed(stream)
    assert len(frames) == 4
    assert p.resyncs == 2
    assert p.resync_bytes == 4


def test_garbage_injection_between_frames():
    rng = random.Random(7)
    parts = []
    for i in range(30):
        parts.append(_frame(float(i)))
        garbage = bytes(rng.randrange(256) for _ in range(rng.randint(0, 10)))
        parts.append(garbage)
    p = FrameParser()
    frames = p.feed(b''.join(parts))
    # Garbage may occasionally contain a fake AA 55 header and swallow a real
    # frame, so we require >= most frames and check that every decoded frame
    # that is aligned is correct.
    assert len(frames) >= 25
    good = [f for f in frames
            if f.force[0] in range(30) and f.force[1] == pytest.approx(f.force[0] + 1)]
    assert len(good) >= 25


def test_double_sync0_prefix():
    # "AA AA 55 <payload>": the first AA is noise.
    raw = bytes([0xAA]) + _frame(3.0)
    p = FrameParser()
    frames = p.feed(raw)
    assert len(frames) == 1
    assert frames[0].force[0] == pytest.approx(3.0)
    assert p.resyncs == 1


def test_sync0_inside_payload_does_not_confuse():
    # Craft a payload whose bytes include AA 55.
    payload = struct.pack(PAYLOAD_FORMAT, *[struct.unpack('<f', b'\xaa\x55\xaa\x55')[0]] * 7)
    raw = HEADER + payload
    p = FrameParser()
    frames = p.feed(raw * 3)
    assert len(frames) == 3
    assert p.resyncs == 0


def test_nan_marks_invalid():
    raw = pack_frame(math.nan, 0, 0, 0, 0, 0, 0)
    p = FrameParser()
    frames = p.feed(raw)
    assert len(frames) == 1
    assert not frames[0].valid
    assert p.invalid_frames == 1


def test_unpack_wrong_size_raises():
    with pytest.raises(ValueError):
        unpack_payload(b'\x00' * 27)


def test_random_bytes_never_raise():
    p = FrameParser()
    p.feed(os.urandom(10000))
