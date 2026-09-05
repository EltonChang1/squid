"""Background serial reader feeding ``FrameParser`` and timestamping frames.

Every decoded frame is stamped with the host clock at the moment the read()
returned the bytes completing it. Inter-frame gaps larger than
``gap_threshold_s`` are counted as drops (at 1 kHz a healthy link shows ~1 ms
spacing; USB CDC batching can legitimately cluster a few frames per read).
"""

from __future__ import annotations

import collections
from dataclasses import dataclass
import threading
import time
from typing import Callable, Deque, List, Optional

from .protocol import Frame, FRAME_RATE_HZ, FRAME_SIZE, FrameParser

try:
    import serial  # pyserial
except ImportError:  # pragma: no cover
    serial = None

ClockFn = Callable[[], int]


@dataclass(frozen=True)
class StampedFrame:
    frame: Frame
    stamp_ns: int
    seq: int


class SerialFrameReader:
    """Reads a byte stream (pyserial port or any object with ``read(n)``)."""

    def __init__(self, port: str, baud: int = 115200, clock_ns: ClockFn = time.time_ns,
                 max_queue: int = 4096, gap_threshold_s: float = 3.0 / FRAME_RATE_HZ,
                 stream=None, on_frames: Optional[Callable[[List['StampedFrame']], None]] = None):
        self.port = port
        self.baud = baud
        self._clock_ns = clock_ns
        self._stream = stream  # injected for tests / pty emulator
        # If given, each batch of decoded frames is handed to on_frames() on the
        # reader thread instead of being queued for drain().
        self._on_frames = on_frames
        self.parser = FrameParser()
        self._queue: Deque[StampedFrame] = collections.deque(maxlen=max_queue)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._seq = 0
        self._last_stamp_ns: Optional[int] = None
        self.gap_threshold_ns = int(gap_threshold_s * 1e9)
        self.gaps = 0  # inter-frame gaps above threshold
        self.max_gap_ns = 0
        self.bytes_read = 0
        self.overflow = 0  # frames evicted from the queue before consumption
        self.callback_errors = 0

    # -- lifecycle -----------------------------------------------------------
    def start(self) -> None:
        if self._thread is not None:
            return
        if self._stream is None:
            if serial is None:
                raise RuntimeError('pyserial is required for SerialFrameReader')
            self._stream = serial.Serial(self.port, self.baud, timeout=0.002)
            try:
                self._stream.reset_input_buffer()
            except Exception:  # noqa: BLE001
                pass
        self._thread = threading.Thread(target=self._run, name='serial-reader', daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._stream is not None and hasattr(self._stream, 'close'):
            try:
                self._stream.close()
            except Exception:  # noqa: BLE001
                pass

    # -- consumer API --------------------------------------------------------
    def drain(self) -> List[StampedFrame]:
        with self._lock:
            out = list(self._queue)
            self._queue.clear()
        return out

    # -- internals -----------------------------------------------------------
    def _read_chunk(self) -> bytes:
        stream = self._stream
        n = FRAME_SIZE
        waiting = getattr(stream, 'in_waiting', None)
        if waiting:
            n = max(n, int(waiting))
        return stream.read(n) or b''

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                chunk = self._read_chunk()
            except Exception:  # noqa: BLE001
                time.sleep(0.01)
                continue
            if not chunk:
                continue
            self.bytes_read += len(chunk)
            stamp = self._clock_ns()
            frames = self.parser.feed(chunk)
            if not frames:
                continue
            batch: List[StampedFrame] = []
            with self._lock:
                for f in frames:
                    self._seq += 1
                    if self._last_stamp_ns is not None:
                        gap = stamp - self._last_stamp_ns
                        if gap > self.max_gap_ns:
                            self.max_gap_ns = gap
                        if gap > self.gap_threshold_ns:
                            self.gaps += 1
                    self._last_stamp_ns = stamp
                    sf = StampedFrame(frame=f, stamp_ns=stamp, seq=self._seq)
                    if self._on_frames is not None:
                        batch.append(sf)
                    else:
                        if len(self._queue) == self._queue.maxlen:
                            self.overflow += 1
                        self._queue.append(sf)
            if batch:
                try:
                    self._on_frames(batch)
                except Exception:  # noqa: BLE001
                    self.callback_errors += 1
