"""Threaded camera capture with a latest-frame slot.

Each camera runs its own capture thread so a slow ``read()`` on one device
never stalls the others (the blueprint's single 120 Hz timer calling two
blocking ``read()`` calls cannot sustain 120 fps). The frame is timestamped
immediately after ``grab()`` returns, which is the closest software proxy to
the exposure time available through V4L2/OpenCV.

``SimCameraSource`` produces synthetic frames so the whole pipeline can run
without hardware (e.g. inside Docker on macOS).
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Callable, Optional

import numpy as np

try:  # OpenCV is only needed for real devices.
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


@dataclass
class CapturedFrame:
    image: np.ndarray  # HxWx3 uint8 BGR
    stamp_ns: int  # monotonic/system-clock ns at grab time (see clock_ns)
    seq: int


ClockFn = Callable[[], int]


FrameCallback = Callable[[CapturedFrame], None]


class CameraSource:
    """Base class: a background thread captures frames.

    Consumers either poll ``pop()`` for the newest unconsumed frame, or pass
    ``on_frame`` to have every frame delivered synchronously on the capture
    thread (used by the publisher: publishing from the capture thread avoids a
    high-rate ROS timer and adds no latency).
    """

    def __init__(self, name: str, fps: float, clock_ns: ClockFn = time.time_ns,
                 on_frame: Optional[FrameCallback] = None):
        self.name = name
        self.fps = fps
        self._clock_ns = clock_ns
        self._on_frame = on_frame
        self._lock = threading.Lock()
        self._latest: Optional[CapturedFrame] = None
        self._seq = 0
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.frames_captured = 0
        self.frames_dropped = 0  # overwritten before being consumed (pop mode)
        self.errors = 0

    # -- lifecycle -----------------------------------------------------------
    def start(self) -> None:
        if self._thread is not None:
            return
        self._open()
        self._thread = threading.Thread(target=self._run, name=f'cam-{self.name}', daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._close()

    # -- consumer API --------------------------------------------------------
    def pop(self) -> Optional[CapturedFrame]:
        """Return the newest unconsumed frame, or None."""
        with self._lock:
            f = self._latest
            self._latest = None
            return f

    # -- to be implemented ---------------------------------------------------
    def _open(self) -> None:
        pass

    def _close(self) -> None:
        pass

    def _capture(self) -> Optional[tuple]:
        """Block until a frame is available. Return (stamp_ns, image) or None."""
        raise NotImplementedError

    # -- internals -----------------------------------------------------------
    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                result = self._capture()
            except Exception:  # noqa: BLE001 - keep the thread alive
                self.errors += 1
                time.sleep(0.01)
                continue
            if result is None:
                self.errors += 1
                continue
            stamp_ns, image = result
            self._seq += 1
            self.frames_captured += 1
            frame = CapturedFrame(image=image, stamp_ns=stamp_ns, seq=self._seq)
            if self._on_frame is not None:
                try:
                    self._on_frame(frame)
                except Exception:  # noqa: BLE001
                    self.errors += 1
                continue
            with self._lock:
                if self._latest is not None:
                    self.frames_dropped += 1
                self._latest = frame


class V4L2CameraSource(CameraSource):
    """OpenCV/V4L2 device configured for MJPG at the requested size and fps."""

    def __init__(self, name: str, device, width: int, height: int, fps: float,
                 fourcc: str = 'MJPG', clock_ns: ClockFn = time.time_ns,
                 on_frame: Optional[FrameCallback] = None):
        super().__init__(name, fps, clock_ns, on_frame)
        self.device = device
        self.width = width
        self.height = height
        self.fourcc = fourcc
        self._cap = None

    def _open(self) -> None:
        if cv2 is None:
            raise RuntimeError('OpenCV (cv2) is required for V4L2CameraSource')
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2) if hasattr(cv2, 'CAP_V4L2') \
            else cv2.VideoCapture(self.device)
        if not cap.isOpened():
            raise RuntimeError(f'camera {self.name}: cannot open device {self.device!r}')
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # keep latency low
        self._cap = cap

    def _close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def _capture(self):
        if not self._cap.grab():
            return None
        stamp = self._clock_ns()
        ok, img = self._cap.retrieve()
        if not ok:
            return None
        return stamp, img

    def actual_settings(self) -> dict:
        if self._cap is None:
            return {}
        return {
            'width': int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            'height': int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            'fps': float(self._cap.get(cv2.CAP_PROP_FPS)),
        }


class SimCameraSource(CameraSource):
    """Synthetic camera: paced at ``fps`` with a moving pattern and frame counter.

    ``jitter_s`` adds uniform random timing noise to emulate a real device; a
    ``stall_every``/``stall_s`` pair injects periodic long gaps to exercise
    drop detection downstream.
    """

    def __init__(self, name: str, width: int, height: int, fps: float,
                 clock_ns: ClockFn = time.time_ns, jitter_s: float = 0.0,
                 stall_every: int = 0, stall_s: float = 0.0, seed: int = 0,
                 on_frame: Optional[FrameCallback] = None):
        super().__init__(name, fps, clock_ns, on_frame)
        self.width = width
        self.height = height
        self.jitter_s = jitter_s
        self.stall_every = stall_every
        self.stall_s = stall_s
        self._rng = np.random.default_rng(seed)
        self._next_t = None
        # Precompute a base gradient image once; per-frame work is a roll + blit.
        yy, xx = np.mgrid[0:height, 0:width]
        base = np.empty((height, width, 3), dtype=np.uint8)
        base[..., 0] = (xx * 255 // max(width - 1, 1))
        base[..., 1] = (yy * 255 // max(height - 1, 1))
        base[..., 2] = ((xx + yy) * 255 // max(width + height - 2, 1))
        self._base = base
        self._blob = min(width, height) // 8

    def render(self, seq: int) -> np.ndarray:
        phase = (seq % 240) / 240.0
        # Drifting RGB gradient resembles grazing-light GelSight illumination.
        img = np.roll(self._base, int(phase * self.width), axis=1)
        # "Contact patch" moving across the gel.
        cx = int((0.5 + 0.35 * np.sin(2 * np.pi * phase)) * self.width)
        cy = int((0.5 + 0.35 * np.cos(2 * np.pi * phase)) * self.height)
        r = self._blob
        img[max(cy - r, 0):cy + r, max(cx - r, 0):cx + r] = 255
        # Binary frame counter in the top-left corner (16 bits, 4px blocks).
        for bit in range(16):
            img[0:4, bit * 4:(bit + 1) * 4] = 255 if (seq >> bit) & 1 else 0
        return img

    def _open(self) -> None:
        self._next_t = time.monotonic()

    def _capture(self):
        period = 1.0 / self.fps
        self._next_t += period
        if self.jitter_s > 0:
            self._next_t += float(self._rng.uniform(-self.jitter_s, self.jitter_s))
        if self.stall_every and self._seq > 0 and self._seq % self.stall_every == 0:
            self._next_t += self.stall_s
        delay = self._next_t - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            # Fell behind; do not accumulate debt.
            self._next_t = time.monotonic()
        stamp = self._clock_ns()
        return stamp, self.render(self._seq + 1)
