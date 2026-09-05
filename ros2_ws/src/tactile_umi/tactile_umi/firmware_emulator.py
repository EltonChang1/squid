"""Emulates the RP2040 firmware on a pseudo-terminal.

Writes protocol frames at 1 kHz with smooth synthetic loads and jaw motion so
the ROS stack can be exercised without hardware. Optional fault injection:

* ``jitter_s``      - uniform timing noise per frame
* ``stall_every``   - every N frames, pause for ``stall_s`` (simulates a USB hiccup)
* ``garbage_every`` - every N frames, inject a few random bytes (parser resync test)

Usage as a library::

    emu = FirmwareEmulator()
    emu.start()             # emu.port -> '/dev/ttys00X'
    ...
    emu.stop()

Or standalone: ``ros2 run tactile_umi firmware_emulator`` prints the pty path.
"""

from __future__ import annotations

import argparse
import math
import os
import random
import sys
import threading
import time
import tty
from typing import Optional

from .protocol import FRAME_RATE_HZ, pack_frame


class FirmwareEmulator:
    def __init__(self, rate_hz: float = FRAME_RATE_HZ, jitter_s: float = 0.0,
                 stall_every: int = 0, stall_s: float = 0.0, garbage_every: int = 0,
                 seed: int = 0, jaw_period_s: float = 4.0, jaw_amplitude_rad: float = 3.0,
                 max_debt_s: float = 0.05):
        self.rate_hz = rate_hz
        # Like a real MCU, keep the long-run rate exact: if the thread wakes
        # late it emits the frames it owes back-to-back (USB delivers in bursts
        # anyway). Beyond max_debt_s the backlog is discarded as a stall.
        self.max_debt_s = max_debt_s
        self.jitter_s = jitter_s
        self.stall_every = stall_every
        self.stall_s = stall_s
        self.garbage_every = garbage_every
        self.jaw_period_s = jaw_period_s
        self.jaw_amplitude_rad = jaw_amplitude_rad
        self._rng = random.Random(seed)
        self._master_fd: Optional[int] = None
        self._slave_fd: Optional[int] = None
        self.port: Optional[str] = None
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.frames_sent = 0
        self.frames_dropped = 0

    def start(self) -> None:
        self._master_fd, self._slave_fd = os.openpty()
        self.port = os.ttyname(self._slave_fd)
        # Raw mode on the slave: no line discipline, no echo, binary-safe.
        tty.setraw(self._slave_fd)
        # Never block the pacing loop if the reader is slow/absent: drop like
        # a real USB CDC endpoint with no host reading.
        os.set_blocking(self._master_fd, False)
        self._thread = threading.Thread(target=self._run, name='fw-emulator', daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        for fd in (self._master_fd, self._slave_fd):
            if fd is not None:
                try:
                    os.close(fd)
                except OSError:
                    pass
        self._master_fd = self._slave_fd = None

    @staticmethod
    def synth(t: float, jaw_period_s: float, jaw_amplitude_rad: float) -> tuple:
        """Deterministic sensor model at time ``t`` seconds."""
        # Grasp cycle: jaw closes, force builds while closed, torque ripples.
        phase = (t % jaw_period_s) / jaw_period_s
        jaw = jaw_amplitude_rad * 0.5 * (1 - math.cos(2 * math.pi * phase))
        closed = max(0.0, math.sin(2 * math.pi * phase))
        fx = 0.5 * math.sin(2 * math.pi * 1.3 * t)
        fy = 0.5 * math.cos(2 * math.pi * 0.7 * t)
        fz = -9.81 * 0.35 - 12.0 * closed  # gripper weight + grasp load
        tx = 0.05 * math.sin(2 * math.pi * 0.4 * t)
        ty = 0.05 * math.cos(2 * math.pi * 0.5 * t)
        tz = 0.01 * closed
        return fx, fy, fz, tx, ty, tz, jaw

    def _run(self) -> None:
        period = 1.0 / self.rate_hz
        t0 = time.monotonic()
        next_t = t0
        while not self._stop.is_set():
            next_t += period
            if self.jitter_s > 0:
                next_t += self._rng.uniform(-self.jitter_s, self.jitter_s)
            if self.stall_every and self.frames_sent and self.frames_sent % self.stall_every == 0:
                next_t += self.stall_s
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            elif -delay > self.max_debt_s:
                next_t = time.monotonic()  # too far behind: drop the backlog
            t = next_t - t0  # nominal sample time, not wake time
            payload = pack_frame(*self.synth(t, self.jaw_period_s, self.jaw_amplitude_rad))
            if (self.garbage_every and self.frames_sent
                    and self.frames_sent % self.garbage_every == 0):
                n_garbage = self._rng.randint(1, 7)
                payload = bytes(self._rng.randrange(256) for _ in range(n_garbage)) + payload
            try:
                os.write(self._master_fd, payload)
            except BlockingIOError:
                self.frames_dropped += 1  # reader not keeping up / not attached
            except OSError:
                pass  # pty closed
            self.frames_sent += 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='Tactile-UMI firmware emulator (pty)')
    ap.add_argument('--rate', type=float, default=FRAME_RATE_HZ)
    ap.add_argument('--jitter-ms', type=float, default=0.0)
    ap.add_argument('--stall-every', type=int, default=0)
    ap.add_argument('--stall-ms', type=float, default=0.0)
    ap.add_argument('--garbage-every', type=int, default=0)
    ap.add_argument('--link', default=None,
                    help='create a symlink at this path pointing at the pty')
    args = ap.parse_args(argv)

    emu = FirmwareEmulator(rate_hz=args.rate, jitter_s=args.jitter_ms / 1e3,
                           stall_every=args.stall_every, stall_s=args.stall_ms / 1e3,
                           garbage_every=args.garbage_every)
    emu.start()
    if args.link:
        try:
            os.remove(args.link)
        except FileNotFoundError:
            pass
        os.symlink(emu.port, args.link)
    print(emu.port, flush=True)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        emu.stop()
        if args.link:
            try:
                os.remove(args.link)
            except FileNotFoundError:
                pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
