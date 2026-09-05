"""Timestamp alignment and jitter gating (pure Python, no ROS imports).

``ReferenceTimeline`` keeps a bounded, sorted window of reference stamps (the
1 kHz ``/wrist/wrench`` stream). ``AlignmentGate.check(stamp)`` returns the
signed offset from an image stamp to the nearest reference sample and whether
the frame should be kept (|offset| <= max_offset).

At a healthy 1 kHz reference rate the nearest sample is always within 0.5 ms,
so the 2.0 ms gate (blueprint) only trips when the F/T stream stalls, which is
exactly the condition under which a visuo-tactile sample is not trustworthy.

``RateMonitor`` tracks inter-arrival statistics for /system/sync_status.
"""

from __future__ import annotations

import bisect
import collections
from dataclasses import dataclass, field
import math
from typing import Deque, List, Optional

NS_PER_MS = 1_000_000


class ReferenceTimeline:

    def __init__(self, window_ns: int = 2_000_000_000):
        self.window_ns = window_ns
        self._stamps: Deque[int] = collections.deque()

    def add(self, stamp_ns: int) -> None:
        # Stamps arrive in order from a single publisher; tolerate small
        # reordering by inserting sorted.
        if not self._stamps or stamp_ns >= self._stamps[-1]:
            self._stamps.append(stamp_ns)
        else:
            lst = list(self._stamps)
            bisect.insort(lst, stamp_ns)
            self._stamps = collections.deque(lst)
        cutoff = self._stamps[-1] - self.window_ns
        while self._stamps and self._stamps[0] < cutoff:
            self._stamps.popleft()

    def __len__(self) -> int:
        return len(self._stamps)

    @property
    def latest(self) -> Optional[int]:
        return self._stamps[-1] if self._stamps else None

    def nearest_offset(self, stamp_ns: int) -> Optional[int]:
        """Signed ns from ``stamp_ns`` to the nearest reference (ref - stamp)."""
        if not self._stamps:
            return None
        i = bisect.bisect_left(self._stamps, stamp_ns)
        candidates = []
        if i < len(self._stamps):
            candidates.append(self._stamps[i] - stamp_ns)
        if i > 0:
            candidates.append(self._stamps[i - 1] - stamp_ns)
        return min(candidates, key=abs)


@dataclass
class GateStats:
    checked: int = 0
    kept: int = 0
    dropped: int = 0
    no_reference: int = 0
    offsets_ns: List[int] = field(default_factory=list)  # bounded by caller

    def summary(self) -> dict:
        offs = sorted(abs(o) for o in self.offsets_ns)
        n = len(offs)

        def pct(p: float) -> float:
            if n == 0:
                return float('nan')
            k = min(n - 1, max(0, int(math.ceil(p / 100.0 * n)) - 1))
            return offs[k] / NS_PER_MS

        return {
            'checked': self.checked, 'kept': self.kept, 'dropped': self.dropped,
            'no_reference': self.no_reference,
            'p50_ms': pct(50), 'p99_ms': pct(99),
            'max_ms': (offs[-1] / NS_PER_MS) if n else float('nan'),
        }


class AlignmentGate:

    def __init__(self, max_offset_ms: float = 2.0, window_ns: int = 2_000_000_000,
                 keep_history: int = 20000, drop_without_reference: bool = True):
        self.max_offset_ns = int(max_offset_ms * NS_PER_MS)
        self.reference = ReferenceTimeline(window_ns)
        self.stats = GateStats()
        self._keep_history = keep_history
        self.drop_without_reference = drop_without_reference

    def add_reference(self, stamp_ns: int) -> None:
        self.reference.add(stamp_ns)

    def check(self, stamp_ns: int):
        """Return (keep: bool, offset_ns: Optional[int])."""
        self.stats.checked += 1
        off = self.reference.nearest_offset(stamp_ns)
        if off is None:
            self.stats.no_reference += 1
            keep = not self.drop_without_reference
        else:
            keep = abs(off) <= self.max_offset_ns
            if len(self.stats.offsets_ns) < self._keep_history:
                self.stats.offsets_ns.append(off)
        if keep:
            self.stats.kept += 1
        else:
            self.stats.dropped += 1
        return keep, off


class CausalGate:
    """``AlignmentGate`` that waits for the reference stream to catch up.

    Topics arrive through independent DDS queues, so an image is frequently
    delivered *before* the 1 kHz wrench samples that bracket it. Deciding
    immediately would compare against a stale reference and drop good frames.
    Instead, ``submit()`` parks the item until either

    * the newest reference stamp is >= item stamp + max_offset (all candidate
      neighbours have arrived; decide exactly), or
    * the item has been held for ``hold_ns`` of wall time (reference stream
      stalled; decide with what is available, which drops it if nothing is
      within threshold).

    ``add_reference()`` and ``flush_expired()`` return decided items as
    ``(keep, offset_ns, stamp_ns, payload)`` tuples in submission order.
    """

    def __init__(self, gate: AlignmentGate, hold_ns: int = 250_000_000):
        self.gate = gate
        self.hold_ns = hold_ns
        self._pending: Deque[tuple] = collections.deque()  # (stamp, payload, submit_wall_ns)
        self.pending_bytes = 0
        self.max_pending = 0

    def __len__(self) -> int:
        return len(self._pending)

    def submit(self, stamp_ns: int, payload, now_wall_ns: int) -> List[tuple]:
        self._pending.append((stamp_ns, payload, now_wall_ns))
        if hasattr(payload, '__len__'):
            self.pending_bytes += len(payload)
        self.max_pending = max(self.max_pending, len(self._pending))
        return self._resolve(now_wall_ns)

    def add_reference(self, stamp_ns: int, now_wall_ns: int) -> List[tuple]:
        self.gate.add_reference(stamp_ns)
        return self._resolve(now_wall_ns)

    def flush_expired(self, now_wall_ns: int) -> List[tuple]:
        return self._resolve(now_wall_ns)

    def flush_all(self) -> List[tuple]:
        out = []
        while self._pending:
            out.append(self._decide(self._pending.popleft()))
        return out

    def _decide(self, item) -> tuple:
        stamp, payload, _ = item
        if hasattr(payload, '__len__'):
            self.pending_bytes -= len(payload)
        keep, off = self.gate.check(stamp)
        return keep, off, stamp, payload

    def _resolve(self, now_wall_ns: int) -> List[tuple]:
        out = []
        latest = self.gate.reference.latest
        horizon = None if latest is None else latest - self.gate.max_offset_ns
        while self._pending:
            stamp, _, submitted = self._pending[0]
            ready = horizon is not None and stamp <= horizon
            expired = now_wall_ns - submitted >= self.hold_ns
            if not (ready or expired):
                break
            out.append(self._decide(self._pending.popleft()))
        return out


class RateMonitor:
    """Inter-arrival statistics over a sliding window of stamps."""

    def __init__(self, expected_hz: float, window: int = 2000):
        self.expected_hz = expected_hz
        self._stamps: Deque[int] = collections.deque(maxlen=window)
        self.count = 0
        self.max_gap_ns = 0
        self.last_stamp_ns: Optional[int] = None

    def add(self, stamp_ns: int) -> None:
        self.count += 1
        if self.last_stamp_ns is not None:
            gap = stamp_ns - self.last_stamp_ns
            if gap > self.max_gap_ns:
                self.max_gap_ns = gap
        self.last_stamp_ns = stamp_ns
        self._stamps.append(stamp_ns)

    def reset_max(self) -> None:
        self.max_gap_ns = 0

    def rate_hz(self) -> float:
        if len(self._stamps) < 2:
            return 0.0
        span = self._stamps[-1] - self._stamps[0]
        return (len(self._stamps) - 1) / (span * 1e-9) if span > 0 else 0.0

    def jitter_ms(self) -> dict:
        """Deviation of inter-arrival gaps from the nominal period."""
        if len(self._stamps) < 3:
            return {'p50': float('nan'), 'p99': float('nan'), 'max': float('nan')}
        period = 1e9 / self.expected_hz
        devs = sorted(abs((b - a) - period) for a, b in zip(self._stamps, list(self._stamps)[1:]))
        n = len(devs)

        def pct(p):
            return devs[min(n - 1, max(0, int(math.ceil(p / 100 * n)) - 1))] / NS_PER_MS

        return {'p50': pct(50), 'p99': pct(99), 'max': devs[-1] / NS_PER_MS}

    def stale_ns(self, now_ns: int) -> Optional[int]:
        return None if self.last_stamp_ns is None else now_ns - self.last_stamp_ns
