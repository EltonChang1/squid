import math

import pytest

from tactile_umi.alignment import AlignmentGate, CausalGate, RateMonitor, ReferenceTimeline

MS = 1_000_000


def test_causal_gate_waits_for_reference_to_pass():
    cg = CausalGate(AlignmentGate(max_offset_ms=2.0), hold_ns=100 * MS)
    wall = 0
    # Image at 50 ms arrives before any wrench sample near it.
    assert cg.submit(50 * MS, 'img50', wall) == []
    assert len(cg) == 1
    # Reference catching up: nothing decided until latest >= 52 ms.
    for t in range(0, 52):
        assert cg.add_reference(t * MS, wall) == []
    decided = cg.add_reference(52 * MS, wall)
    assert len(decided) == 1
    keep, off, stamp, payload = decided[0]
    assert keep and off == 0 and stamp == 50 * MS and payload == 'img50'
    assert len(cg) == 0


def test_causal_gate_hold_timeout_drops_when_reference_stalled():
    cg = CausalGate(AlignmentGate(max_offset_ms=2.0), hold_ns=100 * MS)
    for t in range(0, 40):
        cg.add_reference(t * MS, 0)
    # Reference stalls at 39 ms; image at 60 ms arrives at wall 60 ms.
    assert cg.submit(60 * MS, 'img', 60 * MS) == []
    assert cg.flush_expired(150 * MS) == []          # held for 90 ms, not yet expired
    decided = cg.flush_expired(160 * MS)             # 100 ms hold reached
    assert len(decided) == 1
    keep, off, _, _ = decided[0]
    assert not keep and off == -21 * MS


def test_causal_gate_preserves_order_and_bytes_accounting():
    cg = CausalGate(AlignmentGate(max_offset_ms=2.0), hold_ns=10**12)
    cg.submit(10 * MS, b'aaaa', 0)
    cg.submit(20 * MS, b'bb', 0)
    assert cg.pending_bytes == 6 and cg.max_pending == 2
    out = cg.add_reference(30 * MS, 0)
    assert [p for _, _, _, p in out] == [b'aaaa', b'bb']
    assert cg.pending_bytes == 0
    assert cg.flush_all() == []


def _ref_stream(gate: AlignmentGate, start_ns: int, n: int, period_ns: int = MS):
    for i in range(n):
        gate.add_reference(start_ns + i * period_ns)


def test_nearest_offset_signed():
    tl = ReferenceTimeline()
    for t in (0, 1 * MS, 2 * MS):
        tl.add(t)
    assert tl.nearest_offset(int(0.4 * MS)) == -int(0.4 * MS)
    assert tl.nearest_offset(int(0.6 * MS)) == int(0.4 * MS)
    assert tl.nearest_offset(5 * MS) == -3 * MS
    assert tl.nearest_offset(-MS) == MS


def test_empty_reference():
    tl = ReferenceTimeline()
    assert tl.nearest_offset(123) is None
    g = AlignmentGate(drop_without_reference=True)
    keep, off = g.check(123)
    assert not keep and off is None
    assert g.stats.no_reference == 1 and g.stats.dropped == 1
    g2 = AlignmentGate(drop_without_reference=False)
    keep, _ = g2.check(123)
    assert keep


def test_healthy_1khz_reference_keeps_every_image():
    g = AlignmentGate(max_offset_ms=2.0)
    _ref_stream(g, 0, 2000)  # 2 s of 1 kHz wrench
    # 120 Hz images at arbitrary phase
    for i in range(240):
        stamp = int(i * (1e9 / 120) + 0.37 * MS)
        keep, off = g.check(stamp)
        assert keep
        assert abs(off) <= 0.5 * MS + 1
    s = g.stats.summary()
    assert s['dropped'] == 0 and s['kept'] == 240
    assert s['max_ms'] <= 0.5001


def test_stalled_reference_drops_images():
    g = AlignmentGate(max_offset_ms=2.0)
    _ref_stream(g, 0, 1000)  # 0..999 ms
    # gap: reference resumes at 1050 ms
    _ref_stream(g, 1050 * MS, 1000)
    # Images inside the gap, more than 2 ms from either side, must be dropped.
    dropped = 0
    kept = 0
    for ms in range(995, 1060):
        keep, off = g.check(ms * MS)
        if 999 + 2 < ms < 1050 - 2:
            assert not keep, f'image at {ms} ms should be dropped (offset {off / MS:.1f} ms)'
            dropped += 1
        elif ms <= 999 or ms >= 1050:
            assert keep
            kept += 1
    assert dropped == 1050 - 2 - (999 + 2) - 1
    assert g.stats.dropped == dropped


def test_threshold_boundary_inclusive():
    g = AlignmentGate(max_offset_ms=2.0)
    g.add_reference(10 * MS)
    keep, _ = g.check(12 * MS)  # exactly 2.0 ms
    assert keep
    keep, _ = g.check(12 * MS + 1)
    assert not keep


def test_window_eviction():
    tl = ReferenceTimeline(window_ns=100 * MS)
    for i in range(1000):
        tl.add(i * MS)
    assert len(tl) <= 101
    # Ancient stamps are gone; offset measured against the window edge.
    assert tl.nearest_offset(0) == (999 - 100) * MS


def test_out_of_order_insert():
    tl = ReferenceTimeline()
    tl.add(5 * MS)
    tl.add(3 * MS)  # slight reorder
    tl.add(6 * MS)
    assert tl.nearest_offset(int(3.2 * MS)) == -int(0.2 * MS)


def test_summary_percentiles():
    g = AlignmentGate(max_offset_ms=10.0)
    g.add_reference(0)
    for us in range(1, 101):  # offsets 1..100 us
        g.check(us * 1000)
    s = g.stats.summary()
    assert s['p50_ms'] == pytest.approx(0.050)
    assert s['p99_ms'] == pytest.approx(0.099)
    assert s['max_ms'] == pytest.approx(0.100)


def test_rate_monitor():
    m = RateMonitor(expected_hz=1000.0)
    for i in range(1000):
        m.add(i * MS)
    assert m.rate_hz() == pytest.approx(1000.0, rel=1e-3)
    j = m.jitter_ms()
    assert j['max'] == 0.0
    m.add(1000 * MS + 5 * MS)  # 6 ms gap
    assert m.max_gap_ns == 6 * MS
    assert m.jitter_ms()['max'] == pytest.approx(5.0)
    assert m.stale_ns(1010 * MS) == 5 * MS
    m.reset_max()
    assert m.max_gap_ns == 0


def test_rate_monitor_empty():
    m = RateMonitor(100.0)
    assert m.rate_hz() == 0.0
    assert math.isnan(m.jitter_ms()['p99'])
    assert m.stale_ns(0) is None
