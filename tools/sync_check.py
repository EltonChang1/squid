#!/usr/bin/env python3
"""Offline MCAP timestamp verifier for Tactile-UMI sessions.

Reads a rosbag2 MCAP (ROS 2 CDR messages) with ``mcap-ros2-support`` and
reports, per topic: message count, effective rate, inter-message jitter
(deviation from the nominal period, p50/p99/max) and the largest gap. For each
tactile image topic it computes the offset from every image header stamp to the
nearest ``/wrist/wrench`` header stamp and prints a histogram.

Exit status is 0 when the image<->wrench p99 offset is below ``--max-p99-ms``
(default 1.0 ms, the blueprint's "Timestamp Synchronization" acceptance
criterion) for every tactile stream, otherwise 1.

Usage:
    python3 tools/sync_check.py /data/sessions/session_*/trajectory/trajectory_0.mcap
    python3 tools/sync_check.py <session_dir>          # finds the .mcap inside
    python3 tools/sync_check.py bag.mcap --json report.json

Requires: pip install mcap mcap-ros2-support   (no ROS installation needed)
"""

from __future__ import annotations

import argparse
import bisect
import glob
import json
import math
import os
import sys
from collections import defaultdict
from typing import Dict, List, Optional

try:
    from mcap_ros2.reader import read_ros2_messages
except ImportError:  # pragma: no cover
    sys.stderr.write('missing dependency: pip install mcap mcap-ros2-support\n')
    raise

NOMINAL_HZ = {
    '/tactile/left/image_raw': 120.0,
    '/tactile/right/image_raw': 120.0,
    '/wrist/image_raw': 60.0,
    '/wrist/wrench': 1000.0,
    '/gripper/state': 1000.0,
    '/pose/end_effector': 200.0,
    '/system/sync_status': 10.0,
}
TACTILE = ('/tactile/left/image_raw', '/tactile/right/image_raw')
REFERENCE = '/wrist/wrench'
NS_PER_MS = 1e6


def _pct(sorted_vals: List[float], p: float) -> float:
    if not sorted_vals:
        return float('nan')
    k = min(len(sorted_vals) - 1, max(0, int(math.ceil(p / 100.0 * len(sorted_vals))) - 1))
    return sorted_vals[k]


def _header_stamp_ns(msg) -> Optional[int]:
    h = getattr(msg, 'header', None)
    if h is None:
        return None
    return int(h.stamp.sec) * 1_000_000_000 + int(h.stamp.nanosec)


def resolve_mcap(path: str) -> str:
    if os.path.isfile(path):
        return path
    hits = sorted(glob.glob(os.path.join(path, '**', '*.mcap'), recursive=True))
    if not hits:
        raise FileNotFoundError(f'no .mcap under {path}')
    return hits[0]


def load_stamps(mcap_path: str) -> Dict[str, List[int]]:
    stamps: Dict[str, List[int]] = defaultdict(list)
    for m in read_ros2_messages(mcap_path):
        topic = m.channel.topic
        ns = _header_stamp_ns(m.ros_msg)
        if ns is None:
            ns = m.log_time_ns
        stamps[topic].append(ns)
    for t in stamps:
        stamps[t].sort()
    return stamps


def stream_stats(stamps: List[int], nominal_hz: Optional[float]) -> dict:
    n = len(stamps)
    out = {'count': n}
    if n < 2:
        return out
    span_s = (stamps[-1] - stamps[0]) * 1e-9
    out['duration_s'] = span_s
    out['rate_hz'] = (n - 1) / span_s if span_s > 0 else float('nan')
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    out['max_gap_ms'] = max(gaps) / NS_PER_MS
    if nominal_hz:
        period = 1e9 / nominal_hz
        devs = sorted(abs(g - period) / NS_PER_MS for g in gaps)
        out['jitter_p50_ms'] = _pct(devs, 50)
        out['jitter_p99_ms'] = _pct(devs, 99)
        out['jitter_max_ms'] = devs[-1]
        # A "drop" is a gap of more than 1.5 nominal periods.
        out['dropped_est'] = sum(int(round(g / period)) - 1 for g in gaps if g > 1.5 * period)
    return out


def nearest_offsets_ms(images: List[int], ref: List[int]) -> List[float]:
    out = []
    for s in images:
        i = bisect.bisect_left(ref, s)
        cands = []
        if i < len(ref):
            cands.append(ref[i] - s)
        if i > 0:
            cands.append(ref[i - 1] - s)
        if cands:
            out.append(min(cands, key=abs) / NS_PER_MS)
    return out


def histogram(values_ms: List[float], edges_ms=(0.1, 0.25, 0.5, 1.0, 2.0, 5.0)) -> List[tuple]:
    absvals = [abs(v) for v in values_ms]
    rows = []
    lo = 0.0
    for hi in edges_ms:
        rows.append((lo, hi, sum(1 for v in absvals if lo <= v < hi)))
        lo = hi
    rows.append((lo, float('inf'), sum(1 for v in absvals if v >= lo)))
    return rows


def analyse(mcap_path: str, max_p99_ms: float) -> dict:
    stamps = load_stamps(mcap_path)
    report = {'file': mcap_path, 'streams': {}, 'alignment': {}, 'pass': True}
    for topic in sorted(stamps):
        report['streams'][topic] = stream_stats(stamps[topic], NOMINAL_HZ.get(topic))
    ref = stamps.get(REFERENCE, [])
    for topic in TACTILE:
        if topic not in stamps:
            continue
        offs = nearest_offsets_ms(stamps[topic], ref)
        so = sorted(abs(o) for o in offs)
        mean_signed = sum(offs) / len(offs) if offs else float('nan')
        a = {
            'count': len(offs),
            'p50_ms': _pct(so, 50), 'p99_ms': _pct(so, 99),
            'max_ms': so[-1] if so else float('nan'),
            'mean_signed_ms': mean_signed,
            'over_2ms': sum(1 for v in so if v > 2.0),
            'histogram': histogram(offs),
        }
        a['pass'] = bool(so) and a['p99_ms'] < max_p99_ms
        report['alignment'][topic] = a
        report['pass'] = report['pass'] and a['pass']
    if not ref:
        report['pass'] = False
        report['error'] = f'reference topic {REFERENCE} missing'
    if not any(t in stamps for t in TACTILE):
        report['pass'] = False
        report['error'] = 'no tactile image topics found'
    return report


def print_report(r: dict, max_p99_ms: float) -> None:
    print(f'MCAP: {r["file"]}')
    print()
    print(f'{"topic":<28}{"count":>8}{"rate Hz":>10}{"jit p50":>9}{"jit p99":>9}{"jit max":>9}'
          f'{"max gap":>9}{"drops":>7}')
    for topic, s in r['streams'].items():
        def f(k, fmt='{:.3f}'):
            v = s.get(k)
            return '-' if v is None or (isinstance(v, float) and math.isnan(v)) else fmt.format(v)
        print(f'{topic:<28}{s["count"]:>8}{f("rate_hz", "{:.1f}"):>10}{f("jitter_p50_ms"):>9}'
              f'{f("jitter_p99_ms"):>9}{f("jitter_max_ms"):>9}{f("max_gap_ms"):>9}'
              f'{f("dropped_est", "{}"):>7}')
    print('\n(jitter = |gap - nominal period| in ms; drops estimated from gaps > 1.5 periods)\n')
    for topic, a in r['alignment'].items():
        verdict = 'PASS' if a['pass'] else 'FAIL'
        print(f'{topic} -> nearest {REFERENCE}: n={a["count"]} p50={a["p50_ms"]:.3f} ms '
              f'p99={a["p99_ms"]:.3f} ms max={a["max_ms"]:.3f} ms '
              f'mean(signed)={a["mean_signed_ms"]:+.3f} ms '
              f'over2ms={a["over_2ms"]}  [{verdict} @ p99 < {max_p99_ms} ms]')
        for lo, hi, n in a['histogram']:
            hi_s = 'inf' if math.isinf(hi) else f'{hi:g}'
            bar = '#' * min(60, int(60 * n / max(1, a['count'])))
            print(f'    [{lo:>5g}, {hi_s:>4}) ms {n:>7} {bar}')
    if 'error' in r:
        print(f'\nERROR: {r["error"]}')
    print(f'\nOVERALL: {"PASS" if r["pass"] else "FAIL"}')


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('path', help='.mcap file or a session directory containing one')
    ap.add_argument('--max-p99-ms', type=float, default=1.0,
                    help='pass threshold on image<->wrench p99 offset (default 1.0)')
    ap.add_argument('--json', help='also write the report as JSON to this path')
    args = ap.parse_args(argv)

    mcap_path = resolve_mcap(args.path)
    report = analyse(mcap_path, args.max_p99_ms)
    print_report(report, args.max_p99_ms)
    if args.json:
        with open(args.json, 'w') as fh:
            json.dump(report, fh, indent=2, default=str)
    return 0 if report['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
