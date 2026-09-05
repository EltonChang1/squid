#!/usr/bin/env python3
"""Sanity-check rendered STLs: parse (binary or ASCII), report bounding boxes and
triangle counts, and assert key blueprint dimensions.

    python3 cad/check_stl.py cad/stl
"""

from __future__ import annotations

import os
import struct
import sys

# Expected bounding-box extents in mm (x, y, z) with tolerance. Derived from
# common.scad parameters; update together with the CAD.
EXPECTED = {
    # finger: body_w = 20+2*(3+4) = 34, body_d = 15+14 = 29 (+ dovetail 4 on -Y),
    # body_h = 35+4.5+1+2 = 42.5
    'finger_body.stl': ((34.0, 33.0, 42.5), 2.5),
    # rail_len = 85 + 2*30.8 + 10 = 156.6, base_w 34, tower to z=24
    'rail_base.stl': ((156.6, 34.0, 24.0), 1.5),
    'carriage_adapter.stl': ((30.8, 23.0, 36.0), 1.5),
    'trigger.stl': ((70.0, 29.4, 8.0), 2.0),
    'ft_plate.stl': ((50.0, 50.0, 10.0), 1.0),
}


def read_stl(path: str):
    with open(path, 'rb') as fh:
        data = fh.read()
    if len(data) >= 84:
        n = struct.unpack_from('<I', data, 80)[0]
        if 84 + n * 50 == len(data):
            tris = []
            off = 84
            for _ in range(n):
                vals = struct.unpack_from('<12f', data, off)
                tris.append(vals[3:12])
                off += 50
            return tris
    # ASCII fallback
    tris, cur = [], []
    for line in data.decode('ascii', 'ignore').splitlines():
        parts = line.strip().split()
        if parts and parts[0] == 'vertex':
            cur.extend(float(v) for v in parts[1:4])
            if len(cur) == 9:
                tris.append(tuple(cur))
                cur = []
    return tris


def bbox(tris):
    xs = [v for t in tris for v in t[0::3]]
    ys = [v for t in tris for v in t[1::3]]
    zs = [v for t in tris for v in t[2::3]]
    return (max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))


def main(stl_dir: str) -> int:
    failures = 0
    files = sorted(f for f in os.listdir(stl_dir) if f.endswith('.stl'))
    if not files:
        print(f'no STL files in {stl_dir}')
        return 1
    for f in files:
        tris = read_stl(os.path.join(stl_dir, f))
        if not tris:
            print(f'FAIL {f}: no triangles')
            failures += 1
            continue
        bx = bbox(tris)
        msg = f'{f:<26} tris={len(tris):>7}  bbox={bx[0]:7.2f} x {bx[1]:7.2f} x {bx[2]:7.2f} mm'
        if f in EXPECTED:
            exp, tol = EXPECTED[f]
            ok = all(abs(a - b) <= tol for a, b in zip(bx, exp))
            msg += f'  expected~{exp} tol={tol}  {"OK" if ok else "MISMATCH"}'
            if not ok:
                failures += 1
        print(msg)
    print('all checks passed' if failures == 0 else f'{failures} check(s) failed')
    return 0 if failures == 0 else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else 'stl'))
