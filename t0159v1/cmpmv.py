#!/usr/bin/env python3
# cmpmv.py A.txt B.txt [tol]: compares two mv.cpp dumps after dropping line moves shorter than tol
# from the previous position. Prints SAME-UPTO-TINY or the first difference.
import sys, math
tol = float(sys.argv[3]) if len(sys.argv) > 3 else 1e-6
KINDS = (1, 4, 5) if len(sys.argv) > 4 and sys.argv[4] == "any" else (1,)  # "any": also drop tiny arcs/ramps
def load(f):
    mv, dg, prev = [], [], None
    for l in open(f):
        if l.startswith('D '): dg.append(l.strip()); continue
        t = l.split(); k = int(t[0]); x, y, z = map(float, t[1:4]); cx, cy = float(t[5]), float(t[6])
        if prev and k in KINDS and math.dist(prev, (x, y, z)) < tol: prev = (x, y, z); continue
        mv.append((k, x, y, z, cx, cy)); prev = (x, y, z)
    return mv, dg
a, da = load(sys.argv[1]); b, db = load(sys.argv[2])
if len(a) == len(b) and all(p[0] == q[0] and max(abs(u - v) for u, v in zip(p[1:], q[1:])) < tol for p, q in zip(a, b)):
    print('SAME-UPTO-TINY' + ('' if da == db else ' (diagnostics differ)'))
else:
    n = next((i for i, (p, q) in enumerate(zip(a, b)) if p[0] != q[0] or max(abs(u - v) for u, v in zip(p[1:], q[1:])) >= tol), min(len(a), len(b)))
    print(f'DIFF at {n}: lens {len(a)} {len(b)}', a[n] if n < len(a) else None, b[n] if n < len(b) else None)
