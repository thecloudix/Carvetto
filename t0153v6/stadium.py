#!/usr/bin/env python3
# T-0153 r5: stadium frame (outer radius R+g, hole radius R, cap centres at x=+-L) with islands that are x-slices
# [a,b] of the hole inset by `ins` (a <= -L takes the whole left cap, b >= L the whole right cap).
# usage: stadium.py L R g "a1:b1,a2:b2,..." ins [rot_deg] > file
import math, sys
L, R, g = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
iv = [tuple(float(t) for t in v.split(':')) for v in sys.argv[4].split(',')] if sys.argv[4] else []
ins = float(sys.argv[5]); rot = math.radians(float(sys.argv[6])) if len(sys.argv) > 6 else 0.0
c, s = math.cos(rot), math.sin(rot)
def P(x, y): return (c * x - s * y, s * x + c * y)
def line(a, b): a, b = P(*a), P(*b); print(f"    L {a[0]:.17g} {a[1]:.17g} {b[0]:.17g} {b[1]:.17g}")
def arc(cx, cy, r, a0, a1, cw): q = P(cx, cy); print(f"    A {q[0]:.17g} {q[1]:.17g} r {r:.17g} {a0 + rot:.17g} {a1 + rot:.17g} dir {1 if cw else 0}")
n = 0
def head():
    global n; print(f"contour {n} closed=1"); n += 1
def stad(r, cw):
    head()
    if not cw:
        line((-L, -r), (L, -r)); arc(L, 0, r, -math.pi / 2, math.pi / 2, False); line((L, r), (-L, r)); arc(-L, 0, r, math.pi / 2, 1.5 * math.pi, False)
    else:
        line((-L, r), (L, r)); arc(L, 0, r, math.pi / 2, -math.pi / 2, True); line((L, -r), (-L, -r)); arc(-L, 0, r, 1.5 * math.pi, math.pi / 2, True)
stad(R + g, False); stad(R, True)
y = R - ins
for a, b in iv:
    head()
    a2, b2 = max(a, -L), min(b, L)
    line((a2, -y), (b2, -y))
    if b >= L: arc(L, 0, y, -math.pi / 2, math.pi / 2, False)
    else: line((b2, -y), (b2, y))
    line((b2, y), (a2, y))
    if a <= -L: arc(-L, 0, y, math.pi / 2, 1.5 * math.pi, False)
    else: line((a2, y), (a2, -y))
