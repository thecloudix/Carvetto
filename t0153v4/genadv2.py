#!/usr/bin/env python3
# Natural (no fault injection) attempt on the safety net: a near-full D-hole hugging its circle (misread at a coarse
# chordal with a small distance tolerance) with an island where the hole's skeleton is deepest.
import math, os, sys
out = sys.argv[1]; os.makedirs(out, exist_ok=True)
def poly(pts):
    return "contour 0 closed=1\n" + "\n".join("    L %.17g %.17g %.17g %.17g" % (pts[i][0], pts[i][1], pts[(i+1)%len(pts)][0], pts[(i+1)%len(pts)][1]) for i in range(len(pts)))
def circ(c, r, a0=0.0, cw=False):
    return "contour 0 closed=1\n    A %.17g %.17g r %.17g %.17g %.17g dir %d" % (c[0], c[1], r, a0, a0 - 2*math.pi if cw else a0 + 2*math.pi, 1 if cw else 0)
def arcpts(r, a0, a1, n): return [(r*math.cos(a0+(a1-a0)*i/n), r*math.sin(a0+(a1-a0)*i/n)) for i in range(n+1)]
R = 8.0
for g in [0.0005, 0.002, 0.004, 0.008]:
    for S in [5.0, 5.5, 6.0]:
        for isl in [0, 1.0, 2.5]:
            p = arcpts(R - g, 0.4, 0.4 + S, 170); p = p[86:] + p[:86]
            cs = [circ((0, 0), R, 1.1), poly(p)]
            if isl: cs.append(circ((0.05, -0.03), isl, 0.7))
            open(os.path.join(out, "Disl_g%g_S%g_i%g.txt" % (g, S, isl)), "w").write("\n".join(cs) + "\n")
