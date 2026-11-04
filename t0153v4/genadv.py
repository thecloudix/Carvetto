#!/usr/bin/env python3
# T-0153 round 4: adversarial hole geometry around B1 (holes hugging walls within chordal, thin tips, chords,
# nested, touching, both orientations).  usage: genadv.py <outdir>   writes <name>.txt in vc3's dump format.
import math, os, sys, random
out = sys.argv[1]; os.makedirs(out, exist_ok=True)
def poly(pts):  # closed polyline
    s = ["contour 0 closed=1"]
    for i in range(len(pts)):
        a = pts[i]; b = pts[(i + 1) % len(pts)]
        s.append("    L %.17g %.17g %.17g %.17g" % (a[0], a[1], b[0], b[1]))
    return "\n".join(s)
def circ(c, r, a0=0.0, cw=False):
    a1 = a0 - 2 * math.pi if cw else a0 + 2 * math.pi
    return "contour 0 closed=1\n    A %.17g %.17g r %.17g %.17g %.17g dir %d" % (c[0], c[1], r, a0, a1, 1 if cw else 0)
def arcpts(c, r, a0, a1, n):
    return [(c[0] + r * math.cos(a0 + (a1 - a0) * i / n), c[1] + r * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]
def rot(p, k): k %= len(p); return p[k:] + p[:k]
def w(name, conts):
    open(os.path.join(out, name + ".txt"), "w").write("\n".join(conts) + "\n")
R = 8.0; O = (0.0, 0.0)
# A: D-holes (arc at R-g hugging a circle, closed by a chord), sweeps up to nearly full, cw/ccw, rotated start
for g in [0.0005, 0.001, 0.002, 0.004, 0.01, 0.03, 0.06, 0.09]:
    for S in [0.3, 1.0, 2.5, 4.5, 6.0, 6.25]:
        for n in [24, 170]:
            for cw in [0, 1]:
                p = arcpts(O, R - g, 0.4, 0.4 + S, n)
                p = rot(p, n // 2 + 1)
                if cw: p = p[::-1]
                w("D_g%g_S%g_n%d_%s" % (g, S, n, "cw" if cw else "ccw"), [circ(O, R, 1.1), poly(p)])
# A2: D-hole hugging a square's corner region and a polygonised (many tiny spans) circle wall
for g in [0.001, 0.004, 0.02]:
    wall = arcpts(O, R, 0, 2 * math.pi, 600)[:-1]
    p = rot(arcpts(O, R - g, 1.0, 3.0, 300), 7)
    w("Dpolywall_g%g" % g, [poly(wall), poly(p)])
# B: crescent (lune) hole hugging the wall: thin tips at both ends
for g in [0.001, 0.004, 0.02, 0.06]:
    for t in [0.05, 0.2, 1.0]:  # max thickness
        a0, a1 = 0.5, 2.5; n = 200
        outer = arcpts(O, R - g, a0, a1, n)
        # inner arc: same end points, sagging by t at the middle -> radius through 3 points
        inner = []
        for i in range(n + 1):
            u = i / n; a = a0 + (a1 - a0) * u
            rr = R - g - t * math.sin(math.pi * u)
            inner.append((rr * math.cos(a), rr * math.sin(a)))
        p = outer + inner[::-1][1:-1]
        w("lune_g%g_t%g" % (g, t), [circ(O, R, 0.3), poly(rot(p, 37))])
        w("lune_g%g_t%g_cw" % (g, t), [circ(O, R, 0.3), poly(rot(p, 37)[::-1])])
# C: L-shaped hole hugging two walls of a square (corner) at gap g, and a frame hole hugging all four walls
for g in [0.0005, 0.002, 0.01, 0.05]:
    s = 10.0
    for th in [0.05, 0.3, 2.0]:
        Lh = [(g, g), (s - 3, g), (s - 3, g + th), (g + th, g + th), (g + th, s - 3), (g, s - 3)]
        w("Lhole_g%g_th%g" % (g, th), [poly([(0, 0), (s, 0), (s, s), (0, s)]), poly(Lh[::-1])])
    w("framehole_g%g" % g, [poly([(0, 0), (s, 0), (s, s), (0, s)]), poly([(g, g), (g, s - g), (s - g, s - g), (s - g, g)])])
    w("framehole_isl_g%g" % g, [poly([(0, 0), (s, 0), (s, s), (0, s)]), poly([(g, g), (g, s - g), (s - g, s - g), (s - g, g)]),
                                 poly([(2, 2), (s - 2, 2), (s - 2, s - 2), (2, s - 2)])])
    # three nested within g: outer, hole, island (sandwich, the offsets cross two walls)
    w("sandwich_g%g" % g, [poly([(0, 0), (s, 0), (s, s), (0, s)]), poly([(g, g), (g, s - g), (s - g, s - g), (s - g, g)]),
                           poly([(2 * g + 0.001, 2 * g + 0.001), (s - 2 * g - 0.001, 2 * g + 0.001), (s - 2 * g - 0.001, s - 2 * g - 0.001), (2 * g + 0.001, s - 2 * g - 0.001)])])
# F: annulus: circular hole at R-g inside circle R (region = thin ring), with and without an island inside
for g in [0.0005, 0.002, 0.01, 0.03, 0.06]:
    w("annulus_g%g" % g, [circ(O, R, 0.2), circ(O, R - g, 1.7, True)])
    w("annulus_isl_g%g" % g, [circ(O, R, 0.2), circ(O, R - g, 1.7, True), circ((0.3, 0.1), 3.0, 0.0)])
    w("annulus_ccw_g%g" % g, [circ(O, R, 0.2), circ(O, R - g, 1.7, False)])
    w("annulus_poly_g%g" % g, [circ(O, R, 0.2), poly(arcpts(O, R - g, 0.3, 0.3 + 2 * math.pi, 400)[:-1])])
# G: hole touching the wall (gap 0) along a chord / at one point; hole crossing the wall slightly
for d in [0.0, -0.0005, 0.0003]:
    p = arcpts(O, R + d, 0.4, 2.0, 100)
    w("Dtouch_d%g" % d, [circ(O, R, 1.1), poly(rot(p, 13))])
# H: two holes hugging each other (along a straight cut) and the outer circle, each a half-disc
for g in [0.0005, 0.002, 0.01]:
    top = [p for p in arcpts(O, R - g, math.asin(g / (R - g)), math.pi - math.asin(g / (R - g)), 200)]
    bot = [(x, -y) for (x, y) in top]
    w("halves_g%g" % g, [circ(O, R, 0.0), poly(rot(top, 50)), poly(rot(bot[::-1], 50))])
# I: thin holes (width 0.07..0.14) hugging the wall at gap g, for chordal 0.1
for g in [0.0005, 0.002, 0.01]:
    for t in [0.07, 0.1, 0.14, 0.2]:
        p = arcpts(O, R - g, 0.5, 2.0, 150) + arcpts(O, R - g - t, 2.0, 0.5, 150)
        w("band_g%g_t%g" % (g, t), [circ(O, R, 0.0), poly(rot(p, 11))])
# J: nested holes within a hole within an island etc, alternating, all hugging at g
for g in [0.0005, 0.002, 0.01, 0.05]:
    cs = []; r = R
    for k in range(5):
        cs.append(circ((0.0, 0.0), r, 0.3 * k, k % 2 == 1)); r -= g if k % 2 == 0 else 1.5
    w("nestalt_g%g" % g, cs)
# K: D-hole inside an island inside a hole (depth 3), hugging the island
for g in [0.001, 0.004, 0.02]:
    p = rot(arcpts(O, 4 - g, 0.4, 3.0, 170), 60)
    w("Ddeep_g%g" % g, [poly([(-10, -10), (10, -10), (10, 10), (-10, 10)]), circ(O, 7, 0.0, True), circ(O, 4, 0.5), poly(p)])
print(len(os.listdir(out)))
