#!/usr/bin/env python3
"""near.py <tag v9|g2> <seed> <join> <side> <x> <y> [radius]: polygonised input spans and branch/main output spans near (x,y).
Uses x2 v9one/g2one (POLY=1) for the input and x2 mainone OUTS=1 for both outputs."""
import sys, subprocess, re, math, os
D = '/home/chris/wt-T0009r4/build-verify4-drv'
tag, seed, j, s, x, y = sys.argv[1:7]; R = float(sys.argv[7]) if len(sys.argv) > 7 else 0.05
P = (float(x), float(y))
env = dict(os.environ, POLY='1')
a = subprocess.run([D + '/x2', tag + 'one', seed, j, s], capture_output=True, text=True, env=env, timeout=600).stdout
src = '0' if tag == 'g2' else '1'
b = subprocess.run([D + '/x2', 'mainone', seed, src, j, s], capture_output=True, text=True, env=dict(os.environ, OUTS='1'), timeout=600).stdout
def dpt(p, q): return math.hypot(p[0] - q[0], p[1] - q[1])
def dseg(p, a, b):
    vx, vy = b[0] - a[0], b[1] - a[1]; L2 = vx * vx + vy * vy
    t = 0 if L2 == 0 else max(0, min(1, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / L2))
    return dpt(p, (a[0] + t * vx, a[1] + t * vy))
i = 0; prev = None
for l in a.splitlines():
    if l.startswith('d=') or l.startswith('Contour'): print(l)
    m = re.match(r'in.spans.emplace_back\(LineSpan\{\{(\S+), (\S+)\}, \{(\S+), (\S+)\}\}\);', l)
    m2 = re.match(r'in.spans.emplace_back\(ArcSpan\{\{(\S+), (\S+)\}, (\S+), (\S+), (\S+), ArcDirection::(\w+)\}\);', l)
    if m:
        q = list(map(float, m.groups())); A, B = q[:2], q[2:]
        L = dpt(A, B); dd = dseg(P, A, B); h = math.degrees(math.atan2(B[1] - A[1], B[0] - A[0]))
        turn = '' if prev is None else ' turn %.1f' % (((h - prev + 540) % 360) - 180)
        if dd < R: print('in %d line len %.3g dist %.4g dir %.1f%s (%.12g,%.12g)->(%.12g,%.12g)' % (i, L, dd, h, turn, A[0], A[1], B[0], B[1]))
        prev = h; i += 1
    elif m2:
        q = m2.groups(); c = (float(q[0]), float(q[1])); r = float(q[2])
        dd = abs(dpt(P, c) - r)
        if dd < R or dpt(P, c) < R: print('in %d arc c(%.12g,%.12g) r %.4g sa %s ea %s %s' % (i, c[0], c[1], r, q[3], q[4], q[5]))
        prev = None; i += 1
    if l.startswith('CORE') or l.startswith('TIE'): print(l[:600])
who = ''
for l in b.splitlines():
    if l.startswith('branch') or l.startswith('main'): who = l.split(':')[0].strip(); print(l[:400])
    m = re.match(r'\s+k(\d) \((\S+),(\S+)\)->\((\S+),(\S+)\) c\((\S+),(\S+)\) r (\S+) sw (\S+) len (\S+)', l)
    if m:
        q = m.groups(); A = (float(q[1]), float(q[2])); B = (float(q[3]), float(q[4]))
        if min(dpt(P, A), dpt(P, B), dseg(P, A, B)) < R: print('  %s %s' % (who, l.strip()[:200]))
