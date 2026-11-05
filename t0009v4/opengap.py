#!/usr/bin/env python3
"""opengap.py <v9|g2> <seed> <join> <side>: branch open chains whose end comes back within 0.01 mm of their start (near-closed),
and main's closed chains, from x2 mainone OUTS=1."""
import sys, subprocess, re, math, os
D = '/home/chris/wt-T0009r4/build-verify4-drv'
tag, seed, j, s = sys.argv[1:5]
out = subprocess.run([D + '/x2', 'mainone', seed, '0' if tag == 'g2' else '1', j, s], capture_output=True, text=True, env=dict(os.environ, OUTS='1'), timeout=900).stdout
who = None; ch = []
for l in out.splitlines():
    if l.startswith('branch') or l.startswith('main'): who = l.split(':')[0].strip()
    m = re.match(r'\s+chain closed (\d)', l)
    if m: ch.append([who, int(m.group(1)), []]); continue
    m = re.match(r'\s+k(\d) \((\S+),(\S+)\)->\((\S+),(\S+)\) c\(\S+\) r (\S+) sw (\S+) len (\S+)', l)
    if m: ch[-1][2].append(tuple(map(float, m.groups()[1:5])) + (float(m.group(8)),))
for w, c, sp in ch:
    if not sp: continue
    a = sp[0][:2]; b = sp[-1][2:4]; g = math.hypot(a[0] - b[0], a[1] - b[1]); L = sum(x[4] for x in sp)
    if (w == 'branch' and not c and g < 0.01 and L > 3 * g) or (w == 'main' and c):
        print('%s %s %s %s: %s %s spans %d len %.4g gap %.3g at (%.9f,%.9f)' % (tag, seed, j, s, w, 'closed' if c else 'OPEN', len(sp), L, g, a[0], a[1]))
