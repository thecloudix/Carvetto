#!/usr/bin/env python3
# delta54.py: stored runs (scene VAR) whose output hash differs round 5 (log) vs round 4 (r4 = 999a90e, main c2aa6f4 base).
import glob, os
O = '/home/chris/wt-T0158/build-verify5-rel/out'
lst = [l.strip() for l in open('/home/chris/harness/t0158v5/stored_scenes.lst')]
byname = {f.replace('/home/chris/', '').replace('/', '_'): f for f in lst}
def parse(p):
    r = {}
    for ln in open(p, errors='replace'):
        f = ln.split()
        if f and f[0] == 'R': r[f[1]] = f[7]
    return r
for p in sorted(glob.glob(f'{O}/scn/log/*.txt')):
    n = os.path.basename(p)[:-4]; q = f'{O}/scn_r4/r4/{n}.txt'
    if not os.path.exists(q): continue
    a, b = parse(p), parse(q); m = f'{O}/scn/logoff/{n}.txt'; c = parse(m) if os.path.exists(m) else {}
    for t in a:
        if t in b and a[t] != b[t] and (t not in c or a[t] != c[t]): print(byname.get(n, n), t.rsplit(':', 1)[1])
