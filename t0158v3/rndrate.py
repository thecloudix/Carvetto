#!/usr/bin/env python3
# rndrate.py RNDOUT: random-scene change rate branch (log) vs main (logoff) by mode (m0 single, m1 few, m2 many, m3 pages), and
# start-moved count (first T line differing > 1e-6).
import sys, glob, os, collections, re, math
O = sys.argv[1]; st = collections.defaultdict(collections.Counter)
def parse(p):
    r = {}; calls = collections.defaultdict(list)
    for ln in open(p, errors='replace'):
        f = ln.split()
        if not f: continue
        if f[0] == 'T' and len(f) >= 19: calls[f[1]].append((float(f[9]), float(f[10])))
        elif f[0] == 'R': r[f[1]] = (f[7], calls.pop(f[1], []), f[9])
    return r
for p in sorted(glob.glob(f'{O}/log/*.txt')):
    q = p.replace('/log/', '/logoff/')
    if not os.path.exists(q): continue
    a, b = parse(p), parse(q)
    for t in a:
        if t not in b: continue
        m = re.match(r'r\d+:\d+:(m\d)', t).group(1); st[m]['scenes'] += 1
        if a[t][0] != b[t][0]:
            st[m]['changed'] += 1
            if a[t][2] != b[t][2]: st[m]['leftchanged'] += 1
tot = collections.Counter()
for m in sorted(st):
    tot.update(st[m]); print(m, dict(st[m]), '%.1f%%' % (100.0 * st[m]['changed'] / max(1, st[m]['scenes'])))
print('all', dict(tot), '%.1f%%' % (100.0 * tot['changed'] / max(1, tot['scenes'])))
