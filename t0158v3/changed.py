#!/usr/bin/env python3
# changed.py SCNOUT: list "scenefile VAR" for stored runs whose output hash differs branch (log) vs main (logoff), plain build.
# Also prints change-rate summary per variant to stderr (orig = as stored, vsout/bbc/opt = st VARS variants).
import sys, glob, collections, os
O = sys.argv[1]; st = collections.defaultdict(collections.Counter)
def parse(p):
    r = {}
    for ln in open(p, errors='replace'):
        f = ln.split()
        if f and f[0] == 'R': r[f[1]] = (f[7], f[9])
    return r
lst = [l.strip() for l in open('/home/chris/wt-T0158/build-verify3-rel/stored_scenes.lst')]
byname = {f.replace('/home/chris/', '').replace('/', '_'): f for f in lst}
for p in sorted(glob.glob(f'{O}/log/*.txt')):
    q = p.replace('/log/', '/logoff/'); n = os.path.basename(p)[:-4]
    if not os.path.exists(q): continue
    a, b = parse(p), parse(q)
    for t in a:
        if t not in b: continue
        var = t.rsplit(':', 1)[1]; st[var]['runs'] += 1
        if a[t][0] != b[t][0]:
            st[var]['changed'] += 1
            if a[t][1] != b[t][1]: st[var]['leftchanged'] += 1
            print(byname.get(n, n), var)
tot = collections.Counter()
for v in st:
    tot.update(st[v]); print(v, dict(st[v]), '%.1f%%' % (100.0 * st[v]['changed'] / max(1, st[v]['runs'])), file=sys.stderr)
print('all', dict(tot), '%.1f%%' % (100.0 * tot['changed'] / max(1, tot['runs'])), file=sys.stderr)
