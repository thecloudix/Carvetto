#!/usr/bin/env python3
# hstcmp.py h.txt : hstored.sh output, br vs base per env/class (moves, non-loose diagnostics, loose messages).
import sys, collections
d = {}
for l in open(sys.argv[1]):
    t = l.split()
    if len(t) < 4: continue
    v, e, f = t[0], t[1], t[2]
    if 'H' not in t: d[(v, e, f)] = None; continue
    i = t.index('|'); d[(v, e, f)] = (t[6], t[i + 1], t[i + 2], t[i + 3], int(t[i + 4]), t[i + 5])
c = collections.Counter(); ex = collections.defaultdict(list)
for (v, e, f), x in d.items():
    if v != 'br': continue
    y = d.get(('base', e, f), 'MISSING')
    if x is None or y is None or y == 'MISSING': c[('nohash', e)] += 1; continue
    cls = x[0]; c[(e, cls, 'n')] += 1
    if x[1] != y[1]: c[(e, cls, 'MOVES DIFF')] += 1; ex['moves'].append((e, f))
    if x[3] != y[3]: c[(e, cls, 'OTHER DIAG DIFF')] += 1; ex['diag'].append((e, f))
    if y[4] > 0 and x[5] != y[5]: c[(e, cls, 'LOOSE MSG DIFF vs base')] += 1; ex['loose'].append((e, f))
    if x[4] > y[4]: c[(e, cls, 'gained loose')] += 1
    if x[4] < y[4]: c[(e, cls, 'LOST loose')] += 1
for k in sorted(c, key=str): print(k, c[k])
for k in ex: print(k, len(ex[k]), ex[k][:6])
