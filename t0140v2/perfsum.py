#!/usr/bin/env python3
# perfsum.py OUT... : per page: pairs, median/min/max of br/base CPU ratio (pairs in file order), base and br medians, loose count, moves hash equal.
import sys, statistics, collections
P = collections.OrderedDict()
for fn in sys.argv[1:]:
    rows = [l.split('|') for l in open(fn) if l.count('|') >= 3]
    for r in rows:
        v, f, e, res = r[0].strip(), r[1].strip(), r[2].strip(), r[3].split()
        if len(res) < 10: continue
        P.setdefault((f, e), {'base': [], 'br': [], 'loose': None, 'hash': set()})
        d = P[(f, e)]; d[v].append(float(res[4])); d['hash'].add((v, res[10]))
        if v == 'br': d['loose'] = res[8]
for (f, e), d in P.items():
    n = min(len(d['base']), len(d['br']))
    if n == 0: continue
    rat = [d['br'][i] / d['base'][i] for i in range(n)]
    hs = {h for _, h in d['hash']}
    print(f"{f:14s} {e:28s} n={n} base {statistics.median(d['base'][:n]):9.4f} br {statistics.median(d['br'][:n]):9.4f} ratio med {statistics.median(rat):.3f} min {min(rat):.3f} max {max(rat):.3f} loose {d['loose']} hash {'same' if len(hs) == 1 else 'DIFF'}")
