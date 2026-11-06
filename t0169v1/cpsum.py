#!/usr/bin/env python3
# usage: cpsum.py <coderprobe output>: per seed min CPU main/branch, ratios, totals
import sys, collections
d = collections.defaultdict(lambda: {'main': [], 'br': []}); h = collections.defaultdict(dict)
for l in open(sys.argv[1]):
    t = l.split()
    if len(t) < 4 or t[0] not in ('main', 'br'): continue
    s = t[3]; d[s][t[0]].append(float(t[t.index('cpu') + 1])); h[s][t[0]] = t[t.index('hash') + 1]
rows = [(min(v['br']) / min(v['main']), min(v['br']), min(v['main']), s, h[s]['br'] == h[s]['main']) for s, v in d.items() if v['br'] and v['main'] and min(v['main']) > 0]
rows.sort(reverse=True)
for r in rows[:8]: print('ratio %.3f br %.3f main %.3f seed %s same %s' % r)
print('seeds', len(rows), 'total br %.2f main %.2f ratio %.3f' % (sum(r[1] for r in rows), sum(r[2] for r in rows), sum(r[1] for r in rows) / sum(r[2] for r in rows)))
big = [r for r in rows if r[2] > 0.2]; print('seeds with main > 0.2 s:', len(big), 'max ratio %.3f' % max([r[0] for r in big] or [0]))
