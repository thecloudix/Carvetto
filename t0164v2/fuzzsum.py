#!/usr/bin/env python3
# t0164v2: summarise fuzz.txt: per mode, branch vs main accepted / bypass totals; per (file, c) accepted equality in normal mode
import re, sys, collections
rows = collections.defaultdict(dict); tot = collections.defaultdict(lambda: [0, 0, 0.0])
for l in open(sys.argv[1]):
    m = re.match(r'(\w+) \[(.*?)\] (\S+) c=(\S+) iters=(\d+) accepted=(\d+) BYPASS=(\d+) worstOutside=(\S+) worstDisc=(\S+)', l)
    if not m: continue
    v, mode, f, c, it, acc, byp, wo, wd = m.groups(); mode = mode or 'normal'
    rows[(mode, f.split('/')[-1], c)][v] = (int(acc), int(byp), float(wd))
    t = tot[(v, mode)]; t[0] += int(acc); t[1] += int(byp); t[2] = max(t[2], float(wd))
for k in sorted(tot): print(k, 'accepted=%d bypass=%d worstDisc=%.4f' % tuple(tot[k]))
for k, d in sorted(rows.items()):
    if k[0] == 'normal' and 'br' in d and 'main' in d and d['br'][0] != d['main'][0]: print('ACCEPT DIFF', k, d)
    if 'br' in d and d['br'][1] > 0: print('BR BYPASS', k, d)
