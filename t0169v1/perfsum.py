#!/usr/bin/env python3
# usage: perfsum.py <perfi output>: per case min / median CPU main vs branch, ratio of mins, hash/moves equality
import sys, re, collections, statistics
t = collections.defaultdict(lambda: {'main': [], 'br': []}); info = collections.defaultdict(dict)
for l in open(sys.argv[1]):
    if not l.startswith('r'): continue
    p = [x.strip() for x in l.split('|')]; v = p[0].split()[1]; case = p[1]; out = p[2]
    m = re.search(r'cpu ([0-9.]+)', out) or re.search(r'([0-9.]+) s CPU', out)
    if not m: continue
    t[case][v].append(float(m.group(1)))
    h = re.search(r'hash (\S+)', out); mv = re.search(r'moves (\d+)', out); rt = re.search(r'retr(?:acts)? (\d+)', out)
    info[case][v] = (h.group(1) if h else '', mv.group(1) if mv else '', rt.group(1) if rt else '')
print('%-46s %8s %8s %8s %8s %6s %s' % ('case', 'minMain', 'minBr', 'medMain', 'medBr', 'ratio', 'same output'))
for c, d in t.items():
    if not d['main'] or not d['br']: continue
    a, b = min(d['main']), min(d['br'])
    print('%-46s %8.3f %8.3f %8.3f %8.3f %6.3f %s' % (c[:46], a, b, statistics.median(d['main']), statistics.median(d['br']), b / a if a > 0 else 0, 'same' if info[c].get('main') == info[c].get('br') else 'DIFF %s vs %s' % (info[c].get('main'), info[c].get('br'))))
