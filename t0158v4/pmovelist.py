#!/usr/bin/env python3
# pmovelist.py DIR kind var thr: scenes (and first moved call's rule) whose start moves > thr under the kind transforms (pmove.py logic).
import sys, glob, collections, math, os
D, K, V, THR = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4])
for p in sorted(glob.glob(D + '/*.txt')):
    runs = collections.OrderedDict(); calls = collections.defaultdict(list)
    for ln in open(p, errors='replace'):
        f = ln.split()
        if not f: continue
        if f[0] == 'T' and len(f) >= 19: calls[f[1]].append((int(f[3]), float(f[9]), float(f[10])))
        elif f[0] == 'R': runs[f[1]] = calls.pop(f[1], [])
    g = collections.defaultdict(dict)
    for t, v in runs.items():
        b, x = t.rsplit(':', 1); g[b][x] = v
    for b, gg in g.items():
        if not b.endswith(':' + V) or 'id' not in gg: continue
        c0 = gg['id']; worst = 0; info = None
        for x, c in gg.items():
            if x == 'id' or (K == 'scale') != (x[0] in 'ur') or len(c) != len(c0): continue
            for i, (a, q) in enumerate(zip(c0, c)):
                d = math.hypot(a[1] - q[1], a[2] - q[2])
                if d > 1e-6:
                    if d > worst: worst, info = d, (x, i, a[0], q[0])
                    break
        if worst > THR: print(os.path.basename(p), b, round(worst, 6), info)
