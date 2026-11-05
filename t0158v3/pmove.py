#!/usr/bin/env python3
# pmove.py DIR [kind]: absolute start moves under perturbation. For each scene/variant and each tieFreeStart call (same call
# count as 'id'), the move of the call's start (mapped back) vs 'id'; per family the max move per scene, bucketed, for
# kind 'scale' (u*, r*) or 'frame' (t*, q*). Only the FIRST moved call per run counts (later calls follow from it).
# Also reports call-count changes. Examples of the largest moves.
import sys, glob, collections, math, re, os
D = sys.argv[1]; K = sys.argv[2] if len(sys.argv) > 2 else 'scale'
B = [(1e-2, '>1e-2'), (1e-3, '>1e-3'), (1e-4, '>1e-4'), (1e-6, '>1e-6')]
st = collections.defaultdict(collections.Counter); ex = collections.defaultdict(list)
for p in sorted(glob.glob(D + '/*.txt')):
    base = os.path.basename(p); m = re.search(r'(gen\d)_([a-z0-9]+)_\d+\.scn', base); fam = (m.group(1) + ':' + m.group(2)) if m else 'stored'
    runs = collections.OrderedDict(); calls = collections.defaultdict(list)
    for ln in open(p, errors='replace'):
        f = ln.split()
        if not f: continue
        if f[0] == 'T' and len(f) >= 19: calls[f[1]].append((int(f[3]), float(f[9]), float(f[10])))
        elif f[0] == 'R': runs[f[1]] = calls.pop(f[1], [])
    groups = collections.defaultdict(dict)
    for t, v in runs.items():
        b, x = t.rsplit(':', 1); groups[b][x] = v
    for b, g in groups.items():
        if 'id' not in g: continue
        c0 = g['id']; worst = 0.0; wx = None; cc = 0
        for x, c in g.items():
            if x == 'id' or (K == 'scale') != (x[0] in 'ur'): continue
            if len(c) != len(c0): cc += 1; continue
            for i, (a, q) in enumerate(zip(c0, c)):
                d = math.hypot(a[1] - q[1], a[2] - q[2])
                if d > 1e-6:
                    if d > worst: worst, wx = d, (x, i, a[0], q[0])
                    break
        fam2 = fam if fam != 'stored' else 'stored:' + b.rsplit(':', 1)[-1]
        st[fam2]['scenes'] += 1; st[fam2]['callcount_changed'] += cc > 0
        for t, n in B:
            if worst > t: st[fam2][n] += 1; break
        if worst > 1e-4: ex[fam2].append((round(worst, 6), base[-36:], b[-26:], wx))
for fam in sorted(st):
    print(f'{fam:16s} ' + ' '.join(f'{k} {v}' for k, v in sorted(st[fam].items())))
    for e in sorted(ex[fam], reverse=True)[:3]: print('     e.g.', e)
