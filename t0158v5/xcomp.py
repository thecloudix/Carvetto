#!/usr/bin/env python3
# xcomp.py DIR A B: per run tag, compare R hash and call starts of config A vs B (e.g. log vs logv3 = GCC 15 plain vs
# -march=x86-64-v3 FMA). Counts runs with differing hash / differing start (> 1e-6), per family; examples.
import sys, glob, collections, math, re, os
D, A, Bc = sys.argv[1:4]
def parse(p):
    runs = {}; calls = collections.defaultdict(list)
    for ln in open(p, errors='replace'):
        f = ln.split()
        if not f: continue
        if f[0] == 'T' and len(f) >= 19: calls[f[1]].append((int(f[3]), float(f[9]), float(f[10])))
        elif f[0] == 'R': runs[f[1]] = (f[7], calls.pop(f[1], []), f[9])
    return runs
st = collections.defaultdict(collections.Counter); ex = collections.defaultdict(list)
for p in sorted(glob.glob(f'{D}/{A}/*.txt')):
    q = p.replace(f'/{A}/', f'/{Bc}/')
    if not os.path.exists(q): continue
    base = os.path.basename(p); m = re.search(r'(gen\d)_([a-z0-9]+)_\d+\.scn', base); fam = (m.group(1) + ':' + m.group(2)) if m else 'other'
    a, b = parse(p), parse(q)
    for t in a:
        if t not in b: st[fam]['missing'] += 1; continue
        fam2 = fam if fam != 'other' else 'other:' + (t.rsplit(':', 1)[-1] if not re.match(r'r\d+:\d+:m', t) else 'rnd')
        st[fam2]['runs'] += 1
        if a[t][0] != b[t][0]:
            st[fam2]['hashdiff'] += 1
            sd = len(a[t][1]) != len(b[t][1]) or any(math.hypot(x[1] - y[1], x[2] - y[2]) > 1e-6 for x, y in zip(a[t][1], b[t][1]))
            if sd:
                st[fam2]['startdiff'] += 1
                if len(ex[fam2]) < 4: ex[fam2].append((base[-40:], t[-40:], [x[0] for x in a[t][1]][:4]))
            if a[t][2] != b[t][2]: st[fam2]['leftdiff'] += 1
for fam in sorted(st):
    print(f'{fam:18s} ' + ' '.join(f'{k} {v}' for k, v in sorted(st[fam].items())))
    for e in ex[fam]: print('     e.g.', e)
