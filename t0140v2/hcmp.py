#!/usr/bin/env python3
# hcmp.py DIR [A B]: compares A_* (br) vs B_* (base) H lines of hrand.sh (moves hash, non-loose diagnostics hash, loose messages) per class.
import sys, glob, os, collections
D = sys.argv[1]; A = sys.argv[2] if len(sys.argv) > 2 else 'br'; Bv = sys.argv[3] if len(sys.argv) > 3 else 'base'; c = collections.Counter(); ex = collections.defaultdict(list)
def rd(p):
    d = {}
    for l in open(p):
        t = l.split()
        if not l.startswith('H ') or '|' not in t: continue
        i = t.index('|'); d[(t[1], t[2])] = (t[3], t[i + 1], t[i + 3], int(t[i + 4]), t[i + 5])
    return d
for p in sorted(glob.glob(f'{D}/{A}_*.txt')):
    q = p.replace(f'/{A}_', f'/{Bv}_')
    if not os.path.exists(q): c['pending'] += 1; continue
    a = rd(p); b = rd(q); tag = os.path.basename(p)[len(A) + 1:-4]
    for k in a:
        if k not in b: c['missing in base'] += 1; continue
        x, y = a[k], b[k]; cls = x[0]; env = tag.split('_')[-1]; c[(cls, env, 'n')] += 1
        if x[1] != y[1]: c[(cls, env, 'MOVES DIFF')] += 1; ex['moves'].append((tag, k))
        if x[2] != y[2]: c[(cls, env, 'OTHER DIAG DIFF')] += 1; ex['diag'].append((tag, k))
        if y[3] and x[4] != y[4]: c[(cls, env, 'LOOSE MSGS CHANGED')] += 1; ex['loose'].append((tag, k))
        if x[3] > y[3]: c[(cls, env, 'gained loose')] += 1
        if x[3] < y[3]: c[(cls, env, 'LOST loose')] += 1
    for k in b:
        if k not in a: c['missing in br'] += 1
for k in sorted(c, key=str): print(k, c[k])
for k in ex: print(k, len(ex[k]), ex[k][:8])
