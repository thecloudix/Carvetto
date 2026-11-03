#!/usr/bin/env python3
# difflist.py OUTDIR A B : prints "seed mode env sc" for scenes whose dl3 hash differs between builds A and B
import sys, os
D, A, B = sys.argv[1:4]
def load(v):
    r = {}
    for f in os.listdir(D):
        if not f.startswith(v + '_'): continue
        tag = f[len(v) + 1:-4]
        for l in open(os.path.join(D, f)):
            if l.startswith('H ') and 'PV' in l: h = l.split(); r[(tag, h[2])] = l.split('|')[1]
    return r
a, b = load(A), load(B)
for k in sorted(set(a) & set(b)):
    if a[k] != b[k]:
        s, m, e = k[0].split('_', 2); print(s, m, e, k[1])
