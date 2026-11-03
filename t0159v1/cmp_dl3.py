#!/usr/bin/env python3
# cmp_dl3.py OUTDIR: compares dl3 HASH outputs. Pairs: mainrel/brrel (expect bit-identical),
# mainv3/brv3 (left-out per scene: branch never more), brrel/brv3 (left-out spread between builds).
import sys, os, re, collections
D = sys.argv[1]
def load(v):
    r = {}
    for f in os.listdir(D):
        if not f.startswith(v + '_'): continue
        tag = f[len(v) + 1:]
        for line in open(os.path.join(D, f)):
            if not line.startswith('H ') or 'PV' not in line: continue
            p = line.split('|')
            h = line.split()
            key = (tag, h[1], h[2])
            mv = p[1].split(); pv = line.split('PV')[1].split()
            left = sum(int(x.split(':')[1].split('/')[0]) for x in pv if ':' in x and '/' in x)
            r[key] = (mv[0], mv[1], left, line.strip())
    return r
V = {v: load(v) for v in ['mainrel', 'brrel', 'mainv3', 'brv3', 'hybrel', 'hybv3']}
def cmp(a, b, strict):
    A, B = V[a], V[b]; keys = sorted(set(A) & set(B))
    diff = 0; more = 0; fewer = 0; ex = []
    for k in keys:
        if A[k][:2] != B[k][:2]:
            diff += 1
            if B[k][2] > A[k][2]: more += 1; ex.append(('MORE-LEFT', k, A[k][2], B[k][2]))
            elif B[k][2] < A[k][2]: fewer += 1; ex.append(('fewer-left', k, A[k][2], B[k][2]))
            else: ex.append(('same-left', k, A[k][2], B[k][2]))
    print(f'{a} vs {b}: scenes {len(keys)} differing {diff} | {b} leaves more out {more}, fewer {fewer}')
    for e in [x for x in ex if x[0] == 'MORE-LEFT'] + [x for x in ex if x[0] != 'MORE-LEFT'][:int(os.environ.get('NEX', '5'))]: print('   ', e)
cmp('mainrel', 'brrel', True); cmp('mainrel', 'hybrel', True); cmp('hybrel', 'brrel', True); cmp('mainv3', 'hybv3', False); cmp('hybv3', 'brv3', False); cmp('mainv3', 'brv3', False); cmp('brrel', 'brv3', False); cmp('mainrel', 'mainv3', False)
