#!/usr/bin/env python3
# usage: cmp2.py <sweep dir>: changed cases WITHOUT a main clearance violation (worst >= 0): how they differ
import sys, glob, os
def load(f):
    out = {}
    for l in open(f):
        if l.startswith('case '):
            if ' worst ' not in l: continue
            t = l.split(); k = {key: t[t.index(key) + 1] for key in ['hash', 'retract', 'plunge', 'cut', 'moves', 'worst', 'ch', 'dist', 'strat', 'tools']}
            out[t[1]] = k
    return out
agg = dict(n=0, sameRP=0, dRet=0, dPl=0, moreRet=0, fewerRet=0, absCutMax=0.0, relCutMax=0.0)
ex = []
for fb in sorted(glob.glob(os.path.join(sys.argv[1], '*_br.txt'))):
    fm = fb[:-7] + '_main.txt'
    if not os.path.exists(fm): continue
    b, m = load(fb), load(fm)
    for k in b:
        if k not in m or b[k]['hash'] == m[k]['hash'] or float(m[k]['worst']) < 0: continue
        agg['n'] += 1; dr = int(b[k]['retract']) - int(m[k]['retract']); dp = int(b[k]['plunge']) - int(m[k]['plunge'])
        dc = float(b[k]['cut']) - float(m[k]['cut'])
        agg['dRet'] += dr; agg['dPl'] += dp; agg['sameRP'] += (dr == 0 and dp == 0); agg['moreRet'] += dr > 0; agg['fewerRet'] += dr < 0
        agg['absCutMax'] = max(agg['absCutMax'], abs(dc)); rc = abs(dc) / max(1e-9, float(m[k]['cut'])); agg['relCutMax'] = max(agg['relCutMax'], rc)
        ex.append((abs(dc), dr, dp, os.path.basename(fb)[:-7], k, m[k]['ch'], m[k]['dist'], m[k]['strat'], m[k]['tools'], int(b[k]['moves']) - int(m[k]['moves'])))
print(agg)
ex.sort(reverse=True)
for e in ex[:15]: print('dCut %.6f dRet %d dPl %d %s case %s ch %s dist %s strat %s tools %s dMoves %d' % e)
print('with retract/plunge change:'); [print('  dCut %.6f dRet %d dPl %d %s case %s ch %s dist %s strat %s tools %s dMoves %d' % e) for e in ex if e[1] or e[2]][:20]
