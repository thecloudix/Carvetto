#!/usr/bin/env python3
# bdiff.py OUT: list runs whose first-call cut start differs plain vs v3 on the BRANCH (log vs logv3), with variant,
# rule, nearest-cutd hint (mCut), and the start mode of the scene; plus the "unexplained" br-vs-main changes with
# any fired call (dpt) detail.
import sys, os, glob, collections, math, re
sys.argv = [sys.argv[0]] + sys.argv[1:]
exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cmp2.py')).read().split('chg = {}')[0])
byv = collections.Counter(); rows = []
for k in keys:
    A, B = data['log'][k], data['logv3'][k]
    s, ln = sd(A['calls'], B['calls'])
    if s:
        i, dd = s[0]; c = A['calls'][i]; byv[variant(k)] += 1
        rows.append((variant(k), k, i, c['rule'], round(dd, 6), c['mCut'], c['cutd']))
print('branch plain-vs-v3 start differences by variant:', dict(byv))
for r in rows:
    if r[0] != 'bbc': print('  ', r)
print('unexplained br-vs-main (plain):')
for k in keys:
    A, B = data['log'][k], data['logoff'][k]
    if A['hash'] != B['hash']:
        s, ln = sd(A['calls'], B['calls'])
        if not s:
            fired = [(c['i'], c['rule'], c['dpt']) for c in A['calls'] if c['rule']]
            print('  ', k, 'fired calls (i, rule, dpt):', fired[:6], 'ncalls', len(A['calls']), len(B['calls']))
