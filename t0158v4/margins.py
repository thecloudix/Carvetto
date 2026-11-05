#!/usr/bin/env python3
# margins.py DIR...: over all T lines (st K4 format) in DIR/*.txt: counts of rule/kind, and calls near a round-4 tolerance:
# mOnC < 0.5 (a tested span start within a factor 2 of the join-circle tolerance), mEqui < 0.5 (a junction's |ga-gb| within a
# factor 2 of the equidistance tolerance), mLop < 1e-6 mm (two candidate junctions with near-equal "most even feet"),
# mNear/mCut < 1e-9 (gate knife-edges). Examples.
import sys, glob, collections
c = collections.Counter(); ex = collections.defaultdict(list)
for D in sys.argv[1:]:
    for p in glob.glob(D + '/*.txt'):
        for ln in open(p, errors='replace'):
            f = ln.split()
            if len(f) < 31 or f[0] != 'T' or f[24] != 'K4': continue
            rule, kind = int(f[3]), int(f[25]); mCut, mNear, mGap = float(f[14]), float(f[15]), float(f[16])
            mOnC, mEqui, mLop = float(f[27]), float(f[28]), float(f[29])
            c[f'calls'] += 1; c[f'rule{rule}'] += 1
            if rule >= 2: c[f'rule{rule} kind{kind}'] += 1
            def flag(k, cond):
                if cond:
                    c[k] += 1
                    if len(ex[k]) < 4: ex[k].append((p.split('/')[-1][-40:], f[1][-40:], 'rule', rule, 'kind', kind, 'du', f[6]))
            flag('mOnC<0.5', rule >= 2 and mOnC < 0.5)
            flag('mEqui<0.5', rule >= 2 and mEqui < 0.5)
            flag('mLop<1e-6', rule >= 2 and kind in (2, 3) and mLop < 1e-6)
            flag('mCut<1e-9', mCut < 1e-9)
            flag('mNear<1e-9', mNear < 1e-9)
            flag('mGap<1e-9', mGap < 1e-9)
for k in sorted(c): print(k, c[k])
for k in ex:
    for e in ex[k]: print(' ', k, e)
