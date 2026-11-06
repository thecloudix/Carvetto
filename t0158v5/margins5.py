#!/usr/bin/env python3
# margins5.py DIR... : over st T lines of the instrumented round-5 lib (tfs_instr5.py), per directory: deciding-path counts
# (kind 0 projection, 1 arc rule, 2 own-edge corner, 3 fallback round first point, 4 fallback equidistant junction) x rule fired,
# corner-place counts (back) and met, and the calls closest to each new band: mEqui = | |g-d|/noise - 1 | (labelling),
# mOnC = | |dist-d|/onCircle - 1 | (on-circle), mLop = min |along.edgeDir| (travel sign test).
import sys, glob, collections
for D in sys.argv[1:]:
    kinds = collections.Counter(); cnt = collections.Counter(); close = {'mEqui': [], 'mOnC': [], 'mLop': []}; calls = 0
    for p in glob.glob(D + '/*.txt'):
        for ln in open(p, errors='replace'):
            f = ln.split()
            if not f or f[0] != 'T' or len(f) < 31: continue
            calls += 1; rule = f[3]; kind = f[25]; back = f[26]; met = f[30]
            kinds[(kind, rule)] += 1
            if kind in ('2', '3', '4') or back != '-1': cnt[(back, met)] += 1
            for k, i in (('mEqui', 28), ('mOnC', 27), ('mLop', 29)):
                v = float(f[i])
                if v < 1e299: close[k].append((v, p.split('/')[-1][-50:], f[1]))
    print(f'== {D}: {calls} calls')
    print('  kind,rule:', dict(sorted(kinds.items())))
    print('  corner-path count,met:', dict(sorted(cnt.items(), key=lambda x: -x[1])[:12]))
    for k in close:
        v = sorted(close[k])
        b = collections.Counter('<1e-6' if x[0] < 1e-6 else '<1e-3' if x[0] < 1e-3 else '<0.1' if x[0] < 0.1 else '<0.5' if x[0] < 0.5 else '>=0.5' for x in v)
        print(f'  {k}: n {len(v)} buckets {dict(b)}; closest', v[:3])
