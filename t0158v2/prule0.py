#!/usr/bin/env python3
# prule0.py OUT cfg: perturbed runs (scale transforms) where the first moved call had rule 0 in the id run;
# print scene, variant, call, mCut (| |nearest-cutd| - tol |), cutd, move. Small |nearest-cutd| (mCut < 0.02) = target ~on the vector.
import sys, glob, os, collections, math
O, c = sys.argv[1], sys.argv[2]; seen = collections.OrderedDict()
for p in sorted(glob.glob(f'{O}/{c}/*.txt')):
    runs = collections.OrderedDict(); calls = collections.defaultdict(list)
    for ln in open(p, errors='replace'):
        f = ln.split()
        if not f: continue
        if f[0] == 'T' and len(f) >= 19: calls[f[1]].append((int(f[3]), float(f[9]), float(f[10]), float(f[14]), float(f[18])))
        elif f[0] == 'R': runs[f[1]] = calls.pop(f[1], [])
    g = collections.defaultdict(dict)
    for t, v in runs.items(): b, x = t.rsplit(':', 1); g[b][x] = v
    for b, gg in g.items():
        if 'id' not in gg: continue
        c0 = gg['id']
        for x, cc in gg.items():
            if x == 'id' or x[0] not in 'ur' or len(cc) != len(c0): continue
            for i, (q0, q) in enumerate(zip(c0, cc)):
                d = math.hypot(q0[1]-q[1], q0[2]-q[2])
                if d > 1e-5 * max(1, abs(q0[1]), abs(q0[2])):
                    if q0[0] == 0:
                        k = (os.path.basename(p)[:-4], b.split(':')[-1], i)
                        if k not in seen: seen[k] = (q0[3], q0[4], round(d, 4), x)
                    break
for k, v in seen.items():
    if os.environ.get('ALL') or k[1] in ('orig', 'vsout'): print(k, 'mCut %.4g cutd %.4g move %s (%s)' % v)
