#!/usr/bin/env python3
# pcmp2.py OUT [cfg...]: perturbation sweep analysis. For each scene+variant, compare every perturbed run against 'id':
# the FIRST tieFreeStart call whose start moved (> TOL*scale) -- later calls may follow from it. Split transforms into
# 'scale' (u+-k ulps, r+-1e-12..1e-9) and 'frame' (translations, quarter turns). Classify by the id call's rule and by
# whether the call is a tip start (VectorStart-like: |nearest - cutd| small => mCut ~ eps) or not.
import sys, glob, os, collections, math, re
O = sys.argv[1]; cfgs = sys.argv[2:] or ['log', 'logoff', 'logv3']; TOL = float(os.environ.get('TOL', '1e-5'))
for c in cfgs:
    st = collections.defaultdict(collections.Counter); ex = collections.defaultdict(list)
    for p in sorted(glob.glob(f'{O}/{c}/*.txt')):
        runs = collections.OrderedDict(); calls = collections.defaultdict(list)
        for ln in open(p, errors='replace'):
            f = ln.split()
            if not f: continue
            if f[0] == 'T' and len(f) >= 19: calls[f[1]].append((int(f[3]), float(f[9]), float(f[10]), float(f[14]), float(f[18])))
            elif f[0] == 'R': runs[f[1]] = (int(f[3]), int(f[9]), calls.pop(f[1], []))
        groups = collections.defaultdict(dict)
        for t, v in runs.items():
            b, x = t.rsplit(':', 1); groups[b][x] = v
        base = os.path.basename(p)
        m = re.search(r'build-verify2-rel_(gen\d)_([a-z]+)_\d+\.scn', base)
        for b, g in groups.items():
            if 'id' not in g: continue
            fam = (m.group(1) + ':' + m.group(2)) if m else ('stored:' + b.split(':')[-1])
            m0, l0, c0 = g['id']; st[fam]['scenes'] += 1
            for x, (mv, l, cc) in g.items():
                if x == 'id': continue
                kind = 'scale' if x[0] in 'ur' else 'frame'
                st[fam][kind + '_runs'] += 1
                if len(cc) != len(c0): st[fam][kind + '_callcount'] += 1; continue
                for i, (q0, q) in enumerate(zip(c0, cc)):
                    d = math.hypot(q0[1] - q[1], q0[2] - q[2]); sc = max(1.0, abs(q0[1]), abs(q0[2]))
                    if d > TOL * sc:
                        tip = q0[3] < 2e-3 * max(1.0, q0[4]) or abs(q0[3] - 1e-3) < 1e-9  # |nearest-cutd| within ~tol (mCut ~ eps)
                        key = f'{kind}_moved_rule{q0[0]}' + ('_tip' if tip else '')
                        st[fam][key] += 1
                        if len(ex[fam + key]) < 3: ex[fam + key].append((base[-60:], b.split(':')[-1], x, i, q0[0], q[0], round(d, 6)))
                        break
                st[fam][kind + '_left_changed'] += l != l0
    print(f'== {c}')
    for fam in sorted(st):
        print(f'  {fam:16s} ' + ' '.join(f'{k} {v}' for k, v in sorted(st[fam].items())))
        for k, e in ex.items():
            if k.startswith(fam) and k[len(fam):].startswith('scale'):
                for z in e[:2]: print('       e.g.', k[len(fam):], z)
