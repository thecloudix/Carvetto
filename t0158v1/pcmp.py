#!/usr/bin/env python3
# pcmp.py OUT [cfg...]: per config, per family: perturbed runs whose cut start moved vs the unperturbed run ('id'),
# split by whether the call was an arc-centre tie (fired) in the id run; tab-left/moves changes; fired-flag flips.
import sys, glob, os, collections, math, re
O = sys.argv[1]; cfgs = sys.argv[2:] or ['log', 'logoff', 'logv3']; TOL = float(os.environ.get('TOL', '1e-5'))
for c in cfgs:
    st = collections.defaultdict(collections.Counter); ex = collections.defaultdict(list)
    for p in sorted(glob.glob(f'{O}/{c}/*.txt')):
        runs = collections.OrderedDict(); calls = collections.defaultdict(list)
        for ln in open(p, errors='replace'):
            f = ln.split()
            if not f: continue
            if f[0] == 'T' and len(f) >= 13: calls[f[1]].append((int(f[3]), float(f[9]), float(f[10]), float(f[7])))
            elif f[0] == 'R': runs[f[1]] = (int(f[3]), int(f[9]), calls.pop(f[1], []))
        groups = collections.defaultdict(dict)
        for t, v in runs.items():
            b, x = t.rsplit(':', 1); groups[b][x] = v
        for b, g in groups.items():
            if 'id' not in g: continue
            fam = re.sub(r'_\d+\.scn.*', '', os.path.basename(p).split('build-verify1-rel_gen_')[-1]) if 'gen_' in p else ('stored:' + b.split(':')[-1])
            m0, l0, c0 = g['id']; st[fam]['scenes'] += 1
            st[fam]['id_fired_calls'] += sum(1 for q in c0 if q[0])
            for x, (m, l, cc) in g.items():
                if x == 'id': continue
                st[fam]['runs'] += 1
                if len(cc) != len(c0): st[fam]['callcount'] += 1; continue
                moved_t = moved_o = flip = 0
                for i, (q0, q) in enumerate(zip(c0, cc)):
                    d = math.hypot(q0[1] - q[1], q0[2] - q[2]); sc = max(1.0, abs(q0[1]), abs(q0[2]))
                    if q0[0] != q[0]: flip += 1
                    if d > TOL * sc:
                        if q0[0] or q[0]: moved_t += 1
                        else: moved_o += 1
                        if len(ex[fam]) < 6: ex[fam].append((os.path.basename(p), b, x, i, q0[0], q[0], round(d, 6)))
                        break  # later calls (Optimise) depend on earlier starts
                st[fam]['moved_at_tie'] += moved_t > 0; st[fam]['moved_other'] += moved_o > 0; st[fam]['fired_flip'] += flip > 0
                st[fam]['left_changed'] += l != l0; st[fam]['moves_changed'] += m != m0
    print(f'== {c}')
    for fam in sorted(st):
        print(f'  {fam:14s} ' + ' '.join(f'{k} {v}' for k, v in st[fam].items()))
        for e in ex[fam][:3]: print('       e.g.', e)
