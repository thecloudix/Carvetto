#!/usr/bin/env python3
# cmp2.py OUT [cfgs]: compare st outputs in OUT/<cfg>/*.txt (sweep.sh scn / random).
# (A) branch vs main change rate (plain log/logoff, v3 logv3/logv3off); each change must be explained by a call whose
#     rule fired and moved the start (first differing start is a fired call); per-rule tally.
# (B) cross-compiler: plain vs v3 differing cut starts, branch and main, classified by rule of the first differing call.
# (M) merged (mlogv3) vs branch plain (log): hash and start agreement.
# (K) knife-edges: fired calls whose decisive margins are small (mCut, mNear, mGap, mArcC); rule flag mismatch plain/v3.
import sys, os, glob, collections, math
O = sys.argv[1]; TOL = float(os.environ.get('TOL', '1e-6')); KN = float(os.environ.get('KN', '1e-6'))
def parse(path):
    d = collections.OrderedDict(); calls = collections.defaultdict(list)
    for ln in open(path, errors='replace'):
        f = ln.split()
        if not f: continue
        if f[0] == 'T' and len(f) >= 19:
            calls[f[1]].append(dict(i=int(f[2]), rule=int(f[3]), nc=int(f[4]), runs=int(f[5]), du=float(f[6]), dpt=float(f[7]), dRule=float(f[8]), x=float(f[9]), y=float(f[10]),
                mArcC=float(f[11]), mArcR=float(f[12]), mArcS=float(f[13]), mCut=float(f[14]), mNear=float(f[15]), mGap=float(f[16]), zda=int(f[17]), cutd=float(f[18])))
        elif f[0] == 'R':
            d[f[1]] = dict(moves=int(f[3]), diags=int(f[5]), hash=f[7], left=int(f[9]), free=int(f[11]), calls=calls.pop(f[1], []))
    return d
cfgs = [c for c in ['log', 'logoff', 'logv3', 'logv3off', 'mlogv3'] if os.path.isdir(f'{O}/{c}')]
data = {c: {} for c in cfgs}
for c in cfgs:
    for p in sorted(glob.glob(f'{O}/{c}/*.txt')):
        for k, v in parse(p).items(): data[c][os.path.basename(p) + '|' + k] = v
keys = [k for k in data['log'] if all(k in data[c] for c in cfgs)]
print(f'runs with all configs {cfgs}: {len(keys)} ' + ' '.join(f'{c} {len(data[c])}' for c in cfgs))
def sd(a, b):
    out = []
    for i in range(min(len(a), len(b))):
        dd = math.hypot(a[i]['x'] - b[i]['x'], a[i]['y'] - b[i]['y']); sc = max(1.0, abs(a[i]['x']), abs(a[i]['y']))
        if not dd <= TOL * sc: out.append((i, dd))
    return out, len(a) != len(b)
def variant(k):
    t = k.split('|')[1]
    import re
    return 'rnd' if re.match(r'r\d+:\d+:m', t) else t.split(':')[1]
chg = {}
for on, off, name in (('log', 'logoff', 'plain'), ('logv3', 'logv3off', 'v3')):
    if on not in data or off not in data: continue
    ch = 0; unexpl = []; perv = collections.Counter(); tot = collections.Counter(); byrule = collections.Counter(); leftd = collections.Counter(); freed = collections.Counter()
    fired_runs = 0; moved_same = 0; changed = []
    for k in keys:
        A, B = data[on][k], data[off][k]; v = variant(k); tot[v] += 1
        if any(c['rule'] and c['dpt'] > 1e-9 for c in A['calls']): fired_runs += 1
        if A['hash'] != B['hash']:
            ch += 1; perv[v] += 1; changed.append(k)
            s, ln = sd(A['calls'], B['calls'])
            first = s[0][0] if s else None
            if first is None or not A['calls'][first]['rule']: unexpl.append((k, first, ln))
            else: byrule[A['calls'][first]['rule']] += 1
            leftd[(A['left'] > B['left']) - (A['left'] < B['left'])] += 1; freed[(A['free'] > B['free']) - (A['free'] < B['free'])] += 1
        elif any(c['rule'] and c['dpt'] > 1e-9 for c in A['calls']): moved_same += 1
    chg[name] = changed
    print(f'(A {name}) changed br vs main {ch}/{len(keys)} = {100.0*ch/max(1,len(keys)):.1f}%; runs with a fired call that moved the start {fired_runs}; of those, output identical {moved_same}')
    print('   per variant: ' + ', '.join(f'{v} {perv[v]}/{tot[v]} ({100.0*perv[v]/max(1,tot[v]):.1f}%)' for v in sorted(tot)))
    print(f'   first moved call by rule: {dict(byrule)}; tabs-left br vs main (+1 = br leaves more): {dict(leftd)}; comes-free diags: {dict(freed)}')
    print(f'   UNEXPLAINED changes: {len(unexpl)}', unexpl[:10])
    if name == 'plain':
        open(f'{O}/changed_plain.lst', 'w').write('\n'.join(changed) + '\n')
for a, b, name in (('log', 'logv3', 'branch'), ('logoff', 'logv3off', 'main'), ('log', 'mlogv3', 'branch-plain vs merged-v3')):
    if a not in data or b not in data: continue
    sdiff = 0; cls = collections.Counter(); ex = collections.defaultdict(list); leftdiff = []; hashdiff = 0; rulemis = []
    for k in keys:
        A, B = data[a][k], data[b][k]
        if A['hash'] != B['hash']: hashdiff += 1
        if A['left'] != B['left']: leftdiff.append((k, A['left'], B['left']))
        for i in range(min(len(A['calls']), len(B['calls']))):
            if A['calls'][i]['rule'] != B['calls'][i]['rule']: rulemis.append((k, i, A['calls'][i]['rule'], B['calls'][i]['rule'])); break
        s, ln = sd(A['calls'], B['calls'])
        if s or ln:
            sdiff += 1
            if s:
                i = s[0][0]; ca, cb = A['calls'][i], B['calls'][i]
                key = f"rule{ca['rule']}/{cb['rule']} nc{ca['nc']} runs{ca['runs']}"
            else: key = 'callcount'
            cls[key] += 1
            if len(ex[key]) < 6: ex[key].append((k, s[0] if s else None, A['left'], B['left']))
    print(f'(B {name}) runs with differing cut start {sdiff}; hash differs {hashdiff}; tab-left differs {len(leftdiff)}; rule flag differs {len(rulemis)}')
    for kk, n in cls.most_common(): print(f'    {kk}: {n}', ex[kk][:4])
    for t in leftdiff[:10]: print('    left', t)
    for t in rulemis[:10]: print('    rulemis', t)
# (K) knife edges among fired calls of the branch plain run
kn = collections.Counter(); kex = collections.defaultdict(list); zda = []
for k in keys:
    for c in data['log'][k]['calls']:
        if c['zda']: zda.append((k, c['i']))
        for m in ('mCut', 'mNear', 'mGap', 'mArcC', 'mArcR', 'mArcS'):
            if c[m] < KN * max(1.0, c['cutd']):
                kn[m] += 1
                if len(kex[m]) < 5: kex[m].append((k, c['i'], c['rule'], c[m]))
print(f'(K) calls with a threshold margin < {KN}: {dict(kn)}; near-zero-direction arcs in sharp rule: {len(zda)}', zda[:5])
for m, e in kex.items(): print('   ', m, e)
