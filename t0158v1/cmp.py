#!/usr/bin/env python3
# cmp.py OUT : compare st outputs in OUT/{log,logoff,logv3,logv3off}/*.txt (see scnsweep.sh / rndsweep.sh)
# Reports: (A) branch vs main output change rate (plain and v3), each change explained by a fired rule call
# with a moved start; (B) plain vs v3 cut-start / tab-count agreement, branch and main; (C) fired flag
# agreement across compilers; tie validity (dRule).
import sys, os, glob, collections, math
O = sys.argv[1]; TOL = float(os.environ.get('TOL', '1e-6'))
def parse(path):
    d = collections.OrderedDict(); calls = collections.defaultdict(list)
    for ln in open(path, errors='replace'):
        f = ln.split()
        if not f: continue
        if f[0] == 'T' and len(f) >= 13:
            calls[f[1]].append(dict(i=int(f[2]), fired=int(f[3]), nq=int(f[4]), wrap=int(f[5]), du=float(f[6]), dpt=float(f[7]), dRule=float(f[8]), x=float(f[9]), y=float(f[10]), nmc=float(f[11]), nmr=float(f[12])))
        elif f[0] == 'R':
            d[f[1]] = dict(moves=int(f[3]), diags=int(f[5]), hash=f[7], left=int(f[9]), free=int(f[11]), calls=calls.pop(f[1], []))
    return d
cfgs = ['log', 'logoff', 'logv3', 'logv3off']
data = {c: {} for c in cfgs}
for c in cfgs:
    for p in sorted(glob.glob(f'{O}/{c}/*.txt')):
        for k, v in parse(p).items(): data[c][os.path.basename(p) + '|' + k] = v
keys = [k for k in data['log'] if all(k in data[c] for c in cfgs)]
print(f'scenes(runs) with all 4 configs: {len(keys)}  (log {len(data["log"])}, logoff {len(data["logoff"])}, logv3 {len(data["logv3"])}, logv3off {len(data["logv3off"])})')
def startsdiff(a, b):
    n = min(len(a), len(b)); out = []
    for i in range(n):
        dd = math.hypot(a[i]['x'] - b[i]['x'], a[i]['y'] - b[i]['y'])
        sc = max(1.0, abs(a[i]['x']), abs(a[i]['y']))
        if not dd <= TOL * sc: out.append((i, dd))
    return out, len(a) != len(b)
def variant(k): return k.split('|')[1].split(':')[1] if ':' in k.split('|')[1] else 'rnd'
# (A)
for on, off, name in (('log', 'logoff', 'plain'), ('logv3', 'logv3off', 'v3')):
    ch = 0; unexpl = []; tiebad = []; firedSame = 0; perv = collections.Counter(); perv_tot = collections.Counter(); withfire = 0; leftd = collections.Counter(); freed = collections.Counter()
    for k in keys:
        A, B = data[on][k], data[off][k]; v = variant(k); perv_tot[v] += 1
        fired = [c for c in A['calls'] if c['fired'] and c['dpt'] > 1e-9]
        if any(c['fired'] for c in A['calls']): withfire += 1
        for c in A['calls']:
            if c['fired'] and not (c['dRule'] <= 1e-3): tiebad.append((k, c['i'], c['dRule']))
        if A['hash'] != B['hash']:
            ch += 1; perv[v] += 1
            sd, ln = startsdiff(A['calls'], B['calls'])
            first = sd[0][0] if sd else None
            ok = first is not None and A['calls'][first]['fired'] == 1
            if not ok: unexpl.append((k, first, ln))
            leftd[(A['left'] > B['left']) - (A['left'] < B['left'])] += 1
            freed[(A['free'] > B['free']) - (A['free'] < B['free'])] += 1
        elif fired: firedSame += 1
    print(f'(A {name}) runs changed br vs main: {ch}/{len(keys)} = {100.0*ch/max(1,len(keys)):.1f}%; runs where rule fired at all: {withfire}; fired with moved start but identical output: {firedSame}')
    print(f'   per variant changed/total: ' + ', '.join(f'{v} {perv[v]}/{perv_tot[v]}' for v in sorted(perv_tot)))
    print(f'   tabs left out br vs main among changed (+1 = br leaves more): {dict(leftd)}; "comes free" diags: {dict(freed)}')
    print(f'   unexplained changes (first differing start not a fired rule call): {len(unexpl)}', unexpl[:10])
    print(f'   fired calls whose rule point is not equally near (dRule > 1e-3): {len(tiebad)}', tiebad[:5])
# (B)/(C)
for a, b, name in (('log', 'logv3', 'branch'), ('logoff', 'logv3off', 'main')):
    sdiff = 0; tiediff = []; nontie = []; leftdiff = []; movesdiff = 0; hashdiff = 0; firemis = []; lenmis = 0
    for k in keys:
        A, B = data[a][k], data[b][k]
        if A['hash'] != B['hash']: hashdiff += 1
        if A['moves'] != B['moves']: movesdiff += 1
        if A['left'] != B['left']: leftdiff.append((k, A['left'], B['left']))
        sd, ln = startsdiff(A['calls'], B['calls'])
        if ln: lenmis += 1
        for i in range(min(len(A['calls']), len(B['calls']))):
            if A['calls'][i]['fired'] != B['calls'][i]['fired']: firemis.append((k, i)); break
        if sd:
            sdiff += 1
            i = sd[0][0]; ca, cb = A['calls'][i], B['calls'][i]
            # for main (rule off) 'fired' still tells whether the call was an arc-centre tie
            if ca['fired'] or cb['fired']: tiediff.append((k, i, sd[0][1]))
            else: nontie.append((k, i, sd[0][1], ca['nmc'], cb['nmc'], ca['nmr'], cb['nmr']))
    print(f'(B {name}) plain vs v3: runs with a differing cut start: {sdiff}; first difference at an arc-centre tie: {len(tiediff)}; elsewhere: {len(nontie)}; tab-left count differs: {len(leftdiff)}; moves count differs: {movesdiff}; call-count mismatch {lenmis}; hash differs {hashdiff}')
    for t in tiediff[:12]: print('    tie  ', t)
    for t in nontie[:12]: print('    other', t)
    for t in leftdiff[:12]: print('    left ', t)
    print(f'(C {name}) fired flag differs plain vs v3: {len(firemis)}', firemis[:8])
