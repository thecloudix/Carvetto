#!/usr/bin/env python3
"""Summarise x2 logs (round 3): AC2 violations under the round-3 checker.
Violation classes per offset: dist (xk dv), round (xk rv), cov (non-thin miss, core or tie, any stretch),
covlong (non-thin, stretch > tol), side ((c): valid offset on the requested side of a collapsed run, > 3 tol from
the output), coll (collapsed-corner locus miss, non-thin, > 3 tol). Excess (Sharp/Square output beyond d outside
every corner region) is informational."""
import re, sys, glob, collections
files = sys.argv[1:]
rows = []
for f in files:
    for l in open(f, errors='replace'):
        if not (l.startswith('CORE') or l.startswith('TIE')): continue
        m = re.search(r'(v9|g2|v9all|g2all) seed=(0x[0-9a-f]+) n=(\d+) st=(\d+) closed=(\d) d=([\d.e+-]+) (\w+) side=(\d)', l)
        x = re.search(r'x: dv(\d+) u(\S+) rv(\d+) o(\S+) cm(\d+) miss(\S+) len(\S+) tie(\d+) tlen(\S+) thin(\d+) long(\d+)', l)
        c = re.search(r'c: runs(\d+) exm(\d+) exd(\S+) coll(\d+) side(\d+) sided(\S+)', l)
        ex = re.search(r'X-excess out \d+ at \S+ (\S+) beyond', l)
        if not (m and x and c): continue
        r = dict(file=f, tag=m.group(1), seed=m.group(2), n=int(m.group(3)), closed=int(m.group(5)), d=float(m.group(6)), join=m.group(7), side=int(m.group(8)),
                 dv=int(x.group(1)), under=float(x.group(2)), rv=int(x.group(3)), over=float(x.group(4)), cm=int(x.group(5)), mlen=float(x.group(7)), tie=int(x.group(8)), tlen=float(x.group(9)), long=int(x.group(11)),
                 coll=int(c.group(4)), side_=int(c.group(5)), sided=float(c.group(6)), excess=float(ex.group(1)) if ex else 0.0, line=l.strip())
        rows.append(r)
cls = collections.Counter(); worst = {}
for r in rows:
    k = []
    if r['dv']: k.append('dist')
    if r['rv']: k.append('round')
    if r['cm'] or r['tie']: k.append('cov')
    if r['long']: k.append('covlong')
    if r['side_']: k.append('side')
    if r['coll']: k.append('coll')
    if r['excess'] > 0: k.append('excess')
    r['cls'] = k
    for q in k: cls[q] += 1
print('offsets with violations by class:', dict(cls))
bad = [r for r in rows if set(r['cls']) - {'excess'}]
print('offsets with any AC2 violation (dist/round/cov/side/coll):', len(bad))
for r in sorted(bad, key=lambda r: -max(r['mlen'], r['tlen'], r['sided'])):
    print(f"{r['tag']} {r['seed']} {r['join']} side={r['side']} closed={r['closed']} d={r['d']:.6g} cls={','.join(r['cls'])} under={r['under']:.3g} over={r['over']:.3g} covlen={max(r['mlen'], r['tlen']):.3g} sideDout={r['sided']:.3g}")
