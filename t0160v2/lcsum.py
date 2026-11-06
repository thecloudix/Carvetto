#!/usr/bin/env python3
# t0160v2: summarise lc2 output files: per case ins/cl/dg, inserted-feed clearance, global-scan worst vs ridge, flags.
import sys, re, glob
rows = []
for f in sys.argv[1:]:
    cur = None
    for l in open(f):
        m = re.match(r'^(\S+)\s+ins=(\d) cuts=(\d+) cl=(\d) dg=(\d) minClr-c/2=(\S+) ringClr-c/2=(\S+) ridge=(\S+)', l)
        if m:
            cur = dict(name=m[1], ins=int(m[2]), cuts=int(m[3]), cl=int(m[4]), dg=int(m[5]), clr=float(m[6]), ring=float(m[7]), ridge=float(m[8]), g='', flags=set(), corner=0.0); rows.append(cur); continue
        if l.startswith('    REFUSED') or re.match(r'^\S+\s+REFUSED', l):
            rows.append(dict(name=l.split()[0], refused=l.strip())); cur = None; continue
        if cur is None: continue
        if 'GSCAN gs' in l:
            m = re.search(r'onWorst=(\S+) at \((\S+),(\S+)\) offWorst=(\S+) ridge=(\S+) \| oraclePred=(\S+)', l)
            cur['on'] = float(m[1]); cur['off'] = float(m[4]); cur['pred'] = float(m[6]); cur['at'] = (m[2], m[3])
            for fl in ('GABOVE-RIDGE', 'PRED>CH-NOCUTS', 'PRED<CH-CUTS', 'unchanged'):
                if fl in l: cur['flags'].add(fl)
        if 'ABOVE-RIDGE' in l and 'corner' in l: cur['flags'].add('CORNER-ABOVE')
        m = re.search(r'worstCornerOn=(\S+)', l)
        if m: cur['corner'] = float(m[1])
for r in rows:
    if 'refused' in r: print('REFUSED', r['refused']); continue
    bad = []
    if not (r['ins'] and r['cl'] and r['dg']): bad.append('INSERTION')
    if r['cuts'] and r['clr'] < -1e-9: bad.append('CLEARANCE<c/2')
    print(f"{r['name']:<40} cuts={r['cuts']:<4} clr-c/2={r['clr']:+.2e} ring-c/2={r['ring']:+.2e} ridge={r['ridge']:.4f} gOn={r.get('on',0):.5f} gOff={r.get('off',0):.4f} cornerOn={r['corner']:.4f} pred={r.get('pred',0):.4f} {' '.join(sorted(r['flags']))} {' '.join(bad)}")
