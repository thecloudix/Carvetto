#!/usr/bin/env python3
"""re4sum.py <re outputs...>: round-4 summary of RV4 lines (branch vs main on the same polygonised input).
REG (strict): branch coverage miss stretch > tol, non-thin, at points main covers within tol. REG3: main within 3 tol.
regSide: (c)-side miss (> 3 tol) main covers within 3 tol, not in a strip thinner than tol (sidethin counted apart).
EXW: Sharp/Square excess beyond d outside the checker's corner regions, worse than main's by more than tol."""
import re, sys, collections
rows = []
for f in sys.argv[1:]:
    for l in open(f):
        if not l.startswith('RV4'): continue
        m = re.search(r'RV4 (\S+) (\S+) (\w+) side=(\d) d=(\S+) closed=(\d) \| REG stretches>tol (\d+) \(tieonly (\d)\) maxlen (\S+) dout (\S+) at \((\S+),(\S+)\) side (\d+) \((\S+)\) \| excess br (\d+) \((\S+)\) main (\d+) \((\S+)\) \| closedchains br (\d+) main (\d+) \| dv br (\d+) \((\S+)\) rv br (\d+) \((\S+)\) \| REG3 (\d+) len (\S+) dout (\S+) sidethin (\d+)', l)
        g = m.groups()
        rows.append(dict(tag=g[0], seed=g[1], join=g[2], side=int(g[3]), d=float(g[4]), closed=int(g[5]), reg=int(g[6]), tieonly=int(g[7]), reglen=float(g[8]), regdout=float(g[9]), regx=g[10], regy=g[11],
                         rside=int(g[12]), rsided=float(g[13]), exb=int(g[14]), exbw=float(g[15]), exm=int(g[16]), exmw=float(g[17]), clb=int(g[18]), clm=int(g[19]), dv=int(g[20]), rv=int(g[22]),
                         reg3=int(g[24]), reg3len=float(g[25]), reg3dout=float(g[26]), sidethin=int(g[27])))
tol = 1e-3
print('rechecked offsets', len(rows), '| branch dv', sum(r['dv'] > 0 for r in rows), 'rv', sum(r['rv'] > 0 for r in rows))
reg = [r for r in rows if r['reg']]; reg3 = [r for r in rows if r['reg3']]
print('REG strict (main within tol):', len(reg), '| of which branch dout <= 3 tol:', sum(r['regdout'] <= 3 * tol + 1e-9 for r in reg), '| > 3 tol:', sum(r['regdout'] > 3 * tol + 1e-9 for r in reg))
print('REG3 (main within 3 tol):', len(reg3))
print('regSide (non-thin):', sum(r['rside'] > 0 for r in rows), '| side misses main covers that lie in strips thinner than tol:', sum(r['sidethin'] > 0 for r in rows))
exw = [r for r in rows if r['exb'] and r['exbw'] > r['exmw'] + tol]
print('EXW (excess worse than main by > tol):', len(exw), '| excess flagged but within tol of main:', sum(1 for r in rows if r['exb'] and r['exbw'] <= r['exmw'] + tol))
print('fewer closed chains than main (among rechecked):', sum(r['clb'] < r['clm'] for r in rows))
K = int(sys.argv[0] and 40)
for r in sorted(reg, key=lambda r: -r['reglen'])[:K]:
    print('REG  %s %s %s side=%d d=%.6g closed=%d len %.4g branch-dout %.4g tieonly %d at (%s,%s)' % (r['tag'], r['seed'], r['join'], r['side'], r['d'], r['closed'], r['reglen'], r['regdout'], r['tieonly'], r['regx'], r['regy']))
for r in rows:
    if r['rside']: print('SIDE %s %s %s side=%d d=%.6g dout %.4g' % (r['tag'], r['seed'], r['join'], r['side'], r['d'], r['rsided']))
for r in sorted(exw, key=lambda r: -(r['exbw'] - r['exmw']))[:K]:
    print('EXW  %s %s %s side=%d d=%.6g branch %.4g main %.4g' % (r['tag'], r['seed'], r['join'], r['side'], r['d'], r['exbw'], r['exmw']))
for r in rows:
    if r['clb'] < r['clm']: print('CL   %s %s %s side=%d d=%.6g br %d main %d' % (r['tag'], r['seed'], r['join'], r['side'], r['d'], r['clb'], r['clm']))
