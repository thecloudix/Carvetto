#!/usr/bin/env python3
"""advsum.py <adv logs>: summary of FLAG lines of x2 adv."""
import re, sys, glob
def f(p, l, t=float):
    m = re.search(p, l); return t(m.group(1)) if m else None
rows = []
for fn in sys.argv[1:]:
    for l in open(fn):
        if not l.startswith('FLAG'): continue
        br, mn = l.split('| main ')[0], '| main ' + l.split('| main ')[1]
        rows.append(dict(seed=f(r'adv (\S+)', l, str), fam=f(r'fam(\d)', l, int), fac=f(r'fac (\S+)', l), d=f(r' d=(\S+)', l), join=f(r' (Round|Sharp|Square) side', l, str), side=f(r'side=(\d)', l, int),
            dv=f(r'br dv(\d+)', br, int), rv=f(r' rv(\d+)\(', br, int), reg=f(r'REG (\d+)\(', br, int), reglen=f(r'REG \d+\(tieonly \d len (\S+)', br), regdout=f(r'REG \d+\(tieonly \d len \S+ dout (\S+)', br),
            rside=f(r'regSide (\d+)', br, int), exb=f(r' excess (\d+)\(', br, int), exbw=f(r' excess \d+\((\S+)\)', br), bcl=f(r' closed (\d+) chains', br, int), cpu=f(r'cpu (\S+)', br),
            mexbw=f(r'excess \d+\((\S+)\)', mn), mcl=f(r' closed (\d+) chains', mn, int), mch=f(r'chains (\d+)', mn, int)))
print('flag lines', len(rows))
print('dv', sum(r['dv'] > 0 for r in rows), 'rv', sum(r['rv'] > 0 for r in rows), 'REG', sum(r['reg'] > 0 for r in rows), 'regSide', sum(r['rside'] > 0 for r in rows), 'fewer closed', sum(r['bcl'] < r['mcl'] for r in rows), 'cpu>2s', sum(r['cpu'] > 2 for r in rows))
ex = [r for r in rows if r['exb'] and r['exbw'] > r['mexbw'] + 1e-3]
print('excess worse than main', len(ex), '| main empty', sum(1 for r in ex if r['mch'] == 0), '| main non-empty', sum(1 for r in ex if r['mch'] > 0))
for r in rows:
    if r['reg'] or r['rside'] or r['bcl'] < r['mcl'] or r['dv'] or r['rv'] or r['cpu'] > 2:
        print('  %s fam%d fac %.4g d %.4g %s side %d REG %d (len %.4g dout %.4g) regSide %d closed br/main %d/%d cpu %.2f' % (r['seed'], r['fam'], r['fac'], r['d'], r['join'], r['side'], r['reg'], r['reglen'], r['regdout'], r['rside'], r['bcl'], r['mcl'], r['cpu']))
for r in sorted([r for r in ex if r['mch'] > 0], key=lambda r: -(r['exbw'] - r['mexbw']))[:8]:
    print('  EX %s fam%d d %.4g %s side %d br %.4g main %.4g' % (r['seed'], r['fam'], r['d'], r['join'], r['side'], r['exbw'], r['mexbw']))
