#!/usr/bin/env python3
# usage: cmp.py <sweep dir>: br vs main per config/family: changed hashes, violations (worst < -1e-5), neg (< 0),
# changed cases with/without a main violation, retract/plunge/cut deltas, worst values.
import sys, glob, os, re, collections
d = sys.argv[1]
def load(f):
    out = {}
    for l in open(f):
        if l.startswith('case '):
            t = l.split(); k = {}
            if ' worst ' not in l: continue
            for key in ['hash', 'retract', 'plunge', 'cut', 'moves', 'cpu', 'worst']: k[key] = t[t.index(key) + 1]
            out[t[1]] = dict(hash=k['hash'], ret=int(k['retract']), pl=int(k['plunge']), cut=float(k['cut']), moves=int(k['moves']), cpu=float(k['cpu']), worst=float(k['worst']), line=l.strip())
    return out
tot = collections.Counter(); rows = []
for fb in sorted(glob.glob(os.path.join(d, '*_br.txt'))):
    fm = fb[:-7] + '_main.txt'
    if not os.path.exists(fm): continue
    b, m = load(fb), load(fm); name = os.path.basename(fb)[:-7]
    common = [k for k in b if k in m]
    ch = [k for k in common if b[k]['hash'] != m[k]['hash']]
    mv = [k for k in common if m[k]['worst'] < -1e-5]; bv = [k for k in common if b[k]['worst'] < -1e-5]
    mn = [k for k in common if m[k]['worst'] < 0]; bn = [k for k in common if b[k]['worst'] < 0]
    chv = [k for k in ch if m[k]['worst'] < 0]
    dr = sum(b[k]['ret'] - m[k]['ret'] for k in ch); dp = sum(b[k]['pl'] - m[k]['pl'] for k in ch)
    dc = sum(b[k]['cut'] - m[k]['cut'] for k in ch)
    cb = sum(b[k]['cpu'] for k in common); cm = sum(m[k]['cpu'] for k in common)
    wm = min([m[k]['worst'] for k in common] or [0]); wb = min([b[k]['worst'] for k in common] or [0])
    rows.append((name, len(common), len(ch), len(chv), len(mv), len(mn), len(bv), len(bn), dr, dp, dc, wm, wb, cb, cm))
    for i, key in enumerate(['n', 'changed', 'changedMainNeg', 'mainViol', 'mainNeg', 'brViol', 'brNeg', 'dRetract', 'dPlunge']):
        tot[key] += rows[-1][1 + i]
print('%-12s %4s %4s %5s %5s %5s %4s %4s %5s %5s %10s %11s %11s %7s %7s' % ('cfg_fam', 'n', 'chg', 'chgMN', 'mViol', 'mNeg', 'bV', 'bN', 'dRet', 'dPl', 'dCut', 'worstMain', 'worstBr', 'cpuBr', 'cpuMain'))
for r in rows: print('%-12s %4d %4d %5d %5d %5d %4d %4d %5d %5d %10.3f %11.3e %11.3e %7.2f %7.2f' % r)
print('TOTAL', dict(tot))
