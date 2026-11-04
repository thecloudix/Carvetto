#!/usr/bin/env python3
"""Classify 're' output lines: per offset, is the branch's worst coverage/side miss point covered by main (<= 3 tol)?"""
import re, sys, collections
tol = 1e-3
C = collections.Counter(); out = collections.defaultdict(list)
for l in open(sys.argv[1]):
    if not l.startswith('RE'): continue
    m = re.search(r'RE (\S+) (\S+) (\w+) side=(\d) d=(\S+) closed=(\d) \| branch dv(\d+)\(u(\S+)\) rv(\d+)\(o(\S+)\) cov(\d+)\+tie(\d+)\(len (\S+)\) side(\d+)\((\S+)\) coll(\d+) chains (\d+) \| main dv(\d+)\(u(\S+)\) rv(\d+)\(o(\S+)\) cov(\d+)\+tie(\d+)\(len (\S+)\) side(\d+)\((\S+)\) coll(\d+) chains (\d+) \| covP->branch (\S+) main (\S+) \| sideP->branch (\S+) main (\S+)', l)
    g = m.groups()
    tag, seed, join, side, d, closed = g[0], g[1], g[2], int(g[3]), float(g[4]), int(g[5])
    bcov, btie, blen, bside, bsided, bcoll = int(g[10]), int(g[11]), float(g[12]), int(g[13]), float(g[14]), int(g[15])
    mdv, mu, mcov, mtie, mlen, mside, msided = int(g[17]), float(g[18]), int(g[21]), int(g[22]), float(g[23]), int(g[24]), float(g[25])
    cb, cm, sb, sm = float(g[28]), float(g[29]), float(g[30]), float(g[31])
    worst = max(blen if (bcov or btie) else 0, bsided if bside else 0)
    regress = (bcov + btie > 0 and cb > 3 * tol and 0 <= cm <= 3 * tol) or (bside > 0 and sb > 3 * tol and 0 <= sm <= 3 * tol)
    k = 'branch-only (main covers the point)' if regress else 'main also misses / both empty'
    big = worst > 0.01
    key = k + (' | >0.01mm' if big else ' | <=0.01mm (tol-scale)')
    C[key] += 1
    out[key].append((worst, f"{tag} {seed} {join} side={side} closed={closed} d={d:.6g} branch: cov{bcov}+tie{btie} len {blen:.3g} side{bside} dout {bsided:.3g} coll{bcoll} | main: dv{mdv}(u {mu:.2g}) cov{mcov}+tie{mtie} len {mlen:.3g} side{mside} | worst point -> branch {max(cb,sb):.3g} main {cm if cm>=0 else sm:.3g}"))
for k in sorted(C): print(f"{C[k]:5d}  {k}")
for k in sorted(out):
    print('==', k)
    for w, s in sorted(out[k], reverse=True)[:int(sys.argv[2]) if len(sys.argv) > 2 else 12]: print('  ', s)
