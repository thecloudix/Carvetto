#!/usr/bin/env python3
# rnddump2.py RNDDIR DUMPDIR [skipseeds]: parallel rnddump (12 threads); dumps every random scene whose output differs br vs main
# (log vs logoff) and prints a floodcmp.sh list ("file orig") of those with tabs. Seeds in skipseeds (comma list) are skipped.
import sys, glob, os, subprocess
from concurrent.futures import ThreadPoolExecutor
R, D = sys.argv[1], sys.argv[2]; skip = set(sys.argv[3].split(',')) if len(sys.argv) > 3 else set()
os.makedirs(D, exist_ok=True); B = '/home/chris/wt-T0158/build-verify3-rel/bin/st_br'
jobs = []
for p in sorted(glob.glob(f'{R}/log/*.txt')):
    seed, S, O = os.path.basename(p)[:-4].split('_')
    if seed in skip: continue
    h = lambda q: {f[1]: f[7] for f in (l.split() for l in open(q)) if f and f[0] == 'R'}
    a, b = h(p), h(p.replace('/log/', '/logoff/'))
    for k in a:
        if k in b and a[k] != b[k]:
            sc = int(k.split(':')[1]); jobs.append((seed, S, O, sc, f'{D}/r{seed}_{S}_{O}_{sc}.scn'))
def run(j):
    seed, S, O, sc, fn = j
    if not os.path.exists(fn):
        subprocess.run([B, seed, str(sc + 1), S, O], env=dict(os.environ, TARGET=str(sc), DUMP=fn), stdout=subprocess.DEVNULL, timeout=3000)
    P = [l for l in open(fn) if l.startswith('P ')][0].split()
    return fn if P[14] == '1' else None
with ThreadPoolExecutor(12) as pool:
    for r in pool.map(run, jobs):
        if r: print(r, 'orig', flush=True)
