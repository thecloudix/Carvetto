#!/usr/bin/env python3
# rnddump.py RNDDIR DUMPDIR: dump (st_br TARGET/DUMP) every random scene whose output differs br vs main (log vs logoff)
# and has tabs; prints a floodcmp.sh list ("file orig") on stdout.
import sys, glob, os, subprocess
R, D = sys.argv[1], sys.argv[2]; os.makedirs(D, exist_ok=True); B = '/home/chris/wt-T0158/build-verify2-rel/bin/st_br'
for p in sorted(glob.glob(f'{R}/log/*.txt')):
    seed, S, O = os.path.basename(p)[:-4].split('_')
    def h(q): return {f[1]: f[7] for f in (l.split() for l in open(q)) if f and f[0] == 'R'}
    a, b = h(p), h(p.replace('/log/', '/logoff/'))
    for k in a:
        if k in b and a[k] != b[k]:
            sc = int(k.split(':')[1]); fn = f'{D}/r{seed}_{S}_{O}_{sc}.scn'
            if not os.path.exists(fn):
                subprocess.run([B, seed, str(sc + 1), S, O], env=dict(os.environ, TARGET=str(sc), DUMP=fn), stdout=subprocess.DEVNULL, timeout=1800)
            P = [l for l in open(fn) if l.startswith('P ')][0].split()
            if P[14] == '1': print(fn, 'orig')
