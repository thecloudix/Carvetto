#!/usr/bin/env python3
# worse.py FLOODDIR LIST: for each scene the flood calls worse on the branch, check whether MAIN produces the branch's
# cut starts (and tab/move counts) under some ulp/1e-9 perturbation or under -march=x86-64-v3 -- i.e. whether the
# branch's outcome is one main already reaches (the rule only picks one of the tied starts).
import sys, subprocess, os, math, collections
B = '/home/chris/wt-T0158/build-verify2-rel/bin'
def run(binary, mode, f, var, off):
    env = dict(os.environ, VARS='1'); 
    if off: env['TFS_OFF'] = '1'
    out = subprocess.run([f'{B}/st_{binary}', mode, f], env=env, capture_output=True, text=True, timeout=3000).stdout
    runs = collections.OrderedDict(); calls = collections.defaultdict(list)
    for ln in out.splitlines():
        x = ln.split()
        if not x: continue
        if x[0] == 'T': calls[x[1]].append((float(x[9]), float(x[10])))
        elif x[0] == 'R': runs[x[1]] = (calls.pop(x[1], []), int(x[3]), int(x[9]), int(x[11]))
    return {k: v for k, v in runs.items() if k.split(':')[1] == var}
def same(a, b): return len(a[0]) == len(b[0]) and all(math.hypot(p[0]-q[0], p[1]-q[1]) <= 1e-6 * max(1, abs(p[0]), abs(p[1])) for p, q in zip(a[0], b[0])) and a[1:] == b[1:]
for ln in open(sys.argv[2]):
    f, var = ln.split()
    br = run('log', 'scn', f, var, False); k = next(iter(br)); b = br[k]
    hits = []
    for binary in ('log', 'logv3'):
        for t, v in run(binary, 'pert', f, var, True).items():
            if same(b, v): hits.append(f'{"main" if binary == "log" else "mainv3"}:{t.rsplit(":", 1)[1]}')
    print(os.path.basename(f), var, 'branch left', b[2], 'free', b[3], '| main reaches it under:', hits[:8] if hits else 'NONE', flush=True)
