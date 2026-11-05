#!/usr/bin/env python3
# T-0153 r6 (bdc932c): geometry aimed at the new net's reasoning, run end to end through the sabotaged build.
#   full     one island = the hole inset by d (wrong skeleton lies over region everywhere; gaps g, d vs 2c+d)
#   slivers  islands of width w every s along the hole (thin walls crossing the wrong skeleton between points)
#   stepgap  islands whose gaps equal k x the wrong skeleton's point spacing (membership carried across a gap)
#   rot      the above rotated (ray-count query box is horizontal: rings straddling p.y at odd angles)
# usage: autoattack6.py <vc3_sab> <out dir> [chordals]   -> prints one line per run; FLAG = carved outside beyond 2c+d
import itertools, os, re, subprocess, sys
vc3, out = sys.argv[1], sys.argv[2]; chs = [float(x) for x in (sys.argv[3] if len(sys.argv) > 3 else '0.05,0.1,1.0').split(',')]
here = os.path.dirname(os.path.abspath(__file__)); os.makedirs(out, exist_ok=True)
def gen(path, L, R, g, iv, ins, rot):
    with open(path, 'w') as f:
        subprocess.run([sys.executable, '-I', os.path.join(here, 'stadium.py'), str(L), str(R), str(g), ','.join(f'{a}:{b}' for a, b in iv), str(ins), str(rot)], stdout=f, check=True)
def run(path, ch, sab, tool):
    env = dict(os.environ, SAB=str(sab), VC_FILE=path, VC_FIX=f'{tool} {ch} 0 0')
    try: o = subprocess.run([vc3, '1', '1'], capture_output=True, text=True, env=env, timeout=600).stdout
    except subprocess.TimeoutExpired: return 'TIMEOUT', None
    m = re.search(r'ok=(\d+) refused=(\d+).*?realViolations=(\d+).*?worstCone=([-0-9.e+]+)', o)
    return (m.group(0) if m else o[-200:]), (None if not m or m.group(2) == '1' else float(m.group(4)))
cases = []
for ch in chs:
    c = ch / 3; clear = 2 * c + min(0.001, c); L, R = 20, 1.5
    for g in (0.3 * clear, 0.6 * clear, 1.2 * clear, 3 * clear):
        for d in (0.005, 0.5 * clear, clear):
            cases.append((ch, 'full', g, [(-99, 99)], d, 0))
    for w, s in ((0.002, 0.5), (0.02, 1.0), (0.2, 2.0), (2 * clear, 0.3)):
        iv = [(-L + 0.3 + k * s, -L + 0.3 + k * s + w) for k in range(int((2 * L - 0.6) / s))]
        for g in (0.5 * clear, 2 * clear):
            cases.append((ch, f'slivers_w{w:g}_s{s:g}', g, iv, 0.005, 0)); cases.append((ch, f'slivers_w{w:g}_s{s:g}', g, iv, 0.005, 37))
    for k in (0.5, 1, 2, 5):   # island blocks with gaps of k x the wrong skeleton's mean point spacing (~ c * 3 measured)
        gap = k * 3 * c; iv = []; x = -99
        while True:
            a = (iv[-1][1] if iv else -L) + gap; b = a + 1.0
            if b > L - 0.3: break
            iv.append((a, b))
        iv = [(-99, iv[0][0] - gap)] + iv
        for g in (0.5 * clear, 2 * clear):
            cases.append((ch, f'stepgap_k{k:g}', g, iv, 0.005, 0)); cases.append((ch, f'stepgap_k{k:g}', g, iv, 0.005, 23))
flags = 0
for ch, fam, g, iv, d, rot in cases:
    c = ch / 3; clear = 2 * c + min(0.001, c)
    path = os.path.join(out, f'{fam}_c{ch:g}_g{g:.4g}_d{d:.4g}_r{rot}.txt'); gen(path, 20, 1.5, g, iv, d, rot)
    for sab, tool in itertools.product((0, 2, 3, 4), ('0 12 90', '1 6 0')):
        s, cone = run(path, ch, sab, tool)
        flag = cone is not None and cone < -clear - 1e-9
        if sab == 0 and cone is not None and cone < 0: flag = True
        flags += flag
        print(f"{'FLAG ' if flag else ''}{os.path.basename(path)} SAB={sab} [{tool}] clear={clear:.4f} :: {s}", flush=True)
print(f'autoattack6 done: {len(cases)} geometries, FLAGS={flags}')
