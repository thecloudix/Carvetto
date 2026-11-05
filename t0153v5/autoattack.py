#!/usr/bin/env python3
# T-0153 r5: place islands of a stadium frame exactly at the points SkeletonCheck samples on the wrong skeletons.
# usage: autoattack.py <net binary> <out file> L R g chordal rot_deg <mode: 4 (wrong={0},{1}) | 2 (SAB=2 skeleton of {0,1})>
import math, os, re, subprocess, sys
net, out, L, R, g, ch, rot, mode = sys.argv[1], sys.argv[2], *map(float, sys.argv[3:8]), sys.argv[8]
here = os.path.dirname(os.path.abspath(__file__))
def gen(iv, path):
    with open(path, 'w') as f:
        subprocess.run([sys.executable, '-I', os.path.join(here, 'stadium.py'), str(L), str(R), str(g), ','.join(f'{a}:{b}' for a, b in iv), '0.005', str(rot)], stdout=f, check=True)
base = out + '.base'; gen([], base)
env = dict(os.environ); xs = []
runs = [('0', {}), ('1', {})] if mode == '4' else [('01', {'SAB': '2', 'SABI': '1'})]
c, s = math.cos(math.radians(rot)), math.sin(math.radians(rot))
for w, e in runs:
    o = subprocess.run([net, base, str(ch), w, 'v'], capture_output=True, text=True, env={**env, **e}).stdout
    for m in re.finditer(r'\[\d+ \(([-\d.]+),([-\d.]+)\) r', o): x, y = float(m.group(1)), float(m.group(2)); xs.append(c * x + s * y)
xs.sort(); hw = R + g  # cover >= r - clear on each side
iv = []
for x in xs:
    a, b = x - hw, x + hw
    if iv and a <= iv[-1][1] + 0.05: iv[-1] = (iv[-1][0], b)
    else: iv.append((a, b))
iv = [(-99 if a < -L + 0.5 else a, 99 if b > L - 0.5 else b) for a, b in iv]
gen(iv, out); print(out, 'islands', len(iv), iv)
