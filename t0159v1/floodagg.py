#!/usr/bin/env python3
# floodagg.py DIR vA vB: per scene, compares dl3 flood verdicts (hold/nohold/sev/part, FREE pieces, left) of two builds
import sys, os, re
D, A, B = sys.argv[1:4]
def parse(f):
    l = open(f).read().strip()
    if not l: return None
    m = re.search(r'G (\d+) hold (\d+) nohold (\d+) sev (\d+) part (\d+) thin (\d+) \| left (\d+) req (\d+)', l)
    fr = re.search(r'FREE (\d+) biggest ([\d.]+) mm2 at \S+ allLost (\d+)', l)
    return dict(G=int(m[1]), hold=int(m[2]), nohold=int(m[3]), sev=int(m[4]), part=int(m[5]), left=int(m[7]), free=int(fr[1]) if fr else 0, lost=int(fr[3]) if fr else 0)
tot = dict(n=0, worse=0, better=0, same=0)
for f in sorted(os.listdir(D)):
    if not f.startswith(A + '_'): continue
    g = B + f[len(A):]
    if not os.path.exists(os.path.join(D, g)): continue
    a, b = parse(os.path.join(D, f)), parse(os.path.join(D, g))
    if not a or not b: continue
    tot['n'] += 1
    bad = lambda r: (r['nohold'] + r['sev'] + r['part'], r['free'], r['lost'])
    if bad(b) > bad(a) or b['nohold'] + b['sev'] + b['part'] > a['nohold'] + a['sev'] + a['part'] or b['free'] > a['free'] or b['lost'] > a['lost']:
        tot['worse'] += 1; print('WORSE', f[len(A) + 1:], a, b)
    elif b != a and (b['left'] < a['left'] or b['free'] < a['free']): tot['better'] += 1
    else: tot['same'] += 1
print(tot)
