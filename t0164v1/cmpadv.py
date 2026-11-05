#!/usr/bin/env python3
# t0164v1: compare advhash outputs (sorted, one line per run) branch vs main: moves hash, diag list, refusals, timeouts
import re, sys
pat = re.compile(r'^(.*?) :: HASH case \d+ .+? chord \S+ ok (\d) moves (\d+) diags (\d+) h (\S+) mh (\S+) dl (\S*)')
def load(f):
    d = {}
    for l in open(f):
        m = pat.match(l)
        d[l.split(' :: ')[0]] = m.groups()[1:] if m else ('TIMEOUT/ERR', l.strip()[-80:])
    return d
A, B = load(sys.argv[1]), load(sys.argv[2])
assert A.keys() == B.keys()
n = same = mh = dl = 0; refA = refB = 0; odd = 0
for k in sorted(A):
    a, b = A[k], B[k]; n += 1
    if len(a) == 2 or len(b) == 2: odd += 1; print('ODD', k, a, b); continue
    refA += a[0] == '0'; refB += b[0] == '0'
    if a[4] != b[4]: mh += 1; print('MOVES DIFFER', k, a[:3], b[:3]); continue
    if a[3] != b[3]: dl += 1; print('DIAGS DIFFER', k, a[5], b[5]); continue
    same += 1
print(f'runs={n} identical={same} movesDiffer={mh} diagsDiffer={dl} odd={odd} refused br={refA} main={refB}')
