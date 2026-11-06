#!/usr/bin/env python3
# t0160v1: compare HASH lines of two sweep dirs. usage: cmp.py <A dir> <B dir> [expect: all|ncl0]
# all: every h must match. ncl0: h must match where ncl == 0; where ncl > 0 report same/differ counts.
import re, sys, glob, os
pat = re.compile(r'HASH case (\d+) (.+?) chord (\S+) ok (\d) moves (\d+) diags (\d+) h (\S+) mh (\S+) dl (\S*) so (\S+) width (\S+) ncl (\d+)')
mode = sys.argv[3] if len(sys.argv) > 3 else 'all'
n = same = bad = diff_ncl = same_ncl = 0; exits = []
for fa in sorted(glob.glob(sys.argv[1] + '/*.txt')):
    fb = os.path.join(sys.argv[2], os.path.basename(fa))
    A = [pat.search(l).groups() for l in open(fa) if l.startswith('HASH')]; B = [pat.search(l).groups() for l in open(fb) if l.startswith('HASH')]
    exits += [l.strip() for l in open(fa) if l.startswith('exit')] + [l.strip() for l in open(fb) if l.startswith('exit')]
    if len(A) != len(B): print('COUNT', fa, len(A), len(B)); bad += 1
    for a, b in zip(A, B):
        n += 1
        if a[6] == b[6]:
            same += 1; same_ncl += a[11] != '0'; continue
        if mode == 'ncl0' and a[11] != '0': diff_ncl += 1; continue
        bad += 1; print('DIFF', os.path.basename(fa), a[:6], a[11], b[:6])
print(f'n={n} same={same} bad={bad} ncl>0: same={same_ncl} differ={diff_ncl} exits={sorted(set(exits))}')
