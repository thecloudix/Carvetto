#!/usr/bin/env python3
# t0164v1 (d): for every clamped case (ncl>0, stepover>width) compare the moves with the carver's stepover at 0.5 x width:
# moves change <=> the carver did rest clearing <=> StepoverLimited (29) should be present.
import re, sys, glob, os
pat = re.compile(r'HASH case (\d+) (.+?) chord (\S+) ok (\d) moves (\d+) diags (\d+) h (\S+) mh (\S+) dl (\S*) so (\S+) width (\S+) ncl (\d+)')
def rows(f): return [pat.search(l).groups() for l in open(f) if 'HASH' in l]
agree = dis = 0
for fb in sorted(glob.glob(sys.argv[1] + '/*.txt')):
    A = rows(fb); C = rows(os.path.join(sys.argv[2], os.path.basename(fb)))
    for a, c in zip(A, C):
        if not (int(a[11]) > 0 and float(a[9]) > float(a[10]) and a[3] == '1'): continue
        rest = a[7] != c[7]; diag = '29' in a[8].split(',')
        ok = rest == diag; agree += ok; dis += not ok
        print(('OK  ' if ok else 'BAD ') + os.path.basename(fb), a[0], a[1], 'chord', a[2], 'so', a[9], 'width', a[10], 'rest_work', rest, 'diag', diag)
print('agree', agree, 'disagree', dis)
