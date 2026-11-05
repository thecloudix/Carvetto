#!/usr/bin/env python3
# t0164v1: compare branch vs main HASH lines file by file: moves hash (mh) must match; diag lists may differ only by
# StepoverLimited (29), which must then satisfy ncl>0 and stepover > width (independent width = 2*footprint(limit)).
import re, sys, glob, os
pat = re.compile(r'HASH case (\d+) (.+?) chord (\S+) ok (\d) moves (\d+) diags (\d+) h (\S+) mh (\S+) dl (\S*) so (\S+) width (\S+) ncl (\d+)')
def rows(f): return [pat.search(l).groups() for l in open(f) if 'HASH' in l]
stats = dict(n=0, same=0, mhdiff=0, so_only=0, so_bad=0, other=0, so_cases=0, clamp_nodiag=0, refused_br=0, refused_main=0)
for fb in sorted(glob.glob(sys.argv[1] + '/*.txt')):
    fm = os.path.join(sys.argv[2], os.path.basename(fb)); A, B = rows(fb), rows(fm)
    assert len(A) == len(B), fb
    for a, b in zip(A, B):
        stats['n'] += 1; stats['refused_br'] += a[3] == '0'; stats['refused_main'] += b[3] == '0'
        if a[7] != b[7]: stats['mhdiff'] += 1; print('MOVES DIFFER', os.path.basename(fb), a[:6], b[:6]); continue
        so, w, ncl = float(a[9]), float(a[10]), int(a[11])
        if a[6] == b[6]:
            stats['same'] += 1
            if ncl > 0 and so > w and a[3] == '1': stats['clamp_nodiag'] += 1
            continue
        da = [x for x in a[8].split(',') if x]; db = [x for x in b[8].split(',') if x]
        if [x for x in da if x != '29'] == db and da.count('29') == 1:
            stats['so_only'] += 1
            if not (ncl > 0 and so > w): stats['so_bad'] += 1; print('SO NOT JUSTIFIED', os.path.basename(fb), a)
            elif so <= w * (1 + 1e-12): print('SO AT ROUNDING', os.path.basename(fb), a[:3], so, w)
        else: stats['other'] += 1; print('DIAG DIFFER', os.path.basename(fb), a[:6], a[8], b[8])
print(stats)
