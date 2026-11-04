#!/usr/bin/env python3
# cmp2.py DIR [DIR...]: pair br_*/main_* files line by line by tag (first word); every common tag must
# have the same 'hash' (or the same trailing result for edge lines); tags only on br = main crashed/killed.
import glob, os, sys, collections, re
st = collections.Counter(); bad = []
def parse(f):
    m = {}
    for l in open(f, errors='replace'):
        w = l.split()
        if not w or ':' not in w[0]: continue
        h = re.search(r'hash ([0-9a-f]+)', l); c = 'CRASH' in l or re.search(r' rc [1-9]', l)
        m[w[0]] = ('TIMEOUT' if 'TIMEOUT' in l else 'CRASH' if c else (h.group(1) if h else None), l.rstrip())
    return m
for d in sys.argv[1:]:
    for fb in sorted(glob.glob(os.path.join(d, 'br_*.txt'))):
        fm = fb.replace('/br_', '/main_'); B = parse(fb); M = parse(fm) if os.path.exists(fm) else {}
        for t, (hb, lb) in B.items():
            if hb is None: continue
            if t not in M or M[t][0] is None: st['br_only(main crashed/killed)'] += 1; bad.append(('br_only', lb)); continue
            hm, lm = M[t]
            if 'TIMEOUT' in (hb, hm): st['timeout'] += 1; continue
            if hb == 'CRASH': st['BR_CRASH'] += 1; bad.append(('BR_CRASH', lb)); continue
            if hm == 'CRASH': st['main_crash_br_ok'] += 1; bad.append(('main_crash', lb)); continue
            if hb == hm: st['match'] += 1
            else: st['DIFF'] += 1; bad.append(('DIFF', lb, lm))
        for t in M:
            if t not in B and M[t][0] is not None: st['main_only'] += 1; bad.append(('main_only', M[t][1]))
for k in sorted(st): print(k, st[k])
for b in bad[:40]: print(b)
