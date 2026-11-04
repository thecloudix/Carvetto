#!/usr/bin/env python3
# cmp.py OUTDIR (PARTIAL=1: compare common tags of killed/partial runs): pair br_*/main_* lines by tag; 'same' must hash-match; 'onabove' must be refused on br
import glob, os, sys, collections
d = sys.argv[1]; st = collections.Counter(); bad = []
def parse(f):
    m = {}
    for l in open(f):
        if ' hash ' not in l: continue
        w = l.split(' | ')[0].split(); m[w[0]] = (w, l.rstrip())
    return m
for fb in sorted(glob.glob(os.path.join(d, 'br_*.txt'))):
    fm = fb.replace('/br_', '/main_'); B = parse(fb); M = parse(fm)
    if len(B) != len(M):
        if os.environ.get('PARTIAL'): st['partial_files'] += 1
        else: bad.append(('count', fb, len(B), len(M)))
    for t, (wb, lb) in B.items():
        if t not in M:
            if not os.environ.get('PARTIAL'): bad.append(('missing', t))
            continue
        wm, lm = M[t]; exp = wb[2]; kv = dict(zip(wb[3::2], wb[4::2])); km = dict(zip(wm[3::2], wm[4::2]))
        side = kv['side']; key = (exp, 'side' + side, 'sep' + kv['sep'])
        if exp == 'same':
            if kv['hash'] == km['hash']: st[key + ('match',)] += 1
            else: st[key + ('DIFF',)] += 1; bad.append(('diff', lb, lm))
            if kv['moves'] == '0': st['same_zero_moves'] += 1
        else:
            ok = kv['moves'] == '0' and kv['diags'] == '1' and 'above the tool radius' in lb
            st[key + ('refused' if ok else 'NOTREFUSED',)] += 1
            if not ok: bad.append(('notrefused', lb))
            if km['moves'] != '0': st['onabove_main_cut'] += 1
for k in sorted(st, key=str): print(k, st[k])
print('BAD', len(bad))
for b in bad[:30]: print(b)
