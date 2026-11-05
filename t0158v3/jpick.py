#!/usr/bin/env python3
# jpick.py OUT/cfg: for 'id' (or scn) runs, rule-2/3 calls whose picked junction is farther from the target than the run's
# nearest junction by > JPM (default 1e-6 mm): the start went to a neighbouring corner, not the corner nearest the target.
# Per family: calls with rule 2/3, of which pick != nearest; distribution of the extra distance; examples.
import sys, glob, collections, re, os
D = sys.argv[1]; JPM = float(os.environ.get('JPM', '1e-6'))
st = collections.defaultdict(collections.Counter); ex = collections.defaultdict(list)
for p in sorted(glob.glob(D + '/*.txt')):
    base = os.path.basename(p); m = re.search(r'(gen\d)_([a-z0-9]+)_\d+\.scn', base); fam0 = (m.group(1) + ':' + m.group(2)) if m else 'stored'
    for ln in open(p, errors='replace'):
        f = ln.split()
        if not f or f[0] != 'T' or len(f) < 24: continue
        tag = f[1]; x = tag.rsplit(':', 1)[-1]
        if x not in ('id', 'orig', 'vsout', 'bbc', 'opt') and not re.match(r'r\d+:\d+:m', tag): continue
        fam = fam0 if fam0 != 'stored' else 'stored:' + (x if x in ('orig', 'vsout', 'bbc', 'opt') else 'rnd')
        rule = int(f[3]); 
        if rule not in (2, 3): continue
        st[fam]['rule23'] += 1
        jpm = float(f[23]); rl = int(f[21])
        if rl >= 3: st[fam]['runLen>=3'] += 1
        if jpm > JPM:
            st[fam]['pickNotNearest'] += 1
            b = '>1mm' if jpm > 1 else '>0.1mm' if jpm > 0.1 else '>0.01mm' if jpm > 0.01 else '>1e-3' if jpm > 1e-3 else '>1e-6'
            st[fam]['extra' + b] += 1
            if len(ex[fam]) < 4: ex[fam].append((base[-40:], tag[-30:], 'call', f[2], 'runLen', rl, 'pick', f[20], 'jmin', f[22], 'extra %.4g' % jpm))
for fam in sorted(st):
    print(f'{fam:16s} ' + ' '.join(f'{k} {v}' for k, v in sorted(st[fam].items())))
    for e in ex[fam]: print('     e.g.', *e)
