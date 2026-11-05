#!/usr/bin/env python3
# floodagg.py OUT: per scene, branch vs main whole-scene flood verdicts (hold/nohold/sev/part, FREE, UNREP, REP)
import glob, os, re, sys, collections
O = sys.argv[1]; c = collections.Counter(); worse = []; better = []
def parse(p):
    for ln in open(p, errors='replace'):
        if ln.startswith('S '):
            m = re.search(r'G (\d+) hold (\d+) nohold (\d+) sev (\d+) part (\d+) thin (\d+) \| left (\d+)', ln)
            g = list(map(int, m.groups()))
            fr = re.search(r'\[FREE (\d+) ge05 (\d+)', ln); unrep = '[UNREP]' in ln; cnt = re.search(r'\[CNT free04 (\d+) rep (\d+)', ln)
            return dict(G=g[0], hold=g[1], nohold=g[2], sev=g[3], part=g[4], left=g[6], free=int(fr.group(1)) if fr else 0, free05=int(fr.group(2)) if fr else 0,
                        unrep=unrep, unrepN=(int(cnt.group(1)) - int(cnt.group(2))) if cnt else 0)
    return None
for p in sorted(glob.glob(f'{O}/br/*.txt')):
    q = p.replace('/br/', '/main/'); a = parse(p); b = parse(q) if os.path.exists(q) else None
    if not a or not b: c['incomplete'] += 1; continue
    c['scenes'] += 1
    key = os.path.basename(p)
    w = (a['sev'] > b['sev']) or (a['part'] > b['part']) or (a['free05'] > b['free05']) or (a['unrepN'] > b['unrepN']) or (a['unrep'] and not b['unrep'])
    bt = (a['sev'] < b['sev']) or (a['free05'] < b['free05']) or (a['unrepN'] < b['unrepN'])
    if a == b: c['identical verdict'] += 1
    if w: worse.append((key, a, b))
    if bt: better.append((key, a, b))
    for k in ('sev', 'part', 'free05', 'unrepN', 'nohold'): c[f'br {k} total'] += a[k]; c[f'main {k} total'] += b[k]
print(dict(c)); print('WORSE on branch:', len(worse))
for w in worse: print('  ', w)
print('better on branch:', len(better))
for w in better[:20]: print('  ', w)
