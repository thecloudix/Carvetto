import re,collections,statistics,sys
d=collections.defaultdict(list)
for l in open(sys.argv[1]):
    m=re.match(r'(SIDE=\S+ ALLOW=\S+) (perf\d)(_base|_main)? (\d+ \d+ \d+):.*cpu ([\d.]+)s',l)
    if m: d[(m.group(1),m.group(2),m.group(4))+((m.group(3) or '_new'),)].append(float(m.group(5)))
keys=sorted(set(k[:3] for k in d))
for k in keys:
    med={v:statistics.median(d[k+(v,)]) for v in ['_new','_base','_main'] if k+(v,) in d}
    print('%-22s %s %-9s new %.2f base %.2f main %.2f  new/base %.3f new/main %.3f'%(k[0],k[1],k[2],med['_new'],med['_base'],med['_main'],med['_new']/med['_base'],med['_new']/med['_main']))
