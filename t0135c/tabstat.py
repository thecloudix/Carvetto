# tabstat.py file... : sum kept/left for tabbed runs, by class (sep In/Out a>=0, sep In/Out a<0, other)
import sys, re, collections
tot=collections.Counter()
for fn in sys.argv[1:]:
    seg=[]
    for l in open(fn):
        if l.startswith('H '):
            t=l.split(); side=int(t[4]); a=float(t[8]); sep=int(t[14]); tabs=int(t[12])
            if tabs:
                left=req=0
                for d in seg:
                    if d.startswith('M '): break
                    m=re.search(r': (\d+) of (\d+) tab\(s\) left out', d)
                    if m: left+=int(m.group(1)); req+=int(m.group(2))
                cls=('On' if side==2 else 'IO')+(' sep' if sep else '')+(' a<0' if a<0 else ' a>=0')
                tot[(cls,'left')]+=left; tot[(cls,'scenes')]+=1
            seg=[]
        else: seg.append(l)
for k in sorted(tot): print(*k, tot[k])
