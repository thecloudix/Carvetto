import re,glob,math,collections,sys
def parse(lines):
    d={}; cur=[]
    for l in lines:
        if l.startswith('T '):
            t=l.split(); cur.append((float(t[3]),float(t[4]),float(t[5]),int(t[6]),int(t[7]))); continue
        if l.startswith('S ') and ' | G ' in l:
            t=l.split(); key=(t[1],t[2])
            side=int(t[4]); a=float(t[10]); sep=int(t[12]); V=int(t[8])
            d[key]=dict(tabs=cur,side=side,a=a,sep=sep,V=V,line=l.strip())
            cur=[]
        elif l.startswith('S '): cur=[]
    return d
br={}
for f in glob.glob('sw/wsf8x_*.log'): br.update(parse(open(f)))
def cls(s):
    c='On' if s['side']==2 else 'IO'
    c+=' a<0' if s['a']<0 else ' a>=0'
    c+=' V' if s['V'] else ' EM'
    c+=' sep' if s['sep'] else ''
    return c
for name in sys.argv[1:]:
    other=parse(open(name))
    miss=collections.Counter(); ex=collections.defaultdict(list); n=0; held=0
    for k,s in other.items():
        if k not in br: continue
        n+=1
        b=br[k]
        for x,y,L,v0,v5 in s['tabs']:
            if v0==1 and v5==1:
                held+=1
                if not any(math.hypot(x-p,y-q)<1.0 for p,q,_,_,_ in b['tabs']):
                    miss[cls(s)]+=1; ex[cls(s)].append((k,(x,y)))
    print(name,'scenes',n,'holding tabs',held,'missing in branch by class',dict(miss))
    for c in ex: print('  EX',c,ex[c][:12])
print('--- per-scene holding-tab counts (branch < other)')
for name in sys.argv[1:]:
    other=parse(open(name)); fewer=[]
    for k,s in other.items():
        if k not in br: continue
        hb=sum(1 for t in br[k]['tabs'] if t[3]==1 and t[4]==1); ho=sum(1 for t in s['tabs'] if t[3]==1 and t[4]==1)
        if hb<ho: fewer.append((k,hb,ho,cls(s)))
    print(name,'scenes where branch holds fewer:',fewer)
