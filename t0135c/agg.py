# agg.py: aggregate wsf8 sweeps: swB (branch), swO (e4da396), swT (T-0127)
import glob, collections, math, sys
D='/home/chris/wt-T0135/build-drv/sw/'
def rd(b):
    S={}; T=collections.defaultdict(list)
    for f in glob.glob(D+b+'_[0-9]*.log'):
        for l in open(f):
            t=l.split()
            if l.startswith('S ') and 'side' in l: S[(t[1],t[2])]=l
            elif l.startswith('T '): T[(t[1],t[2])].append((float(t[3]),float(t[4]),float(t[5]),int(t[6]),int(t[7])))
    return S,T
def g(l,k):
    t=l.split(); return t[t.index(k)+1]
def cls(l):
    side=int(g(l,'side')); a=float(g(l,'a')); V=int(g(l,'V')); sep=int(g(l,'sep'))
    s='On' if side==2 else 'IO'
    if side==2: s+=' a<0' if a<0 else (' a=0' if a==0 else ' a>0')
    return s+(' V' if V else ' EM')+(' sep' if sep else '')
BR=sys.argv[1] if len(sys.argv)>1 else 'swB2'
B,BT=rd(BR); O,OT=rd('swO'); T,TT=rd('swT')
c=collections.Counter(); bad=[]
for k,l in B.items():
    if 'empty' in l or 'nocut' in l: c['empty/nocut']+=1; continue
    c['scenes']+=1; c['scenes mode'+('0' if k[0].startswith('501') else '1' if k[0].startswith('502') else '2')]+=1
    for key in ('sev','part','nohold','thin'):
        v=int(g(l,key)); c['branch '+key]+=v
        if v: bad.append((key,k,l.strip()))
    c['branch kept']+=int(g(l,'G')); c['branch left']+=int(g(l,'left')); c['branch req']+=int(g(l,'req'))
    if '[FREE' in l: c['branch scenes with free pieces']+=1
    for f in ('UNREP','FALSEREP','CNT '):
        if f in l: c['branch '+f.strip()]+=1; bad.append((f.strip(),k,l.strip()))
def near(a,b): return math.hypot(a[0]-b[0],a[1]-b[1])<0.6
for name,(X,XT) in (('old',(O,OT)),('t127',(T,TT))):
    more=collections.Counter(); ex=collections.defaultdict(list)
    for k,l in X.items():
        if k not in B or 'side' not in B[k] or 'empty' in l or 'nocut' in l: continue
        c[name+' scenes']+=1
        for key in ('sev','part','nohold','thin'): c[name+' '+key]+=int(g(l,key))
        c[name+' kept']+=int(g(l,'G'))
        if 'CNT ' in l: c[name+' CNT']+=1
        bt=BT.get(k,[]); cl=cls(B[k])
        c[name+' holdOK']+=sum(1 for t in XT.get(k,[]) if t[3]==1 and t[4]==1)
        c['branch holdOK (on '+name+' scenes)']+=sum(1 for t in bt if t[3]==1 and t[4]==1)
        for t in XT.get(k,[]):
            if t[3]==1 and t[4]==1 and not any(near(t,u) for u in bt):
                more[cl]+=1; ex[cl].append((k,t))
    fewer=collections.Counter(); fewEx=[]
    for k,l in X.items():
        if k not in B or 'side' not in B[k] or 'empty' in l or 'nocut' in l: continue
        hb=sum(1 for t in BT.get(k,[]) if t[3]==1 and t[4]==1); hx=sum(1 for t in XT.get(k,[]) if t[3]==1 and t[4]==1)
        if hb<hx: fewer[cls(B[k])]+=hx-hb; fewEx.append((k,hx,hb))
    print(name,'scenes where branch keeps fewer holding tabs, by class (tabs short):',dict(fewer),fewEx[:8])
    print(name,'holding tabs absent in branch (by position), by class:',dict(more))
    for cl in ex: print('  EX',name,cl,ex[cl][:6])
for k in sorted(c): print(k,c[k])
print('BAD',len(bad))
for b in sorted(bad)[:80]: print(' ',b[0],b[1],b[2][-260:])
