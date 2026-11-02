# agg3.py: aggregate wsf8 sweeps (branch wsf8, main wsf8_old, keep-all mutant wsf8_mut)
import glob, re, collections, math, sys
D='/home/chris/wt-T0127/build-verify3/drv/sw/'
def rd(b):
    S={}; T=collections.defaultdict(list)
    for f in glob.glob(D+b+'_4*.log'):
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
B,BT=rd('wsf8'); O,OT=rd('wsf8_old'); M,MT=rd('wsf8_mut')
c=collections.Counter(); bad=[]
for k,l in B.items():
    if 'empty' in l or 'nocut' in l: continue
    c['scenes']+=1; cl=cls(l); c['scenes '+cl.split()[0]]+=1
    for key in ('sev','part','nohold','thin'):
        v=int(g(l,key)); c['branch '+key]+=v
        if v: bad.append((key,k,l.strip()))
    c['branch kept']+=int(g(l,'G')); c['branch left']+=int(g(l,'left')); c['branch req']+=int(g(l,'req'))
    if 'UNREP' in l: bad.append(('UNREP',k,l.strip()))
    if 'FALSEREP' in l: bad.append(('FALSEREP',k,l.strip()))
    if 'CNT ' in l: bad.append(('CNT',k,l.strip()))
def near(a,b): return math.hypot(a[0]-b[0],a[1]-b[1])<0.6
dropHold=collections.Counter(); mainMore=collections.Counter(); ex=collections.defaultdict(list)
for k,l in B.items():
    bt=BT.get(k,[]); cl=cls(l)
    if k in M:
        for t in MT.get(k,[]):
            if t[3]==1 and t[4]==1 and not any(near(t,u) for u in bt):
                dropHold[cl]+=1; ex[('mut',cl)].append((k,t))
        c['mut scenes']+=1
        for key in ('sev','part','nohold','thin'): c['mut '+key]+=int(g(M[k],key))
        c['mut holdOK']+=sum(1 for t in MT.get(k,[]) if t[3]==1 and t[4]==1)
        c['branch holdOK (mut scenes)']+=sum(1 for t in bt if t[3]==1 and t[4]==1)
    if k in O:
        c['main scenes']+=1
        for key in ('sev','part','nohold','thin'): c['main '+key]+=int(g(O[k],key))
        c['main kept']+=int(g(O[k],'G'))
        c['main UNREP04']+= 1 if 'CNT ' in O[k] else 0
        for t in OT.get(k,[]):
            if t[3]==1 and t[4]==1 and not any(near(t,u) for u in bt):
                mainMore[cl]+=1; ex[('main',cl)].append((k,t))
for k in sorted(c): print(k,c[k])
print('mutant holding (non-thin) tabs dropped by branch, by class:',dict(dropHold))
print('main holding (non-thin) tabs absent in branch, by class:',dict(mainMore))
for k in sorted(ex): print('EX',k,ex[k][:6])
print('BAD',len(bad))
for b in bad: print(*b)
