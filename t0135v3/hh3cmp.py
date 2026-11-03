# hh3cmp.py: compare dl3 hash outputs branch vs base/main; writes cand.txt (tabbed-move diffs) and pv.txt
import glob,collections,os,re
def rd(f):
    d={}
    for l in open(f):
        if not l.startswith('H '): continue
        head,pv=l.split('| PV') if '| PV' in l else (l,'')
        t=head.split()
        d[(t[1],t[2])]=dict(side=int(t[4]),V=int(t[6]),a=float(t[8]),neg0=t[8].startswith('-'),r=float(t[10]),tabs=int(t[12]),sep=int(t[14]),m1=t[16],d1=t[17],m0=t[21],d0=t[22],pv=dict((x.split(':')[0],tuple(map(int,x.split(':')[1].split('/')))) for x in pv.split()))
    return d
def cls(a):
    if a['side']==2: c='On'
    elif a['a']>0: c='IO a>0'
    elif a['a']==0: c='IO a=-0' if a['neg0'] else 'IO a=0'
    elif a['a']==-a['r']: c='IO a=-r'
    else: c='IO -r<a<0'
    return c+(' sep' if a['sep'] else '')
cand=set(); pvl=[]
for other in ['base','main']:
    c=collections.Counter(); ex=collections.defaultdict(list)
    for fn in glob.glob('hh/dl3_[0-9]*.txt'):
        suf=fn.split('/dl3_',1)[1]; of='hh/dl3_'+other+'_'+suf
        if not os.path.exists(of): continue
        s,m,tag=suf[:-4].split('_',2)
        n=rd(fn); o=rd(of)
        for k in n:
            if k not in o: continue
            a=n[k]; b=o[k]; cl=cls(a)
            c[('notab',cl,'n')]+=1
            if a['m0']!=b['m0']: c[('notab',cl,'MOVES DIFF')]+=1; ex[('notab',cl)].append((s,k[1],m,tag))
            elif a['d0']!=b['d0']: c[('notab',cl,'DIAG DIFF')]+=1
            if a['tabs']:
                c[('tabs',cl,'n')]+=1
                if a['m1']!=b['m1']:
                    c[('tabs',cl,'MOVES DIFF')]+=1; ex[('tabs',cl)].append((s,k[1],m,tag)); cand.add((s,k[1],m,tag))
                elif a['d1']!=b['d1']: c[('tabs',cl,'DIAG DIFF')]+=1
                if other=='base':
                    for v,(lo,req) in a['pv'].items():
                        lb=b['pv'].get(v,(0,req))[0]
                        if lo>lb: pvl.append((s,k[1],m,tag,v,lo,lb,req,sum(x[0] for x in a['pv'].values()),sum(x[0] for x in b['pv'].values())))
    print('=== vs',other)
    for k in sorted(c): print(' ',*k,c[k])
    for k in sorted(ex):
        if k[0]=='notab' or not k[1].startswith('On'): print('  EX',k,len(ex[k]),ex[k][:6])
with open('cand.txt','w') as f:
    for x in sorted(cand): f.write(' '.join(x)+'\n')
with open('pv.txt','w') as f:
    for x in pvl: f.write(' '.join(map(str,x))+'\n')
print('cand',len(cand),'per-vector fewer-kept-than-base entries',len(pvl))
