import sys,math,collections
def rd(f):
    tp=collections.defaultdict(list)
    for l in open(f):
        if not l.startswith("t") or len(l.split())!=5: continue
        a,b=l.split(' ',1); k,x,y,z=b.strip().rstrip('|').split(); tp[a].append((int(k),float(x),float(y),float(z)))
    return tp
A=rd(sys.argv[1]); B=rd(sys.argv[2])
def segs(mv):
    out=[]; p=None
    for k,x,y,z in mv:
        q=(x,y,z)
        if p is not None and k==1 and abs(p[2]-z)<1e-9: out.append((p,q))
        p=q
    return out
def stats(mv):
    c=collections.Counter(k for k,*_ in mv); L=sum(math.dist(a[:2],b[:2]) for a,b in segs(mv)); return dict(c),round(L,4)
import bisect
def hd(SA,SB):
    # vertices of A to segments of B at same z via grid
    cell=0.05; g=collections.defaultdict(list)
    for a,b in SB:
        n=int(math.dist(a[:2],b[:2])/cell)+1
        seen=set()
        for i in range(n+1):
            t=i/n; x=a[0]+t*(b[0]-a[0]); y=a[1]+t*(b[1]-a[1]); kk=(int(math.floor(x/cell)),int(math.floor(y/cell)),round(a[2],6))
            if kk not in seen: seen.add(kk); g[kk].append((a,b))
    w=0; wp=None
    for a,b in SA:
        for p in (a,b):
            best=1e9; cx=int(math.floor(p[0]/cell)); cy=int(math.floor(p[1]/cell))
            for dx in (-1,0,1):
                for dy in (-1,0,1):
                    for s,e in g.get((cx+dx,cy+dy,round(p[2],6)),()):
                        vx=e[0]-s[0]; vy=e[1]-s[1]; l=vx*vx+vy*vy; t=0 if l==0 else max(0,min(1,((p[0]-s[0])*vx+(p[1]-s[1])*vy)/l))
                        best=min(best,math.hypot(p[0]-s[0]-t*vx,p[1]-s[1]-t*vy))
            if best>w: w=best; wp=p
    return (w, wp if w>0 else None)
for t in sorted(set(A)|set(B)):
    sa,sb=segs(A[t]),segs(B[t])
    print(t,'old',stats(A[t]),'new',stats(B[t]),'H old->new',hd(sa,sb),'new->old',hd(sb,sa))
