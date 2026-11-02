import math,sys
def load(f):
    r=[]
    for l in open(f):
        if l.startswith('diag') or l.startswith('scene'): continue
        p=l.strip().rstrip('|').split(); r.append((int(p[0]),float(p[1]),float(p[2]),float(p[3])))
    return r
a=load(sys.argv[1]); b=load(sys.argv[2])
def dseg(p,q,s):
    d=[s[i]-q[i] for i in range(3)]; l=sum(t*t for t in d)
    t=0 if l==0 else max(0,min(1,sum((p[i]-q[i])*d[i] for i in range(3))/l))
    return math.sqrt(sum((p[i]-q[i]-t*d[i])**2 for i in range(3)))
def h(A,B):
    w=0; Bp=[x[1:] for x in B]
    # grid index of B segments for global nearest
    import collections
    cell=1.0; G=collections.defaultdict(list)
    for j in range(1,len(Bp)):
        q,s=Bp[j-1],Bp[j]
        for cx in range(int(math.floor(min(q[0],s[0])/cell)),int(math.floor(max(q[0],s[0])/cell))+1):
            for cy in range(int(math.floor(min(q[1],s[1])/cell)),int(math.floor(max(q[1],s[1])/cell))+1):
                if len(G[(cx,cy)])<4000: G[(cx,cy)].append(j)
    for x in A:
        p=x[1:]; cx,cy=int(math.floor(p[0]/cell)),int(math.floor(p[1]/cell)); best=1e9
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                for j in G.get((cx+dx,cy+dy),()):
                    best=min(best,dseg(p,Bp[j-1],Bp[j]))
        w=max(w,min(best,1.0))
    return w
print(len(a),len(b),'dev new->old %.3g old->new %.3g'%(h(a,b),h(b,a)))
