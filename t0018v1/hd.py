import math
a=[l.split() for l in open('out/dump_pc_new.txt')]; b=[l.split() for l in open('out/dump_pc_old.txt')]
def P(x): return tuple(float(x[i].rstrip('|')) for i in (2,3,4))
def dseg(p,a,b):
    d=[b[i]-a[i] for i in range(3)]; l=sum(t*t for t in d)
    t=0 if l==0 else max(0,min(1,sum((p[i]-a[i])*d[i] for i in range(3))/l))
    return math.sqrt(sum((p[i]-a[i]-t*d[i])**2 for i in range(3)))
def h(A,B):
    worst=0
    for i in range(len(A)):
        if A[i]==B[i]: continue
        p=P(A[i]); best=1e9
        for j in range(max(1,i-3),min(len(B),i+4)):
            best=min(best,dseg(p,P(B[j-1]),P(B[j])))
        worst=max(worst,best)
    return worst
if len(a)==len(b): print('path-to-path deviation new->old %.3g old->new %.3g'%(h(a,b),h(b,a)))
