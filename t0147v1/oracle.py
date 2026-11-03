import sys
from decimal import Decimal as D, getcontext
getcontext().prec = 160
def pi_():
    getcontext().prec += 10
    def atan_inv(x):
        x=D(x); s=D(0); t=1/x; n=1; x2=x*x; sign=1
        while True:
            term=t/n
            if term < D(10)**(-getcontext().prec): break
            s += sign*term; t/=x2; n+=2; sign=-sign
        return s
    p = 4*(4*atan_inv(5)-atan_inv(239)); getcontext().prec -= 10; return +p
PI=pi_()
def sincos(a):
    a=D(a)
    k=(a/(2*PI)).to_integral_value(); a=a-k*2*PI
    s=D(0); c=D(0); term=D(1); n=0
    # series for cos and sin
    tc=D(1); ts=a; i=0
    eps=D(10)**(-155)
    while abs(tc)>eps or abs(ts)>eps:
        c+=tc; s+=ts
        tc = -tc*a*a/((2*i+1)*(2*i+2)); ts=-ts*a*a/((2*i+2)*(2*i+3)); i+=1
    return s,c
f=lambda h: D(float.fromhex(h))
def seg(p,q): return (p[0]*q[1]-q[0]*p[1])/2
def bez(P):
    # power basis
    def coeffs(v0,v1,v2,v3):
        return [v0, 3*(v1-v0), 3*(v2-2*v1+v0), v3-3*v2+3*v1-v0]
    X=coeffs(*[p[0] for p in P]); Y=coeffs(*[p[1] for p in P])
    dX=[X[1],2*X[2],3*X[3]]; dY=[Y[1],2*Y[2],3*Y[3]]
    tot=D(0)
    for i in range(4):
        for j in range(3):
            tot += (X[i]*dY[j]-Y[i]*dX[j])/(i+j+1)
    return tot/2
def arcint(c,r,a0,sw):
    a1=a0+sw
    s0,c0=sincos(a0); s1,c1=sincos(a1)
    I=(r*r*sw + r*(c[0]*(s1-s0)-c[1]*(c1-c0)))/2
    return I,(c[0]+r*c0,c[1]+r*s0),(c[0]+r*c1,c[1]+r*s1)
eps=D(2)**-52
rows=[]
cur=None
def finish(c):
    kind,cid,closed,lib,Q,spans=c
    pts=[]
    if not closed:
        rows.append((kind,cid,lib,D(0),D(0),D(0),D(1),Q)); return
    o=spans[0][1]
    A=D(0); B=D(0)  # A documented semantic, B pure geometric
    prevC=o; prevG=spans[0][2]
    S=D(0)
    def upd(p):
        nonlocal S
        d=max(abs(p[0]-o[0]),abs(p[1]-o[1])); S=max(S,d)
    for sp in spans:
        t=sp[0]
        if t=='L':
            a,b=sp[1],sp[3]
            A+=seg(prevC,a)+seg(a,b); B+=seg(prevG,a)+seg(a,b); prevC=b; prevG=b; upd(a); upd(b)
        elif t=='B':
            P=sp[3]; A+=seg(prevC,P[0])+bez(P); B+=seg(prevG,P[0])+bez(P); prevC=P[3]; prevG=P[3]
            for p in P: upd(p)
        else:
            c,r,a0,sw,cs,ce=sp[3]
            I,es,ee=arcint(c,r,a0,sw)
            A+=seg(prevC,cs)+seg(cs,es)+I+seg(ee,ce); B+=seg(prevG,es)+I; prevC=ce; prevG=ee
            upd(cs); upd(ce); S=max(S, max(abs(c[0]-o[0]),abs(c[1]-o[1]))+r)
    A+=seg(prevC,o); B+=seg(prevG,spans[0][2])
    rows.append((kind,cid,lib,A,B,S,len(spans),Q))
for line in open(sys.argv[1]):
    w=line.split()
    if w[0]=='C': cur=[w[1],int(w[2]),w[3]=='1',f(w[4]),None,[]]
    elif w[0]=='Q': cur[4]=f(w[1])
    elif w[0]=='L':
        a=(f(w[1]),f(w[2])); b=(f(w[3]),f(w[4])); cur[5].append(('L',a,a,b))
    elif w[0]=='B':
        v=[f(x) for x in w[1:]]; P=[(v[0],v[1]),(v[2],v[3]),(v[4],v[5]),(v[6],v[7])]; cur[5].append(('B',P[0],P[0],P))
    elif w[0]=='A':
        v=[f(x) for x in w[1:]]; c=(v[0],v[1]); r=v[2]; a0=v[3]; sw=v[4]; cs=(v[5],v[6]); ce=(v[7],v[8])
        s0,c0=sincos(a0); es=(c[0]+r*c0,c[1]+r*s0)
        cur[5].append(('A',cs,es,(c,r,a0,sw,cs,ce)))
    elif w[0]=='E': finish(cur)
import collections
worst=collections.defaultdict(lambda:(D(0),None)); worstB=collections.defaultdict(lambda:(D(0),None)); cnt=collections.Counter(); bad=0; signflip=0
for kind,cid,lib,A,B,S,n,Q in rows:
    cnt[kind]+=1
    scale=eps*S*S*n if S>0 else D(1)
    if kind=='open':
        if lib!=0: print('OPEN NONZERO',cid,lib); bad+=1
        continue
    e=abs(lib-A)/scale if scale>0 else abs(lib-A)
    if e>worst[kind][0]: worst[kind]=(e,cid)
    eb=abs(lib-B)/scale
    if eb>worstB[kind][0]: worstB[kind]=(eb,cid)
    if Q is not None:
        eq=abs(Q-A)/scale
        if eq>worst['Q'][0]: worst['Q']=(eq,cid)
    if e>8: bad+=1; print('BAD',kind,cid,float(lib),float(A),float(e))
    if A!=0 and abs(A)>8*scale and (lib>0)!=(A>0): signflip+=1; print('FLIP',kind,cid)
print('counts',dict(cnt)); print('bad',bad,'flips',signflip)
for k,v in worst.items(): print('worst/eps*S^2*n',k,float(v[0]),v[1])
for k,v in worstB.items(): print('vs pure-geometric',k,float(v[0]),v[1])
