import re,sys
txt=open(sys.argv[1]).read().split('\n')
def chains(prefix):
    res=[];cur=None
    for l in txt:
        if l.startswith(prefix+' chain'):
            cur={'closed':'closed=1' in l,'sp':[]};res.append(cur)
        elif l.startswith('branch') or l.startswith('main'):
            cur=None
        elif cur is not None and l.startswith('  '):
            cur['sp'].append(l)
    return res
import math
def ends(sp):
    m=re.match(r'  L \(([^,]+),([^)]+)\)->\(([^,]+),([^)]+)\)',sp)
    if m: return (float(m[1]),float(m[2])),(float(m[3]),float(m[4]))
    m=re.match(r'  A c\(([^,]+),([^)]+)\) r (\S+) sa (\S+) ea (\S+)',sp)
    cx,cy,r,sa,ea=map(float,m.groups())
    return (cx+r*math.cos(sa),cy+r*math.sin(sa)),(cx+r*math.cos(ea),cy+r*math.sin(ea))
for pre in ['branch','main']:
    cs=chains(pre)
    for i,c in enumerate(cs):
        if c['closed']: continue
        e=ends(c['sp'][-1])[1]
        for j,c2 in enumerate(cs):
            if j==i or c2['closed']: continue
            s=ends(c2['sp'][0])[0]
            dd=math.dist(e,s)
            if dd<=1e-3: print(pre,'chain',i,'end -> chain',j,'start gap %.3g'%dd, 'at', e)
def dseg(p,sp):
    m=re.match(r'  L \(([^,]+),([^)]+)\)->\(([^,]+),([^)]+)\)',sp)
    if m:
        ax,ay,bx,by=map(float,m.groups()); vx,vy=bx-ax,by-ay; L2=vx*vx+vy*vy
        t=0 if L2==0 else max(0,min(1,((p[0]-ax)*vx+(p[1]-ay)*vy)/L2))
        return math.dist(p,(ax+t*vx,ay+t*vy))
    m=re.match(r'  A c\(([^,]+),([^)]+)\) r (\S+) sa (\S+) ea (\S+) (\S+)',sp)
    cx,cy,r,sa,ea=map(float,m.groups()[:5]); ccw=m[6]=='CCW'
    a=math.atan2(p[1]-cy,p[0]-cx)
    sw=(ea-sa) if ccw else (sa-ea)
    sw%= 2*math.pi
    rel=((a-sa) if ccw else (sa-a))%(2*math.pi)
    if rel<=sw: return abs(math.dist(p,(cx,cy))-r)
    s,e=ends(sp); return min(math.dist(p,s),math.dist(p,e))
print('-- crossing check')
for pre in ['branch','main']:
    cs=chains(pre)
    for i,c in enumerate(cs):
        if c['closed']: continue
        e=ends(c['sp'][-1])[1]
        for j,c2 in enumerate(cs):
            if j==i or c2['closed']: continue
            if math.dist(e,ends(c2['sp'][0])[0])<=1e-3:
                others=[(k,q) for k,c3 in enumerate(cs) for q,sp in enumerate(c3['sp']) if dseg(e,sp)<1e-7 and not (k==i and q==len(c['sp'])-1) and not (k==j and q==0)]
                print(pre,'split',i,'->',j,'other spans through the node:',others)
