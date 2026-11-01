import sys,math,re
# usage: misschk.py dumpfile px py
px,py=float(sys.argv[2]),float(sys.argv[3])
best={}
prev={}
for line in open(sys.argv[1]):
    m=re.match(r't(\d+) (\d+) (\S+) (\S+) (\S+)\|',line)
    if not m: continue
    t=int(m.group(1)); k=int(m.group(2)); x,y,z=map(float,m.group(3,4,5))
    if t in prev and k!=0:
        ax,ay,az=prev[t]
        dx,dy=x-ax,y-ay; L=dx*dx+dy*dy
        tt=0 if L==0 else max(0,min(1,((px-ax)*dx+(py-ay)*dy)/L))
        d=math.hypot(px-ax-tt*dx,py-ay-tt*dy)
        zz=az+tt*(z-az)
        if zz<0 and (t not in best or d<best[t][0]): best[t]=(d,zz,(ax,ay,az),(x,y,z))
    prev[t]=(x,y,z)
for t,v in sorted(best.items()): print('tool',t,'nearest in-material move dist %.4f at z %.3f'%(v[0],v[1]),v[2],v[3])
