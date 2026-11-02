import sys
def load(p):
    d={};k=None
    for l in open(p):
        if l[0]=='K': k=l.split()[1]; d[k]=[]
        elif l[0]=='R': d[k].append(l.split()[1:])
    return d
a=load(sys.argv[1]); b=load(sys.argv[2])
same=diff=added=suspicious=removed=0; maxbb=0.0; maxhw=0.0
for k in a:
    ra=a[k]; rb=b.get(k)
    if rb is None: print("missing key",k); continue
    if ra==rb: same+=1; continue
    diff+=1
    rest=list(ra)
    for r in rb:
        if r in rest: rest.remove(r)
        else: added+=1; print("ADDED/CHANGED",k,r)
    for r in rest:
        removed+=1; area=abs(float(r[1])); per=float(r[2]); import math; ir=float(r[7]); n=int(r[0]); ub=2*area/per if n==3 else math.sqrt(area/math.pi); half=ub; maxgrid=max(globals().get('maxgrid',0),ir); globals()['maxgrid']=maxgrid
        maxhw=max(maxhw,area/per if per>0 else 0); maxbb=max(maxbb,half)
        if half>4.5e-6: suspicious+=1; print("REMOVED-WIDE?",k,r)
print("max grid inradius estimate of removed",globals().get("maxgrid",0)); print(f"keys same={same} differ={diff} removedRings={removed} added={added} removedWithInradiusUpperBound>4.5nm={suspicious} max(area/perim)={maxhw:.3e} maxRemovedInradiusUB={maxbb:.3e}")
