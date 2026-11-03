import sys
from fractions import Fraction as F
def sgn(x): return (x>0)-(x<0)
from collections import Counter
cnt=Counter(); ex=[]
for line in sys.stdin:
    t=line.split(); vals=[float.fromhex(x) for x in t[1:-1]]; v=[F(x) for x in vals]; got=int(t[-1])
    if t[0]=='O':
        ax,ay,bx,by,cx,cy=v; exp=sgn((ax-cx)*(by-cy)-(ay-cy)*(bx-cx))
    else:
        ax,ay,bx,by,cx,cy,dx,dy=v
        adx,ady,bdx,bdy,cdx,cdy=ax-dx,ay-dy,bx-dx,by-dy,cx-dx,cy-dy
        exp=sgn((adx*adx+ady*ady)*(bdx*cdy-cdx*bdy)+(bdx*bdx+bdy*bdy)*(cdx*ady-adx*cdy)+(cdx*cdx+cdy*cdy)*(adx*bdy-bdx*ady))
    small=any(x!=0 and abs(x)<1e-140 for x in vals)
    if exp!=got:
        cnt[(t[0],'tiny<1e-140' if small else 'in-range')]+=1
        if not small and len(ex)<8: ex.append(line.strip()+" exp %d"%exp)
print(cnt)
for e in ex: print(" ",e)
