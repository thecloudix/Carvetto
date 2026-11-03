import sys
from fractions import Fraction as F
def sgn(x): return (x>0)-(x<0)
bad=0; n=0; zeroes=0; badlines=[]
for line in sys.stdin:
    t=line.split()
    if not t: continue
    v=[F(float.fromhex(x)) for x in t[1:-1]]; got=int(t[-1]); n+=1
    if t[0]=='O':
        ax,ay,bx,by,cx,cy=v; exp=sgn((ax-cx)*(by-cy)-(ay-cy)*(bx-cx))
    else:
        ax,ay,bx,by,cx,cy,dx,dy=v
        adx,ady,bdx,bdy,cdx,cdy=ax-dx,ay-dy,bx-dx,by-dy,cx-dx,cy-dy
        exp=sgn((adx*adx+ady*ady)*(bdx*cdy-cdx*bdy)+(bdx*bdx+bdy*bdy)*(cdx*ady-adx*cdy)+(cdx*cdx+cdy*cdy)*(adx*bdy-bdx*ady))
    if exp==0: zeroes+=1
    if exp!=got:
        bad+=1
        if len(badlines)<10: badlines.append(line.strip()+"  expected %d"%exp)
print("cases",n,"exact-zero",zeroes,"mismatches",bad)
for b in badlines: print("  ",b)
