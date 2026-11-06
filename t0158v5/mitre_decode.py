#!/usr/bin/env python3
# mitre_decode.py CHK4OUT: for chk4 runs off the mitre, decode the start vertex T of the first contour (line edges): turn,
# mitre ratio 1/cos(turn/2) vs the scene's mitre limit, offset d (= |S-T| - (|S-T|-d)), and the distances from S to the two
# chamfer corners of a chamfer at distance d from T (perpendicular to the bisector), and to the mitre and feet.
import sys, math
def rd(path):
    cs=[]; P=None
    for ln in open(path):
        f=ln.split()
        if not f: continue
        if f[0]=='C': cs.append([])
        elif f[0]=='L': cs[-1].append(tuple(float.fromhex(x) for x in f[1:5]))
        elif f[0] in 'AB': cs[-1].append(None)
        elif f[0]=='P': P=f
    return cs,P
for ln in open(sys.argv[1]):
    f=ln.split()
    if len(f)<14 or float(f[-1])<=1e-6: continue
    path=f[0]; S=(float(f[10]),float(f[11])); cs,P=rd(path); c=cs[0]
    if c[0] is None or c[-1] is None: print(path.split('/')[-1],'curved edge'); continue
    T=(c[0][0],c[0][1]); a=c[-1]; b=c[0]
    ui=(a[2]-a[0],a[3]-a[1]); L=math.hypot(*ui); ui=(ui[0]/L,ui[1]/L); uo=(b[2]-b[0],b[3]-b[1]); L=math.hypot(*uo); uo=(uo[0]/L,uo[1]/L)
    ar=sum(s[0]*s[3]-s[1]*s[2] for s in c if s); sg=1 if ar>0 else -1
    turn=math.atan2(ui[0]*uo[1]-ui[1]*uo[0], ui[0]*uo[0]+ui[1]*uo[1])*sg
    lim=float.fromhex(P[-1]) if P[-1].startswith(('0x','-0x')) else float(P[-1])
    dS=math.hypot(S[0]-T[0],S[1]-T[1]); d=dS-float(f[8])
    ni=(ui[1]*sg,-ui[0]*sg); no=(uo[1]*sg,-uo[0]*sg)
    Fi=(T[0]+ni[0]*d,T[1]+ni[1]*d); Fo=(T[0]+no[0]*d,T[1]+no[1]*d)
    t=d*math.tan(abs(turn)/4)
    Ci=(Fi[0]+ui[0]*t,Fi[1]+ui[1]*t); Co=(Fo[0]-uo[0]*t,Fo[1]-uo[1]*t)
    dist=lambda p:math.hypot(S[0]-p[0],S[1]-p[1])
    print('%s dir %s turn %.4f rad mitreRatio %.4f limit %.4g squared %s d %.4f | S-Fin %.3g S-Fout %.3g S-chamferIn %.3g S-chamferOut %.3g S-mitre %s lenIn %.3g lenOut %.3g'%(path.split('/')[-1],f[4],turn,1/math.cos(turn/2),lim,1/math.cos(turn/2)>lim,d,dist(Fi),dist(Fo),dist(Ci),dist(Co),f[-1],math.hypot(a[2]-a[0],a[3]-a[1]),math.hypot(b[2]-b[0],b[3]-b[1])))
