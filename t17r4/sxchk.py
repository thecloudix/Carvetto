from math import cos,sin,hypot,atan2
B=(4.7312675883853093,0.12127593674145538,0.060637968370727331,0.10384123423698335,-1.4997348578015719)
for name,(cx,cy) in {'br':(-3.9648284655413623e-12,42430.389760319311),'ref':(0.0,42430.389760319318)}.items():
    r=42430.329233577009
    # signed distance of points along span4's last 20 um to big circle (positive = outside)
    out=[]
    for k in range(0,41):
        a=B[4]+ (k*0.5e-6)/B[2]   # walk back along CW arc (angles increase backwards)
        p=(B[0]+B[2]*cos(a),B[1]+B[2]*sin(a))
        out.append((k*0.5, hypot(p[0]-cx,p[1]-cy)-r))
    print(name, ' '.join('%.1fum:%.2e'%x for x in out[::4]))
