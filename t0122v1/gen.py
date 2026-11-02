import sys, math
def poly(pts, layer, closed=True):
    s = f"0\nPOLYLINE\n8\n{layer}\n66\n1\n70\n{1 if closed else 0}\n"
    for x,y in pts: s += f"0\nVERTEX\n8\n{layer}\n10\n{x}\n20\n{y}\n"
    return s + "0\nSEQEND\n"
def circle(cx,cy,r,layer): return f"0\nCIRCLE\n8\n{layer}\n10\n{cx}\n20\n{cy}\n40\n{r}\n"
def arc(cx,cy,r,a0,a1,layer): return f"0\nARC\n8\n{layer}\n10\n{cx}\n20\n{cy}\n40\n{r}\n50\n{a0}\n51\n{a1}\n"
def line(x0,y0,x1,y1,layer): return f"0\nLINE\n8\n{layer}\n10\n{x0}\n20\n{y0}\n11\n{x1}\n21\n{y1}\n"
def wrap(ents): return "0\nSECTION\n2\nENTITIES\n" + "".join(ents) + "0\nENDSEC\n0\nEOF\n"
star=[(100+ (40 if i%2==0 else 18)*math.cos(math.pi/2+i*math.pi/5), 60+(40 if i%2==0 else 18)*math.sin(math.pi/2+i*math.pi/5)) for i in range(10)]
open(sys.argv[1]+"/shapes.dxf","w").write(wrap([
  circle(40,40,30,"Ring"), circle(40,40,10,"Ring"),          # annulus: island
  poly(star,"Star"),
  poly([(150,10),(230,10),(230,90),(150,90)],"Box"), poly([(170,30),(190,30),(190,50),(170,50)],"Box"), circle(210,70,6,"Box"),
  poly([(10,120),(60,140),(110,120),(160,150)],"Open",closed=False), line(10,170,150,170,"Open"), arc(200,160,20,0,180,"Open"),
]))
