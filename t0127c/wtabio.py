# wtabio.py scene mx my plateau [bin]: windowed flood part->waste across one In/Out tab
import sys,subprocess
scn,mx,my,L=sys.argv[1],float(sys.argv[2]),float(sys.argv[3]),float(sys.argv[4])
b=sys.argv[5] if len(sys.argv)>5 else 'flood'
D,a,px,py,wx,wy,tx,ty=map(float,subprocess.run(['./bin/seedsio',scn,str(mx),str(my)],capture_output=True,text=True).stdout.split())
r=D/2; h=max(L/2+1.5, 2*r+abs(a)+3)
# window: square round the tab (may include neighbours when tabs are close)
args=['./bin/'+b,scn,str(px),str(py),str(wx),str(wy),'0.02',str(mx-h),str(my-h),str(mx+h),str(my+h)]
out=subprocess.run(args,capture_output=True,text=True).stdout.strip().split('\n')
g=[l for l in out if l.startswith('grid')]
print(('HELD' if 'YES' in out[-1] else 'FREE'), g[0].split('segs')[1].strip() if g else out[-1])
