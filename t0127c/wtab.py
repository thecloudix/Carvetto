# wtab.py scene mx my plateau [bin] : windowed flood across one tab (direction from the nearest vector)
import sys,subprocess,math
scn,mx,my,L=sys.argv[1],float(sys.argv[2]),float(sys.argv[3]),float(sys.argv[4])
b=sys.argv[5] if len(sys.argv)>5 else 'flood'
D,a,tx,ty=map(float,subprocess.run(['./bin/tangent',scn,str(mx),str(my)],capture_output=True,text=True).stdout.split())
r=D/2; nx,ny=-ty,tx
h=max(L/2+1.5, r+3); off=r+abs(a)+1.2
args=['./bin/'+b,scn,str(mx+nx*off),str(my+ny*off),str(mx-nx*off),str(my-ny*off),'0.02',str(mx-h),str(my-h),str(mx+h),str(my+h)]
out=subprocess.run(args,capture_output=True,text=True).stdout.strip().split('\n')[-1]
allo=subprocess.run(args,capture_output=True,text=True).stdout.strip().split('\n')
g=[l for l in allo if l.startswith('grid')]
seeds=(' ['+g[0].split('segs')[1].split(None,1)[1]+']') if g else ''
print(('HELD' if 'YES' in out else ('FREE' if 'NO' in out else 'ERR '+out))+seeds)
