import sys,subprocess,math
scn,mx,my,ax,ay,bx,by,D=sys.argv[1],*map(float,sys.argv[2:9])
r=D/2; tx,ty=bx-ax,by-ay; L=math.hypot(tx,ty); tx/=L; ty/=L; nx,ny=-ty,tx
h=max(L/2+r+1.5, r+3); off=r+1.2
for b in ('flood','flood_old'):
    args=['./bin/'+b,scn,str(mx+nx*off),str(my+ny*off),str(mx-nx*off),str(my-ny*off),'0.02',str(mx-h),str(my-h),str(mx+h),str(my+h)]
    out=subprocess.run(args,capture_output=True,text=True).stdout.strip().split('\n')[-1]
    print(b, out)
