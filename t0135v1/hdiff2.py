import glob,subprocess,os,sys
def rd(pat):
    d={}
    for f in glob.glob(pat):
        for l in open(f):
            t=l.split()
            if not l.startswith('H ') or len(t)<24: continue
            d[(t[1],t[2])]=t
    return d
n=rd('hs2/wsh3_6*.txt'); o=rd('hs2/wsh3_base_6*.txt')
os.makedirs('hd2',exist_ok=True)
out=[]
for k in sorted(set(n)&set(o)):
    a=n[k]; b=o[k]
    side=int(a[4]); al=float(a[8]); tabs=int(a[12]); sep=int(a[14])
    if side==2: continue
    if not (al>=0): continue
    nt=(a[20:23]!=b[20:23]); mv=a[16]!=b[16]; dg=a[17]!=b[17]
    if not (nt or mv or dg): continue
    f='hd2/m3_%s_%s.scn'%k
    if not os.path.exists(f):
        subprocess.run('FORCEIOPOS=1 HDUMP=hd2 SC=%s HASH=1 ./bin/wsh3 %s %d 3 > /dev/null'%(k[1],k[0],int(k[1])+1),shell=True)
    out.append((k,sep,tabs,al,'NOTAB' if nt else '','MOVES' if mv else '','DIAG' if dg else ''))
for x in out: print(*x)
