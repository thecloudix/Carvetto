import sys,glob,collections,math
def rd(pat):
    d={}
    for f in glob.glob(pat):
        for l in open(f):
            if not l.startswith('H ') or len(l.split())<24: continue
            t=l.split()
            k=(t[1],t[2]); side=int(t[4]); a=float(t[8]); r=float(t[10]); tabs=int(t[12]); sep=int(t[14])
            neg0 = t[8].startswith('-')
            d[k]=(side,a,r,tabs,sep,neg0,t[16],t[17],t[18],t[21],t[22],t[23])
    return d
n=rd('hs4/new_*.txt'); o=rd('hs4/old_*.txt')
c=collections.Counter(); ex=collections.defaultdict(list)
for k in sorted(set(n)&set(o)):
    side,a,r,tabs,sep,neg0,m1,d1,n1,m0,d0,n0=n[k]; g=o[k]
    if side==2: cls='On'
    elif a>0: cls='IO a>0'
    elif a==0: cls='IO a=-0' if neg0 else 'IO a=0'
    elif a==-r: cls='IO a=-r'
    elif a<-r: cls='IO a<-r'
    else: cls='IO -r<a<0'
    c[('notab',cls,'n')]+=1
    if m0!=g[9]: c[('notab',cls,'MOVES DIFF')]+=1; ex[('notab',cls)].append(k)
    if d0!=g[10]: c[('notab',cls,'DIAG DIFF')]+=1
    if tabs:
        c[('tabs',cls,'n')]+=1
        if m1!=g[6]: c[('tabs',cls,'MOVES DIFF')]+=1; ex[('tabs',cls)].append(k)
        if d1!=g[7]: c[('tabs',cls,'DIAG DIFF')]+=1
for k in sorted(c): print(*k, c[k])
for k in sorted(ex):
    print('EX',k,ex[k][:8])
