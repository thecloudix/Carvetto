import sys,collections
def rd(f):
    d={}
    for l in open(f):
        if not l.startswith('H '): continue
        t=l.split()
        k=(t[1],t[2]); side=int(t[4]); V=int(t[6]); a=float(t[8]); r=float(t[10]); tabs=int(t[12]); sep=int(t[14])
        h1=t[16]; h0=t[20]
        d[k]=(side,V,a,r,tabs,sep,h1,h0,l.strip())
    return d
n=rd(sys.argv[1]); o=rd(sys.argv[2])
c=collections.Counter(); ex=collections.defaultdict(list)
for k in sorted(set(n)&set(o)):
    side,V,a,r,tabs,sep,h1,h0,l=n[k]; _,_,_,_,_,_,g1,g0,_=o[k]
    io = side in (0,1)
    if a > 0 or (a == 0): acls='a>=0'
    elif a == -r: acls='a=-r'
    else: acls='-r<a<0'
    if a < -r: acls='a<-r'
    sname={0:'Out',1:'In',2:'On'}[side]
    # tab-free comparison
    key='notab %s %s'%(sname if not io else 'IO-'+sname, acls)
    c[key+' n']+=1
    if h0!=g0: c[key+' DIFF']+=1; ex[key].append(k)
    if tabs:
        key='tabs %s %s'%(sname, acls)
        c[key+' n']+=1
        if h1!=g1: c[key+' DIFF']+=1; ex[key].append(k)
for k in sorted(c): print(k, c[k])
for k in sorted(ex):
    if ('a>=0' in k and 'On' not in k) or 'notab' in k: print('EX', k, ex[k][:10])
