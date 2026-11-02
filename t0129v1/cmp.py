import glob,collections,sys
def rd(pat):
    d={}
    for f in glob.glob(pat):
        for l in open(f):
            t=l.split()
            if not l.startswith('H ') or len(t)<24: continue
            d[(t[1],t[2])]=(int(t[4]),int(t[6]),float.fromhex(t[8]),float.fromhex(t[10]),int(t[12]),int(t[14]),t[16:19],t[21:24],t[8])
    return d
n=rd(sys.argv[1]+'/new_*.txt'); o=rd(sys.argv[1]+'/old_*.txt')
c=collections.Counter(); ex=collections.defaultdict(list)
for k in sorted(set(n)&set(o)):
    side,V,a,r,tabs,sep,h1,h0,araw=n[k]; g=o[k]
    if a<-r: cls='a<-r'
    elif a==-r: cls='a=-r'
    elif a-(-r)<=1e-9: cls='-r<a<=-r+1e-9'
    elif a<0: cls='-r+1e-9<a<0'
    elif a==0: cls='a=0'
    else: cls='a>0'
    sname=['Out','In','On'][side]
    for lab,hn,ho in (('tabs',h1,g[6]),('notab',h0,g[7])):
        if lab=='tabs' and not tabs: continue
        key=(cls,sname,lab)
        c[key+('n',)]+=1
        if cls=='a<-r':
            if hn[2]!='0': c[key+('NOT REFUSED',)]+=1; ex[key].append(k)
            if ho[2]!='0': c[key+('old had moves',)]+=1
        else:
            if hn[0]!=ho[0] or hn[2]!=ho[2]: c[key+('MOVES DIFF',)]+=1; ex[key].append(k)
            if hn[1]!=ho[1]: c[key+('DIAG DIFF',)]+=1; ex[key].append(k)
for k in sorted(c): print(*k, c[k])
for k in sorted(ex): print('EX',k,ex[k][:8])
print('total',len(set(n)&set(o)), 'missing', len(set(n)^set(o)))
