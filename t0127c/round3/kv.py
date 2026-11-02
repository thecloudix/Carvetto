import collections
def rd(f):
    d={}
    for l in open(f):
        t=l.split()
        if not t or t[0]!='C': continue
        d[(t[1],t[2])]=(int(t[12]), int(t[14]), int(t[16]), int(t[4]), l.strip())
    return d
n=rd('kv_new.txt'); o=rd('kv_old.txt')
c=collections.Counter()
for k in sorted(n):
    gn,ln,rn,side,l=n[k]; go,lo,ro,_,_=o[k]
    key='On' if side==2 else 'IO'
    c[key+' scenes']+=1; c[key+' kept_new']+=gn; c[key+' kept_old']+=go
    if gn>go: c[key+' new>old']+=1
    if gn<go: c[key+' new<old']+=1; print('LESS',k[0],k[1],'new',gn,'old',go, ' '.join(l.split()[3:]))
print(dict(c))
