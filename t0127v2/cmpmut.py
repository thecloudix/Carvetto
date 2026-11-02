import glob,re,sys
def rd(files):
    d={}
    for f in files:
        for l in open(f):
            if not l.startswith('S ') or 'empty' in l or 'nocut' in l: continue
            t=l.split(); g=lambda k:int(t[t.index(k)+1])
            d[(t[1],t[2])]=(g('G'),g('hold'),g('sev'),g('part'),g('nohold'),g('thin'),'FREE' in l,'UNREP' in l,l)
    return d
m=rd(glob.glob('mut/*.log')); b=rd(glob.glob('sw/wsf7_29*.log'))
import collections; c=collections.Counter()
for k in sorted(m):
    if k not in b: continue
    mg,mh,ms,mp,mn,mt,mf,mu,ml=m[k]; bg,bh,bs,bp,bn,bt,bf,bu,bl=b[k]
    side=int(ml.split()[4]); key='On' if side==2 else 'IO'
    c[key+' scenes']+=1; c[key+' hold_mut']+=mh; c[key+' hold_br']+=bh; c[key+' bad_mut']+=ms+mp; c[key+' bad_br']+=bs+bp
    if mh>bh: c[key+' scenes mut keeps more holding']+=1
    if bf and not mf: c[key+' FREE in branch, not in mutant']+=1; print('FREEFIX',key,k)
    if ms+mp>bs+bp: print('MUTBAD',k,ml[ml.index('|'):][:200])
print(dict(c))
