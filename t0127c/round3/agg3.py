import re,glob,collections
def parse(f):
    out={}
    for l in open(f):
        if not l.startswith('S ') or ' side ' not in l: continue
        t=l.split(); key=(t[1],t[2])
        m=re.search(r'\| G (\d+) hold (\d+) nohold (\d+) sev (\d+) part (\d+) thin (\d+) \| left (\d+) req (\d+)',l)
        d=dict(zip(['G','hold','nohold','sev','part','thin','left','req'],map(int,m.groups())))
        d['side']=int(t[4]); d['free']='[FREE' in l; d['unrep']='[UNREP]' in l; d['falserep']='[FALSEREP]' in l; d['line']=l.strip()
        out[key]=d
    return out
for mode in '012':
    D={}
    for b in ('wsf7','wsf7_old','wsf7_mut'):
        D[b]={}
        for f in glob.glob('sw/%s_30*_%s.log'%(b,mode)): D[b].update(parse(f))
    for b,dd in D.items():
        if not dd: continue
        c=collections.Counter()
        for d in dd.values():
            for k in ['G','hold','nohold','sev','part','thin']: c[k]+=d[k]
            c['scenes']+=1; c['free']+=d['free']; c['unrep']+=d['unrep']; c['falserep']+=d['falserep']
        print('mode',mode,b,dict(c))
    for other in ('wsf7_old','wsf7_mut'):
        keys=sorted(set(D['wsf7'])&set(D[other]))
        if not keys: continue
        c=collections.Counter()
        for k in keys:
            n,o=D['wsf7'][k],D[other][k]
            c['pairs']+=1; c['G_new']+=n['G']; c['G_'+other]+=o['G']; c['hold_new']+=n['hold']; c['hold_'+other]+=o['hold']
            c['more']+=n['G']>o['G']; c['fewer']+=n['G']<o['G']; c['free_new']+=n['free']; c['free_'+other]+=o['free']
        print('   paired vs',other,dict(c))
