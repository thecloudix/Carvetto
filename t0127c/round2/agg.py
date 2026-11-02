import re,glob,sys,collections
def parse(f):
    out={}
    for l in open(f):
        if not l.startswith('S ') or ' side ' not in l: continue
        t=l.split()
        key=(t[1],t[2])
        d={'side':int(t[4]),'V':int(t[8]),'a':float(t[10]),'sep':int(t[12])}
        m=re.search(r'\| G (\d+) hold (\d+) nohold (\d+) sev (\d+) part (\d+) thin (\d+) \| left (\d+) req (\d+)',l)
        d.update(dict(zip(['G','hold','nohold','sev','part','thin','left','req'],map(int,m.groups()))))
        d['free']='[FREE' in l; d['allLost']=bool(re.search(r'allLost [1-9]',l)); d['line']=l.strip()
        fm=re.search(r'FREE (\d+) biggest ([0-9.]+)',l); d['freeArea']=float(fm.group(2)) if fm else 0
        out[key]=d
    return out
for mode in ['0','1','2']:
    new={};old={}
    for f in glob.glob('sw4/wsf4_[0-9]*_%s.log*'%mode): new.update(parse(f))
    for f in glob.glob('sw4/wsf4_old_*_%s.log*'%mode): old.update(parse(f))
    for name,D in (('new',new),('old',old)):
        if not D: continue
        tot=collections.Counter()
        for d in D.values():
            for k in ['G','hold','nohold','sev','part','thin','left','req']: tot[k]+=d[k]
            tot['scenes']+=1; tot['free']+=d['free']; tot['freeNoAllLost']+= d['free'] and not d['allLost']
            tot['On']+= d['side']==2; tot['V']+= d['V']>0
        print('mode',mode,name,dict(tot))
    # paired
    keys=sorted(set(new)&set(old))
    if keys:
        c=collections.Counter()
        for k in keys:
            n,o=new[k],old[k]
            c['pairs']+=1; c['G_new']+=n['G']; c['G_old']+=o['G']; c['left_new']+=n['left']; c['left_old']+=o['left']
            c['more']+= n['G']>o['G']; c['fewer']+= n['G']<o['G']
            c['old_sev_or_part']+= o['sev']+o['part']; c['new_sev_or_part']+= n['sev']+n['part']
            if n['free'] and not n['allLost'] and n['nohold']>0 and o['allLost']: c['noholdFreeRegress']+=1; print('  NOHOLD-FREE', k, 'new:', n['line'][n['line'].find('| G'):][:150], '| old G',o['G'],'left',o['left'])
            if n['G']<o['G']: print('  FEWER', k, 'new G',n['G'],'left',n['left'],'old G',o['G'],'left',o['left'], 'side',n['side'],'a',n['a'],'sep',n['sep'],'V',n['V'])
        print('mode',mode,'paired',dict(c))
