import sys,re
# usage: chk.py files... ; checks every scene line for count/area matching of FREEL vs REPL, and tab verdicts
tot=0; bad=[]; stats={'free04':0,'rep':0,'sev':0,'part':0,'thin':0,'nohold':0,'hold':0}
for fn in sys.argv[1:]:
    for l in open(fn):
        m=re.search(r'(?:^|F (\S+) .*?)S (\d+) (\d+) side',l)
        if ' | G ' not in l: continue
        tot+=1
        key=(m.group(1) or '')+' '+m.group(2)+' '+m.group(3)
        g=re.search(r'hold (\d+) nohold (\d+) sev (\d+) part (\d+) thin (\d+)',l)
        for k,v in zip(['hold','nohold','sev','part','thin'],g.groups()): stats[k]+=int(v)
        rep=[float(x) for x in re.search(r'\[REPL([^\]]*)\]',l).group(1).split()] if '[REPL' in l else []
        fr=[float(x.split('@')[0]) for x in re.search(r'\[FREEL([^\]]*)\]',l).group(1).split()] if '[FREEL' in l else []
        stats['free04']+=len(fr); stats['rep']+=len(rep)
        r2=sorted(rep); unm=[]
        for a in sorted(fr,reverse=True):
            best=None
            for i,b in enumerate(r2):
                if abs(a-b)<=0.1*a+0.06 and (best is None or abs(a-b)<abs(a-r2[best])): best=i
            if best is None: unm.append(a)
            else: r2.pop(best)
        flags=[]
        if unm: flags.append('UNREPORTED %s'%unm)
        if r2: flags.append('EXTRA_REPORTS %s'%r2)
        if int(g.group(3)) or int(g.group(4)) or int(g.group(5)): flags.append('TABFAIL sev %s part %s thin %s'%(g.group(3),g.group(4),g.group(5)))
        if '[UNREP]' in l or 'FALSEREP' in l: flags.append('ORACLEFLAG')
        if flags: bad.append((key,flags,l.strip()[:0]))
print('scenes',tot,stats)
for k,f,_ in bad: print('BAD',k,'; '.join(f))
