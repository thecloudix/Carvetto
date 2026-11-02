import sys,re,math
rows={}
for l in open(sys.argv[1]):
    t=l.split(None,2)
    if len(t)<3: continue
    k,b,rest=t
    tabs=[tuple(map(float,m.groups())) for m in re.finditer(r'T 0 0 (\S+) (\S+) (\S+) (\S+) (\S+)',rest)]
    fr=re.search(r'\[FREEL([^\]]*)\]',rest); fr=[float(x.split('@')[0]) for x in fr.group(1).split()] if fr else []
    G=re.search(r'\| G (\d+) hold (\d+) nohold (\d+) sev (\d+) part (\d+) thin (\d+)',rest)
    rows.setdefault(k,{})[b]=(tabs,fr,G.groups() if G else None, 'CNT' in rest)
for k,v in rows.items():
    n=v.get('wsf8x'); o=v.get('wsf8x_base')
    if not n or not o: print(k,'MISSING'); continue
    issues=[]
    for x,y,L,v0,v5 in n[0]:
        if v0!=1 or v5!=1: issues.append('branch tab (%.2f,%.2f) v0=%d v5=%d'%(x,y,v0,v5))
    for x,y,L,v0,v5 in o[0]:
        if v0==1 and v5==1 and not any(math.hypot(x-a,y-b)<1.0 for a,b,_,_,_ in n[0]): issues.append('base holding tab (%.2f,%.2f) missing in branch'%(x,y))
    if len(n[1])>len(o[1]) or sum(n[1])>sum(o[1])+0.5: issues.append('more free: branch %s base %s'%(n[1],o[1]))
    if n[3]: issues.append('branch CNT mismatch')
    print(k,'branch G/hold %s/%s base %s/%s'%(n[2][0],n[2][1],o[2][0],o[2][1]),'free b%s o%s'%(len(n[1]),len(o[1])),'OK' if not issues else 'ISSUES: '+'; '.join(issues))
