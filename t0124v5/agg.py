import glob,re,collections,sys
def parse(f):
    on={}; tabs=[]; summ=None; sev=[]
    for l in open(f):
        if l.startswith('ONSTAT'):
            t=l.split(); d=dict(zip(t[1::2],t[2::2])); on[int(d['sc'])]=d
        elif l.startswith('ONTAB'):
            t=l.split(); d=dict(zip(t[1::2],t[2::2])); tabs.append(d)
        elif l.startswith('SUMMARY'): summ=l
        elif l.startswith('scene') and 'SEVERED' in l: sev.append(l.strip())
    return on,tabs,summ,sev
tot=collections.Counter()
for f in sorted(glob.glob('sw/*_conn6s.log')):
    tag=f.split('/')[1].replace('_conn6s.log','')
    on,tabs,summ,sev=parse(f)
    m=re.search(r'scenes (\d+).*tabs kept (\d+) checked (\d+) severed (\d+) thin\(<0.1\) (\d+).*countBad (\d+)',summ)
    sc,k,ch,sv,th,cb=map(int,m.groups())
    tot['scenes']+=sc; tot['kept_all']+=k; tot['sev_all']+=sv; tot['thin_all']+=th; tot['countBad_all']+=cb
    tot['on_scenes']+=len(on)
    for s,d in on.items():
        tot['on_kept']+=int(d['kept']); tot['on_left']+=int(d['left']); tot['on_req']+=int(d['req'])
    for d in tabs:
        tot['on_tabs_checked']+=1
        if d['joined']=='0':
            tot['on_severed']+=1; print('ON SEVERED',tag,d)
        if float(d['neck'])<0.1 and d['joined']=='1': tot['on_thin']+=1; print('ON THIN', tag, d)
        if float(d['len'])>=2*float(d['D'])+... if False else False: pass
    for s in sev:
        print('SEV',tag,s[:300])
    import os
    fo=f.replace('_conn6s.log','_conn6s_old.log')
    if os.path.exists(fo):
        on0,tabs0,_,_=parse(fo)
        for s,d in on.items():
            d0=on0.get(s)
            if not d0: continue
            tot['cmp_on_scenes']+=1
            tot['cmp_on_req']+=int(d['req'])
            tot['cmp_kept_new']+=int(d['kept']); tot['cmp_kept_old']+=int(d0['kept'])
            if int(d['kept'])<int(d0['kept']): tot['cmp_scenes_fewer']+=1
            if int(d['kept'])>int(d0['kept']): tot['cmp_scenes_more']+=1; print('MORE',tag,s,d['kept'],d0['kept'])
        tot['old_on_sev']+=sum(1 for d in tabs0 if d['joined']=='0')
        tot['old_on_tabs']+=len(tabs0)
for k,v in tot.items(): print(k,v)
