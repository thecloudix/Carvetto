# agg7.py [set prefix...]: aggregate conn7 sweep logs (new = conn7, old = conn7_old)
import glob,re,collections,sys,os
pref=sys.argv[1:] or ['']
def parse(f):
    on={}; tabs=[]; summ=None; sev=[]
    for l in open(f):
        if l.startswith('ONSTAT'):
            t=l.split(); d=dict(zip(t[1::2],t[2::2])); on[int(d['sc'])]=d
        elif l.startswith('ONTAB'):
            t=l.split(); d=dict(zip(t[1::2],t[2::2])); tabs.append(d)
        elif l.startswith('SUMMARY'): summ=l
        elif l.startswith('scene') and ('SEVER' in l or 'GOUGE' in l or 'NONDET' in l or 'count' in l.lower()): sev.append(l.strip())
    return on,tabs,summ,sev
tot={'new':collections.Counter(),'old':collections.Counter()}
lists=collections.defaultdict(list)
for f in sorted(glob.glob('sw/*_conn7.log')):
    tag=os.path.basename(f).replace('_conn7.log','')
    if not any(tag.startswith(p) for p in pref): continue
    for v,fn in (('new',f),('old',f.replace('_conn7.log','_conn7_old.log'))):
        if not os.path.exists(fn): continue
        on,tabs,summ,sev=parse(fn)
        if not summ: continue
        m=re.search(r'scenes (\d+) det (\d+) gouge (\d+) clampBad (\d+) sepOrder (\d+) \| tabs kept (\d+) checked (\d+) severed (\d+) thin\(<0.1\) (\d+).*countBad (\d+)',summ)
        sc,det,gou,cl,so,k,ch,sv,th,cb=map(int,m.groups())
        T=tot[v]; T['files']+=1; T['scenes']+=sc; T['det']+=det; T['gouge']+=gou; T['clampBad']+=cl; T['kept_all']+=k; T['checked_all']+=ch; T['sev_all']+=sv; T['thin_all']+=th; T['countBad']+=cb
        T['on_scenes']+=len(on)
        for s,d in on.items():
            T['on_kept']+=int(d['kept']); T['on_left']+=int(d['left']); T['on_req']+=int(d['req'])
        T['on_tabs_checked']+=len(tabs)
        T['on_severed']+=sum(1 for d in tabs if d['joined']=='0')
        T['on_thin']+=sum(1 for d in tabs if d['joined']=='1' and float(d['neck'])<0.1)
        if v=='new':
            for s in sev: lists['sev'].append((tag,s))
            for d in tabs:
                if d['joined']=='1' and float(d['neck'])<0.1: lists['thin'].append((tag,d))
# per-scene On kept, paired files only
pc=collections.Counter()
for f in sorted(glob.glob('sw/*_conn7.log')):
    tag=os.path.basename(f).replace('_conn7.log','')
    if not any(tag.startswith(p) for p in pref): continue
    fo=f.replace('_conn7.log','_conn7_old.log')
    if not os.path.exists(fo): continue
    on,_,s1,_=parse(f); on0,_,s0,_=parse(fo)
    if not s1 or not s0: continue
    for s,d in on.items():
        d0=on0.get(s)
        if not d0: continue
        pc['scenes']+=1; pc['req']+=int(d['req']); pc['kept_new']+=int(d['kept']); pc['kept_old']+=int(d0['kept'])
        if int(d['kept'])>int(d0['kept']): pc['scenes_more']+=1
        if int(d['kept'])<int(d0['kept']): pc['scenes_fewer']+=1; lists['fewer'].append((tag,f"sc {s} new {d['kept']} old {d0['kept']} req {d['req']} D {d['D']} len {d['len']} sep {d['sep']} allow {d['allow']}"))
print('paired On:',' '.join(f'{k}={c}' for k,c in pc.items()))
for v in ('new','old'):
    print(v,' '.join(f'{k}={c}' for k,c in tot[v].items()))
for k,l in lists.items():
    print(k,len(l))
    for x in l[:400]: print('  ',x[0],str(x[1])[:330])
