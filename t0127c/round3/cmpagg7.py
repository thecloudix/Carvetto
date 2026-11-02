# cmpagg7.py: compare nontab3 hashes new vs old per scene; group by tabs/side/allowance sign
import glob,os,collections
c=collections.Counter(); diffs=[]
for f in sorted(glob.glob('cmp/new_*.txt')):
    o=f.replace('/new_','/old_')
    if not os.path.exists(o): continue
    tag=os.path.basename(f)[4:-4]; tabs=tag.endswith('t1')
    N=[l for l in open(f) if l.startswith('scene')]; O=[l for l in open(o) if l.startswith('scene')]
    if len(N)!=len(O): print('LEN MISMATCH',tag); continue
    other=[l for l in open(f) if not l.startswith('scene')]
    if other: print('STRAY',tag,other[:2])
    for a,b in zip(N,O):
        t=a.split(); side=t[5]; allow=float(t[9])
        if side=='2': key='on'
        elif allow<0: key='io_neg'
        else: key='io_nonneg'
        key=('tabs_' if tabs else 'notabs_')+key
        c[key+'_n']+=1
        if a!=b:
            c[key+'_diff']+=1
            diffs.append((key,tag,a.strip(),b.strip()))
for k in sorted(c): print(k,c[k])
minusR0={-1.0,-1.5875,-3.0,-4.0,-6.0}
for d in diffs:
    if d[0] in ('tabs_on','tabs_io_neg'): continue
    if d[0]=='notabs_io_neg' and float(d[2].split()[9]) in minusR0 and d[2].split()[5]=='1': continue  # Inside at a = -r (T-0127 fix)
    print('UNEXPECTED DIFF',*d)
print('io_neg diffs (tabs):',sum(1 for d in diffs if d[0]=='tabs_io_neg'))
minusR={-1.0,-1.5875,-3.0,-4.0,-6.0}
for key in ('notabs_io_neg','tabs_io_neg'):
    sub=[d for d in diffs if d[0]==key]
    exact=[d for d in sub if float(d[2].split()[9]) in minusR]
    inside_exact=[d for d in exact if d[2].split()[5]=='1']
    print(key,'diffs',len(sub),'of which allowance == -r exactly',len(exact),'(Inside',len(inside_exact),')')
    for d in sub:
        pass

