import glob,os,collections
c=collections.Counter()
for f in sorted(glob.glob('cmp/new_*.txt')):
    o=f.replace('new_','old_')
    if not os.path.exists(o): continue
    tag=os.path.basename(f)[4:-4]; tabs=tag.endswith('t1')
    N=[l for l in open(f) if l.startswith('scene')]; O=[l for l in open(o) if l.startswith('scene')]
    if len(N)!=len(O): print('LEN MISMATCH',tag); continue
    other=[l for l in open(f) if not l.startswith('scene')]
    if other: print('STRAY',tag,other[:2])
    for a,b in zip(N,O):
        side=a.split()[5]
        key=('tabs' if tabs else 'notabs')+('_on' if side=='2' else '_io')
        c[key+'_n']+=1
        if a!=b:
            c[key+'_diff']+=1
            if key!='tabs_on': print('DIFF',tag,a.strip(),'|',b.strip())
for k in sorted(c): print(k,c[k])
