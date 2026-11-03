# fcmp.py VARIANT_A VARIANT_B : compare holding tabs per scene between fc outputs (by tag); print scenes where A < B
import glob,os,re,sys,collections
def load():
    d=collections.defaultdict(dict)
    for f in glob.glob(os.environ.get('FCDIR','fc')+'/*.txt'):
        b=os.path.basename(f)[:-4]
        m=re.match(r'(wsf8y(?:_base|_main|_mut)?)_(\d+)_(\d+)_(\d+)_(.*)$',b)
        if not m: continue
        binn,s,c,mo,tag=m.groups()
        env=tag
        for k in ['_NOADD__1','_NOLINK__1','_OLDORDER__1']:
            if env.endswith(k.replace('__','_')): pass
        var=binn
        for k in ['NOADD','NOLINK','OLDORDER']:
            if k in tag: var+='+'+k; tag=tag.replace('_'+k+'_1','')
        hold=0; flags=''; S=None
        for l in open(f):
            if l.startswith('T '):
                t=l.split()
                if t[6]=='1' and t[7]=='1': hold+=1
            elif l.startswith('S '): S=l.strip()
        if S is None: continue
        d[(s,c,mo,tag)][var]=(hold,S)
    return d
d=load()
A,B=sys.argv[1],sys.argv[2]
n=0; fewer=[]; more=0; bad=[]
for k,v in sorted(d.items()):
    if A in v:
        S=v[A][1]
        if any(x in S for x in ['SEVERED','PARTIAL','THIN']): bad.append((k,re.findall(r'\[(SEVERED|PARTIAL|THIN)[^\]]*\]',S)))
    if A not in v or B not in v: continue
    n+=1
    if v[A][0]<v[B][0]: fewer.append((k,v[A][0],v[B][0]))
    elif v[A][0]>v[B][0]: more+=1
print(A,'vs',B,'scenes',n,'A fewer holding:',len(fewer),'A more:',more)
for f in fewer: print('  FEWER',f)
print('scenes where',A,'has severed/partial/thin:',len(bad))
for b in bad[:30]: print('  BAD',b)
