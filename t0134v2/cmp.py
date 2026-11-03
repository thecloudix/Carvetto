import sys, collections
def load(p):
    d={}
    for l in open(p):
        k,s,h,b,m=(l.rstrip('\n').split('\t')+['']*5)[:5]
        d[k]=(s,h,b.strip(),m)
    return d
br,mn=load(sys.argv[1]),load(sys.argv[2])
cats=collections.defaultdict(list)
for k,(s,h,b,m) in br.items():
    ms,mh,mb,mm=mn[k]
    if s=='OK':
        if b: cats['BRANCH_OK_WITH_BAD_WORD'].append((k,b))
        if ms!='OK': cats['OK_BUT_MAIN_ERR'].append((k,mm))
        elif mh!=h: cats['OK_DIFFERS_FROM_MAIN'].append((k,''))
        else: cats['ok_identical'].append(k)
    else:
        if ms=='ERR': cats['err_both'].append((k,m))
        elif mb: cats['refused_main_had_bad'].append((k,mb,m))
        else: cats['OVER_REFUSAL'].append((k,m))
for c,v in sorted(cats.items()):
    print(c,len(v))
    if c not in('ok_identical',):
        for x in v[:int(sys.argv[3]) if len(sys.argv)>3 else 6]: print('   ',x)
