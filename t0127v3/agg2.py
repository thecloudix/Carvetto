import sys,glob,re,collections
binname=sys.argv[1]
c=collections.Counter(); lists=collections.defaultdict(list)
for f in sorted(f for f in glob.glob('sw/%s_*.log'%binname) if (binname.endswith('_old') or '_old_' not in f)):
    mode=f.split('_')[-1].split('.')[0]
    for l in open(f):
        if not l.startswith('S '): continue
        t=l.split()
        if 'empty' in t or 'nocut' in t: c['empty/nocut']+=1; continue
        seed,sc=t[1],t[2]; side=int(t[4]); V=int(t[8]); a=float(t[10]); sep=int(t[12])
        g=lambda k: int(t[t.index(k)+1])
        c['scenes']+=1; c['scenes side%d'%side]+=1
        if V: c['vbit scenes']+=1
        if sep: c['sep scenes']+=1
        if side==2: c['On a<0' if a<0 else 'On a=0' if a==0 else 'On a>0']+=1
        else: c['IO a=-r..0']+=1
        c['tabs']+=g('G'); c['hold']+=g('hold'); c['req']+=g('req'); c['left']+=g('left')
        key=(seed,sc,mode)
        for fl in ['SEVERED','PARTIAL','NOHOLD','THIN','UNREP','FALSEREP','FREE','REP']:
            if '['+fl+' ' in l or '['+fl+']' in l:
                c['flag '+fl]+=1; lists[fl].append(key)
print(binname)
for k in sorted(c): print(' ',k,c[k])
for fl in ['SEVERED','PARTIAL','NOHOLD','THIN','UNREP','FALSEREP']:
    if lists[fl]: print(fl, ' '.join('m%s_%s_%s'%(m,s,x) for s,x,m in lists[fl]))
