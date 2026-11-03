import glob,collections,sys,os
def rd(pat):
    d={}
    for f in glob.glob(pat):
        tag=os.path.basename(f).split('_',4)[-1] if False else f
        for l in open(f):
            if not l.startswith('H '): continue
            t=l.split()
            k=(t[1],t[2],f.split('/')[-1].split('_',2)[-1] if not f.split('/')[-1].startswith('hh_') else '')
            d[(t[1],t[2])]=dict(side=int(t[4]),V=int(t[6]),a=float(t[8]),neg0=t[8].startswith('-'),r=float(t[10]),tabs=int(t[12]),sep=int(t[14]),m1=t[16],d1=t[17],m0=t[21],d0=t[22])
    return d
def tagof(f): 
    b=f.split('/')[-1]
    return b
res={}
for other in ['hh_base','hh_main']:
    c=collections.Counter(); ex=collections.defaultdict(list)
    for fn in glob.glob('hh/hh_[0-9]*.txt'):
        suffix=fn.split('/hh_',1)[1]
        of='hh/'+other+'_'+suffix
        if not os.path.exists(of): continue
        n=rd(fn); o=rd(of); lead='LEADS' in suffix
        for k in n:
            if k not in o: continue
            a=n[k]; b=o[k]
            if a['side']==2: cls='On'
            elif a['a']>0: cls='IO a>0'
            elif a['a']==0: cls='IO a=-0' if a['neg0'] else 'IO a=0'
            elif a['a']==-a['r']: cls='IO a=-r'
            else: cls='IO -r<a<0'
            cls+=' sep' if a['sep'] else ''
            cls+=' leads' if lead else ''
            c[('notab',cls,'n')]+=1
            if a['m0']!=b['m0'] or a['d0']!=b['d0']: c[('notab',cls,'DIFF')]+=1; ex[('notab',cls)].append(k+(suffix,))
            if a['tabs']:
                c[('tabs',cls,'n')]+=1
                if a['m1']!=b['m1']: c[('tabs',cls,'MOVES DIFF')]+=1; ex[('tabs',cls)].append(k+(suffix,))
                elif a['d1']!=b['d1']: c[('tabs',cls,'DIAG DIFF')]+=1
    print('=== vs',other)
    for k in sorted(c): print(' ',*k,c[k])
    for k in sorted(ex):
        if k[1].startswith('IO a') and not k[1].startswith('IO a=-r'): print('  EX',k,ex[k][:15])
    if other=='hh_base':
        with open('hdiff_base.txt','w') as f:
            for k in ex:
                for e in ex[k]: f.write('%s %s %s %s %s\n'%(e[0],e[1],e[2],k[0],k[1].replace(' ','_')))
