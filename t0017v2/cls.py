import re,sys
for fn in sys.argv[1:]:
    cls={}
    for L in open(fn):
        if not L.startswith('GAPDV'): continue
        m=re.match(r'GAPDV (\S+?)(\(noisy\))? st=(\d) seed=(\S+) gen=(\d+) (\S+) inGap (\S+) \| br gaps (\d+) dv (\d+) \((\S+)\) rv (\d+) \((\S+)\) \| main gaps (\d+) dv (\d+) \((\S+)\) rv (\d+) \((\S+)\) \| (.*)',L)
        if not m: print("??",L[:120]); continue
        nm,noisy,st,seed,gen,join,ig,bg,bdv,bu,brv,bo,mg,mdv,mu,mrv,mo,iss=m.groups()
        bdv,brv,mdv,mrv,bg,mg=map(int,(bdv,brv,mdv,mrv,bg,mg)); bu,bo,mu,mo,ig=map(float,(bu,bo,mu,mo,ig))
        if bdv==0 and brv==0: k='gapsOnly'
        else:
            worse = (bdv>0 and bu>mu+1e-12) or (brv>mrv) or (brv>0 and bo>mo+1e-12)
            k=('ST' if st=='1' else ('inGap' if ig>0 else ('noisyNoGap' if noisy else 'EXACT')))+('-WORSE' if worse else '-sameOrBetter')
        cls.setdefault(k,[]).append((seed,nm,join,bdv,bu,brv,bo,mdv,mu,mrv,mo,ig,bg,mg))
    print(fn)
    for k,v in sorted(cls.items()):
        print(f"  {k}: {len(v)} cases, br dv {sum(x[3] for x in v)} rv {sum(x[5] for x in v)}")
        if 'WORSE' in k or 'EXACT' in k or 'noisyNoGap' in k or 'inGap' in k or k=='gapsOnly':
            for x in v: print("     ",x[0],x[1],x[2],f"br dv {x[3]} under {x[4]:.3g} rv {x[5]} over {x[6]:.3g} | main dv {x[7]} under {x[8]:.3g} rv {x[9]} over {x[10]:.3g} | inGap {x[11]:.3g} gaps b{x[12]} m{x[13]}")
