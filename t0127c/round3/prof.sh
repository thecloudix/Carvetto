#!/bin/bash
# prof.sh out.data cmd... : perf record + symbol aggregation (static symbols via nm)
o=$1; shift; bin=$1
perf record -o $o "$@" > /dev/null 2>&1
nm -C $bin | sort > $o.syms
python3 - $o <<'PY'
import subprocess,bisect,re,collections,sys
o=sys.argv[1]; syms=[]
for l in open(o+'.syms'):
    t=l.split(None,2)
    if len(t)<3: continue
    try: syms.append((int(t[0],16),t[2].strip()))
    except: pass
syms.sort(); addrs=[a for a,_ in syms]
out=subprocess.run(['perf','report','-i',o,'--no-children','--stdio','--sort','sym'],capture_output=True,text=True).stdout
agg=collections.Counter()
for l in out.splitlines():
    m=re.match(r'\s+([0-9.]+)%.*\[\.\] (0x[0-9a-f]+|\S.*)',l)
    if m:
        x=m.group(2)
        if x.startswith('0x'):
            a=int(x,16); i=bisect.bisect_right(addrs,a)-1; name=syms[i][1][:90] if i>=0 else '?'
        else: name=x[:90]
        agg[name]+=float(m.group(1))
for k,v in agg.most_common(10): print('%.1f%% %s'%(v,k))
PY
