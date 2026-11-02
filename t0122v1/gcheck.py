import re, sys
def check(path):
    x=y=z=None; spindle=False; units=None; errs=[]; mode=None; toolchanges=0; rapids=0
    for n,line in enumerate(open(path),1):
        s=re.sub(r"\(.*?\)","",line).strip()
        if not s: continue
        words=dict()
        codes=re.findall(r"([A-Z])(-?\d+(?:\.\d+)?)",s)
        gs=[float(v) for k,v in codes if k=='G']; ms=[int(float(v)) for k,v in codes if k=='M']
        for k,v in codes:
            if k in 'XYZIJFSP': words[k]=float(v)
        if 21 in gs: units='mm'
        if 20 in gs: units='in'
        for m in ms:
            if m==3: spindle=True
            if m==5: spindle=False
            if m in (0,6):
                toolchanges+=1
                if spindle: errs.append(f"{n}: tool change with spindle on")
                if z is None or z<=0: errs.append(f"{n}: tool change not at safe Z (z={z})")
        for g in gs:
            if g in (0,1,2,3): mode=int(g)
        if any(k in words for k in 'XYZ'):
            if units!='mm': errs.append(f"{n}: motion before G21")
            nx=words.get('X',x); ny=words.get('Y',y); nz=words.get('Z',z)
            if mode==0:
                rapids+=1
                xy=(nx!=x or ny!=y)
                if xy and (z is None or z<=0 or nz is None or nz<=0): errs.append(f"{n}: XY rapid below/at surface z={z}->{nz}")
                if nz is not None and nz<=0: errs.append(f"{n}: rapid into material z={nz}")
            else:
                if nz is not None and nz<0 and not spindle: errs.append(f"{n}: cutting below surface with spindle off")
                if 'F' not in words and mode in (1,2,3) and n<10: pass
            x,y,z=nx,ny,nz
    if spindle: errs.append("program ends with spindle on")
    print(f"{path}: units={units} toolchanges={toolchanges} rapids={rapids} final z={z} errors={len(errs)}")
    for e in errs[:15]: print("  ",e)
    return len(errs)
sys.exit(sum(check(p) for p in sys.argv[1:]))
