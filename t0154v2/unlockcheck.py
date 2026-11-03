# Verifier T-0154 r2: independent unlock-pause checker. Per file: pauses with "stays off"/"spindle off"
# (off pauses), "unlock" pauses; each drag section (off cuts) must be followed by exactly one unlock
# pause before the next M3/M4 or program end; no unlock pause while no drag cut happened since last.
import re, sys
exp = {'s1_first':1,'s2_same':1,'s3_mill_to_drag':1,'s4_drag_to_mill':1,'s5_drag_to_drag_other':1,
       's6_same_numname_spin_to_off':1,'s7_same_numname_off_to_spin':1,'s8_mixed':2,'drag':None}
bad=0
for p in sys.argv[1:]:
    lines=open(p,newline='').read().split('\n'); marlin='marlin' in p
    spindle=False; dragcut=False; unlocks=0; offp=0; errs=[]; recent=[]
    for n,raw in enumerate(lines,1):
        recent=(recent+[raw.lower()])[-3:]
        code=raw.split(';')[0]
        if not marlin: code=re.sub(r"\([^)]*\)","",code)
        msg=''
        if marlin and re.match(r"\s*M0\b",code,re.I): msg=code[3:].lower(); code='M0'
        ws=re.findall(r"([A-Z])([-+]?\d*\.?\d+)",re.sub(r"\s+","",code).upper())
        for l,v in ws:
            if l=='M':
                m=float(v)
                if m in (3,4):
                    if dragcut: errs.append(f"{n}: spindle start after drag cuts without unlock")
                    spindle=True
                if m==5: spindle=False
                if m==0:
                    win=' '.join(recent)+' '+msg
                    if 'unlock' in win:
                        unlocks+=1
                        if not dragcut: errs.append(f"{n}: unlock pause without preceding drag cuts")
                        dragcut=False
                    elif 'stays off' in win or 'spindle off' in win: offp+=1
            if l=='G' and float(v) in (1,2,3) and not spindle: dragcut=True
        if re.search(r"\bM30\b|\bM2\b",code.upper()) and dragcut: errs.append(f"{n}: program end after drag cuts without unlock")
    if dragcut: errs.append("EOF after drag cuts without unlock")
    key=p.split('__')[-1].replace('.nc','') if '__' in p else 'drag'
    if exp.get(key) is not None and unlocks!=exp[key]: errs.append(f"unlocks={unlocks} expected {exp[key]}")
    print(f"{p.split('/')[-1]}: offpauses={offp} unlocks={unlocks} {'OK' if not errs else 'ERR'}")
    for e in errs: print('   ',e)
    bad+=bool(errs)
sys.exit(1 if bad else 0)
