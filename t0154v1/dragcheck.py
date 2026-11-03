# Independent drag-rule checker (verifier T-0154).
import re, sys
def words(code):
    c = re.sub(r"\s+", "", code).upper()
    return re.findall(r"([A-Z])([-+]?\d*\.?\d+)", c)
def check(path):
    inch = 'inch' in path; k = 25.4 if inch else 1.0; safe = 5.0
    errs = []; lines = open(path, newline='').read().split('\n')
    spindle = False; S = None; z = None; m5_since_cut = False
    drag = False; announced_recent = []; x = y = None; after_pause_needs_z = False
    cuts_off = cuts_on = 0; pauses = 0
    for n, raw in enumerate(lines, 1):
        if any(ord(ch) < 32 or ord(ch) > 126 for ch in raw.rstrip('\r')): errs.append(f"{n}: non-ascii")
        marlin = 'marlin' in path
        code = raw.split(';')[0]
        if not marlin: code = re.sub(r"\([^)]*\)", "", code)
        comment = raw[len(code):] if marlin else ' '.join(re.findall(r"\(([^)]*)\)", raw))
        announced_recent = (announced_recent + [raw.lower()])[-3:]
        if marlin and code.strip().upper().startswith('M0 '):
            msg = code.strip()[3:]; code = 'M0'; announced_recent[-1] += ' ' + msg.lower()
        ws = words(code)
        ms = [float(v) for l, v in ws if l == 'M']; gs = [float(v) for l, v in ws if l == 'G']
        d = {l: float(v) for l, v in ws if l in 'XYZS'}
        if drag and (any(m in (3, 4) for m in ms) or 'S' in d): errs.append(f"{n}: spindle word in drag section: {raw}")
        if 'S' in d: S = d['S']
        for m in ms:
            if m in (3, 4):
                if not (S and S > 0): errs.append(f"{n}: M3 without S>0")
                spindle = True; m5_since_cut = False
            elif m == 5: spindle = False; m5_since_cut = True
            elif m in (0, 6):
                if spindle: errs.append(f"{n}: stop/toolchange with spindle on")
                if (z is not None or cuts_on + cuts_off > 0) and (z is None or z < safe - 1e-6): errs.append(f"{n}: M{int(m)} not at safe Z (z={z})")
                win = ' '.join(announced_recent)
                if m == 0 and 'spindle' in win and 'stays off' in win or (m == 0 and marlin and 'spindle off' in win):
                    if not m5_since_cut: errs.append(f"{n}: spindle-off pause without M5 since last cut/start")
                    drag = True; pauses += 1; after_pause_needs_z = True
                else:
                    drag = False
        if 'Z' in d: z = d['Z'] * k
        moving = any(a in d for a in 'XYZ')
        if moving and gs and gs[0] == 0 and after_pause_needs_z:
            if not ('Z' in d and 'X' not in d and 'Y' not in d and z >= safe - 1e-6) and ('X' in d or 'Y' in d) and (z is None or z < safe-1e-6):
                errs.append(f"{n}: XY rapid after pause below safe Z")
        if moving and gs and gs[0] in (1, 2, 3):
            m5_since_cut = False
            if z is not None and z < 0:
                if spindle: 
                    cuts_on += 1
                    if abs(z + 0.1) < 1e-3: errs.append(f"{n}: drag-depth cut with spindle on")
                else:
                    if not drag: errs.append(f"{n}: cut with spindle off outside a drag section")
                    if abs(z + 3.0) < 1e-3: errs.append(f"{n}: mill-depth cut with spindle off")
                    cuts_off += 1
    return errs, cuts_on, cuts_off, pauses
bad = 0
for p in sys.argv[1:]:
    e, on, off, pz = check(p)
    print(f"{p.split('/')[-1]}: on={on} off={off} pauses={pz} {'OK' if not e else 'ERRORS'}")
    for x in e: print("   ", x)
    bad += bool(e)
sys.exit(1 if bad else 0)
