# T-0151 (coder self-check, adapted from harness/t0150v1/gcheck_drill.py): quick engrave G-code
# checker. Rules: printable ASCII; units + G90 before motion; XY rapids only at safe Z; no rapid
# below safe Z at all; spindle on with S > 0 before any G1/G2/G3; tool change with spindle off at
# safe Z; no cut below the deepest engrave depth; program ends at safe Z with the spindle off.
# T-0154 drag rule (manifest key drag=1 allows spindle-off cuts): a cut with the spindle off is
# allowed only in a drag section, which starts at an operator stop M0 that comes with the
# spindle off, after an M5 that follows the last cut and spindle start, and is announced as a spindle stop ("spindle" on the M0 line or in the
# two lines before it) and ends at the next tool change stop (M0 not announced so, or M6).
# Inside a drag section no M3/M4 and no S word may appear: the spindle restarts only via a tool
# change. After drag cuts the spindle may start again only behind an unlock pause: an M0 announced
# with "unlock" (on its line or the two before), which also ends the drag section; a program
# that ends after drag cuts needs that unlock pause before its end too.
# Without drag=1 every cut needs the spindle on, as before.
import re, sys
def check(path, post, bottom, safe, plunge=-1, drag=False):
    inch = 'inch' in post; k = 25.4 if inch else 1.0
    eps = 0.0026 if inch else 0.0011
    x = y = z = None; spindle = False; S = 0.0; units = None; absolute = False; mode_g = None; errs = []; cuts = 0; F = None
    def E(n, msg): errs.append(f"{n}: {msg}")
    dragsec = False; dragcuts = 0; recent = []; m5 = False; unlockdue = False  # m5: M5 since the last spindle start / cut
    for n, line in enumerate(open(path, newline=''), 1):
        s = line.rstrip('\r\n')
        window = ' '.join(recent[-2:] + [s]).lower(); recent.append(s)
        announced = 'spindle' in window; unlock = 'unlock' in window
        if any(ord(ch) < 32 or ord(ch) > 126 for ch in s): E(n, "non-printable")
        s = re.sub(r"\(.*?\)", "", s); s = s.split(';')[0].strip()
        if not s: continue
        # Read words as a controller does (T-0154): spaces/tabs ignored, any case, signed values
        # and a leading '.' ("M 3", "m03", "S.5", "M+3").
        codes = re.findall(r"([A-Z])([+-]?(?:\d+\.?\d*|\.\d+))", re.sub(r"[ \t]", "", s).upper())
        if codes and codes[0][0] == 'N': codes = codes[1:]
        if any(c[0] == 'M' and c[1] == '0' for c in codes):
            codes = codes[:[i for i, c in enumerate(codes) if c[0] == 'M' and c[1] == '0'][0] + 1]
        codes = [(kk, v.lstrip('+')) for kk, v in codes]
        gs = [v for kk, v in codes if kk == 'G']; ms = [int(float(v)) for kk, v in codes if kk == 'M']
        w = {kk: float(v) for kk, v in codes if kk in 'XYZFSPIJR'}
        if 'S' in w:
            S = w['S']
            if dragsec: E(n, "S word during a spindle-off (drag) toolpath")
        if 'F' in w: F = w['F'] * k
        for g in gs:
            if g == '21': units = 'mm'
            if g == '20': units = 'in'
            if g == '90': absolute = True
            if g == '91': absolute = False
            if g in ('73', '76', '98', '99') or (g.startswith('8') and len(g) == 2 and g != '80'): E(n, f"canned cycle G{g}")
            if g in ('0', '00', '1', '01', '2', '02', '3', '03'): mode_g = int(g)
            if g == '43': z = None
        for m in ms:
            if m in (3, 4):
                if dragsec: E(n, "spindle started after a spindle-off pause without a tool change stop")
                if unlockdue: E(n, "spindle started after a drag toolpath without an unlock pause")
                dragsec = False; m5 = False
                spindle = True
                if not S > 0: E(n, f"M{m} without positive S")
            if m == 5: spindle = False; m5 = True
            if m in (0, 6):
                if spindle: E(n, "tool change with spindle on")
                if z is not None and z < safe - eps: E(n, "tool change below safe Z")
                z = None
                if m == 0 and not spindle and unlock: dragsec = False; unlockdue = False
                else: dragsec = (m == 0 and not spindle and m5 and announced)
        if '4' in gs or '04' in gs: continue
        if not any(a in w for a in 'XYZ') and not any(a in w for a in 'IJ'): continue
        if units is None or not absolute: E(n, "motion before units/G90")
        if (units == 'in') != inch: E(n, "units mismatch")
        nx = w['X'] * k if 'X' in w else x; ny = w['Y'] * k if 'Y' in w else y; nz = w['Z'] * k if 'Z' in w else z
        xy = (nx != x or ny != y)
        if mode_g == 0:
            if xy and (z is None or z < safe - eps or nz is None or nz < safe - eps): E(n, f"XY rapid not at safe Z z={z}->{nz}")
            if nz is not None and nz < safe - eps: E(n, f"rapid below safe Z ({nz})")
        elif mode_g in (1, 2, 3):
            cuts += 1; m5 = False
            if drag and dragsec and not spindle: dragcuts += 1; unlockdue = True
            elif not spindle or not S > 0: E(n, "cut with spindle off")
            if x is None or y is None or z is None: E(n, "cut from unknown position")
            if nz is not None and nz < bottom - eps: E(n, f"below engrave depth {nz} < {bottom}")
            if nz is not None and nz > safe + eps: E(n, f"cut above safe Z {nz}")
            if mode_g in (2, 3) and not ('I' in w or 'J' in w or 'R' in w): E(n, "arc without centre")
            if F is None or not F > 0: E(n, "cut without positive feed")
            if mode_g == 1 and xy and nz is not None and z is not None and abs(nz - z) > eps: E(n, f"ramped/inclined cut z {z}->{nz}")
            if mode_g == 1 and not xy and nz is not None and z is not None and nz < z - eps and plunge > 0 and abs(F - plunge) > 0.06 * k: E(n, f"plunge at F {F} not plunge rate {plunge}")
            if mode_g in (2, 3):
                if nz is not None and z is not None and abs(nz - z) > eps: E(n, "helical arc")
                if 'I' in w or 'J' in w:
                    cx = x + w.get('I', 0) * k; cy = y + w.get('J', 0) * k
                    r0 = ((x - cx)**2 + (y - cy)**2) ** 0.5; r1 = ((nx - cx)**2 + (ny - cy)**2) ** 0.5
                    if abs(r0 - r1) > 3 * eps: E(n, f"arc radius mismatch {r0} vs {r1}")
            if nz is not None and nz < -eps and xy and z is not None and z < -eps and False: pass
        else: E(n, "motion without mode")
        x, y, z = nx, ny, nz
    if cuts == 0: E('end', "no cuts")
    if drag and dragcuts == 0: E('end', "drag=1 but no spindle-off (drag) cuts")
    if unlockdue: E('end', "program ends after a drag toolpath without an unlock pause")
    if spindle: E('end', "program ends with spindle on")
    if z is None or z < safe - eps: E('end', f"program does not end at safe Z (z={z})")
    return errs
total = 0; files = 0
for line in open(sys.argv[1]):
    parts = line.split(); path, post = parts[0], parts[1]
    P = {kv.split('=')[0]: float(kv.split('=')[1]) for kv in parts[2:]}
    errs = check(path, post, P['bottom'], P['safe'], P.get('plunge', -1), P.get('drag', 0) > 0); files += 1
    if errs:
        total += len(errs); print(path, len(errs), "errors")
        for e in errs[:6]: print("   ", e)
print(f"{files} files, {total} errors"); sys.exit(1 if total else 0)
