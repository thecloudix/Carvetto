# T-0150 verifier: drilling G-code checker, extended from harness/t0122v1/gcheck.py.
# Understands the chip-breaking rule (owner decision 2026-11-03): a rapid below safe Z is allowed
# only as an upward Z-only retract from inside the hole already drilled at that XY. Also checks the
# AboveStart retract height, plunge feed, dwell words, spindle, canned cycles, final state.
import re, sys
def check(path, post, P):
    inch = 'inch' in post; k = 25.4 if inch else 1.0
    eps = 0.0026 if inch else 0.0011
    safe = P['safe']; start = P['start']; bottom = P['bottom']; gap = P['gap']; mode = P['mode']; dwell = P['dwell']; plunge = P['plunge']
    x = y = z = None; spindle = False; S = 0.0; F = None; units = None; absolute = False; mode_g = None; errs = []
    deepest = {}  # (x,y) -> deepest Z drilled
    holes_bottom = {}; dwells = 0; lastcut = None
    def E(n, msg): errs.append(f"{n}: {msg}")
    for n, line in enumerate(open(path, newline=''), 1):
        s = line.rstrip('\r\n')
        if any(ord(ch) < 32 or ord(ch) > 126 for ch in s): E(n, "non-printable")
        s = re.sub(r"\(.*?\)", "", s); s = s.split(';')[0].strip()
        if not s: continue
        codes = re.findall(r"([A-Z])(-?\d+(?:\.\d+)?)", s)
        if codes and codes[0][0] == 'N': codes = codes[1:]
        if any(c[0] == 'M' and c[1] == '0' for c in codes):
            codes = codes[:[i for i, c in enumerate(codes) if c[0] == 'M' and c[1] == '0'][0] + 1]
        gs = [v for kk, v in codes if kk == 'G']; ms = [int(float(v)) for kk, v in codes if kk == 'M']
        w = {kk: float(v) for kk, v in codes if kk in 'XYZFSP'}
        if 'S' in w: S = w['S']
        if 'F' in w: F = w['F'] * (k if True else 1)
        for g in gs:
            if g == '21': units = 'mm'
            if g == '20': units = 'in'
            if g == '90': absolute = True
            if g == '91': absolute = False
            if g in ('73', '76', '98', '99') or (g.startswith('8') and len(g) == 2 and g != '80'): E(n, f"canned cycle G{g}")
            if g in ('0', '00', '1', '01', '2', '02', '3', '03'): mode_g = int(g)
            if g in ('2', '3', '02', '03'): E(n, "arc in drilling")
            if g == '43': z = None
        for m in ms:
            if m == 3:
                spindle = True
                if not S > 0: E(n, "M3 without positive S")
            if m == 5: spindle = False
            if m in (0, 6):
                if spindle: E(n, "tool change with spindle on")
                if z is not None and z < safe - eps: E(n, "tool change below safe Z")
                z = None
        if '4' in gs or '04' in gs:
            dwells += 1
            if not ('P' in w and w['P'] > 0): E(n, "dwell not > 0")
            want = dwell * (1000 if 'marlin' in post else 1)
            if 'P' in w and abs(w['P'] - want) > (0.5 if 'marlin' in post else 0.0051): E(n, f"dwell P{w['P']} want {want}")
            if lastcut is None or (x, y, z) != lastcut: E(n, "dwell not at the bottom of a plunge")
            continue
        if not any(a in w for a in 'XYZ'): continue
        if units is None or not absolute: E(n, "motion before units/G90")
        if (units == 'in') != inch: E(n, "units mismatch")
        nx = w['X'] * k if 'X' in w else x; ny = w['Y'] * k if 'Y' in w else y; nz = w['Z'] * k if 'Z' in w else z
        xy = (nx != x or ny != y)
        if mode_g == 0:
            if xy and (z is None or z < safe - eps or nz is None or nz < safe - eps): E(n, f"XY rapid not at safe Z z={z}->{nz}")
            if nz is not None and nz < safe - eps:
                # in-hole rapid: must be Z-only, upward, start inside drilled depth at this XY
                if xy: E(n, "rapid below safe Z moves XY")
                elif z is None or not nz > z + 1e-9: E(n, f"rapid below safe Z not upward {z}->{nz}")
                elif (x, y) not in deepest or z < deepest[(x, y)] - eps: E(n, "upward rapid starts outside drilled hole")
                else:
                    if mode == 'none': E(n, "in-hole retract without pecking")
                    if mode == 'start' and nz < min(start + gap, safe) - eps: E(n, f"AboveStart retract {nz:.4f} below start+gap {start+gap:.4f}")
                    if mode == 'prev' and abs(nz - min(z + gap, safe)) > eps: E(n, f"AbovePrev retract {nz:.4f} != previous pass {z:.4f} + gap")
                    if nz > 0 and nz < safe - eps and False: pass
        elif mode_g == 1:
            if not spindle or not S > 0: E(n, "cut with spindle off")
            if xy: E(n, "G1 moves XY while drilling")
            if z is None or not nz < z + 1e-9: E(n, f"G1 not downward {z}->{nz}")
            if F is None or abs(F - plunge) > max(0.6 * k if inch else 0.6, plunge * 1e-3): E(n, f"plunge feed {F} != {plunge}")
            deepest[(nx, ny)] = min(deepest.get((nx, ny), 1e300), nz); lastcut = (nx, ny, nz)
            if nz < bottom - eps: E(n, f"below bottom {nz} < {bottom}")
        else: E(n, "motion without mode")
        x, y, z = nx, ny, nz
    if spindle: E('end', "program ends with spindle on")
    if z is None or z < safe - eps: E('end', f"program does not end at safe Z (z={z})")
    holes = [kk for kk, v in deepest.items()]
    for kk, v in deepest.items():
        if abs(v - bottom) > eps: E('end', f"hole at {kk} only reaches {v} (bottom {bottom})")
    if len(holes) != P['holes']: E('end', f"{len(holes)} holes drilled, want {P['holes']}")
    passes_per_hole = None
    if dwell > 0 and dwells == 0: E('end', "no dwells")
    if dwell == 0 and dwells: E('end', "dwell with dwell time 0")
    return errs
total = 0; files = 0
for line in open(sys.argv[1]):
    parts = line.split(); path, post = parts[0], parts[1]
    P = {}
    for kv in parts[2:]:
        kk, v = kv.split('='); P[kk] = v if kk == 'mode' else float(v)
    errs = check(path, post, P); files += 1
    if errs:
        total += len(errs); print(path, len(errs), "errors")
        for e in errs[:6]: print("   ", e)
print(f"{files} files, {total} errors"); sys.exit(1 if total else 0)
