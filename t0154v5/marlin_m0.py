# Model of Marlin 2.1 GCodeParser::parse parameter scan with FASTER_GCODE_PARSER (default on),
# GCODE_CASE_INSENSITIVE off (default), written from the behaviour of Marlin/src/gcode/parser.cpp
# (bugfix-2.1.x): an upper-case letter is a parameter; spaces after it are skipped before the
# value; the value is valid if [-+]?.?[0-9]. M0/M1 then read P (ms) and S (s) as a timeout.
import sys
def valid_signless(s): return len(s) > 0 and (s[0].isdigit() or (s[0] == '.' and len(s) > 1 and s[1].isdigit()))
def valid_float(s): return valid_signless(s) or (len(s) > 1 and s[0] in '+-' and valid_signless(s[1:]))
def params(line, ci=False):
    p = line.lstrip(' ')
    assert p[0] in 'Mm'; p = p[1:]
    while p and p[0].isdigit(): p = p[1:]
    p = p.lstrip(' ')
    out = {}
    i = 0
    while i < len(p):
        c = p[i]; i += 1
        if ci and 'a' <= c <= 'z': c = c.upper()
        if 'A' <= c <= 'Z':
            while i < len(p) and p[i] == ' ': i += 1
            if valid_float(p[i:]):
                j = i + (1 if p[i] in '+-' else 0)
                while j < len(p) and (p[j].isdigit() or p[j] == '.'): j += 1
                out[c] = p[i:j]
            else: out.setdefault(c, None)
        if not (i < len(p) and 'A' <= p[i] <= 'Z'):
            while i < len(p) and (p[i].isdigit() or p[i] in '.-+'): i += 1
            while i < len(p) and p[i] == ' ': i += 1
    return out
for line in sys.argv[1:]:
    for ci in (False, True):
        pr = params(line, ci)
        t = {k: v for k, v in pr.items() if k in 'SP' and v is not None}
        print(f"{line!r:50} case_insensitive={ci!s:5} timeout words: {t or 'none (waits for the user)'}")
