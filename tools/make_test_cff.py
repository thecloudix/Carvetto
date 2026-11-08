#!/usr/bin/env python3
"""Generates core/tests/fixtures/text/CarvettoTestCFF-Regular.otf (T-0130, ADR-0033).

A tiny OpenType/CFF font (cubic outlines, outer contours counter-clockwise) so tests cover the
PostScript-style code path: orientation (FT_OUTLINE_REVERSE_FILL) and genuine cubic Beziers.
Original Carvetto work under the SIL OFL 1.1 (core/tests/fixtures/text/OFL.txt); standard library only,
byte-reproducible. Usage: make_test_cff.py [output.otf]

Glyphs (units per em 1000; see core/tests/fixtures/text/README.md):
  0 .notdef  empty            1 'A' U+0041  box 600x700 with a 400x500 hole, advance 700
  2 'B' U+0042  line, cubic, line, advance 700    3 'C' U+0043  circle r500 with a circle hole r200, advance 1000
"""
import struct
import sys


def num(v):
    v = int(v)
    if -107 <= v <= 107:
        return bytes([v + 139])
    if 108 <= v <= 1131:
        v -= 108
        return bytes([247 + (v >> 8), v & 255])
    if -1131 <= v <= -108:
        v = -v - 108
        return bytes([251 + (v >> 8), v & 255])
    return b"\x1c" + struct.pack(">h", v)


def cs(*items):
    return b"".join(i if isinstance(i, bytes) else num(i) for i in items)


RMOVETO, RLINETO, RRCURVETO, ENDCHAR = b"\x15", b"\x05", b"\x08", b"\x0e"


def index(objs):
    if not objs:
        return struct.pack(">H", 0)
    off = [1]
    for o in objs:
        off.append(off[-1] + len(o))
    return struct.pack(">HB", len(objs), 4) + b"".join(struct.pack(">I", x) for x in off) + b"".join(objs)


def quarter(p0, c1, c2, p3):
    return cs(c1[0] - p0[0], c1[1] - p0[1], c2[0] - c1[0], c2[1] - c1[1], p3[0] - c2[0], p3[1] - c2[1], RRCURVETO)


def circle(cx, cy, r, ccw):
    k = round(r * 0.5523)
    p = [(cx + r, cy), (cx, cy + r), (cx - r, cy), (cx, cy - r), (cx + r, cy)]
    c1 = [(cx + r, cy + k), (cx - k, cy + r), (cx - r, cy - k), (cx + k, cy - r)]
    c2 = [(cx + k, cy + r), (cx - r, cy + k), (cx - k, cy - r), (cx + r, cy - k)]
    segs = [(p[i], c1[i], c2[i], p[i + 1]) for i in range(4)]
    if not ccw:
        segs = [(s[3], s[2], s[1], s[0]) for s in reversed(segs)]
    return segs[0][0], b"".join(quarter(*s) for s in segs)


def move(prev, to):
    return cs(to[0] - prev[0], to[1] - prev[1], RMOVETO)


notdef = cs(ENDCHAR)
# A: outer (0,0) (600,0) (600,700) (0,700) counter-clockwise; hole (100,100) (100,600) (500,600) (500,100) clockwise.
glyph_a = cs(0, 0, RMOVETO, 600, 0, 0, 700, -600, 0, RLINETO, 100, 100, RMOVETO, 0, 500, 400, 0, 0, -500, RLINETO, ENDCHAR)
# B: (0,0) -> (500,0) line; cubic via (800,100), (800,400) to (500,500); line back to (0,500).
glyph_b = cs(0, 0, RMOVETO, 500, 0, RLINETO, 300, 100, 0, 300, -300, 100, RRCURVETO, -500, 0, RLINETO, ENDCHAR)
s1, b1 = circle(500, 500, 500, True)
s2, b2 = circle(500, 500, 200, False)
glyph_c = move((0, 0), s1) + b1 + move(s1, s2) + b2 + ENDCHAR
glyphs = [notdef, glyph_a, glyph_b, glyph_c]
ADV = [500, 700, 700, 1000]
CMAP = {0x41: 1, 0x42: 2, 0x43: 3}


def build():
    charstrings = index(glyphs)
    header = bytes([1, 0, 4, 4])
    name = index([b"CarvettoCFF"])
    strings = index([])
    gsubr = index([])
    private = num(600) + bytes([20]) + num(0) + bytes([21])  # defaultWidthX 600, nominalWidthX 0

    def topdict(cs_off, priv_size, priv_off):
        return (b"\x1d" + struct.pack(">i", cs_off) + bytes([17]) + b"\x1d" + struct.pack(">i", priv_size) +
                b"\x1d" + struct.pack(">i", priv_off) + bytes([18]))

    top_len = len(index([topdict(0, 0, 0)]))
    cs_off = len(header) + len(name) + top_len + len(strings) + len(gsubr)
    top = index([topdict(cs_off, len(private), cs_off + len(charstrings))])
    assert len(top) == top_len
    cff = header + name + top + strings + gsubr + charstrings + private

    n = len(ADV)
    head = struct.pack(">IIIIHHqqhhhhHHhhh", 0x00010000, 0x00010000, 0, 0x5F0F3CF5, 0, 1000, 0, 0, 0, 0, 1000, 1000,
                       0, 8, 2, 0, 0)
    hhea = struct.pack(">IhhhHhhhhhhhhhhhH", 0x00010000, 800, -200, 0, 1000, 0, 0, 1000, 1, 0, 0, 0, 0, 0, 0, 0, n)
    maxp = struct.pack(">IH", 0x00005000, n)
    hmtx = b"".join(struct.pack(">Hh", a, 0) for a in ADV)
    groups = b"".join(struct.pack(">III", c, c, g) for c, g in sorted(CMAP.items()))
    sub = struct.pack(">HHIII", 12, 0, 16 + len(groups), 0, len(CMAP)) + groups
    cmap = struct.pack(">HH", 0, 1) + struct.pack(">HHI", 3, 10, 12) + sub
    fam = "CarvettoCFF".encode("utf-16-be")
    recs = [(1, fam), (2, "Regular".encode("utf-16-be")), (4, fam), (6, fam)]
    rb, st, off = b"", b"", 0
    for nid, d in recs:
        rb += struct.pack(">HHHHHH", 3, 1, 0x409, nid, len(d), off)
        st += d
        off += len(d)
    name_t = struct.pack(">HHH", 0, len(recs), 6 + len(recs) * 12) + rb + st
    post = struct.pack(">IIhhIIIII", 0x00030000, 0, -100, 50, 0, 0, 0, 0, 0)
    os2 = struct.pack(">HhHHHhhhhhhhhhhhh10sIIII4sHHHhhhHH", 3, 500, 400, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      b"\0" * 10, 1, 0, 0, 0, b"NONE", 0x40, 0x41, 0x43, 800, -200, 100, 800, 200)
    tables = {b"CFF ": cff, b"OS/2": os2, b"cmap": cmap, b"head": head, b"hhea": hhea, b"hmtx": hmtx,
              b"maxp": maxp, b"name": name_t, b"post": post}
    tags = sorted(tables)
    out = struct.pack(">4sHHHH", b"OTTO", len(tags), 128, 3, len(tags) * 16 - 128)
    off = 12 + len(tags) * 16
    body = b""
    for t in tags:
        d = tables[t]
        pd = d + b"\0" * (-len(d) % 4)
        ck = sum(struct.unpack(">%dI" % (len(pd) // 4), pd)) & 0xFFFFFFFF
        out += struct.pack(">4sIII", t, ck, off + len(body), len(d))
        body += pd
    return out + body


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "core/tests/fixtures/text/CarvettoTestCFF-Regular.otf"
    with open(path, "wb") as f:
        f.write(build())
