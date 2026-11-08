#!/usr/bin/env python3
"""Generates core/tests/fixtures/text/CarvettoTest-Regular.ttf (T-0130, ADR-0033).

Standard library only; the output is byte-reproducible (fixed timestamps, no randomness).
The font is original work of the Carvetto project, released under the SIL Open Font License 1.1
(core/tests/fixtures/text/OFL.txt). Every glyph is a simple shape with a hand-computable area so
tests can check the outline pipeline against an independent oracle (see
core/tests/fixtures/text/README.md). Usage: make_test_font.py [output.ttf]
"""
import struct
import sys

UPEM = 1000
ASCENDER, DESCENDER, LINEGAP = 800, -200, 100


def poly(*pts):
    return [(x, y, True) for x, y in pts]


# (name, advance, contours). TrueType winds outer contours clockwise and holes counter-clockwise.
GLYPHS = [
    # 0 .notdef: 400x700 box with a 200x500 counter. Net area 400*700 - 200*500 = 180000.
    (".notdef", 500, [poly((100, 0), (100, 700), (500, 700), (500, 0)),
                      poly((200, 100), (400, 100), (400, 600), (200, 600))]),
    # 1 space
    ("space", 300, []),
    # 2 I: 200x700 bar. Area 140000.
    ("I", 400, [poly((100, 0), (100, 700), (300, 700), (300, 0))]),
    # 3 T: 600x100 bar on a 100x600 stem. Area 120000.
    ("T", 700, [poly((0, 600), (0, 700), (600, 700), (600, 600), (350, 600), (350, 0), (250, 0),
                     (250, 600))]),
    # 4 O: rounded square from quadratic curves (on-curve at side midpoints, off-curve at corners).
    #   outer area 245000 + 4 * 2/3 * 61250 = 408333.33..; counter 80000 + 4 * 2/3 * 20000 = 133333.33..
    #   Net area 275000.
    ("O", 800,
     [[(0, 350, True), (0, 700, False), (350, 700, True), (700, 700, False),
       (700, 350, True), (700, 0, False), (350, 0, True), (0, 0, False)],
      [(150, 350, True), (150, 150, False), (350, 150, True), (550, 150, False),
       (550, 350, True), (550, 550, False), (350, 550, True), (150, 550, False)]]),
    # 5 Hebrew alef stand-in: an L shape (asymmetric, so a mirrored outline shows). Area 100000.
    ("alef", 600, [poly((100, 0), (100, 700), (200, 700), (200, 100), (500, 100), (500, 0))]),
    # 6 combining acute: zero advance, 200x100 bar above the cap height, left of the origin. Area 20000.
    ("acutecomb", 0, [poly((-300, 750), (-300, 850), (-100, 850), (-100, 750))]),
]
# 7 composite U+00C9: I plus the acute shifted right by 400. Area 140000 + 20000 = 160000.
CMAP = {0x20: 1, 0x49: 2, 0x54: 3, 0x4F: 4, 0xC9: 7, 0x301: 6, 0x5D0: 5}
KERN = {(3, 4): -120}  # T followed by O


def bbox(contours):
    pts = [p for c in contours for p in c]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return min(xs), min(ys), max(xs), max(ys)


def glyph_bytes(contours):
    if not contours:
        return b""
    x0, y0, x1, y1 = bbox(contours)
    pts = [p for c in contours for p in c]
    out = struct.pack(">hhhhh", len(contours), x0, y0, x1, y1)
    ends, n = [], 0
    for c in contours:
        n += len(c)
        ends.append(n - 1)
    out += struct.pack(">%dH" % len(ends), *ends) + struct.pack(">H", 0)  # no instructions
    out += bytes(1 if p[2] else 0 for p in pts)  # bit 0 = on curve; x and y as int16 deltas
    px = 0
    for p in pts:
        out += struct.pack(">h", p[0] - px)
        px = p[0]
    py = 0
    for p in pts:
        out += struct.pack(">h", p[1] - py)
        py = p[1]
    return out


def composite_bytes():
    out = struct.pack(">hhhhh", -1, 100, 0, 300, 850)
    out += struct.pack(">HHhh", 0x0001 | 0x0002 | 0x0020, 2, 0, 0)  # I at (0,0), more follow
    out += struct.pack(">HHhh", 0x0001 | 0x0002, 6, 400, 0)         # acute at (400,0)
    return out


def checksum(data):
    data += b"\0" * (-len(data) % 4)
    return sum(struct.unpack(">%dI" % (len(data) // 4), data)) & 0xFFFFFFFF


def log2floor(n):
    r = 0
    while (1 << (r + 1)) <= n:
        r += 1
    return r


def build():
    n_glyphs = len(GLYPHS) + 1
    advances = [g[1] for g in GLYPHS] + [400]
    lsbs = [bbox(g[2])[0] if g[2] else 0 for g in GLYPHS] + [100]
    glyf, loca = b"", []
    for g in GLYPHS:
        loca.append(len(glyf))
        data = glyph_bytes(g[2])
        glyf += data + b"\0" * (-len(data) % 4)
    loca.append(len(glyf))
    data = composite_bytes()
    glyf += data + b"\0" * (-len(data) % 4)
    loca.append(len(glyf))

    max_pts = max(sum(len(c) for c in g[2]) for g in GLYPHS)
    max_cont = max(len(g[2]) for g in GLYPHS)
    t = {}
    t["head"] = struct.pack(">IIIIHHqqhhhhHHhhh", 0x00010000, 0x00010000, 0, 0x5F0F3CF5, 0x000B, UPEM,
                            0, 0, -300, 0, 700, 850, 0, 7, 2, 1, 0)
    t["hhea"] = struct.pack(">IhhhHhhhhhhhhhhhH", 0x00010000, ASCENDER, DESCENDER, LINEGAP, 800, -300, 0,
                            700, 1, 0, 0, 0, 0, 0, 0, 0, n_glyphs)
    t["maxp"] = struct.pack(">IHHHHHHHHHHHHHH", 0x00010000, n_glyphs, max_pts, max_cont, max_pts + 4, 2,
                            1, 0, 0, 0, 0, 0, 0, 2, 1)
    t["hmtx"] = b"".join(struct.pack(">Hh", a, l) for a, l in zip(advances, lsbs))
    t["loca"] = struct.pack(">%dI" % len(loca), *loca)
    t["glyf"] = glyf

    os2 = struct.pack(">HhHHHhhhhhhhhhhh", 3, 500, 400, 5, 0, 650, 600, 0, 140, 650, 600, 0, 480, 50, 400, 0)
    os2 += b"\0" * 10                                       # panose
    os2 += struct.pack(">IIII", 1, 0, 0, 0)                 # ulUnicodeRange1-4
    os2 += b"CVTO"                                          # achVendID
    os2 += struct.pack(">HHHhhhHH", 0x40, 0x20, 0xFFFF, ASCENDER, DESCENDER, LINEGAP, ASCENDER, 200)
    os2 += struct.pack(">II", 1, 0)                         # code page ranges
    os2 += struct.pack(">hhHHH", 500, 700, 0, 0x20, 1)      # xHeight, capHeight, default, break, maxContext
    t["OS/2"] = os2

    codes = sorted(CMAP)
    segs = [(c, c, (CMAP[c] - c) & 0xFFFF) for c in codes] + [(0xFFFF, 0xFFFF, 1)]
    sc = len(segs)
    es = log2floor(sc)
    sub = struct.pack(">HHHHHHH", 4, 16 + 8 * sc, 0, sc * 2, 2 * (1 << es), es, sc * 2 - 2 * (1 << es))
    sub += b"".join(struct.pack(">H", s[1]) for s in segs) + struct.pack(">H", 0)
    sub += b"".join(struct.pack(">H", s[0]) for s in segs)
    sub += b"".join(struct.pack(">H", s[2]) for s in segs)
    sub += b"".join(struct.pack(">H", 0) for _ in segs)
    t["cmap"] = struct.pack(">HHHHI", 0, 1, 3, 1, 12) + sub

    pairs = sorted(KERN.items())
    kes = log2floor(len(pairs))
    body = struct.pack(">HHHH", len(pairs), 6 * (1 << kes), kes, 6 * len(pairs) - 6 * (1 << kes))
    body += b"".join(struct.pack(">HHh", a, b, v) for (a, b), v in pairs)
    t["kern"] = struct.pack(">HH", 0, 1) + struct.pack(">HHH", 0, 6 + len(body), 1) + body

    names = {
        0: "Copyright 2026 The Carvetto Authors. Original work.",
        1: "Carvetto Test", 2: "Regular", 3: "Carvetto Test Regular 1.0", 4: "Carvetto Test Regular",
        5: "Version 1.000", 6: "CarvettoTest-Regular",
        13: "This Font Software is licensed under the SIL Open Font License, Version 1.1.",
        14: "https://openfontlicense.org",
    }
    recs, strs = b"", b""
    for nid in sorted(names):
        s = names[nid].encode("utf-16-be")
        recs += struct.pack(">HHHHHH", 3, 1, 0x409, nid, len(s), len(strs))
        strs += s
    t["name"] = struct.pack(">HHH", 0, len(names), 6 + 12 * len(names)) + recs + strs
    t["post"] = struct.pack(">IIhhIIIII", 0x00030000, 0, -100, 50, 0, 0, 0, 0, 0)

    tags = sorted(t)
    nt = len(tags)
    es = log2floor(nt)
    directory = struct.pack(">IHHHH", 0x00010000, nt, 16 * (1 << es), es, 16 * nt - 16 * (1 << es))
    offset = 12 + 16 * nt
    body = b""
    head_off = 0
    for tag in tags:
        d = t[tag]
        if tag == "head":
            head_off = offset + len(body)
        directory += tag.encode("ascii") + struct.pack(">III", checksum(d), offset + len(body), len(d))
        body += d + b"\0" * (-len(d) % 4)
    font = bytearray(directory + body)
    adj = (0xB1B0AFBA - checksum(bytes(font))) & 0xFFFFFFFF
    font[head_off + 8:head_off + 12] = struct.pack(">I", adj)
    return bytes(font)


if __name__ == "__main__":
    out_path = sys.argv[1] if len(sys.argv) > 1 else "core/tests/fixtures/text/CarvettoTest-Regular.ttf"
    with open(out_path, "wb") as f:
        f.write(build())
