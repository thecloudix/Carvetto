#!/usr/bin/env python3
"""T-0363: the committed icon artwork is complete and consistent.

Checks that the SVG sources and every PNG size exist with the right pixel size, that packaging/icons/carvetto.ico
is a valid multi-size icon whose images are exactly the committed PNGs (so it is not stale), and that make_ico.py
rejects bad input. CI has no SVG renderer, so the PNGs are committed renders of the SVGs.
Usage: test_app_icon.py <source-root>
"""
import os
import sys

root = os.path.abspath(sys.argv[1])
sys.path.insert(0, os.path.join(root, "tools"))
import make_ico  # noqa: E402

icons = os.path.join(root, "packaging", "icons")
# Pixel size -> file. 16 and 24 px use the simplified small-size variant (C and pocket only).
FILES = {16: "carvetto-small-16.png", 24: "carvetto-small-24.png", 32: "carvetto-32.png", 48: "carvetto-48.png",
         64: "carvetto-64.png", 128: "carvetto-128.png", 256: "carvetto-256.png"}
errors = []

for svg in ("carvetto.svg", "carvetto-small.svg"):
    text = open(os.path.join(icons, svg), encoding="utf-8").read()
    if "<svg" not in text or 'viewBox="0 0 256 256"' not in text:
        errors.append(svg + " is not a 256x256 SVG")

pngs = {}
for size, name in FILES.items():
    path = os.path.join(icons, name)
    if not os.path.isfile(path):
        errors.append("missing " + name)
        continue
    data = open(path, "rb").read()
    try:
        if make_ico.png_size(data) != (size, size):
            errors.append("%s is not %dx%d" % (name, size, size))
    except ValueError as e:
        errors.append("%s: %s" % (name, e))
    pngs[size] = data

ico_path = os.path.join(icons, "carvetto.ico")
if not os.path.isfile(ico_path):
    errors.append("missing carvetto.ico")
elif len(pngs) == len(FILES):
    ico = open(ico_path, "rb").read()
    try:
        entries = make_ico.read_ico_directory(ico)
        if [e[0] for e in entries] != sorted(FILES):
            errors.append("ico sizes %r, expected %r" % ([e[0] for e in entries], sorted(FILES)))
        for size, length, offset in entries:
            if ico[offset:offset + length] != pngs.get(size):
                errors.append("ico entry %d differs from its PNG (rerun tools/make_ico.py)" % size)
        if ico != make_ico.build_ico(list(pngs.values())):
            errors.append("carvetto.ico is stale (rerun tools/make_ico.py)")
    except ValueError as e:
        errors.append("carvetto.ico: " + str(e))

# Generator error handling.
for bad in (b"not a png", pngs.get(16, b"")[:20]):
    try:
        make_ico.build_ico([bad])
        errors.append("make_ico accepted invalid PNG data")
    except ValueError:
        pass
if 16 in pngs:
    try:
        make_ico.build_ico([pngs[16], pngs[16]])
        errors.append("make_ico accepted a duplicate size")
    except ValueError:
        pass

if errors:
    print("\n".join(errors))
    sys.exit(1)
print("app icon ok")
