#!/usr/bin/env python3
"""T-0363: build the multi-size Windows icon from the committed PNG renders (stdlib only).

Each PNG becomes one PNG-compressed ICO entry (supported since Windows Vista). The sizes come from the PNG
headers, so the file names do not matter. Usage:
    make_ico.py <output.ico> <png> [<png> ...]
    make_ico.py --check <icon.ico> <png> [<png> ...]   (exit 1 when the .ico differs from a fresh build)
Entries are ordered by size. A 256 px image is stored with width/height 0, as the ICO format requires.
"""
import struct
import sys

PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def png_size(data: bytes):
    if len(data) < 24 or data[:8] != PNG_SIGNATURE or data[12:16] != b"IHDR":
        raise ValueError("not a PNG")
    return struct.unpack(">II", data[16:24])


def build_ico(pngs) -> bytes:
    images = []
    for data in pngs:
        w, h = png_size(data)
        if w != h or not 1 <= w <= 256:
            raise ValueError("icon images must be square and at most 256 px, got %dx%d" % (w, h))
        images.append((w, data))
    images.sort(key=lambda e: e[0])
    if len({w for w, _ in images}) != len(images):
        raise ValueError("duplicate icon size")
    out = bytearray(struct.pack("<HHH", 0, 1, len(images)))
    offset = 6 + 16 * len(images)
    for w, data in images:
        dim = 0 if w == 256 else w
        out += struct.pack("<BBBBHHII", dim, dim, 0, 0, 1, 32, len(data), offset)
        offset += len(data)
    for _, data in images:
        out += data
    return bytes(out)


def read_ico_directory(ico: bytes):
    """Returns [(size, byte_length, byte_offset)] for a .ico; raises ValueError when malformed."""
    reserved, kind, count = struct.unpack("<HHH", ico[:6])
    if reserved != 0 or kind != 1:
        raise ValueError("not an .ico")
    entries = []
    for i in range(count):
        w, h, _c, _r, _planes, _bpp, length, offset = struct.unpack("<BBBBHHII", ico[6 + 16 * i:22 + 16 * i])
        if offset + length > len(ico):
            raise ValueError("entry %d runs past the end of the file" % i)
        entries.append((w or 256, length, offset))
    return entries


def main(argv) -> int:
    check = len(argv) > 1 and argv[1] == "--check"
    args = argv[2:] if check else argv[1:]
    if len(args) < 2:
        print(__doc__)
        return 2
    pngs = []
    for p in args[1:]:
        with open(p, "rb") as f:
            pngs.append(f.read())
    ico = build_ico(pngs)
    if check:
        with open(args[0], "rb") as f:
            if f.read() != ico:
                print("%s is stale: rerun tools/make_ico.py" % args[0])
                return 1
        return 0
    with open(args[0], "wb") as f:
        f.write(ico)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
