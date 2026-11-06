#!/usr/bin/env python3
# pgm2png.py in.pgm out.png (P5, 8-bit) - stdlib only
import sys, zlib, struct
d = open(sys.argv[1], 'rb').read(); hdr = d.split(b'\n', 1); w, h = map(int, hdr[0].split()[1:3]); px = hdr[1]
raw = b''.join(b'\x00' + px[y * w:(y + 1) * w] for y in range(h))
def ch(t, b): return struct.pack('>I', len(b)) + t + b + struct.pack('>I', zlib.crc32(t + b) & 0xffffffff)
open(sys.argv[2], 'wb').write(b'\x89PNG\r\n\x1a\n' + ch(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 0, 0, 0, 0)) + ch(b'IDAT', zlib.compress(raw, 9)) + ch(b'IEND', b''))
