#!/usr/bin/env python3
# floodsum.py DIR... : totals of t40 flood sweeps (scenes, free >= 0.04 & thick > 0.05, reported, MISS, FALSE, loose-matched, THIN) + mismatch lines
import sys, glob, re
T = dict(scenes=0, free=0, rep=0, miss=0, false=0, loose=0, thin=0); bad = []
for D in sys.argv[1:]:
    for p in sorted(glob.glob(f'{D}/*.txt')):
        for l in open(p, errors='replace'):
            if l.startswith('SUMMARY'):
                t = l.split()
                for k in T: 
                    if k in t: T[k] += int(t[t.index(k) + 1])
            elif l.startswith('S ') and ('miss 0 false 0' not in l) and not l.rstrip().endswith(('empty', 'nocut')): bad.append(l.strip()[:400])
print(T)
for b in bad: print(' ', b)
