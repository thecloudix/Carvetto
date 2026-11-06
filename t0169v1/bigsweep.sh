#!/bin/sh
# usage: bigsweep.sh <jobs>: hunt for cost outliers: families 0-9 at scale 150 (radius ~1.5 m), chordal 0.002, distance = chordal,
# rest tool, hash only (PK_NOCHECK), branch and main alternating per family -> results/big
B=/home/chris/wt-T0169/build-verify1-bin; R=/home/chris/harness/t0169v1/results/big; mkdir -p $R
for f in 0 1 2 3 4 5 6 7 8 9; do echo $f; done | xargs -P $1 -I{} sh -c 'for v in main br; do PK_SCALE=150 PK_DFR=1 PK_REST=1 PK_CH=0.002 PK_NOCHECK=1 timeout 1800 '$B'/pk_$v {} 1 30 > '$R'/f{}_$v.txt 2>&1; echo "exit $?" >> '$R'/f{}_$v.txt; done'
echo done > $R/DONE
