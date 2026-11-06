#!/bin/sh
# usage: coderprobe.sh <out> <fam> <seeds>: the coder's cost case (curved pocket radius ~1.5 m, chordal 0.002, distance = chordal,
# raster + rest tool), per seed main/branch alternating, 2 reps
B=/home/chris/wt-T0169/build-verify1-bin; O=$1; : > $O
for s in $(seq 1 $3); do for rep in 1 2; do for v in main br; do
  r=$(env PK_SCALE=150 PK_DFR=1 PK_REST=1 ${CP_STRAT:+PK_STRAT=$CP_STRAT} PK_CH=0.002 PK_NOCHECK=1 timeout 600 $B/pk_$v $2 $s 1 | grep '^case'); echo "$v rep$rep $r" | cut -c1-260 >> $O; done; done; done
echo DONE >> $O
