#!/bin/sh
# usage: sweep.sh <dir with vc2> <out dir>   (5 jobs at a time)
H=$1; R=$2; mkdir -p $R
for s in 101 102 103 104 105; do $H/vc2 $s 300 > $R/mix_$s.txt 2>&1 & done; wait
for f in 9 10 11 13 14; do VC_FAM=$f $H/vc2 2$f 120 > $R/fam_$f.txt 2>&1 & done; wait
for f in 12 5 4 8 7; do VC_FAM=$f $H/vc2 2$f 120 > $R/fam_$f.txt 2>&1 & done; wait
VC_FAR=1000 $H/vc2 301 100 > $R/far1e3.txt 2>&1 & VC_FAR=20000 $H/vc2 302 100 > $R/far2e4.txt 2>&1 & VC_FAR=200000 $H/vc2 305 60 > $R/far2e5.txt 2>&1 &
VC_SCALE=0.05 $H/vc2 303 100 > $R/scale005.txt 2>&1 & VC_SCALE=20 $H/vc2 304 60 > $R/scale20.txt 2>&1 & wait
echo done
