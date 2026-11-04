#!/bin/sh
# usage: sweep.sh <dir with vc3> <out dir> [jobs]   round-3 sweep (xargs -P jobs, default 3), line-buffered output
H=$1; R=$2; J=${3:-3}; mkdir -p $R
{
for f in 16 17 18 19 20 21 22 23; do echo "fam_$f|3$f 40|VC_FAM=$f"; done
for s in 301 302 303 304; do echo "mix_$s|$s 60|"; done
for f in 4 9 10 11 13 14 15; do echo "fam_$f|4$f 30|VC_FAM=$f"; done
echo "f15exact|715 60|VC_FAM=15 VC_F15=1"
echo "far1e3|501 30|VC_FAR=1000"; echo "far2e4|502 30|VC_FAR=20000"; echo "far2e5|503 30|VC_FAR=200000"
echo "scale005|504 40|VC_SCALE=0.05"; echo "scale20|505 30|VC_SCALE=20"
for f in 1 6 10 14 16 17; do echo "ch3e4_fam$f|6$f 10|VC_CH=0.0003 VC_FAM=$f"; done
} | xargs -P $J -I{} sh -c 'n=$(echo "{}" | cut -d"|" -f1); a=$(echo "{}" | cut -d"|" -f2); e=$(echo "{}" | cut -d"|" -f3); env $e stdbuf -oL '$H'/vc3s $a > '$R'/$n.txt 2>&1'
echo done
