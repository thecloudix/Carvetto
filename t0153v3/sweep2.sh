#!/bin/sh
# usage: sweep2.sh <dir with vc3r> <out dir> [jobs]: far / scale / minimum-chordal part of the sweep with the far-safe oracle
H=$1; R=$2; J=${3:-2}; mkdir -p $R
{
echo "far1e3|501 25|VC_FAR=1000"; echo "far2e4|502 25|VC_FAR=20000"; echo "far2e5|503 25|VC_FAR=200000"
echo "scale005|504 30|VC_SCALE=0.05"; echo "scale20|505 20|VC_SCALE=20"
for f in 1 6 10 14 16 17; do echo "ch3e4_fam$f|6$f 8|VC_CH=0.0003 VC_FAM=$f"; done
echo "f15exact|715 40|VC_FAM=15 VC_F15=1"
} | xargs -P $J -I{} sh -c 'n=$(echo "{}" | cut -d"|" -f1); a=$(echo "{}" | cut -d"|" -f2); e=$(echo "{}" | cut -d"|" -f3); env $e stdbuf -oL '$H'/vc3r $a > '$R'/$n.txt 2>&1'
echo done
