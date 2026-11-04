#!/bin/sh
# usage: sweep4.sh <vc3 binary> <out dir> [jobs]   round-4 sweep: every family 0..23 (25 cases, seeds 40xx), mixes,
# far/scale, chordal 0.0003 (flat areas now allowed) on 6 families; 5400 s cap per job.
V=$1; R=$2; J=${3:-6}; mkdir -p $R
{
for f in $(seq 0 23); do echo "fam_$f|40$f 25|VC_FAM=$f"; done
for f in 18; do echo "fam18b|4918 40|VC_FAM=18"; echo "fam18_ch01|4818 30|VC_FAM=18 VC_CH=0.1"; done
for s in 401 402; do echo "mix_$s|$s 40|"; done
echo "far2e4|4502 20|VC_FAR=20000"; echo "scale005|4504 30|VC_SCALE=0.05"; echo "scale20|4505 20|VC_SCALE=20"
for f in 1 6 10 14 16 18; do echo "ch3e4_fam$f|46$f 8|VC_CH=0.0003 VC_FAM=$f"; done
} | xargs -P $J -I{} sh -c 'n=$(echo "{}" | cut -d"|" -f1); a=$(echo "{}" | cut -d"|" -f2); e=$(echo "{}" | cut -d"|" -f3); env $e VCVIOL=1 VCDUMP=1 timeout 5400 stdbuf -oL '$V' $a > '$R'/$n.txt 2>&1; echo "exit $?" >> '$R'/$n.txt'
echo done
