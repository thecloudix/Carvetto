#!/bin/sh
# t0160v1: vc3 cone/end-mill/structure oracle with flat depth + clearance tools forced (VC_FM=2), branch (sharpening on by default).
# usage: conesweep.sh <vc3> <out dir> <jobs> <N per family> [extra env]
V=$1; R=$2; J=$3; N=$4; shift 4; E="$*"; mkdir -p $R
{ for f in $(seq 0 23); do echo "fam_$f|61$f $N|VC_FAM=$f"; done; } |
 xargs -P $J -I{} sh -c 'n=$(echo "{}" | cut -d"|" -f1); a=$(echo "{}" | cut -d"|" -f2); e=$(echo "{}" | cut -d"|" -f3); env $e VC_FM=2 '"$E"' VCVIOL=1 timeout 3000 '$V' $a > '$R'/$n.txt 2>&1; echo "exit $?" >> '$R'/$n.txt'
