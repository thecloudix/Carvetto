#!/bin/sh
# usage: hashsweep.sh <vc3 binary> <out dir> <jobs> <cases per family> [env...]: families 0..23 (seeds 60xx) + 2 mixes, hash only.
# extra env (e.g. VC_CS=0, VC_FM=2, VC_INS=1 VC_INSONLY=1) passed to every run.
V=$1; R=$2; J=$3; N=$4; shift 4; E="$*"; mkdir -p $R
{ for f in $(seq 0 23); do echo "fam_$f|60$f $N|VC_FAM=$f"; done; for s in 601 602; do echo "mix_$s|$s $N|VC_X=1"; done; } |
 xargs -P $J -I{} sh -c 'n=$(echo "{}" | cut -d"|" -f1); a=$(echo "{}" | cut -d"|" -f2); e=$(echo "{}" | cut -d"|" -f3); env $e '"$E"' VC_HASH=1 VC_HASHONLY=1 timeout 1800 '$V' $a > '$R'/$n.txt 2>&1; echo "exit $?" >> '$R'/$n.txt'
