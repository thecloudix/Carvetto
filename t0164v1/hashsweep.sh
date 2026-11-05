#!/bin/sh
# usage: hashsweep.sh <vc3 binary built with VC_HASH> <out dir> [jobs]: families 0..23 x 15 cases (seeds 50xx) + 2 mixes x 20,
# hash only (no oracle); compare two out dirs with: diff <(cat A/*.txt|grep HASH) <(cat B/*.txt|grep HASH)
V=$1; R=$2; J=${3:-4}; mkdir -p $R
{ for f in $(seq 0 23); do echo "fam_$f|50$f 15|VC_FAM=$f"; done; for s in 501 502; do echo "mix_$s|$s 20|"; done; } |
 xargs -P $J -I{} sh -c 'n=$(echo "{}" | cut -d"|" -f1); a=$(echo "{}" | cut -d"|" -f2); e=$(echo "{}" | cut -d"|" -f3); env $e VC_HASH=1 VC_HASHONLY=1 timeout 900 '$V' $a > '$R'/$n.txt 2>&1; echo "exit $?" >> '$R'/$n.txt'
