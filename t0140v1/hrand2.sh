#!/bin/bash
# hrand2.sh (VS="merged main" to pick variants; hcmp.py needs br_/base_ names: rename) OUT P "seeds" "modes" N : HASH=1 random pages (leads random and NOLEAD), variants br and base.
B=/home/chris/wt-T0140/build-verify1-rel/bin; O=$1; P=$2; S=$3; M=$4; N=${5:-50}; mkdir -p $O; echo $$ > $O/sweep.pid
for v in ${VS:-br base}; do for m in $M; do for s in $S; do for e in plain NOLEAD; do echo "$v $m $s $e"; done; done; done; done | \
 xargs -P $P -L 1 bash -c 'v=$0; m=$1; s=$2; e=$3; E=""; [ $e = NOLEAD ] && E="NOLEAD=1"; env $E HASH=1 timeout 3600 '$B'/t40_$v $s '$N' $m > '$O'/${v}_m${m}_${s}_$e.txt 2>&1 || echo "rc=$? $v $m $s $e"' >> $O/run.log 2>&1
echo ALLDONE >> $O/run.log
