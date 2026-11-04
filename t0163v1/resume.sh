#!/bin/bash
# resume.sh BINDIR seed N S O: main crashed in the sweep; rerun main from the scene after each crash
# (FROM=k) until it finishes, appending to out/main_<seed>_<S>_<O>.txt; crash scenes are logged.
D=$1; s=$2; n=$3; S=$4; O=$5; f=$D/out/main_${s}_${S}_${O}.txt
while true; do
  last=$(grep -o "^r$s:[0-9]*" $f | /usr/bin/tail -n 1 | cut -d: -f2); next=$(( ${last:--1} + 2 ))  # last ok + 1 crashed -> resume at +2
  [ $next -ge $n ] && break
  echo "seed $s main crashed at scene $((next - 1)), resuming at $next" >> $D/out/resume.log
  FROM=$next timeout 3600 $D/hs_main $s $n $S $O >> $f 2>&1; rc=$?
  [ $rc = 0 ] && break
done
