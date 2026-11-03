#!/bin/bash
# perf.sh BIN OUT: branch scaling runs, two repetitions (order reversed in the second), one at a time
P=$1; OUT=$2; : > $OUT
run() { local e="$1"; shift; local r; r=$(env $e $P "$@" 2>&1); echo "$e | $r | load $(cut -d' ' -f1 /proc/loadavg)" >> $OUT; }
cases=()
for a in 0 30 45 90; do for n in 250 500 1000; do cases+=("ANG=$a|comb $n"); done; done
for n in 250 500 1000; do cases+=("RD=0.1 TOOL=1|comb $n"); cases+=("OFFSET=1|comb $n"); done
for v in 0 1 2; do for n in 1250 2500 5000; do cases+=("X=1|sep $n $v" "TABS=1|sep $n $v" "X=1|rest $n $v" "X=1|tabs $n $v"); done; done
for n in 1250 2500 5000; do cases+=("REV=1 SIDE=1|sep $n 0" "RASTER=1 ANG=30|rest $n 0" "RD=0.5|rest $n 2" "TABS=1 LEADS=1 ORDER=1|sep $n 1"); done
for rep in 1 2; do
  if [ $rep = 1 ]; then list=("${cases[@]}"); else list=(); for ((i=${#cases[@]}-1;i>=0;i--)); do list+=("${cases[$i]}"); done; fi
  for c in "${list[@]}"; do e=${c%%|*}; a=${c#*|}; run "$e" $a; done
done
echo DONE >> $OUT
