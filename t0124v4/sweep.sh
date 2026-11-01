#!/bin/bash
# usage: sweep.sh chainId  -- runs 50-scene chunks; resumable (skips chunks whose log has SUMMARY)
cd /home/claude/vscratch/t0124v4/sw
export CELL=0.05
if [ "$1" = 1 ]; then
 cfgs=("A 0 -1 4 ONP=4" "C 2 -1 3 ONP=3 TLMIN=0.3 TLMAX=4" "E 8 -1 4 ONP=4" "G 3 -1 3 ONP=4" "I 1 -1 2 ONP=3")
else
 cfgs=("B 0 1 4 ONP=4 TLMIN=0.3 TLMAX=4" "D 6 -1 3 ONP=3 TLMIN=0.3 TLMAX=4" "F 5 -1 4 X=1" "H 8 1 2 ONP=4 TLMIN=0.3 TLMAX=4" "J 4 -1 2 ONP=3" "K 7 -1 2 ONP=3")
fi
for c in "${cfgs[@]}"; do
  set -- $c; n=$1; f=$2; sp=$3; chunks=$4; shift 4
  for ((k=0;k<chunks;k++)); do
    seed=$((81000 + $(printf '%d' "'$n") * 10 + k))
    log=sw_${n}_$seed.log
    grep -q SUMMARY $log 2>/dev/null && continue
    env "$@" ../conn4 $seed 50 $f $sp > $log.tmp 2>&1 && mv $log.tmp $log
  done
done
echo done > chain$1.done
