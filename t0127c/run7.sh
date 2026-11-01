#!/bin/bash
# run7.sh <job line>  (resumable)
H=$(dirname $(readlink -f $0)); cd $H
kind=$1; shift
if [ $kind = conn ]; then
  b=$1; tag=$2; seed=$3; f=$4; sp=$5; shift 5
  log=sw/${tag}_${b}.log
  grep -q SUMMARY $log 2>/dev/null && exit 0
  env CELL=0.05 "$@" ./bin/$b $seed 50 $f $sp > $log.tmp 2>&1 && mv $log.tmp $log
else
  tag=$1; seed=$2; f=$3; sp=$4; N=$5; shift 5
  for v in new old; do b=./bin/nontab3; [ $v = old ] && b=./bin/nontab3_old
    o=cmp/${v}_$tag.txt; [ -s $o ] && continue
    env "$@" $b $seed $N $f $sp > $o.tmp 2>&1 && mv $o.tmp $o
  done
fi
