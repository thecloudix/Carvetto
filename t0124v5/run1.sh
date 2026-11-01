#!/bin/bash
# run1.sh bin tag seed fam sep env...
H=$(dirname $(readlink -f $0)); cd $H/sw
b=$1; tag=$2; seed=$3; f=$4; sp=$5; shift 5
log=${tag}_${b}.log
grep -q SUMMARY $log 2>/dev/null && exit 0
env CELL=0.05 "$@" ../bin/$b $seed 50 $f $sp > $log.tmp 2>&1 && mv $log.tmp $log
