#!/bin/bash
# runs one job for variants base, br, in
cd $(dirname $(readlink -f $0)); mkdir -p ab
read d s n m <<<"$*"
for v in base br in; do timeout 3000 ./${d}_$v $s $n $m > ab/${d}_${s}_$v.txt 2>&1; done
echo "done $d $s"
