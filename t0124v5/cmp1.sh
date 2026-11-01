#!/bin/bash
H=$(dirname $(readlink -f $0)); cd $H/cmp
tag=$1; seed=$2; f=$3; sp=$4; shift 4
N=50; [[ $tag == c_* ]] && N=120
for v in new old; do b=../bin/nontab2; [ $v = old ] && b=../bin/nontab2_old
  [ -s ${v}_$tag.txt ] && continue
  env "$@" $b $seed $N $f $sp > ${v}_$tag.tmp 2>&1 && mv ${v}_$tag.tmp ${v}_$tag.txt; done
