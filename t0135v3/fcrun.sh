#!/bin/bash
# fcrun.sh seed sc mode tag : floods one scene with branch/base/main
cd /home/chris/wt-T0135/build-verify3/drv
s=$1; c=$2; m=$3; tag=$4; env=$(echo $tag | sed 's/_\([^_]*\)$/=\1/')
for b in wsf8w wsf8w_base wsf8w_main; do ./fcand.sh $b $s $c $m $env; done
