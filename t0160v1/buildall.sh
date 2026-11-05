#!/bin/bash
# t0160v1: Release builds of branch (3fcccae) and main (f7151a3), plain and -march=x86-64-v3
H=/home/chris/harness/t0160v1; BR=/home/chris/wt-T0160; MA=/home/chris/wt-T0160v1-main
bash $H/buildtree.sh $BR $BR/build-verify1-rel3 Release "-march=x86-64-v3" &
bash $H/buildtree.sh $MA $MA/build-verify1-rel3 Release "-march=x86-64-v3" &
wait
bash $H/buildtree.sh $BR $BR/build-verify1-rel Release "" &
bash $H/buildtree.sh $MA $MA/build-verify1-rel Release "" &
wait
echo done > $H/buildall.done
