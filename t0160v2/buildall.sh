#!/bin/bash
# t0160v2: Debug -Werror (branch) + Release plain/v3 libs for branch (923d66c) and base (681327a)
H=/home/chris/harness/t0160v2; BR=/home/chris/wt-T0160; MA=/home/chris/wt-T0160v2-base
bash $H/buildtree.sh $BR $BR/build-verify2 Debug "" 3 &
bash $H/buildtree.sh $BR $BR/build-verify2-rel Release "" 2 &
bash $H/buildtree.sh $MA $MA/build-verify2-rel Release "" 2 &
wait
bash $H/buildtree.sh $BR $BR/build-verify2-rel3 Release "-march=x86-64-v3" 3 &
bash $H/buildtree.sh $MA $MA/build-verify2-rel3 Release "-march=x86-64-v3" 3 &
wait
echo done > $H/buildall.done
