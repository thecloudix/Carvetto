#!/bin/bash
# t0164v2: all builds (branch Debug -Werror, branch/main Release x86-64-v3, sabotaged branch/main Release v3)
H=/home/chris/harness/t0164v2; S=/home/chris/scratch-t0164v2; BR=/home/chris/wt-T0164; MA=/home/chris/wt-T0164v2-main
bash $H/buildtree.sh $BR $BR/build-verify2 Debug "" &
bash $H/buildtree.sh $BR $BR/build-verify2-rel Release "-march=x86-64-v3" &
bash $H/buildtree.sh $MA $MA/build-verify2-rel Release "-march=x86-64-v3" &
wait
bash $H/buildtree.sh $S/src-sab $S/src-sab/b-rel Release "-march=x86-64-v3" &
bash $H/buildtree.sh $S/src-sabmain $S/src-sabmain/b-rel Release "-march=x86-64-v3" &
wait
echo done > $S/buildall.done
