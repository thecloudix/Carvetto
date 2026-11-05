#!/bin/sh
# t0164v1: build the drivers against branch (ff45327) and main (6de8648) Release -march=x86-64-v3 libs, and the sabotaged ones
H=/home/chris/harness/t0164v1; S=/home/chris/scratch-t0164v1; mkdir -p $S/bin
BR=/home/chris/wt-T0164; BRB=$BR/build-verify1-rel; MA=/home/chris/wt-T0164-main; MAB=$MA/build-verify1-rel
for d in vc3 net fuzz6 grow perf; do
  [ -n "$1" ] && [ "$1" != "rel" ] && continue
  sh $H/build.sh $BR $BRB $H/$d.cpp $S/bin/${d}_br & sh $H/build.sh $MA $MAB $H/$d.cpp $S/bin/${d}_main &
done; wait
if [ "$1" = "sab" ]; then
  for d in vc3 net; do sh $H/build.sh $S/src-sab $S/src-sab/b-rel $H/$d.cpp $S/bin/${d}_sab & sh $H/build.sh $S/src-sab-main $S/src-sab-main/b-rel $H/$d.cpp $S/bin/${d}_sabmain & done; wait
fi
ls -la $S/bin
