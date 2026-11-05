#!/bin/sh
# t0164v2: build the drivers against branch (cce938c) and main (origin/main 6e537a1) Release -march=x86-64-v3 libs, and the sabotaged ones
H=/home/chris/harness/t0164v2; S=/home/chris/scratch-t0164v2; mkdir -p $S/bin
BR=/home/chris/wt-T0164; BRB=$BR/build-verify2-rel; MA=/home/chris/wt-T0164v2-main; MAB=$MA/build-verify2-rel
if [ -z "$1" ] || [ "$1" = rel ]; then
for d in vc3 net fuzz6 grow grow2 perf chunk; do
  sh $H/build.sh $BR $BRB $H/$d.cpp $S/bin/${d}_br & sh $H/build.sh $MA $MAB $H/$d.cpp $S/bin/${d}_main &
done; wait; fi
if [ "$1" = "sab" ]; then
  for d in vc3 net fuzz6; do sh $H/build.sh $S/src-sab $S/src-sab/b-rel $H/$d.cpp $S/bin/${d}_sab & sh $H/build.sh $S/src-sabmain $S/src-sabmain/b-rel $H/$d.cpp $S/bin/${d}_sabmain & done; wait
fi
if [ "$1" = "instr" ]; then for d in grow2; do sh $H/build.sh $S/src-instr $S/src-instr/b-rel $H/$d.cpp $S/bin/${d}_instr; done; fi
ls -la $S/bin
