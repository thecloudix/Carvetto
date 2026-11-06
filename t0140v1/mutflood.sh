#!/bin/bash
# mutflood.sh : for mutants M4_covered, M9_nowedge: build the mutant core lib (build-verify1-rel/mut), link t40_mut, flood mode 5 seeds
# 1410001-1410002 (N 50, leads off) -> build-verify1-rel/mutfl/<M>_<seed>.txt; restores profile.cpp.
R=/home/chris/wt-T0140/build-verify1-rel; H=/home/chris/harness/t0140v1; O=$R/mutfl; mkdir -p $O; echo $$ > $O/pid
for m in ${MUTS:-M9_nowedge M4_covered}; do
  python3 $H/mutants.py $R/profile.orig.cpp $R/mutsrc/core/src/toolpath/profile.cpp $m || continue
  timeout 3000 cmake --build $R/mut -j1 --target carvetto_core > $O/$m.build.log 2>&1 || { echo "$m BUILDFAIL"; continue; }
  OUT=$R/bin VARIANTS=mut $H/mk.sh $H/t40.cpp t40 || continue
  for s in 1410001 1410002; do NOLEAD=1 timeout 7200 $R/bin/t40_mut $s 50 5 > $O/${m}_$s.txt 2>&1; done
done
cp $R/profile.orig.cpp $R/mutsrc/core/src/toolpath/profile.cpp
echo ALLDONE > $O/done
