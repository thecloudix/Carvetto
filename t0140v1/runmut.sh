#!/bin/bash
# runmut.sh : builds each T-0140 mutant (mutants.py) into build-verify1-rel/mut (Release carvetto_tests from mutsrc) and runs the
# T-0140 tests; restores the original profile.cpp at the end.
R=/home/chris/wt-T0140/build-verify1-rel; H=/home/chris/harness/t0140v1; echo $$ > $R/runmut.pid
for m in ${MUTS:-M1_gate M2_clipperonly M3_arcinner M4_covered M5_nothick M6_attrib M7_islands M8_lowered M9_nowedge M10_nocap M11_reach M12_area}; do
  python3 $H/mutants.py $R/profile.orig.cpp $R/mutsrc/core/src/toolpath/profile.cpp $m || { echo "$m PATCHFAIL"; continue; }
  timeout 3000 cmake --build $R/mut -j1 --target carvetto_tests > $R/mut_$m.build.log 2>&1 || { echo "$m BUILDFAIL"; continue; }
  timeout 3000 $R/mut/core/tests/carvetto_tests "*T-0140*" > $R/mut_$m.test.log 2>&1; rc=$?
  echo "$m rc=$rc $(grep -E '^test cases' $R/mut_$m.test.log | tr -s ' ')"
done
cp $R/profile.orig.cpp $R/mutsrc/core/src/toolpath/profile.cpp
echo ALLDONE
