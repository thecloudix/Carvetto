#!/bin/bash
# runmut.sh : Release carvetto_tests from mutsrc (build dir mut), baseline then each mutant; runs the T-0140 tests ("*T-0140*").
R=/home/chris/wt-T0140/build-verify2-rel; H=/home/chris/harness/t0140v2; cd $R; echo $$ > runmut.pid
[ -f mut/build.ninja ] || cmake -S mutsrc -B mut -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF > mut.cfg.log 2>&1
cp origsrc/core/src/toolpath/*.cpp mutsrc/core/src/toolpath/
timeout 5400 cmake --build mut -j${J:-1} --target carvetto_tests > mut_base.build.log 2>&1 || { echo "BASE BUILDFAIL"; exit 1; }
timeout 3000 mut/core/tests/carvetto_tests "*T-0140*" > mut_base.test.log 2>&1; echo "base rc=$? $(grep -E '^test cases' mut_base.test.log | tr -s ' ')"
for m in ${MUTS:-E1_nojoin E2_touchjoin E3_nowinding E4_nocrop E5_noislands E6_noframe E7_nofallback E8_firstpart E9_largest E10_no45rule E11_minside0 E12_walkflip E13_windsign E14_enters0 E15_nocentre E16_sideonly1 P1_gate P2_closedopen P3_nothick P4_attrib P5_area P6_reach P7_covered P8_nowedge P9_closing}; do
  python3 $H/mutants2.py mutsrc origsrc $m || { echo "$m PATCHFAIL"; continue; }
  timeout 3000 cmake --build mut -j${J:-1} --target carvetto_tests > mut_$m.build.log 2>&1 || { echo "$m BUILDFAIL"; continue; }
  timeout 3000 mut/core/tests/carvetto_tests "*T-0140*" > mut_$m.test.log 2>&1; rc=$?
  echo "$m rc=$rc $(grep -E '^test cases' mut_$m.test.log | tr -s ' ')"
done
cp origsrc/core/src/toolpath/*.cpp mutsrc/core/src/toolpath/
echo ALLDONE
