#!/bin/sh
# usage: runmut.sh <mutant names...>: builds each T-0169 mutant of pocket.cpp into the mutant source tree (Release)
# and runs carvetto_tests "[pocket]" and "[t0169]"; prints failing test names per mutant.
M=/home/chris/wt-T0169/build-verify1-mutsrc; ORIG=/home/chris/wt-T0169/core/src/toolpath/pocket.cpp; H=/home/chris/harness/t0169v1; OUT=$H/results/mut; mkdir -p $OUT
for m in "$@"; do
  if [ $m = none ]; then cp $ORIG $M/core/src/toolpath/pocket.cpp; else python3 $H/mutants.py $ORIG $m $M/core/src/toolpath/pocket.cpp || { echo "$m APPLYFAIL"; continue; }; fi
  if ! timeout 1200 cmake --build $M/build-verify1-mut --target carvetto_tests -j2 > $OUT/$m.build.log 2>&1; then echo "$m BUILDFAIL"; continue; fi
  timeout 2400 $M/build-verify1-mut/core/tests/carvetto_tests "[pocket],[t0169]" > $OUT/$m.test.log 2>&1; rc=$?
  fails=$(grep -B1 -h "^.*FAILED:" $OUT/$m.test.log | head -0; grep -h "^[^ ].*" $OUT/$m.test.log | grep -c FAILED)
  echo "$m rc=$rc $(grep -E '^test cases:' $OUT/$m.test.log)"
  grep -h -A3 "^-------" $OUT/$m.test.log | grep -v "^---\|^\.\.\.\|^$\|^/" | sort -u | head -12 | sed 's/^/    /'
done
cp $ORIG $M/core/src/toolpath/pocket.cpp
