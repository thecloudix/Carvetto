#!/bin/sh
# usage: mutlib.sh <mutant names...>: builds carvetto_core with each mutant (mutant source tree, Release) and links
# pk against it: build-verify1-bin/pk_mut_<name>
M=/home/chris/wt-T0169/build-verify1-mutsrc; ORIG=/home/chris/wt-T0169/core/src/toolpath/pocket.cpp; H=/home/chris/harness/t0169v1; BIN=/home/chris/wt-T0169/build-verify1-bin; MB=$M/build-verify1-mut
for m in "$@"; do
  python3 $H/mutants.py $ORIG $m $M/core/src/toolpath/pocket.cpp || continue
  timeout 1200 cmake --build $MB --target carvetto_core -j2 > /dev/null 2>&1 || { echo "$m BUILDFAIL"; continue; }
  g++ -std=c++20 -O2 -w -I$M/core/include -I$H -isystem $MB/_deps/clipper2-src/CPP/Clipper2Lib/include -o $BIN/pk_mut_$m $H/pk.cpp $MB/core/libcarvetto_core.a $MB/_deps/clipper2-build/libClipper2.a $MB/core/libcarvetto_miniz.a -lpthread && echo "$m ok"
done
cp $ORIG $M/core/src/toolpath/pocket.cpp
