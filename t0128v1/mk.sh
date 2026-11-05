#!/bin/bash
# usage: mk.sh <src.cpp> <out>   branch Release lib (BLIB, default build-verify1-rel) + origin/main namespaced cvmain (build-verify1-mainns)
W=/home/chris/wt-T0128; H=/home/chris/harness/t0128v1; H3=/home/chris/harness/t0009v3; D=$W/build-verify1-drv; mkdir -p $D
B=${BLIB:-$W/build-verify1-rel}; R=${MLIB:-$W/build-verify1-mainns}; MS=$W/build-verify1-mainsrc
cd $D
[ -f mw.o ] || g++ -std=c++20 -O2 -Dcarvetto=cvmain -c $H/main_wrap.cpp -I$H -I$MS/core/include -o mw.o
g++ -std=c++20 -O2 -Wall $EXTRA -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable $H/$1 mw.o -I$H -I$H3 -I$W/core/include -I$W/core/tests/geometry $B/core/libcarvetto_core.a $R/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a -o $2 2>&1 | grep -E "error" | head -20
