#!/bin/bash
# usage: mk.sh <src.cpp> <out>   driver vs branch Release lib (BLIB, default build-verify1-rel) + reference namespaced cvmain
#   REF=main (origin/main, build-verify1-mainns, default) | pre (8cd3568, build-verify1-prens) | brns (branch namespaced control)
W=/home/chris/wt-T0172; H=/home/chris/harness/t0172v1; H3=/home/chris/harness/t0009v3; D=$W/build-verify1-drv; mkdir -p $D
REF=${REF:-main}; B=${BLIB:-$W/build-verify1-rel}; R=$W/build-verify1-${REF}ns; MS=$W/build-verify1-${REF}src
[ $REF = brns ] && MS=$W && R=$W/build-verify1-brns
cd $D
[ -f mw_$REF.o ] || g++ -std=c++20 -O2 -Dcarvetto=cvmain -c $H/main_wrap.cpp -I$H -I$MS/core/include -o mw_$REF.o
g++ -std=c++20 -O2 -Wall $EXTRA -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable $H/$1 mw_$REF.o -I$H -I$H3 -I$W/core/include -I$W/core/tests/geometry $B/core/libcarvetto_core.a $R/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a -o $2 2>&1 | grep -E "error|multiple|undefined" | head -8
