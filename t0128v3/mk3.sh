#!/bin/bash
# usage: [BLIB=dir-or-lib.a] [MREF=main|old|br] mk3.sh <src.cpp> <out>
# driver vs BLIB (default build-verify3-rel lib) + reference namespaced cvmain: main=origin/main e9bf85e, old=e87f549, br=cae3fa7.
W=/home/chris/wt-T0128; H=/home/chris/harness/t0128v3; H3=/home/chris/harness/t0009v3; D=$W/build-verify3-drv; mkdir -p $D
B=$W/build-verify3-rel; BL=${BLIB:-$B/core/libcarvetto_core.a}
REF=${MREF:-main}; R=$W/build-verify3-${REF}ns; MS=$W/build-verify3-${REF}src; [ $REF = br ] && MS=$W
cd $D
[ -f mw_$REF.o ] || g++ -std=c++20 -O2 -Dcarvetto=cvmain -c $H/main_wrap.cpp -I$H -I$MS/core/include -o mw_$REF.o
g++ -std=c++20 -O2 -Wall $EXTRA -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable $H/$1 mw_$REF.o -I$H -I$H3 -I$W/core/include -I$W/core/tests/geometry $BL $R/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a $LIBS -o $2 2>&1 | grep -E "error" | head -20
