#!/bin/bash
# usage: mk.sh <variant: br|mg> <src.cpp> <out>
#   br: branch Release lib (build-verify2-rel) + base-namespaced main (60413ec, cvmain)
#   mg: merged (branch + origin/main) Release lib + origin/main namespaced (cvmain)
W=/home/chris/wt-T0009; H=/home/chris/harness/t0009v2; D=$W/build-verify2-drv; mkdir -p $D
case $1 in
  br) SRC=$W; B=$W/build-verify2-rel; R=$W/build-verify2-basens; MS=$W/build-verify2-basesrc;;
  mg) SRC=$W/build-verify2-merge/src; B=$W/build-verify2-merge/rel; R=$W/build-verify2-mainns; MS=$W/build-verify2-mainsrc;;
esac
cd $D; S2=$(realpath -m $H/$2)
[ -f mw_$1.o ] || g++ -std=c++20 -O2 -Dcarvetto=cvmain -c $H/main_wrap.cpp -I$H -I$MS/core/include -o mw_$1.o
g++ -std=c++20 -O2 -Wall $EXTRA -Wno-unused-function -Wno-unused-variable $S2 mw_$1.o -I$H -I$SRC/core/include -I$SRC/core/tests/geometry $B/core/libcarvetto_core.a $R/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a -o ${3} 2>&1 | grep -E "error" | head -20
