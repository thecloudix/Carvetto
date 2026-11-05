#!/bin/bash
# usage: mk.sh br <src.cpp> <out>
#   br: branch task/T-0009-subtol-r4 eab77ce Release lib (build-verify4-rel)
#       + origin/main e2e5345 namespaced cvmain (build-verify4-mainns, source build-verify4-mainsrc)
W=/home/chris/wt-T0009r4; H=/home/chris/harness/t0009v4; D=$W/build-verify4-drv; mkdir -p $D
case $1 in
  br) SRC=$W; B=$W/build-verify4-rel; R=$W/build-verify4-mainns; MS=$W/build-verify4-mainsrc;;
esac
cd $D; S2=$(realpath -m $H/$2)
[ -f mw_$1.o ] || g++ -std=c++20 -O2 -Dcarvetto=cvmain -c $H/main_wrap.cpp -I$H -I$MS/core/include -o mw_$1.o
g++ -std=c++20 -O2 -Wall $EXTRA -Wno-unused-function -Wno-unused-variable $S2 mw_$1.o -I$H -I$SRC/core/include -I$SRC/core/tests/geometry $B/core/libcarvetto_core.a $R/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a -o ${3} 2>&1 | grep -E "error" | head -20
