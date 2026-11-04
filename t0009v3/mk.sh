#!/bin/bash
# usage: mk.sh <variant: br|mg> <src.cpp> <out>
#   br: branch Release lib (build-verify3-rel; branch 8b4e69e already contains origin/main 0fa0699)
#       + origin/main 0fa0699 namespaced cvmain (build-verify3-mainns)
W=/home/chris/wt-T0009; H=/home/chris/harness/t0009v3; D=$W/build-verify3-drv; mkdir -p $D
case $1 in
  br) SRC=$W; B=$W/build-verify3-rel; R=$W/build-verify3-mainns; MS=$W/build-verify3-mainsrc;;
esac
cd $D; S2=$(realpath -m $H/$2)
[ -f mw_$1.o ] || g++ -std=c++20 -O2 -Dcarvetto=cvmain -c $H/main_wrap.cpp -I$H -I$MS/core/include -o mw_$1.o
g++ -std=c++20 -O2 -Wall $EXTRA -Wno-unused-function -Wno-unused-variable $S2 mw_$1.o -I$H -I$SRC/core/include -I$SRC/core/tests/geometry $B/core/libcarvetto_core.a $R/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a -o ${3} 2>&1 | grep -E "error" | head -20
