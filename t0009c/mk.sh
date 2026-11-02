#!/bin/bash
# usage: mk.sh <src.cpp> <out>
W=/home/chris/wt-T0009; B=$W/build-rel; R=$W/build-h/ref-ns
cd $W/build-h/drv && g++ -std=c++20 -O2 -Wall ${1:-vdrv4.cpp} main_wrap.o -I$W/core/include -I. -I$W/core/tests/geometry $B/core/libcarvetto_core.a $R/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a -o ${2:-vdrv4} 2>&1 | grep -E "error|warning" | head -20
