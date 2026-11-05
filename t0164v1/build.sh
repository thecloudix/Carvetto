#!/bin/sh
# usage: build.sh <source tree> <release build dir> <src.cpp> <out>  (links libcarvetto_core.a + Clipper2 + miniz)
W=$1; B=$2; g++ -std=c++20 -O2 -I$W/core/include -I$W/core/src/toolpath -I$B/_deps/clipper2-src/CPP/Clipper2Lib/include -o $4 $3 $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a -lpthread
