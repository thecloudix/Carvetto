#!/bin/sh
# usage: mk.sh <src.cpp> <out> <br|main> [extra flags]: links against the branch / main Release core
case $3 in br) W=/home/chris/wt-T0169; B=$W/build-verify1-rel;; main) W=/home/chris/wt-T0169/build-verify1-mainsrc; B=$W/build-verify1-mrel;; esac
g++ -std=c++20 -O2 $4 -w -I$W/core/include -I$W/core/src -I$W/core/src/toolpath -I$W/core/tests -I$(dirname $0) -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -o $2 $1 $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a -lpthread
