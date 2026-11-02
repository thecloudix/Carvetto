#!/bin/bash
# usage: mk.sh <src.cpp> <out> <worktree> <builddir>
WT=$3; B=$4
g++ -std=c++20 -O1 -I$WT/core/include -I$WT/core/tests/post -I$WT/core/tests -I$WT/core/src/post -I$B/_deps/nlohmann_json-src/include "$1" -o "$2" $B/core/libcarvetto_core.a $B/core/libcarvetto_miniz.a $B/_deps/clipper2-build/libClipper2.a
