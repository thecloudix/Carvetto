#!/bin/sh
# usage: build.sh <src.cpp> <dbg|rel|asan|r2>
W=/home/chris/wt-T0132; B=$W/build-verify3; SRC=$W
case "$2" in
 rel) LIB=$B/rel; OPT="-O2 -DNDEBUG";;
 r2) LIB=$B/r2rel; OPT="-O2 -DNDEBUG"; SRC=$B/r2src;;
 asan) LIB=$B/asan; OPT="-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer";;
 *) LIB=$B; OPT="-g -O0";;
esac
out=$B/drv/$(basename $1 .cpp)-$2
mkdir -p $B/drv
g++ -std=c++20 $OPT -I$SRC/core/include -I$SRC/core/src/document -isystem $LIB/_deps/miniz-src -isystem $LIB/_deps/nlohmann_json-src/include -isystem $LIB/_deps/clipper2-src/CPP/Clipper2Lib/include "$1" -o $out $LIB/core/libcarvetto_core.a $LIB/core/libcarvetto_miniz.a $LIB/_deps/clipper2-build/libClipper2.a && echo $out
