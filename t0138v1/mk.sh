#!/bin/bash
# mk.sh src.cpp out <builddir> <srcroot>
B=$3; W=$4
c++ -std=c++20 -O2 -w -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I$W/core/include -I/home/chris/harness/t0123v1 -I$W/core/tests/toolpath -I$W/core/tests -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include $EXTRA "$1" -o "$2" $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $(ls $B/core/libcarvetto_miniz.a 2>/dev/null)
