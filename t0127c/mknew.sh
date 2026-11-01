#!/bin/bash
# rebuild only the "new" binaries
H=$(dirname $(readlink -f $0)); W=/home/chris/wt-T0127; B=$W/build-hr
for t in "$@"; do c++ -std=c++20 -O2 -w -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I$W/core/include -I$H/tools -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include tools/$t.cpp -o bin/$t $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a & done; wait
