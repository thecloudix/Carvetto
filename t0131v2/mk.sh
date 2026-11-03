#!/bin/bash
# mk.sh src.cpp out -- links a t0131v1 driver against the round-2 branch Release build (build-verify2-rel)
W=/home/chris/wt-T0131; B=$W/build-verify2-rel
T=$(readlink -f $(dirname $(readlink -f $0))/../t0131v1)
c++ $EXTRA -std=c++20 -O2 -w -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I$W/core/include -I$T -I$W/core/tests/toolpath -I$W/core/tests/geometry -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include "$1" -o "$2" $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a
