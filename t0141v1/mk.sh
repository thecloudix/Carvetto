#!/bin/bash
# mk.sh src.cpp name [main]  (branch: rel ; main: mainrel)
W=/home/chris/wt-T0141; if [ "$3" = main ]; then B=$W/build-verify/mainrel; S=$W/build-verify/mainsrc; else B=$W/build-verify/rel; S=$W; fi
c++ -std=c++20 -O2 -w -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I$S/core/include -I$S/core/src/geometry -I. -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include "$1" -o "$2" $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a
