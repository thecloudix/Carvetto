#!/bin/bash
# mk.sh src.cpp out {br|main}  -- links against the T-0137 verifier's Release builds (build-verify1-rel/{br,main})
W0=/home/chris/wt-T0137; R=$W0/build-verify1-rel
if [ "$3" = main ]; then W=$R/mainsrc; B=$R/main; else W=$W0; B=$R/br; fi
T=$(dirname $(readlink -f $0))
c++ $EXTRA -std=c++20 -O2 -w -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I$W/core/include -I$T -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include "$1" -o "$2" $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a
