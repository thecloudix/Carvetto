#!/bin/bash
# mk.sh src.cpp out {br|main}  -- links a driver against the T-0142 verifier's Release (-march=x86-64-v3) builds
W=/home/chris/wt-T0142
if [ "$3" = main ]; then S=$W/build-verify1-mainsrc; B=$W/build-verify1-mainrel; else S=$W; B=$W/build-verify2-rel; fi
T=$(dirname $(readlink -f $0))
c++ $EXTRA -std=c++20 -O2 -march=x86-64-v3 -w -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I$S/core/include -I$S/core/src/geometry -I$S/core/src/toolpath -I$T -I/home/chris/harness/t0131v1 -I/home/chris/harness/t0141v1 -I$S/core/tests/toolpath -I$S/core/tests/geometry -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include "$1" -o "$2" $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a
