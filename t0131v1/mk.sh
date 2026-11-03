#!/bin/bash
# mk.sh src.cpp out {br|main}  -- links against the T-0131 verifier's Release builds
S=/tmp/claude-1000/-home-chris-Carvetto/120b15af-a822-46aa-8270-71a70ec2ea48/scratchpad
if [ "$3" = main ]; then W=$S/main-src; B=$S/main-rel; else W=/home/chris/wt-T0131; B=$W/build-verify1-rel; fi
T=$(dirname $(readlink -f $0))
c++ $EXTRA -std=c++20 -O2 -w -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I$W/core/include -I$T -I$W/core/tests/toolpath -I$W/core/tests/geometry -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include "$1" -o "$2" $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a
