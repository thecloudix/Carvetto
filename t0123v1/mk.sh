#!/bin/bash
# mk.sh src.cpp out worktree  (links against <worktree>/build-verify/rel)
W=$3; B=$W/build-verify/rel; T=$(dirname $(readlink -f $0))
c++ $EXTRA -std=c++20 -O2 -w -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I$W/core/include -I$T -I$W/core/tests/toolpath -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include "$1" -o "$2" $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a
