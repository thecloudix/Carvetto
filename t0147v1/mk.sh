#!/bin/bash
# mk.sh src name -> name_new / name_old
for v in new old; do
 if [ $v = new ]; then W=/home/chris/wt-T0147; B=$W/build-verify/rel; else W=/home/chris/wt-T0147/build-verify/base-src; B=/home/chris/wt-T0147/build-verify/relbase; fi
 c++ $EXTRA -std=c++20 -O2 -w -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I$W/core/include -I$(dirname $1) -I$W/core/tests/toolpath -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include "$1" -o ${2}_$v $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a || exit 1
done
