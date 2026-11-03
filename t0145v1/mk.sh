#!/bin/bash
# mk.sh src name extra-flags -> bin/name_base, bin/name_new
set -e; mkdir -p bin
for v in base new; do
  if [ $v = base ]; then W=/home/chris/wt-T0145/build-verify/basesrc; else W=/home/chris/wt-T0145; fi
  B=/home/chris/wt-T0145/build-verify/rel-$v
  c++ -std=c++20 -O2 -w $3 -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I$W/core/include -I$(dirname $1) -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include "$1" -o bin/${2}_$v $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a
done
