#!/bin/bash
# mk.sh src.cpp name -> name (branch rel-new) and name_old (main rel-main)
H=/home/chris/harness/t0127v3; V=/home/chris/wt-T0129/build-verify
for v in new old; do
  if [ $v = new ]; then W=/home/chris/wt-T0129; B=$V/rel-new; o=$V/drv/$2; else W=$V/mainsrc; B=$V/rel-main; o=$V/drv/${2}_old; fi
  c++ -std=c++20 -O2 -w -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I$W/core/include -I$H -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include "$1" -o "$o" $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a || exit 1
done
