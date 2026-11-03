#!/bin/bash
# mk.sh src.cpp name -> drv/bin/name (branch) and drv/bin/name_old (base 2d2c23a)
V=/home/chris/wt-T0143/build-verify; O=$V/drv/bin
for v in new old; do
  if [ $v = new ]; then W=/home/chris/wt-T0143; B=$V/rel; o=$O/$2; else W=$V/main-src; B=$V/main-rel; o=$O/${2}_old; fi
  c++ -std=c++20 -O2 -w -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I$W/core/include -I${HINC:-.} -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include "$1" -o "$o" $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a || exit 1
done
