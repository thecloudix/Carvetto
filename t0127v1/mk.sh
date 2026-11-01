#!/bin/bash
# mk.sh src.cpp name -> bin/name (branch, build-verify/rel) and bin/name_old (main e4da396, wt-main-T0127/build-rel)
H=/home/chris/wt-T0127/build-verify/drv
for v in new old; do
  if [ $v = new ]; then W=/home/chris/wt-T0127; B=$W/build-verify/rel; o=$H/bin/$2; else W=/home/chris/wt-main-T0127; B=$W/build-rel; o=$H/bin/${2}_old; fi
  c++ -std=c++20 -O2 -w $EXTRA -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I$W/core/include -I$H -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include "$1" -o "$o" $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a || exit 1
done
