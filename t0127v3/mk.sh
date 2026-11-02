#!/bin/bash
# mk.sh src.cpp name [lib] -> bin/name (branch build-verify3/rel) and bin/name_old (main e4da396). LIB=dir overrides branch lib (mutant)
H=/home/chris/harness/t0127v3; O=/home/chris/wt-T0127/build-verify3/drv/bin
for v in new old; do
  if [ $v = new ]; then W=/home/chris/wt-T0127; B=$W/build-verify3/rel; o=$O/$2; L=${LIB:-$B/core}; else [ -n "$NEWONLY" ] && continue; W=/home/chris/wt-main-T0127; B=$W/build-rel; o=$O/${2}_old; L=$B/core; fi
  c++ -std=c++20 -O2 -w -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I$W/core/include -I$H -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include "$1" -o "$o" $L/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a || exit 1
done
