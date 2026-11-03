#!/bin/bash
# mk.sh src.cpp name -> bin/name (branch 42d38af), bin/name_base (6991133), bin/name_main (main toolpath == origin/main)
D=/home/chris/wt-T0135/build-verify2/drv; H=$D
for v in new base main; do
  case $v in
    new) W=/home/chris/wt-T0135; B=$W/build-verify2/rel; o=$D/bin/$2; L=${LIB:-$B/core};;
    base) [ -n "$NEWONLY" ] && continue; W=/home/chris/wt-T0135/build-verify/base-src; B=/home/chris/wt-T0135/build-verify/base-rel; o=$D/bin/${2}_base; L=$B/core;;
    main) [ -n "$NEWONLY" ] && continue; W=/home/chris/wt-main-T0135; B=$W/build-rel; o=$D/bin/${2}_main; L=$B/core;;
  esac
  c++ -std=c++20 -O2 -w -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I$W/core/include -I$H -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include "$1" -o "$o" $L/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a || exit 1
done
