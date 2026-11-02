#!/bin/bash
# mk.sh src.cpp name -> bin/name (worktree build-hr) and bin/name_old (main build-hr)
H=$(dirname $(readlink -f $0)); S=$(dirname $H)
for v in new old; do
  if [ $v = new ]; then W=/home/chris/wt-T0127; o=$H/bin/$2; else W=$S/main-src; o=$H/bin/${2}_old; fi
  [ "$NEWONLY" = 1 ] && [ $v = old ] && continue
  B=$W/build-hr
  c++ -std=c++20 -O2 -w -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I$W/core/include -I$H -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include "$1" -o "$o" $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a || exit 1
done
