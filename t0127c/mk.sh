#!/bin/bash
# mk.sh src.cpp outname   -> builds bin/outname (new: worktree build-hr) and bin/outname_old (main build-hr)
H=$(dirname $(readlink -f $0)); S=$(dirname $H)
src=$1; out=$2
for v in new old; do
  if [ $v = new ]; then W=/home/chris/wt-T0127; o=$H/bin/$out; else W=$S/main-src; o=$H/bin/${out}_old; fi
  B=$W/build-hr
  c++ -std=c++20 -O2 -w -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I$W/core/include -I$H/tools -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include "$src" -o "$o" $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a || exit 1
done
