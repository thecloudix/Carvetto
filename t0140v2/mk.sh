#!/bin/bash
# mk.sh src.cpp name -> $OUT/name_<variant>; links against /home/chris/wt-T0140/build-verify2-rel/<variant>:
# br = task/T-0140 40ccd46 (worktree), base = fa856ec (merge base with origin/main 4bb7542; Profile identical to main).
R=/home/chris/wt-T0140/build-verify2-rel; OUT=${OUT:-$R/bin}; mkdir -p $OUT
T=$(dirname $(readlink -f $0))
for v in ${VARIANTS:-br base}; do
  case $v in br*) W=/home/chris/wt-T0140;; base*) W=$R/basesrc;; mut*|tim*|dbg*) W=$R/${v}src;; esac
  B=$R/$v
  c++ -ffp-contract=off -std=c++20 -O2 -w -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I$W/core/include -I$W/core/src/toolpath -I$T -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include "$1" -o $OUT/${2}_$v $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a || exit 1
done
