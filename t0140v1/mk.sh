#!/bin/bash
# mk.sh src.cpp name -> $OUT/name_<variant>; links against /home/chris/wt-T0140/build-verify1-rel/<variant> (mkall.sh there):
# br = task/T-0140 334a87d (worktree), base = 51c2dd6 (merge base, T-0158 merged), main = origin/main b637c16, merged = git merge-tree result 113b836, t135 = 1c36940 (T-0135 merge).
R=/home/chris/wt-T0140/build-verify1-rel; OUT=${OUT:-$R/bin}; mkdir -p $OUT
T=$(dirname $(readlink -f $0))
for v in ${VARIANTS:-br base}; do
  case $v in br*) W=/home/chris/wt-T0140;; base*) W=$R/basesrc;; main*) W=$R/mainsrc;; merged*) W=$R/mergedsrc;; t135*) W=$R/t135src;; mut*) W=$R/${v}src;; esac
  B=$R/$v
  c++ -ffp-contract=off -std=c++20 -O2 -w -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I$W/core/include -I$T -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include "$1" -o $OUT/${2}_$v $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a || exit 1
done
