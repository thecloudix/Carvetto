#!/bin/bash
# mk.sh src.cpp name -> $OUT/name_<variant>; links against /home/chris/wt-T0158/build-verify3-rel/<variant> (mkall.sh).
# Harness code is compiled with -ffp-contract=off in every variant so generated scenes are bit-identical; only the
# library differs (v3 = library built with -march=x86-64-v3).
R=/home/chris/wt-T0158/build-verify3-rel; OUT=${OUT:-$R/bin}; mkdir -p $OUT
T=$(dirname $(readlink -f $0))
for v in ${VARIANTS:-log logv3 br brv3 main mainv3}; do
  case $v in br*) W=/home/chris/wt-T0158;; main*) W=$R/mainsrc;; mlog*) W=$R/mlogsrc;; log*) W=$R/logsrc;; esac
  F=; case $v in *v3) F=-march=x86-64-v3;; esac
  B=$R/$v
  c++ $F -ffp-contract=off -std=c++20 -O2 -w -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I$W/core/include -I$T -I$W/core/tests/toolpath -I$W/core/tests/geometry -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include "$1" -o $OUT/${2}_$v $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a || exit 1
done
