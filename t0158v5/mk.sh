#!/bin/bash
# mk.sh src.cpp name -> $OUT/name_<variant>; links against /home/chris/wt-T0158/build-verify5-rel/<variant> (mkall5.sh).
# Harness code compiled with -ffp-contract=off in every variant (bit-identical scenes); only the library differs.
R=/home/chris/wt-T0158/build-verify5-rel; OUT=${OUT:-$R/bin}; mkdir -p $OUT
T=$(dirname $(readlink -f $0))
for v in ${VARIANTS:-br5 main br5fma mainfma r4}; do
  case $v in logfma) W=$R/logfmasrc;; log) W=$R/logsrc;; br5fma) W=$R/br5fmasrc;; br5*) W=/home/chris/wt-T0158;; mainfma) W=$R/mainfmasrc;; main*) W=$R/mainsrc;; r4*) W=$R/r4src;; mut*) W=$R/mutsrc;; esac
  B=$R/$v
  c++ -ffp-contract=off -std=c++20 -O2 -w -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I$W/core/include -I$T -I$W/core/tests/toolpath -I$W/core/tests/geometry -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include "$1" -o $OUT/${2}_$v $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a || exit 1
done
