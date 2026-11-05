#!/bin/bash
# mk.sh src.cpp name -> $OUT/name_<variant> for variants (default: log logv3 br brv3 main mainv3)
# Links against /home/chris/wt-T0158/build-verify1-rel/<variant> (built by that dir's mkall.sh):
#   br/brv3 = branch 18f0d20, main/mainv3 = origin/main 69cc49b export, log/logv3 = branch + logging in
#   tieFreeStart (TFS_LOG=1 logs each call; TFS_OFF=1 disables the rule = main), v3 = -march=x86-64-v3 (FMA contraction).
R=/home/chris/wt-T0158/build-verify1-rel; OUT=${OUT:-$R/bin}; mkdir -p $OUT
T=$(dirname $(readlink -f $0))
for v in ${VARIANTS:-log logv3 br brv3 main mainv3}; do
  case $v in br*) W=/home/chris/wt-T0158;; main*) W=$R/mainsrc;; log*) W=$R/logsrc;; esac
  F=; case $v in *v3) F=-march=x86-64-v3;; esac
  B=$R/$v
  c++ $F -std=c++20 -O2 -w -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I$W/core/include -I$T -I$W/core/tests/toolpath -I$W/core/tests/geometry -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include "$1" -o $OUT/${2}_$v $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a || exit 1
done
