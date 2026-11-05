#!/bin/bash
# mk.sh src.cpp out {br|main|mut*}  -- links a driver against the T-0142 round-3 verifier's Release (-march=x86-64-v3)
# builds: br = build-verify3-rel (fb08fc9), main = build-verify1-mainrel (core == origin/main 53ae278/2df9aba),
# mutA/mutB = build-verify3-mutA-rel / -mutB-rel (sources in build-verify3-mutA / -mutB). -ffp-contract=off as core.
W=/home/chris/wt-T0142
case "$3" in
  main) S=$W/build-verify1-mainsrc; B=$W/build-verify1-mainrel;;
  mut*) S=$W/build-verify3-$3; B=$W/build-verify3-$3-rel;;
  *) S=$W; B=$W/build-verify3-rel;;
esac
T=$(dirname $(readlink -f $0))
c++ $EXTRA -std=c++20 -O2 -march=x86-64-v3 -ffp-contract=off -w -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I$S/core/include -I$S/core/src/geometry -I$S/core/src/toolpath -I$T -I/home/chris/harness/t0131v1 -I/home/chris/harness/t0141v1 -I$S/core/tests/toolpath -I$S/core/tests/geometry -I$S/core/tests -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include "$1" -o "$2" $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a
