#!/bin/bash
# mk.sh src.cpp name -> $OUT/name_{mainrel,mainv3,brrel,brv3}
# Links against: origin/main 69eb25a export (scratch v159/main/build-{rel,v3}) and the branch
# worktree /home/chris/wt-T0159b build-verify1-{rel,v3}. -march=x86-64-v3 for the v3 variants.
S=/tmp/claude-1000/-home-chris-Carvetto/120b15af-a822-46aa-8270-71a70ec2ea48/scratchpad/v159
OUT=${OUT:-$S/bin}; mkdir -p $OUT
T=$(dirname $(readlink -f $0)); H=$T/../t0131v1
for v in ${VARIANTS:-mainrel mainv3 brrel brv3 hybrel}; do
  case $v in
    mainrel) W=$S/main; B=$W/build-rel; F=;;
    mainv3) W=$S/main; B=$W/build-v3; F=-march=x86-64-v3;;
    brrel) W=/home/chris/wt-T0159b; B=$W/build-verify1-rel; F=;;
    brv3) W=/home/chris/wt-T0159b; B=$W/build-verify1-v3; F=-march=x86-64-v3;;
    hybrel) W=$S/hyb; B=$W/build-rel; F=;;  # main + branch offset.cpp only (no Profile/tab change)
    hybv3) W=$S/hyb; B=$W/build-v3; F=-march=x86-64-v3;;  # same, FMA
  esac
  c++ $F -std=c++20 -O2 -w -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I$W/core/include -I$T -I$H -I$W/core/tests/toolpath -I$W/core/tests/geometry -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include "$1" -o $OUT/${2}_$v $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a || exit 1
done
