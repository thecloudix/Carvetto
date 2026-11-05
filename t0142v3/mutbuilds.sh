#!/bin/bash
# Mutants of fb08fc9 (Release -march=x86-64-v3, carvetto_tests + core only):
#  mutA: offset.cpp restored to round 2 (15848d4: per-separableBatches left-edge scan) -- "fix reverted"
#  mutB: edge index disabled (kRingIndexMinVertices = SIZE_MAX)
#  mutC: box hierarchy replaced by a linear scan over all rings wound like the lowest (last-outer shortcut kept)
W=/home/chris/wt-T0142
for m in mutA mutB mutC; do
  timeout 2400 cmake -S $W/build-verify3-$m -B $W/build-verify3-$m-rel -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF "-DCMAKE_CXX_FLAGS=-march=x86-64-v3 -Wno-error" > $W/build-verify3-$m-rel.cfg.log 2>&1
  timeout 2400 cmake --build $W/build-verify3-$m-rel -j5 --target carvetto_tests > $W/build-verify3-$m-rel.build.log 2>&1; echo "$m rc=$?"
done
