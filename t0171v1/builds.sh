#!/bin/bash
# real Debug (full, -Werror default), real Release, mutC Debug, mutC Release (carvetto_tests only)
W=/home/chris/wt-T0171; H=/home/chris/harness/t0171v1
timeout 3000 bash -c "cmake -S $W -B $W/build-verify1 -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCARVETTO_BUILD_APP=OFF && cmake --build $W/build-verify1 -j4" > $H/results/build_debug.log 2>&1; echo "realDebug rc=$?"
timeout 3000 bash -c "cmake -S $W -B $W/build-verify1-rel -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF && cmake --build $W/build-verify1-rel -j4 --target carvetto_tests" > $H/results/build_rel.log 2>&1; echo "realRel rc=$?"
for t in Debug Release; do
timeout 3000 bash -c "cmake -S $W/build-verify1-src-mutC -B $W/build-verify1-mutC-$t -G Ninja -DCMAKE_BUILD_TYPE=$t -DCARVETTO_BUILD_APP=OFF -DCMAKE_CXX_FLAGS=-Wno-error && cmake --build $W/build-verify1-mutC-$t -j4 --target carvetto_tests" > $H/results/build_mutC_$t.log 2>&1; echo "mutC$t rc=$?"
done
