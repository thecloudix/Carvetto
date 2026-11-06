#!/bin/bash
# Release libs: branch (build-verify3-rel), origin/main namespaced cvmain (build-verify3-mainns), e87f549 namespaced cvmain (build-verify3-oldns)
W=/home/chris/wt-T0128; cd $W
cmake -S . -B build-verify3-rel -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF -DBUILD_TESTING=OFF > /dev/null && cmake --build build-verify3-rel -j2 --target carvetto_core || echo REL FAIL
for s in main old; do
cmake -S build-verify3-${s}src -B build-verify3-${s}ns -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF -DBUILD_TESTING=OFF "-DCMAKE_CXX_FLAGS=-Dcarvetto=cvmain -Wno-error=maybe-uninitialized" > /dev/null && cmake --build build-verify3-${s}ns -j2 --target carvetto_core || echo $s FAIL
done
echo mkrel done
