#!/bin/bash
# Release branch lib + namespaced origin/main lib (cvmain), inside the worktree.
W=/home/chris/wt-T0128; cd $W
( cmake -S . -B build-verify1-rel -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF > build-verify1-scratch/rel.log 2>&1 && timeout 1800 cmake --build build-verify1-rel -j3 --target carvetto_core >> build-verify1-scratch/rel.log 2>&1; echo "rel rc=$?" >> build-verify1-scratch/rel.log ) &
( cmake -S build-verify1-mainsrc -B build-verify1-mainns -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF -DCARVETTO_BUILD_TESTS=OFF "-DCMAKE_CXX_FLAGS=-Dcarvetto=cvmain -Wno-error=maybe-uninitialized" > build-verify1-scratch/mainns.log 2>&1 && timeout 1800 cmake --build build-verify1-mainns -j3 --target carvetto_core >> build-verify1-scratch/mainns.log 2>&1; echo "mainns rc=$?" >> build-verify1-scratch/mainns.log ) &
wait
