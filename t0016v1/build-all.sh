#!/bin/bash
W=/home/chris/wt-T0016
cd $W
( cmake -S . -B build-verify1 -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCARVETTO_BUILD_APP=OFF > build-verify1.cfg.log 2>&1 && cmake --build build-verify1 -j6 > build-verify1.build.log 2>&1; echo "dbg rc=$?" >> build-verify1.build.log ) &
( cmake -S . -B build-verify1-rel -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF > build-verify1-rel.cfg.log 2>&1 && cmake --build build-verify1-rel -j4 > build-verify1-rel.build.log 2>&1; echo "rel rc=$?" >> build-verify1-rel.build.log ) &
( cmake -S build-verify1-mainsrc -B build-verify1-mainns -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF -DCARVETTO_BUILD_TESTS=OFF "-DCMAKE_CXX_FLAGS=-Dcarvetto=cvmain -Wno-error=maybe-uninitialized" > build-verify1-mainns.cfg.log 2>&1 && cmake --build build-verify1-mainns -j3 --target carvetto_core > build-verify1-mainns.build.log 2>&1; echo "mainns rc=$?" >> build-verify1-mainns.build.log ) &
wait
