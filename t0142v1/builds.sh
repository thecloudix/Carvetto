#!/bin/bash
W=/home/chris/wt-T0142
set -x
cd $W
timeout 3000 cmake -S . -B build-verify1 -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCARVETTO_BUILD_APP=OFF > build-verify1.cfg.log 2>&1
timeout 3000 cmake --build build-verify1 -j6 > build-verify1.build.log 2>&1; echo "debug rc=$?" 
timeout 3000 cmake -S . -B build-verify1-rel -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF -DCMAKE_CXX_FLAGS=-march=x86-64-v3 > build-verify1-rel.cfg.log 2>&1
timeout 3000 cmake --build build-verify1-rel -j6 > build-verify1-rel.build.log 2>&1; echo "rel rc=$?"
timeout 3000 cmake -S build-verify1-mainsrc -B build-verify1-mainrel -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF -DCMAKE_CXX_FLAGS=-march=x86-64-v3 > build-verify1-mainrel.cfg.log 2>&1
timeout 3000 cmake --build build-verify1-mainrel -j6 > build-verify1-mainrel.build.log 2>&1; echo "mainrel rc=$?"
