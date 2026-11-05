#!/bin/bash
# T-0142 round-3 verifier builds (branch fb08fc9). Release -march=x86-64-v3 (build-verify3-rel), Debug -Werror (build-verify3).
W=/home/chris/wt-T0142
cd $W
timeout 3000 cmake -S . -B build-verify3-rel -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF -DCMAKE_CXX_FLAGS=-march=x86-64-v3 > build-verify3-rel.cfg.log 2>&1
timeout 3000 cmake --build build-verify3-rel -j6 > build-verify3-rel.build.log 2>&1; echo "rel rc=$?"
timeout 3000 cmake -S . -B build-verify3 -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCARVETTO_BUILD_APP=OFF > build-verify3.cfg.log 2>&1
timeout 3000 cmake --build build-verify3 -j6 > build-verify3.build.log 2>&1; echo "debug rc=$?"
