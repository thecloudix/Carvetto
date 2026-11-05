#!/bin/bash
# T-0142 round-2 verifier builds (branch 15848d4). Debug -Werror; Release -march=x86-64-v3.
W=/home/chris/wt-T0142
cd $W
echo $$ > $W/build-verify2.pid
timeout 3000 cmake -S . -B build-verify2-rel -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF -DCMAKE_CXX_FLAGS=-march=x86-64-v3 > build-verify2-rel.cfg.log 2>&1
timeout 3000 cmake --build build-verify2-rel -j6 > build-verify2-rel.build.log 2>&1; echo "rel rc=$?"
timeout 3000 cmake -S . -B build-verify2 -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCARVETTO_BUILD_APP=OFF > build-verify2.cfg.log 2>&1
timeout 3000 cmake --build build-verify2 -j6 > build-verify2.build.log 2>&1; echo "debug rc=$?"
