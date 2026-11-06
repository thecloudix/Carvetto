#!/bin/bash
cd /home/chris/wt-T0140; echo $$ > build-verify2-rel/dbg.pid
( cmake -S . -B build-verify2 -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCARVETTO_BUILD_APP=OFF && timeout 3600 cmake --build build-verify2 -j3 ) > build-verify2-rel/dbg.build.log 2>&1; echo "build rc=$?" > build-verify2-rel/dbg.rc
timeout 5400 ctest --test-dir build-verify2 --output-on-failure -j3 > build-verify2-rel/dbg.ctest.log 2>&1; echo "ctest rc=$?" >> build-verify2-rel/dbg.rc
