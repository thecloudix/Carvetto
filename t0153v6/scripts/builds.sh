#!/bin/sh
cd /home/chris/wt-T0153
( timeout 3000 sh -c 'cmake -S . -B build-verify6 -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCARVETTO_BUILD_APP=OFF > build-verify6.cfg.log 2>&1 && cmake --build build-verify6 -j6 > build-verify6.build.log 2>&1 && ctest --test-dir build-verify6 --output-on-failure -j6 > build-verify6.ctest.log 2>&1'; echo "debug rc=$?" > /home/chris/scratch-t0153v6/debug.done ) &
( timeout 3000 sh -c 'cmake -S . -B build-verify6-rel -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF > /dev/null 2>&1 && cmake --build build-verify6-rel -j5 --target carvetto_core > build-verify6-rel.log 2>&1'; echo "rel rc=$?" > /home/chris/scratch-t0153v6/rel.done ) &
( cd /home/chris/scratch-t0153v6/src-sab && timeout 3000 sh -c 'cmake -S . -B b-rel -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF -DCARVETTO_BUILD_TESTS=OFF > cfg.log 2>&1 && cmake --build b-rel -j5 --target carvetto_core > build.log 2>&1'; echo "sab rc=$?" > /home/chris/scratch-t0153v6/sab.done ) &
wait
