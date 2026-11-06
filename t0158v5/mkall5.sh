#!/bin/bash
# T-0158 round 5 Release core libs: br5 (worktree 1156cad), br5fma (export, -march=x86-64-v3 -ffp-contract=fast core+Clipper2),
# main (origin/main 34db4d0), mainfma, r4 (999a90e export).
cd /home/chris/wt-T0158/build-verify5-rel; echo $$ > mkall5.pid
b() { cmake -S $2 -B $1 -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF -DCMAKE_CXX_FLAGS="$3" -DCMAKE_C_FLAGS="$3" > $1.cfg.log 2>&1 && timeout 3000 cmake --build $1 -j2 --target carvetto_core > $1.build.log 2>&1; echo "$1 rc=$?"; }
b br5 /home/chris/wt-T0158 "" & b main mainsrc "" & b br5fma br5fmasrc "-march=x86-64-v3 -ffp-contract=fast" & wait
b mainfma mainfmasrc "-march=x86-64-v3 -ffp-contract=fast" & b r4 r4src "" & wait
echo ALLDONE
