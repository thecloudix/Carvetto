#!/bin/bash
# T-0158 round-3: builds Release core libs in /home/chris/wt-T0158/build-verify3-rel: br (5649b2b), brv3 (-march=x86-64-v3 = FMA),
# main/mainv3 (origin/main 2df9aba export in mainsrc), log/logv3 (5649b2b export in logsrc + tfs_instr3.py).
cd /home/chris/wt-T0158/build-verify3-rel
b() { cmake -S $2 -B $1 -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF -DCMAKE_CXX_FLAGS="$3" -DCMAKE_C_FLAGS="$3" > $1.cfg.log 2>&1 && timeout 3000 cmake --build $1 -j3 --target carvetto_core > $1.build.log 2>&1; echo "$1 rc=$?"; }
b br /home/chris/wt-T0158 "" & b brv3 /home/chris/wt-T0158 "-march=x86-64-v3" & b main mainsrc "" & b mainv3 mainsrc "-march=x86-64-v3" & b log logsrc "" & b logv3 logsrc "-march=x86-64-v3" & wait
echo ALLDONE
