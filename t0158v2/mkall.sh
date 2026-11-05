#!/bin/bash
# builds Release core libs in build-verify2-rel: br (5e7ce1d), brv3 (x86-64-v3), main/mainv3 (origin/main 41649cb, core
# built with -ffp-contract=off since T-0153), log/logv3 (branch + tfs_instr.py), mlog/mlogv3 (branch merged onto 41649cb + tfs_instr.py)
cd /home/chris/wt-T0158/build-verify2-rel
b() { cmake -S $2 -B $1 -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF -DCMAKE_CXX_FLAGS="$3" -DCMAKE_C_FLAGS="$3" > $1.cfg.log 2>&1 && timeout 3000 cmake --build $1 -j4 --target carvetto_core > $1.build.log 2>&1; echo "$1 rc=$?"; }
b br /home/chris/wt-T0158 "" & b brv3 /home/chris/wt-T0158 "-march=x86-64-v3" & b main mainsrc "" & b mainv3 mainsrc "-march=x86-64-v3" & wait
b log logsrc "" & b logv3 logsrc "-march=x86-64-v3" & b mlog mlogsrc "" & b mlogv3 mlogsrc "-march=x86-64-v3" & wait
echo ALLDONE
