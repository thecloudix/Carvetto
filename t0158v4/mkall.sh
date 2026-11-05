#!/bin/bash
# T-0158 round-4: Release core libs in /home/chris/wt-T0158/build-verify4-rel: br (999a90e worktree), main (origin/main c2aa6f4
# export), log/logv3 (999a90e export + tfs_instr4.py; TFS_OFF=1 = main), logfma (log without -ffp-contract=off, built with
# -march=x86-64-v3 -ffp-contract=fast, core and Clipper2), r3 (520ca28 = round 3 merged with main).
cd /home/chris/wt-T0158/build-verify4-rel
b() { cmake -S $2 -B $1 -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF -DCMAKE_CXX_FLAGS="$3" -DCMAKE_C_FLAGS="$3" > $1.cfg.log 2>&1 && timeout 3000 cmake --build $1 -j2 --target carvetto_core > $1.build.log 2>&1; echo "$1 rc=$?"; }
b log logsrc "" & b logv3 logsrc "-march=x86-64-v3" & b logfma logfmasrc "-march=x86-64-v3 -ffp-contract=fast" & wait
b br /home/chris/wt-T0158 "" & b main mainsrc "" & b r3 r3src "" & wait
echo ALLDONE
