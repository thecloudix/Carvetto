#!/bin/bash
# log (1156cad + tfs_instr5.py; TFS_OFF=1 = main) and logfma (same, -march=x86-64-v3 -ffp-contract=fast core+Clipper2)
cd /home/chris/wt-T0158/build-verify5-rel; echo $$ > mklog5.pid
timeout 7000 bash -c 'until grep -q ALLDONE mkall5.log; do sleep 15; done'
b() { cmake -S $2 -B $1 -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF -DCMAKE_CXX_FLAGS="$3" -DCMAKE_C_FLAGS="$3" > $1.cfg.log 2>&1 && timeout 3000 cmake --build $1 -j3 --target carvetto_core > $1.build.log 2>&1; echo "$1 rc=$?"; }
b log logsrc "" & b logfma logfmasrc "-march=x86-64-v3 -ffp-contract=fast" & wait
echo ALLDONE
