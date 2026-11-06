#!/bin/bash
cd /home/chris/wt-T0140/build-verify2-rel; echo $$ > mkall.pid
b() { cmake -S $2 -B $1 -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF > $1.cfg.log 2>&1 && timeout 3000 cmake --build $1 -j2 --target carvetto_core > $1.build.log 2>&1; echo "$1 rc=$?"; }
b br /home/chris/wt-T0140 &
b base basesrc &
wait
echo ALLDONE
