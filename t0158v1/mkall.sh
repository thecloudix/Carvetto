#!/bin/bash
# builds Release core libs: br, brv3 (x86-64-v3), main, mainv3, log (instrumented branch), logv3
cd /home/chris/wt-T0158/build-verify1-rel
b() { # name src flags
  cmake -S $2 -B $1 -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF -DCMAKE_CXX_FLAGS="$3" -DCMAKE_C_FLAGS="$3" > $1.cfg.log 2>&1 && cmake --build $1 -j4 --target carvetto_core > $1.build.log 2>&1; echo "$1 rc=$?"
}
b br /home/chris/wt-T0158 "" &
b brv3 /home/chris/wt-T0158 "-march=x86-64-v3" &
wait
b main mainsrc "" &
b mainv3 mainsrc "-march=x86-64-v3" &
wait
b log logsrc "" &
b logv3 logsrc "-march=x86-64-v3" &
wait
echo ALLDONE
