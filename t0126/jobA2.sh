#!/bin/bash
S=/home/claude/vscratch/t0126; W=/home/claude/wt-T-0126; FC=$(cat $S/fc.txt)
cd $W && rm -rf build && cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCARVETTO_BUILD_APP=OFF -DCMAKE_CXX_FLAGS=-Werror $FC > $S/A2_cfg.log 2>&1
cmake --build build -j2 > $S/A2_build.log 2>&1; echo "DBG BUILD rc=$?" >> $S/A2_build.log
echo "warnings: $(grep -c warning: $S/A2_build.log)" >> $S/A2_build.log
ctest --test-dir build > $S/A2_serial.log 2>&1; echo "SERIAL rc=$?" >> $S/A2_serial.log
echo DONE > $S/A2.done
