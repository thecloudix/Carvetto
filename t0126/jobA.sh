#!/bin/bash
# Full test bar: clean Debug -Werror build, serial ctest, ctest -j8 --repeat until-fail:3, then Release ctest.
S=/home/claude/vscratch/t0126; W=/home/claude/wt-T-0126; FC=$(cat $S/fc.txt)
cd $W && rm -rf build && cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCARVETTO_BUILD_APP=OFF -DCMAKE_CXX_FLAGS=-Werror $FC > $S/A_cfg.log 2>&1
cmake --build build -j2 > $S/A_build.log 2>&1; echo "DBG BUILD rc=$?" >> $S/A_build.log
grep -c "warning" $S/A_build.log >> $S/A_build.log
ctest --test-dir build > $S/A_serial.log 2>&1; echo "SERIAL rc=$?" >> $S/A_serial.log
ctest --test-dir build -j8 --repeat until-fail:3 > $S/A_j8.log 2>&1; echo "J8 rc=$?" >> $S/A_j8.log
echo DONE > $S/A.done
