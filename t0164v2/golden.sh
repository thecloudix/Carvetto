#!/bin/sh
# t0164v1: regenerate goldens in a scratch copy of the branch and list changes
S=/home/chris/scratch-t0164v2; cd $S/src-golden &&
timeout 3600 sh -c 'cmake -S . -B b -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF > cfg.log 2>&1 && nice ninja -C b -j4 > build.log 2>&1' &&
git init -q . && git add -A >/dev/null && git -c user.name=v -c user.email=v@v commit -qm base && timeout 1800 python3 tools/regen_golden.py --build-dir b > regen.log 2>&1; echo "rc=$?" >> regen.log
git -C $S/src-golden status --short > $S/golden_status.txt; echo done > $S/golden.done
