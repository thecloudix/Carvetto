#!/bin/sh
mkdir -p /home/chris/scratch-t0153v6/src-golden && cd /home/chris/wt-T0153 && git archive bdc932c | tar -x -C /home/chris/scratch-t0153v6/src-golden && cd /home/chris/scratch-t0153v6/src-golden &&
timeout 3000 sh -c 'cmake -S . -B b -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF > cfg.log 2>&1 && cmake --build b -j6 > build.log 2>&1' &&
git init -q . && git add -A >/dev/null && git -c user.name=v -c user.email=v@v commit -qm base && timeout 1200 python3 tools/regen_golden.py --build-dir b > regen.log 2>&1; echo "rc=$?" >> regen.log
git -C /home/chris/scratch-t0153v6/src-golden status --short > /home/chris/scratch-t0153v6/golden_status.txt; echo done > /home/chris/scratch-t0153v6/golden.done
