#!/bin/bash
# T-0016 zig-zag timings (t0016v1 perf.cpp): branch vs origin/main / 8cd3568 / branch-namespaced, and the branch without the step-2
# boundary-split change (varNOBS) vs origin/main.  logs/perf16.log
D=/home/chris/wt-T0172/build-verify1-drv; L=/home/chris/harness/t0172v1/logs/perf16.log; cd $D
for b in main pre brns nobs_main; do echo "== $b"
  for s in 0 1; do timeout 900 ./perf16_$b zz 1000 0.02 1 7 $s 2 7; done
  timeout 900 ./perf16_$b zz 4000 0.05 0.2 5 0 2 5; timeout 900 ./perf16_$b hatch 1000 0.02 3 7 0 2 5; timeout 900 ./perf16_$b arc 1000 0.02 1 7 0 2 5
done > $L 2>&1
echo done >> $L
