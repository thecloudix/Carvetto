#!/bin/bash
# follow-up: is the super-linear growth on hatch/arc/zz Round pre-existing (main), and hatch 8000
D=/home/chris/wt-T0016/build-verify1-drv; L=/home/chris/harness/t0016v1/logs; cd $D
while ! grep -q "^done" $L/perf.log; do sleep 10; done
{
for n in 1000 2000 4000; do timeout 1800 ./perf zz $n 0.02 1 7 0 2 2 1; done
for n in 2000 4000; do timeout 1800 ./perf hatch $n 0.02 1 7 0 2 2 1; done
timeout 1800 ./perf arc 2000 0.02 1 7 0 2 2 1
timeout 1800 ./perf hatch 4000 0.02 1 7 0 2 1 2
timeout 1800 ./perf hatch 8000 0.02 1 7 0 2 1 3 nomain
} > $L/perf2.log 2>&1
echo done >> $L/perf2.log
