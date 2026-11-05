#!/bin/bash
# re-measure AC1 at lower machine load
D=/home/chris/wt-T0016/build-verify1-drv; L=/home/chris/harness/t0016v1/logs; cd $D
{ uptime
timeout 1800 ./perf zz 1000 0.02 1 7 0 2 3; timeout 1800 ./perf zz 1000 0.02 1 7 1 2 3
for n in 1000 2000 4000 8000; do timeout 1800 ./perf zz $n 0.02 1 7 0 2 3 7 nomain; timeout 1800 ./perf zz $n 0.02 1 7 1 2 3 7 nomain; done
uptime; } > $L/perf3.log 2>&1
echo done >> $L/perf3.log
