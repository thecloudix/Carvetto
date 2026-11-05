#!/bin/bash
# AC1 timing matrix (Release, CPU time, branch vs main interleaved in one process; min over reps)
D=/home/chris/wt-T0016/build-verify1-drv; L=/home/chris/harness/t0016v1/logs; cd $D
P="timeout 1800 ./perf"
{
echo "== card case, with main"
$P zz 1000 0.02 1 7 0 2 3; $P zz 1000 0.02 1 7 1 2 3
echo "== scaling (branch only)"
for n in 1000 2000 4000 8000; do $P zz $n 0.02 1 7 0 2 2 7 nomain; $P zz $n 0.02 1 7 1 2 2 7 nomain; done
echo "== other dense hatch-like inputs"
$P zz 4000 0.05 0.2 5 0 2 2; $P zz 1000 0.01 1 7 0 2 2; $P zz 1000 0.02 5 3 1 2 2; $P zz 1000 0.02 0.2 10 0 2 2; $P zz 2000 0.1 1 20 1 2 2
$P saw 1000 0.02 1 7 0 2 2; $P saw 1000 0.02 1 7 1 2 2
$P hatch 1000 0.02 3 7 0 2 2; $P hatch 1000 0.02 3 7 1 2 2
$P zc 1000 0.02 1 7 0 2 2; $P zc 1000 0.02 1 7 1 2 2
$P arc 1000 0.02 1 7 0 2 2; $P arc 1000 0.02 1 7 1 2 2
$P bez 500 0.02 1 7 0 2 2; $P bez 500 0.02 1 7 1 2 2
for m in 1.0 1.5 4 10 100; do $P zz 1000 0.02 1 7 0 $m 2; done
echo "== scaling of other shapes (branch only)"
for s in hatch arc zc saw; do for n in 1000 2000 4000; do $P $s $n 0.02 1 7 0 2 2 7 nomain; done; done
for n in 500 1000 2000; do $P bez $n 0.02 1 7 0 2 2 7 nomain; done
} > $L/perf.log 2>&1
echo done >> $L/perf.log
