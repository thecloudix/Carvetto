#!/bin/sh
for round in 1 2; do
 for mode in plain frame flat; do for v in 209499a bdc932c; do
  b=/home/chris/scratch-t0153v5/bin/perf_209499a; [ $v = bdc932c ] && b=/home/chris/scratch-t0153v6/bin/perf_bdc932c
  case $mode in plain) e="";; frame) e="VPERF_FRAME=1";; flat) e="VPERF_FLAT=0.5";; esac
  echo "$v $mode round$round load=$(cut -d' ' -f1 /proc/loadavg) $(env $e timeout 300 $b 5000 2>&1 | tail -1)"
 done; done
done > /home/chris/harness/t0153v6/results/perf.txt
echo done > /home/chris/scratch-t0153v6/perf.done
