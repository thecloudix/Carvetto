#!/bin/sh
# t0164v1 AC2: 5000 glyph-like regions, plain / frame / flat 0.5, Release x86-64-v3, interleaved main/branch, 2 rounds
B=/home/chris/scratch-t0164v1/bin
for round in 1 2; do for mode in plain frame flat; do for v in main br; do
  case $mode in plain) e="";; frame) e="VPERF_FRAME=1";; flat) e="VPERF_FLAT=0.5";; esac
  echo "$v $mode round$round load=$(cut -d' ' -f1 /proc/loadavg) $(env $e timeout 400 $B/perf_$v 5000 2>&1 | grep " regions ")"
done; done; done
