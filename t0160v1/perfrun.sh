#!/bin/sh
# t0160v1: AC2 (5000 glyph-like regions: plain / frame / flat 0.5 self-clear; no clearance tools, sharpening not run) and the
# cost of sharpening with end mills (flat 0.5 + EM 0.2 / EM 1.0 / frame + EM 0.5), Release plain, CPU time, interleaved
# main / branch (on) / branch (VPERF_CS=0), 2 rounds.
B=/home/chris/scratch-t0160v1/bin
for round in 1 2; do
 for mode in plain frame flat; do for v in main br; do
  case $mode in plain) e="";; frame) e="VPERF_FRAME=1";; flat) e="VPERF_FLAT=0.5";; esac
  echo "AC2 $v $mode round$round load=$(cut -d' ' -f1 /proc/loadavg) $(env $e timeout 600 $B/perf_$v 5000 2>&1 | grep " regions ")"
 done; done
 for mode in em02 em10 frameem05; do for v in main br br_off; do
  case $mode in em02) e="VPERF_FLAT=0.5 VPERF_EM=0.2";; em10) e="VPERF_FLAT=0.5 VPERF_EM=1.0";; frameem05) e="VPERF_FRAME=1 VPERF_FLAT=0.5 VPERF_EM=0.5";; esac
  b=$v; x=""; [ $v = br_off ] && { b=br; x="VPERF_CS=0"; }
  echo "COST $v $mode round$round load=$(cut -d' ' -f1 /proc/loadavg) $(env $e $x timeout 900 $B/perf_$b 5000 2>&1 | grep " regions ")"
 done; done
done
