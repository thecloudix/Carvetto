#!/bin/sh
# t0160v1: sharpening cost on one big part (frame round n letters: letters are holes) with a flat depth and a 0.5 mm end
# mill, branch on vs off vs main, interleaved, n = 100 / 200 (500 takes > 4 min on main: Pocket on one part).
B=/home/chris/scratch-t0160v1/bin
for n in 100 200; do for v in main br br_off; do
  b=$v; x=""; [ $v = br_off ] && { b=br; x="VPERF_CS=0"; }
  echo "FRAMEEM $v n=$n load=$(cut -d' ' -f1 /proc/loadavg) $(env VPERF_FRAME=1 VPERF_FLAT=0.5 VPERF_EM=0.5 $x timeout 900 $B/perf_$b $n 2>&1 | grep -E ' regions |diag' | tr '\n' ' ')"
done; done
