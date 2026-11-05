#!/bin/sh
# (c) growth: interleaved branch/main, pairs M and chain N, 2 rounds, Release x86-64-v3
B=/home/chris/scratch-t0164v1/bin
for round in 1 2; do for M in 200 400 800 1600; do for v in br main; do
  echo "$v round$round load=$(cut -d' ' -f1 /proc/loadavg) $(timeout 900 $B/grow_$v pairs $M 0.5 | cut -c1-200)"
done; done; done
for round in 1 2; do for N in 1000 2000 4000; do for v in br main; do
  echo "$v round$round load=$(cut -d' ' -f1 /proc/loadavg) $(timeout 900 $B/grow_$v chain $N 0.05 | cut -c1-200)"
done; done; done
