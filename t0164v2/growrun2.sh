#!/bin/sh
# t0164v2 (c): pairs growth interleaved branch/main, Release x86-64-v3, 2 rounds (main capped at 3200), then main 6400 once (identity)
B=/home/chris/scratch-t0164v2/bin; O=/home/chris/scratch-t0164v2/grow_pairs.txt; : > $O
for round in 1 2; do for M in 200 400 800 1600 3200 6400; do for v in br main; do
  [ $v = main ] && [ $M -gt 3200 ] && continue
  echo "$v round$round load=$(cut -d' ' -f1 /proc/loadavg) $(timeout 1800 $B/grow2_$v pairs $M 0.5 | cut -c1-170)" >> $O
done; done; done
echo "main once load=$(cut -d' ' -f1 /proc/loadavg) $(timeout 3000 $B/grow2_main pairs 6400 0.5 | cut -c1-170)" >> $O
echo done >> $O
