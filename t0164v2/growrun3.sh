#!/bin/sh
# t0164v2 (c): growth of other absorption patterns, interleaved branch/main (main skipped above a cap), Release x86-64-v3
B=/home/chris/scratch-t0164v2/bin; O=/home/chris/scratch-t0164v2/grow_other.txt; : > $O
until grep -q done /home/chris/scratch-t0164v2/densefuzz.txt 2>/dev/null; do sleep 20; done
run() { echo "$1 $2 load=$(cut -d' ' -f1 /proc/loadavg) $(timeout 1800 $B/grow2_$1 $2 $3 $4 $5 | cut -c1-150)" >> $O; }
for round in 1 2; do
  for N in 200 400 800 1600 3200 6400; do run br pairsmix $N 0.5 1; [ $N -le 1600 ] && run main pairsmix $N 0.5 1; done
  for N in 200 400 800 1600 3200; do run br dense $N 0.05 1; [ $N -le 800 ] && run main dense $N 0.05 1; done
  for N in 1000 2000 4000 8000; do run br star $N 0.05 1; run main star $N 0.05 1; done
  for N in 400 800 1600 3200 6400; do run br rand $N 0.05 1; [ $N -le 1600 ] && run main rand $N 0.05 1; done
  for N in 25 50 100 200; do run br nest $N 0.5 1; run main nest $N 0.5 1; done
done
run br pairs 12800 0.5 1
echo done >> $O
