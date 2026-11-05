#!/bin/bash
# AC2 fuzz, round 3: v9 injector (st) 8 x 3000 seeds, g2 8 x 1200 seeds, allj (every join x side) v9 2 x 250, g2 2 x 250.
D=/home/chris/wt-T0009r4/build-verify4-drv; L=/home/chris/harness/t0009v4/logs; cd $D
jobs=()
for i in 0 1 2 3 4 5 6 7; do echo "timeout 5400 nice ./x2 st 3000 $((0x31000000 + i*0x100000)) > $L/v9_$i.log 2>&1"; done > $L/jobs.txt
for i in 0 1 2 3 4 5 6 7; do echo "timeout 5400 nice ./x2 g2 1200 $((0x32000000 + i*0x100000)) > $L/g2_$i.log 2>&1"; done >> $L/jobs.txt
for i in 0 1; do echo "timeout 5400 nice ./x2 allj 250 $((0x33000000 + i*0x100000)) 1 > $L/allj_v9_$i.log 2>&1"; echo "timeout 5400 nice ./x2 allj 250 $((0x34000000 + i*0x100000)) 0 > $L/allj_g2_$i.log 2>&1"; done >> $L/jobs.txt
DUMP=$L/dump.inc xargs -P 8 -I{} bash -c "{}" < $L/jobs.txt
echo done > $L/fuzz.done
