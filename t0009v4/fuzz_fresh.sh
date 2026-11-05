#!/bin/bash
# AC2 fresh seeds (round 4 verifier; not seen by the coder): v9 8 x 1500, g2 8 x 600, allj v9 2 x 125, g2 2 x 125
D=/home/chris/wt-T0009r4/build-verify4-drv; L=/home/chris/harness/t0009v4/logs/fresh; mkdir -p $L; cd $D
for i in 0 1 2 3 4 5 6 7; do echo "timeout 7000 nice ./x2 st 1500 $((0x41000000 + i*0x100000)) > $L/v9_$i.log 2>&1"; done > $L/jobs.txt
for i in 0 1 2 3 4 5 6 7; do echo "timeout 7000 nice ./x2 g2 600 $((0x42000000 + i*0x100000)) > $L/g2_$i.log 2>&1"; done >> $L/jobs.txt
for i in 0 1; do echo "timeout 7000 nice ./x2 allj 125 $((0x43000000 + i*0x100000)) 1 > $L/allj_v9_$i.log 2>&1"; echo "timeout 7000 nice ./x2 allj 125 $((0x44000000 + i*0x100000)) 0 > $L/allj_g2_$i.log 2>&1"; done >> $L/jobs.txt
DUMP=$L/dump.inc xargs -P ${P:-4} -I{} bash -c "{}" < $L/jobs.txt
echo done > $L/fuzz.done
