#!/bin/bash
# AC4: fuzz-family CPU time (min of 5 alternating reps per offset), fresh seeds 0x7b2Fxxxx.
D=/home/chris/wt-T0128/build-verify2-drv; L=/home/chris/harness/t0128v2/logs/ft; mkdir -p $L; cd $D
: > $L/jobs.txt
for f in 0 2 3 4 5 21 22 23 25; do n=300; [ $f -ge 20 ] && n=100; echo "timeout 2400 ./ftime $f $n $(printf 0x7b2%x0000 $f) 5 > $L/ft$f.log 2>&1" >> $L/jobs.txt; done
xargs -P ${P:-2} -I{} bash -c "{}" < $L/jobs.txt
echo done > $L/ft.done
