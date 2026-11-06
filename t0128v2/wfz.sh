#!/bin/bash
# White-box D check (W lib): adversarial fam 90 + fuzz families; logs/w/*.err: WVIOL lines + WADD counts per kind.
H=/home/chris/harness/t0128v2; L=$H/logs/w; mkdir -p $L; rm -f $L/*
: > $L/jobs.txt
for k in 0 1 2 3 4 5 6 7; do echo "$H/wrun.sh 90 2500 $(printf 0x9b%02x0000 $k) w90_$k" >> $L/jobs.txt; done
for f in 5 2 3 4; do for k in 0 1; do echo "$H/wrun.sh $f 400 $(printf 0x9c%x%01x0000 $f $k) w${f}_$k" >> $L/jobs.txt; done; done
xargs -P ${P:-2} -I{} bash -c "{}" < $L/jobs.txt
echo done > $L/w.done
