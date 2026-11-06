#!/bin/bash
# White-box D check with the strict allowance of the tests (W3 lib): fresh seeds; logs/w3/*.err.
H=/home/chris/harness/t0128v2; export WLOG=$H/logs/w3; L=$WLOG; mkdir -p $L
: > $L/jobs.txt
for k in 0 1 2 3 4 5; do echo "$H/wrun3.sh 90 2500 $(printf 0x9d%02x0000 $k) w90_$k" >> $L/jobs.txt; done
for f in 5 3 4; do echo "$H/wrun3.sh $f 600 $(printf 0x9e%x00000 $f) w${f}" >> $L/jobs.txt; done
xargs -P ${P:-2} -I{} bash -c "{}" < $L/jobs.txt
echo done > $L/w.done
