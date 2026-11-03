#!/bin/bash
cd /home/chris/wt-T0143/build-verify/drv
run() { "$@"; }
for v in "" _old; do
  for s in 9101 9102 9103 9104; do ./bin/pc2$v $s 400 0 > cmp/pc2_${s}$v.txt 2>&1 & done; wait
  for m in 0 1 2 3; do HASH=1 ./bin/wsh2$v 92$m 1500 $m > cmp/wsh2_92${m}$v.txt 2>&1 & done; wait
done
echo DONE > cmp/done
