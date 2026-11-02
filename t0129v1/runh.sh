#!/bin/bash
cd /home/chris/wt-T0129/build-verify/drv
for pair in "7201 7202" "7203 7204" "7205 7206"; do
  for s in $pair; do HASH=1 ./wsh $s 1000 4 > hs/new_$s.txt & HASH=1 ./wsh_old $s 1000 4 > hs/old_$s.txt & done; wait
done
for s in 7301 7302; do HASH=1 ./wsh $s 800 2 > hs/new_$s.txt & HASH=1 ./wsh_old $s 800 2 > hs/old_$s.txt & done; wait
echo ALLDONE > hs/done
