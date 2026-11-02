#!/bin/bash
cd /home/chris/wt-T0135/build-verify/drv
while [ ! -f hs/done ]; do sleep 10; done
for i in $(seq 0 19); do s=$((630000+i)); for b in wsh3 wsh3_base; do o=hs2/${b}_$s.txt; grep -q SUMMARY $o 2>/dev/null && continue; FORCEIOPOS=1 HASH=1 ./bin/$b $s 50 3 > $o.tmp 2>&1 && mv $o.tmp $o; done; done
echo DONE > hs2/done
