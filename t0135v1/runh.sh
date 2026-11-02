#!/bin/bash
# runh.sh : hashing branch vs base, mode 3, seeds 620000..620039 (interleaved)
cd /home/chris/wt-T0135/build-verify/drv
for i in $(seq 0 39); do s=$((620000+i)); for b in wsh3 wsh3_base; do o=hs/${b}_$s.txt; grep -q SUMMARY $o 2>/dev/null && continue; HASH=1 ./bin/$b $s 50 3 > $o.tmp 2>&1 && mv $o.tmp $o; done; done
echo DONE > hs/done
