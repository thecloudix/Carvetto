#!/bin/bash
# (b) re-checks on round 3: hn (nested outsets byte hashes, branch vs main) and lo (leftover/lost oracle)
cd /home/chris/harness/t0142v3
for s in 5 6 7 8; do timeout 900 bin/hn_br $s 40 > results/hn_br_$s.txt; timeout 900 bin/hn_main $s 40 > results/hn_main_$s.txt; done
for s in 31 32 33 34; do timeout 900 bin/lo_br $s 80 > results/lo_br_$s.txt; done
echo done > results/runb3.done
