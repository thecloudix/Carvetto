#!/bin/bash
# (b) re-checks: hn (nested outsets byte hashes, branch vs main) and lo (leftover-ring oracle)
cd /home/chris/harness/t0142v2
for s in 5 6 7; do timeout 900 bin/hn_br $s 40 > results/hn_br_$s.txt; timeout 900 bin/hn_main $s 40 > results/hn_main_$s.txt; done
for s in 31 32 33 34; do timeout 900 bin/lo_br $s 80 > results/lo_br_$s.txt; done
for f in 0 1 2 3 4 5 6; do timeout 900 bin/lo_br 41 30 $f > results/lo_br_41_f$f.txt; done
echo done > results/runb.done
