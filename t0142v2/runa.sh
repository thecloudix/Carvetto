#!/bin/bash
# (a)/(e) re-checks on the round-2 branch build
cd /home/chris/harness/t0142v2
for s in 5 6; do timeout 1200 bin/bat_br $s 40 > results/bat_br_$s.txt; done
for s in 31 32; do DIAG=1 timeout 1200 bin/strip_br $s 15 > results/strip_br_$s.txt; done
for m in row grid holed nest; do for op in union diff inter; do timeout 600 bin/pb_br $m $op 10000 3; done; done > results/pb_br.txt
echo done > results/runa.done
