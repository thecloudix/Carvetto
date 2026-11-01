#!/bin/bash
cd /home/claude/vscratch/t0124v4
export CELL=0.04
run(){ name=$1; shift; echo "=== $name" ; env "$@" 2>&1 | grep -E "^scene|SUMMARY|tab \(|diag 10"; }
for sc in 60 235; do run B$sc TLMIN=0.3 TLMAX=4 ./conn3 41002 300 0 1 $sc; done
for sc in 9 93; do run F$sc ./conn3 41006 150 5 -1 $sc; done
for sc in 26 131; do run G$sc ./conn3 41007 150 3 -1 $sc; done
for sc in 6 45; do run J$sc ./conn3 41010 150 2 -1 $sc; done
echo done
