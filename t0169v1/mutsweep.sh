#!/bin/sh
# usage: mutsweep.sh <jobs> <mutants...>: pk_mut_<m> over families 0-8 (base and PK_DFR=1, 100 cases) -> results/mutpk
B=/home/chris/wt-T0169/build-verify1-bin; R=/home/chris/harness/t0169v1/results/mutpk; J=$1; shift; mkdir -p $R
for m in "$@"; do for cfg in "base|" "dfr1|PK_DFR=1"; do for f in 0 1 2 3 4 5 6 7 8; do echo "$m|$cfg|$f"; done; done; done |
 xargs -P $J -I{} sh -c 'm=$(echo "{}" | cut -d"|" -f1); n=$(echo "{}" | cut -d"|" -f2); e=$(echo "{}" | cut -d"|" -f3); f=$(echo "{}" | cut -d"|" -f4);
   env $e timeout 1800 '$B'/pk_mut_$m $f 1000 100 > '$R'/${m}_${n}_f$f.txt 2>&1; echo "exit $?" >> '$R'/${m}_${n}_f$f.txt'
echo done > $R/DONE
