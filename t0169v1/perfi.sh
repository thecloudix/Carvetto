#!/bin/bash
# usage: perfi.sh <out> <rounds>: Release CPU, interleaved main/branch (order alternates per round), one at a time.
# T-0131 pf cases, pp probes (big curved pockets, super-linear families), V-Carve AC2 (vperf, t0160v2 perf.cpp).
B=/home/chris/wt-T0169/build-verify1-bin; OUT=$1; R=${2:-3}; : > $OUT
cases=("pf|ANG=30|comb 1000" "pf|ANG=0|comb 1000" "pf|OFFSET=1|comb 1000" "pf|RD=0.1 TOOL=1|comb 1000" "pf|X=1|rest 5000 0" "pf|RASTER=1 ANG=30|rest 5000 0" "pf|RD=0.5|rest 5000 2" "pf|X=1|pockpage 5000 2"
 "pp|X=1|bigcirc 500" "pp|X=1|bigcirc 1000" "pp|X=1|bigcirc 1500" "pp|X=1|bigcircoff 1500" "pp|PP_DFR=0.1|bigcirc 1500"
 "pp|X=1|concave 200" "pp|X=1|concave 400" "pp|X=1|concave 800" "pp|X=1|concave 1600" "pp|X=1|concave 3000"
 "pp|X=1|gear 100" "pp|X=1|gear 200" "pp|X=1|gear 400" "pp|X=1|gear 800"
 "pp|X=1|wavy 50" "pp|X=1|wavy 100" "pp|X=1|wavy 200" "pp|X=1|wavy 400"
 "pp|X=1|islands 5" "pp|X=1|islands 10" "pp|X=1|reflex 12" "pp|X=1|reflex 25"
 "pk|PK_SCALE=100 PK_NOCHECK=1|9 1095 1" "pk|PK_SCALE=100 PK_NOCHECK=1|9 1004 1" "pk|PK_SCALE=100 PK_NOCHECK=1|9 1056 1" "pk|PK_SCALE=100 PK_NOCHECK=1|7 1095 1"
 "vperf|X=1|5000" "vperf|VPERF_FLAT=0.5|5000" "vperf|VPERF_FLAT=0.5 VPERF_EM=0.5|5000")
for ((r=1;r<=R;r++)); do
  for c in "${cases[@]}"; do b=${c%%|*}; rest=${c#*|}; e=${rest%%|*}; a=${rest#*|}
    if [ $((r % 2)) = 1 ]; then order="main br"; else order="br main"; fi
    for v in $order; do out=$(env $e timeout 900 $B/${b}_$v $a 2>&1 | grep -E "cpu| regions " | head -1); echo "r$r $v | $b $e $a | $out | load $(cut -d' ' -f1 /proc/loadavg)" >> $OUT; done
  done
done
echo DONE >> $OUT
