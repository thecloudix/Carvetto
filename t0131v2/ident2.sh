#!/bin/bash
# ident2.sh BIN OUT -- re-runs a subset of t0131v1/ident.sh workloads on the round-2 branch build
# and compares each hash line with round 1's main results (t0131v1/results/ident_main.txt).
BIN=$1; OUT=$2; M=$(dirname $(readlink -f $0))/../t0131v1/results/ident_main.txt
: > $OUT
while IFS='|' read e a; do
  e=$(echo $e); r=$(env $e $BIN $a 2>&1); h=${r##*hash }
  ref=$(grep -F "$e | ${r%% cpu*}" $M | head -1); rh=${ref##*hash }
  [ "$h" = "$rh" ] && s=SAME || s="DIFF(main $rh)"
  echo "$e | $r | $s" | tee -a $OUT
done <<'L'
ANG=0 |comb 250
ANG=90 |comb 250
ANG=45 |comb 1000
RD=0.1 TOOL=1 |comb 500
PP=1 ANG=90 |comb 300
X=1 |sep 1250 0
TABS=1 |sep 1250 1
X=1 |rest 1250 2
X=1 |tabs 1250 0
REV=1 SIDE=1 |sep 1250 0
TABS=1 LEADS=1 ORDER=1 LPA=0.05 |sep 1250 1
RASTER=1 ANG=90 RD=0.5 |rest 1250 2
RASTER=1 ANG=45 |pockpage 2000 1
SIDE=2 |tabs 1250 2
L
