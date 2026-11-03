#!/bin/bash
cd /home/chris/wt-T0135/build-verify2/drv
s=$1; m=$2; shift 2; tag=$(echo "$@" | tr ' =' '__')
for b in hh hh_base hh_main; do
o=hh/${b}_${s}_${m}_${tag}.txt; grep -q SUMMARY $o 2>/dev/null && continue
if [[ "$*" == *LEADS=1* ]]; then env HASH=1 "$@" ./bin/$b $s 50 $m > $o.tmp 2>&1 && mv $o.tmp $o; continue; fi; env HASH=1 NOLEAD=1 "$@" ./bin/$b $s 50 $m > $o.tmp 2>&1 && mv $o.tmp $o
done
