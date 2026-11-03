#!/bin/bash
cd /home/chris/wt-T0135/build-verify3/drv
s=$1; m=$2; shift 2; tag=$(echo "$@" | tr ' =' '__')
o=hha/dl3_allA_${s}_${m}_${tag}.txt; grep -q SUMMARY $o 2>/dev/null && exit 0
env HASH=1 NOLEAD=1 "$@" ./bin/dl3_allA $s 50 $m > $o.tmp 2>&1 && mv $o.tmp $o
