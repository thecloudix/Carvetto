#!/bin/bash
# hhjobs.sh seed mode [env...] -> hh/{dl3,dl3_base,dl3_main}_seed_mode_tag.txt (hashes, leads off)
cd /home/chris/wt-T0135/build-verify3/drv
s=$1; m=$2; shift 2; tag=$(echo "$@" | tr ' =' '__')
for b in dl3 dl3_base dl3_main; do
o=hh/${b}_${s}_${m}_${tag}.txt; grep -q SUMMARY $o 2>/dev/null && continue
env HASH=1 NOLEAD=1 "$@" ./bin/$b $s 50 $m > $o.tmp 2>&1 && mv $o.tmp $o
done
