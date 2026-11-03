#!/bin/bash
# fcand.sh bin seed sc mode [env...] -> fc/bin_seed_sc_mode_tag.txt (flood oracle on one scene, leads off)
cd /home/chris/wt-T0135/build-verify2/drv
b=$1; s=$2; c=$3; m=$4; shift 4; tag=$(echo "$@" | tr ' =' '__')
o=fc/${b}_${s}_${c}_${m}_${tag}.txt; grep -q SUMMARY $o 2>/dev/null && exit 0; [ -e $o.tmp ] && exit 0
bin=${b%%+*}
env NOLEAD=1 TABS=1 SC=$c "$@" ./bin/$bin $s 50 $m > $o.tmp 2>&1 && mv $o.tmp $o
