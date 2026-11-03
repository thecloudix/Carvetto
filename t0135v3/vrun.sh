#!/bin/bash
# vrun.sh bin seed mode N [env...] -> vs/bin_seed_mode_tag.log (vector free-piece oracle; leads off)
cd /home/chris/wt-T0135/build-verify3/drv
b=$1; s=$2; m=$3; n=$4; shift 4; tag=$(echo "$@" | tr ' =' '__')
log=vs/${b}_${s}_${m}_${tag}.log
grep -q SUMMARY $log 2>/dev/null && exit 0
D=dv/$b; mkdir -p $D
env NOLEAD=1 DUMP=$D "$@" timeout 3000 ./bin/$b $s $n $m > $log.tmp 2>&1; mv $log.tmp $log
