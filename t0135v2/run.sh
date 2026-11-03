#!/bin/bash
# run.sh bin seed mode [env...] -> sw/bin_seed_mode.log ; flagged scenes dumped to d/bin
cd /home/chris/wt-T0135/build-verify2/drv
b=$1; s=$2; m=$3; shift 3
log=sw/${b}_${s}_${m}.log
grep -q SUMMARY $log 2>/dev/null && exit 0
[ -e $log.tmp ] && exit 0
D=d/$b; mkdir -p $D
env NOLEAD=1 TABS=1 DUMP=$D "$@" ./bin/$b $s 50 $m > $log.tmp 2>&1 && mv $log.tmp $log
