#!/bin/bash
# run.sh bin seed mode  -> sw/bin_seed_mode.log ; flagged scenes dumped to d/bin
cd /home/chris/wt-T0127/build-verify3/drv
log=sw/$1_$2_$3.log
grep -q SUMMARY $log 2>/dev/null && exit 0
D=d/$1; mkdir -p $D sw
TABS=1 DUMP=$D ./bin/$1 $2 50 $3 > $log.tmp 2>&1 && mv $log.tmp $log
