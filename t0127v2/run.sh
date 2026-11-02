#!/bin/bash
cd /home/chris/wt-T0127/build-verify2/drv
case "$1 $2" in "wsf7_old 29100"[2-7]|"wsf7_old 29200"[2-3]) exit 0;; esac
log=sw/$1_$2_$3.log
grep -q SUMMARY $log 2>/dev/null && exit 0
D=d/$1; mkdir -p $D
DUMP=$D ./bin/$1 $2 50 $3 > $log.tmp 2>&1 && mv $log.tmp $log
