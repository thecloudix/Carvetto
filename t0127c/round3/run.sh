#!/bin/bash
# run.sh bin seed mode   (resumable; dumps flagged scenes to d/bin)
cd $(dirname $(readlink -f $0))
log=sw/$1_$2_$3.log
grep -q SUMMARY $log 2>/dev/null && exit 0
D=d/$1; mkdir -p $D
DUMP=$D ./bin/$1 $2 50 $3 > $log.tmp 2>&1 && mv $log.tmp $log
