#!/bin/bash
cd $(dirname $(readlink -f $0))
log=sw4/$1_$2_$3.log
grep -q SUMMARY $log 2>/dev/null && exit 0
D=d4new; [ "${1%_old}" != "$1" ] && D=d4old
DUMP=$D ./bin/$1 $2 50 $3 > $log.tmp 2>&1 && mv $log.tmp $log
