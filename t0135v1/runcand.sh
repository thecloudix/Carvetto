#!/bin/bash
# runcand.sh bin listfile out [env...] : run oracle on single scenes (seed sc mode per line)
cd /home/chris/wt-T0135/build-verify/drv
b=$1; L=$2; o=$3; shift 3
while read s c m; do grep -q "^S $s $c " $o 2>/dev/null && continue; env NOLEAD=1 TABS=1 SC=$c "$@" ./bin/$b $s 50 $m | grep -v SUMMARY >> $o; done < $L
echo DONE >> $o.done
