#!/bin/bash
# floodcmp5.sh OUT P LIST [configs]: LIST lines "scenefile VAR" (orig|vsout|bbc|opt). Whole-scene flood oracle (wsf.cpp):
# br = wsf_br5 (1156cad), main = wsf_main (origin/main 34db4d0), r4 = wsf_r4 (999a90e, round 4; based on main c2aa6f4).
B=/home/chris/wt-T0158/build-verify5-rel/bin; O=$1; P=$2; L=$3; C=${4:-br main r4}; mkdir -p $O; for c in $C; do mkdir -p $O/$c; done; echo $$ > $O/sweep.pid
while read f v; do for c in $C; do echo "$c $f $v"; done; done < $L | \
 xargs -P $P -L 1 bash -c 'c=$0; f=$1; v=$2; n=$(echo $f | sed "s|/home/chris/||; s|/|_|g"); W=wsf_br5; [ $c = main ] && W=wsf_main; [ $c = r4 ] && W=wsf_r4; VV=""; [ $v != orig ] && VV="VAR=$v";
   env $VV SCENE=$f timeout 1800 '$B'/$W 0 1 99 > '$O'/$c/${n}_$v.txt 2>&1; rc=$?; [ $rc = 0 ] || echo "rc=$rc $c $f $v"' > $O/run.log 2>&1
echo ALLDONE >> $O/run.log
