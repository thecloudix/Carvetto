#!/bin/bash
# floodcmp4.sh OUT P LIST [configs]: LIST lines "scenefile VAR" (orig|vsout|bbc|opt). Whole-scene flood oracle (wsf.cpp):
# br = wsf_log (rule on, 999a90e), main = wsf_log TFS_OFF=1 (origin/main), r3 = wsf_r3 (520ca28, round 3 merged with main).
# -> OUT/<cfg>/<scene>_<VAR>.txt
B=/home/chris/wt-T0158/build-verify4-rel/bin; O=$1; P=$2; L=$3; C=${4:-br main}; mkdir -p $O; for c in $C; do mkdir -p $O/$c; done; echo $$ > $O/sweep.pid
while read f v; do for c in $C; do echo "$c $f $v"; done; done < $L | \
 xargs -P $P -L 1 bash -c 'c=$0; f=$1; v=$2; n=$(echo $f | sed "s|/home/chris/||; s|/|_|g"); E="X=1"; W=wsf_log; [ $c = main ] && E="TFS_OFF=1"; [ $c = r3 ] && W=wsf_r3; VV=""; [ $v != orig ] && VV="VAR=$v";
   env $E $VV SCENE=$f timeout 1800 '$B'/$W 0 1 99 > '$O'/$c/${n}_$v.txt 2>&1; rc=$?; [ $rc = 0 ] || echo "rc=$rc $c $f $v"' > $O/run.log 2>&1
echo ALLDONE >> $O/run.log
