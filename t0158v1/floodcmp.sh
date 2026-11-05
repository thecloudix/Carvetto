#!/bin/bash
# floodcmp.sh OUT P LIST: LIST lines "scenefile VAR" (VAR = orig|vsout|bbc|opt). Whole-scene flood (wsf.cpp, from
# t0135v3 wsf8w) of the branch (wsf_log, rule on) and main (TFS_OFF=1) -> OUT/{br,main}/<scene>_<VAR>.txt
B=/home/chris/wt-T0158/build-verify1-rel/bin; O=$1; P=$2; L=$3; mkdir -p $O/br $O/main
while read f v; do for c in br main; do echo "$c $f $v"; done; done < $L | \
 xargs -P $P -L 1 bash -c 'c=$0; f=$1; v=$2; n=$(echo $f | sed "s|/home/chris/||; s|/|_|g"); E="X=1"; [ $c = main ] && E="TFS_OFF=1"; VV=""; [ $v != orig ] && VV="VAR=$v";
   env $E $VV SCENE=$f timeout 1800 '$B'/wsf_log 0 1 99 > '$O'/$c/${n}_$v.txt 2>&1; rc=$?; [ $rc = 0 ] || echo "rc=$rc $c $f $v"' > $O/run.log 2>&1
echo ALLDONE >> $O/run.log
