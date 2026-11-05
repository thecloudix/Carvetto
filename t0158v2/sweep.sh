#!/bin/bash
# sweep.sh OUT P LIST MODE [configs]: run st_<v> MODE (scn|pert) on every scene file in LIST, one process per scene and config.
# config names: <variant>[off]; "off" sets TFS_OFF=1 (rule disabled = main). Default configs: log logoff logv3 logv3off mlogv3.
# VARS=1 is passed through (orig + vsout/bbc/opt). Output OUT/<cfg>/<scene>.txt; writes OUT/sweep.pid.
B=/home/chris/wt-T0158/build-verify2-rel/bin; O=$1; P=$2; L=$3; M=$4; C=${5:-log logoff logv3 logv3off mlogv3}
mkdir -p $O; echo $$ > $O/sweep.pid
for c in $C; do mkdir -p $O/$c; done
for f in $(cat $L); do for c in $C; do echo "$c $f"; done; done | \
 xargs -P $P -L 1 bash -c 'c=$0; f=$1; n=$(echo $f | sed "s|/home/chris/||; s|/|_|g"); v=${c%off}; E=""; [ "$c" != "$v" ] && E="TFS_OFF=1";
   env $E timeout 1800 '$B'/st_$v '$M' $f > '$O'/$c/$n.txt 2>&1; rc=$?; [ $rc = 0 ] || echo "rc=$rc $c $f"' > $O/run.log 2>&1
echo ALLDONE >> $O/run.log
