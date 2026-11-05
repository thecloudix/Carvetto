#!/bin/bash
# pertsweep.sh OUT P LIST [configs]: st pert (21 transforms: +-1..4 ulp scale, 1+-1e-12/1e-10/1e-9, 3 translations,
# exact quarter-turn rotations) per scene in LIST, configs default "log logoff logv3" -> OUT/<cfg>/<scene>.txt
B=/home/chris/wt-T0158/build-verify1-rel/bin; O=$1; P=$2; L=$3; C=${4:-log logoff logv3}
for c in $C; do mkdir -p $O/$c; done
for f in $(cat $L); do for c in $C; do echo "$c $f"; done; done | \
 xargs -P $P -L 1 bash -c 'c=$0; f=$1; n=$(echo $f | sed "s|/home/chris/||; s|/|_|g"); v=${c%off}; E=""; [ "$c" != "$v" ] && E="TFS_OFF=1";
   env $E $PEXTRA timeout 1200 '$B'/st_$v pert $f > '$O'/$c/$n.txt 2>&1; rc=$?; [ $rc = 0 ] || echo "rc=$rc $c $f"' > $O/run.log 2>&1
echo ALLDONE >> $O/run.log
