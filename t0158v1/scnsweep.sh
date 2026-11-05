#!/bin/bash
# scnsweep.sh OUT P [list]: stored harness scenes (VARS=1: orig + vsout/bbc/opt variants), one process per scene and config.
# configs: log (rule on), logoff (TFS_OFF=1 = main), logv3, logv3off. Output OUT/<cfg>/<scene>.txt
B=/home/chris/wt-T0158/build-verify1-rel/bin; O=$1; P=${2:-8}; mkdir -p $O
L=${3:-$O/all.lst}
[ -f $L ] || find /home/chris/harness -name '*.scn' -not -path "*/t0163v1/*" -not -path "*/t0158v1/*" | sort > $L
for c in log logoff logv3 logv3off; do mkdir -p $O/$c; done
for f in $(cat $L); do for c in log logoff logv3 logv3off; do echo "$c $f"; done; done | \
 xargs -P $P -L 1 bash -c 'c=$0; f=$1; n=$(echo $f | sed "s|/home/chris/harness/||; s|/|_|g"); v=${c%off}; E=""; [ "$c" != "$v" ] && E="TFS_OFF=1";
   env $E VARS=1 timeout 900 '$B'/st_$v scn $f > '$O'/$c/$n.txt 2>&1; rc=$?; [ $rc = 0 ] || echo "rc=$rc $c $f"' > $O/run.log 2>&1
echo ALLDONE >> $O/run.log
