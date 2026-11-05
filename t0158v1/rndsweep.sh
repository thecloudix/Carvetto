#!/bin/bash
# rndsweep.sh OUT P: random scenes (hs generator; seeds as t0163v1 sweep.sh, smaller N), 4 configs as scnsweep.sh
B=/home/chris/wt-T0158/build-verify1-rel/bin; O=$1; mkdir -p $O
for c in log logoff logv3 logv3off; do mkdir -p $O/$c; done
{
for s in $(seq 101 ${SEEDMAX:-112}); do echo "$s 120 1 0"; done
for s in 201 202 203; do echo "$s 80 1 100000"; done
for s in 221 222 223 224; do echo "$s 80 0.01 0"; done
echo "242 60 37.5 -3.3e6"
for s in 243 244; do echo "$s 60 0.001 0"; done
} | while read s n S X; do for c in log logoff logv3 logv3off; do echo "$c $s $n $S $X"; done; done | \
 xargs -P ${2:-8} -L 1 bash -c 'c=$0; v=${c%off}; E=""; [ "$c" != "$v" ] && E="TFS_OFF=1";
   env $E timeout 5400 '$B'/st_$v $1 $2 $3 $4 > '$O'/$c/$1_$3_$4.txt 2>&1; echo "done $c $1 rc=$?"' > $O/run.log 2>&1
echo ALLDONE >> $O/run.log
