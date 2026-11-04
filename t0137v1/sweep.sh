#!/bin/bash
# sweep.sh BINDIR P: random-scene hash sweep, main and branch, P jobs at a time -> BINDIR/out/<v>_<seed>_<S>_<O>.txt
D=$1; mkdir -p $D/out
{
for s in $(seq 101 118); do echo "$s 120 1 0"; done
for s in 201 202 203; do echo "$s 80 1 100000"; done
for s in 211 212; do echo "$s 60 1 10000000"; done
for s in 221 222; do echo "$s 80 0.01 0"; done
for s in 231 232; do echo "$s 80 1000 0"; done
echo "241 60 0.001 -500"; echo "242 60 37.5 -3.3e6"
} | while read s n S O; do for v in br main; do echo "$v $s $n $S $O"; done; done | \
 xargs -P ${2:-4} -L 1 bash -c 'timeout 3600 '$D'/hs_$0 $1 $2 $3 $4 > '$D'/out/$0_$1_$3_$4.txt 2>&1; echo "done $0 $1 $3 $4 rc=$?"' > $D/sweep.log 2>&1
echo ALLDONE >> $D/sweep.log
