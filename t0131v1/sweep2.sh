#!/bin/bash
# sweep2.sh BINDIR P: rest of the hash sweep (far offsets, scales); Profile far from origin with fewer scenes
D=$1; mkdir -p $D/out
{
for s in 211 212 213; do echo "pock $s 12 1 10000000"; done
for s in 214 215; do echo "prof $s 4 1 1000000"; done
for s in 221 222 223; do echo "prof $s 12 0.01 0"; echo "pock $s 12 0.01 0"; done
for s in 231 232 233; do echo "prof $s 12 1000 0"; echo "pock $s 12 1000 0"; done
for s in 241 242; do echo "prof $s 12 0.001 -500"; echo "pock $s 12 37.5 -3.3e6"; done
} | while read w s n S O; do for v in br main; do echo "$v $w $s $n $S $O"; done; done | \
 xargs -P ${2:-4} -L 1 bash -c 'timeout 1500 '$D'/hs_$0 $1 $2 $3 $4 $5 > '$D'/out/$0_$1_$2_$4_$5.txt 2>&1; echo "done $0 $1 $2 $4 $5 rc=$?"' >> $D/sweep.log 2>&1
echo ALLDONE2 >> $D/sweep.log
