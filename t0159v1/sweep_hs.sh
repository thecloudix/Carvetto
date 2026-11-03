#!/bin/bash
# sweep_hs.sh OUTDIR P: t0131v1 hs bit-exact Profile/Pocket hash sweep (scale/offset) for 4 builds
B=/tmp/claude-1000/-home-chris-Carvetto/120b15af-a822-46aa-8270-71a70ec2ea48/scratchpad/v159/bin
D=$1; mkdir -p $D
{
for s in $(seq 101 112); do echo "prof $s 15 1 0"; echo "pock $s 15 1 0"; done
for s in 201 202; do echo "prof $s 15 1 100000"; echo "pock $s 15 1 100000"; done
for s in 211 212; do echo "prof $s 12 1 10000000"; echo "pock $s 12 1 10000000"; done
for s in 221; do echo "prof $s 12 0.01 0"; echo "pock $s 12 0.01 0"; done
for s in 231; do echo "prof $s 12 1000 0"; echo "pock $s 12 1000 0"; done
} | while read w s n S O; do for v in mainrel brrel mainv3 brv3; do echo "$v $w $s $n $S $O"; done; done | \
xargs -P ${2:-4} -L 1 bash -c 'o='$D'/$0_$1_$2_$4_$5.txt; [ -s $o ] || timeout 3600 '$B'/hs_$0 $1 $2 $3 $4 $5 > $o 2>&1'
echo ALLDONE
