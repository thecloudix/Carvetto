#!/bin/bash
# hsdump.sh OUT P: hs runs with move dumps for mainrel and brrel (pocket seeds 101-106, prof 101-102, one 1e5 offset each)
B=/tmp/claude-1000/-home-chris-Carvetto/120b15af-a822-46aa-8270-71a70ec2ea48/scratchpad/v159/bin; D=$1
{ for s in 101 102 103 104 105 106; do echo "pock $s 15 1 0"; done; echo "pock 201 15 1 100000"; echo "prof 101 15 1 0"; echo "prof 102 15 1 0"; echo "prof 201 15 1 100000"; } | \
while read w s n S O; do for v in mainrel brrel; do echo "$v $w $s $n $S $O"; done; done | \
xargs -P ${2:-2} -L 1 bash -c 'mkdir -p '$D'/$0_$4_$5; DUMPMV='$D'/$0_$4_$5 '$B'/hs_$0 $1 $2 $3 $4 $5 > '$D'/$0_$1_$2_$4_$5.txt 2>&1'
echo ALLDONE
