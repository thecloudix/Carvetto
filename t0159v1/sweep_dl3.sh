#!/bin/bash
# sweep_dl3.sh OUTDIR P: dl3 HASH screen (Profile, tabs, per-vector left-out) for 4 builds
B=/tmp/claude-1000/-home-chris-Carvetto/120b15af-a822-46aa-8270-71a70ec2ea48/scratchpad/v159/bin
D=$1; mkdir -p $D
{ for s in $(seq 1301 1340); do for m in 0 1 2; do echo "$s $m FORCESEP=1"; done; echo "$s 2 X=1"; echo "$s 1 FORCEIOPOS=1"; done; } | \
while read s m e; do for v in ${VARIANTS:-mainrel brrel mainv3 brv3}; do echo "$v $s $m $e"; done; done | \
xargs -P ${2:-4} -L 1 bash -c 'o='$D'/$0_$1_$2_$3.txt; [ -s $o ] || env HASH=1 NOLEAD=1 $3 '$B'/dl3_$0 $1 50 $2 > $o 2>&1' 
echo ALLDONE
