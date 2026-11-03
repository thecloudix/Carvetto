#!/bin/bash
# floodcmp.sh LIST OUT P vA vB : runs the dl3 flood oracle (bin2/dl3f_*) for each listed scene on both builds
B=/tmp/claude-1000/-home-chris-Carvetto/120b15af-a822-46aa-8270-71a70ec2ea48/scratchpad/v159/bin2
mkdir -p $2; cat $1 | while read s m e sc; do for v in $4 $5; do echo "$v $s $m $e $sc"; done; done | \
xargs -P ${3:-3} -L 1 bash -c 'o='$2'/$0_$1_$2_$3_$4.txt; [ -s $o ] || env NOLEAD=1 $3 SC=$4 '$B'/dl3f_$0 $1 50 $2 | grep "^S " > $o'
