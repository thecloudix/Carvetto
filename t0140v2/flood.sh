#!/bin/bash
# flood.sh OUT P VARIANT "seedlist" MODE N [ENV...]: whole-scene flood sweep (t40), leads off (NOLEAD=1), DUMP of MISS/FALSE scenes.
B=/home/chris/wt-T0140/build-verify2-rel/bin; O=$1; P=$2; V=$3; S=$4; M=$5; N=${6:-50}; X="${7:-}"; mkdir -p $O/dump; echo $$ > $O/sweep.pid
for s in $S; do echo $s; done | xargs -P $P -I{} bash -c "[ -f $O/{}_m$M.txt ] && grep -q SUMMARY $O/{}_m$M.txt && exit 0; env $X NOLEAD=1 DUMP=$O/dump timeout 7200 $B/t40_$V {} $N $M > $O/{}_m$M.txt.tmp 2>&1; rc=\$?; mv $O/{}_m$M.txt.tmp $O/{}_m$M.txt; [ \$rc = 0 ] || echo rc=\$rc {}" >> $O/run.log 2>&1
echo ALLDONE >> $O/run.log
