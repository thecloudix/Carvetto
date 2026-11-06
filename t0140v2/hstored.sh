#!/bin/bash
# hstored.sh OUT P : HASH=1 over the stored harness scenes (minus t0127v1/d4old and t0127v2/d/wsf7_old duplicates):
# as stored (leads etc. as saved) and NOLEAD + FORCEIO (Outside/Inside alternating by line, allowance >= 0), variants br and base.
B=/home/chris/wt-T0140/build-verify2-rel/bin; O=$1; P=$2; mkdir -p $O; echo $$ > $O/sweep.pid
find /home/chris/harness -name "*.scn" | grep -v "d4old\|wsf7_old" | sort > $O/list.txt
for v in br base; do n=0; while read f; do n=$((n+1)); echo "$v plain $f"; echo "$v FIO$((n%2)) $f"; done < $O/list.txt; done | \
 xargs -P $P -L 1 bash -c 'v=$0; e=$1; f=$2; E=""; case $e in FIO0) E="NOLEAD=1 FORCEIO=0";; FIO1) E="NOLEAD=1 FORCEIO=1";; esac; r=$(env $E HASH=1 SCENE=$f timeout 600 '$B'/t40_$v 0 1 99 | grep "^H "); echo "$v $e $f $r"' > $O/h.txt 2>&1
echo ALLDONE >> $O/h.txt
