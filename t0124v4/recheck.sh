#!/bin/bash
# re-run each SEVERED scene: with and without the seam extension; print verdict
cd /home/claude/vscratch/t0124v4
declare -A ENV=( [A]="ONP=4" [B]="ONP=4 TLMIN=0.3 TLMAX=4" [C]="ONP=3 TLMIN=0.3 TLMAX=4" [D]="ONP=3 TLMIN=0.3 TLMAX=4" [E]="ONP=4" [F]="X=1" [G]="ONP=4" [H]="ONP=4 TLMIN=0.3 TLMAX=4" [I]="ONP=3" [J]="ONP=3" [K]="ONP=3" )
declare -A FAM=( [A]="0 -1" [B]="0 1" [C]="2 -1" [D]="6 -1" [E]="8 -1" [F]="5 -1" [G]="3 -1" [H]="8 1" [I]="1 -1" [J]="4 -1" [K]="7 -1" )
for f in sw/*.log; do b=$(basename $f .log); n=${b:3:1}; seed=${b:5}
  grep -h "SEVERED" $f | while read -r line; do sc=$(echo "$line" | awk '{print $2}')
    for ns in 0 1; do e="${ENV[$n]}"; [ $ns = 1 ] && e="$e NOSEAM=1"
      r=$(env CELL=0.03 $e DUMPSCENE=repro/sev_${n}_${seed}_${sc}.scn ./conn6 $seed 50 ${FAM[$n]} $sc | grep SUMMARY | sed -E 's/.*tabs kept ([0-9]+) checked ([0-9]+) severed ([0-9]+).*requested ([0-9]+) left ([0-9]+) countBad ([0-9]+).*/kept \1 sev \3 req \4 left \5 cbad \6/')
      echo "$n $seed sc$sc noseam=$ns: $r"; done; done; done
