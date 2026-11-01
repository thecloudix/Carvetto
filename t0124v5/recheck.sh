#!/bin/bash
# recheck every ON SEVERED (new binary): CELL 0.03 NOSEAM=1 and CELL 0.02
cd $(dirname $(readlink -f $0))
declare -A FAM=( [A]="0 -1" [B]="0 1" [C]="2 -1" [D]="6 -1" [E]="8 -1" [F]="5 -1" [G]="3 -1" [H]="8 1" [I]="1 -1" [J]="4 -1" [K]="7 -1" )
declare -A ENV=( [A]="ONP=4" [B]="ONP=4 TLMIN=0.3 TLMAX=4" [C]="ONP=3 TLMIN=0.3 TLMAX=4" [D]="ONP=3 TLMIN=0.3 TLMAX=4" [E]="ONP=4" [F]="X=1" [G]="ONP=4" [H]="ONP=4 TLMIN=0.3 TLMAX=4" [I]="ONP=3" [J]="ONP=3" [K]="ONP=3" )
python3 agg.py | grep "^ON SEVERED" | sed -E "s/ON SEVERED ([a-z0-9]+)_([A-K])_([0-9]+) .*'sc': '([0-9]+)'.*/\1 \2 \3 \4/" | sort -u | while read set n seed sc; do
  e="${ENV[$n]}"; [ $set = n2 ] && e="$(echo $e | sed -E 's/ONP=[0-9]//') ONP=1"
  a=$(env CELL=0.03 NOSEAM=1 $e ./bin/conn6s $seed 50 ${FAM[$n]} $sc | grep SUMMARY | grep -oE "severed [0-9]+|countBad [0-9]+" | tr '\n' ' ')
  b=$(env CELL=0.02 $e ./bin/conn6s $seed 50 ${FAM[$n]} $sc | grep SUMMARY | grep -oE "severed [0-9]+|countBad [0-9]+" | tr '\n' ' ')
  echo "$set $n $seed sc$sc | noseam0.03: $a | cell0.02: $b"
done
