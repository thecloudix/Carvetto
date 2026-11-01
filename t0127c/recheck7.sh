#!/bin/bash
# recheck7.sh [sides regex] : every SEVERED tab in new conn7 logs -> dump scene, windowed flood (new and old lib)
cd $(dirname $(readlink -f $0))
SIDES=${1:-2}
python3 agg7.py | grep "^   " | grep SEVERED | while read tag rest; do
  sc=$(echo "$rest" | sed -E 's/^scene ([0-9]+) .*/\1/'); side=$(echo "$rest" | sed -E 's/.* side ([0-9]) .*/\1/')
  echo "$side" | grep -qE "^($SIDES)$" || continue
  set=${tag%%_*}; rest2=${tag#*_}; cfg=${rest2%%_*}; seed=${rest2#*_}
  line=$(grep -m1 " conn7 ${tag} " jobs_conn.txt); envs=$(echo $line | cut -d' ' -f7-); f=$(echo $line | cut -d' ' -f5); sp=$(echo $line | cut -d' ' -f6)
  scn=s/rc_${tag}_$sc.scn
  [ -s $scn ] || env CELL=0.05 $envs DUMPSCENE=$scn ./bin/conn7 $seed 50 $f $sp $sc > /dev/null 2>&1
  echo "$rest" | grep -oE "SEVERED tab at \([-0-9.]+,[-0-9.]+\) plateau [0-9.]+" | while read a b c xy d L; do
    xy=${xy#(}; xy=${xy%)}; x=${xy%,*}; y=${xy#*,}
    echo "$tag sc $sc side $side tab $x $y L $L: wflood new $(python3 -I wtab.py $scn $x $y $L) old $(python3 -I wtab.py $scn $x $y $L flood_old)"
  done
done
