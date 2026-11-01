#!/bin/bash
# recheckio.sh : every SEVERED (In/Out) scene in new conn7 logs -> rerun that scene with CELL 0.03 NOSEAM=1 and CELL 0.02, new and old
cd $(dirname $(readlink -f $0))
python3 agg7.py "$@" | grep "^   " | grep SEVERED | while read tag rest; do
  sc=$(echo "$rest" | sed -E 's/^scene ([0-9]+) .*/\1/'); side=$(echo "$rest" | sed -E 's/.* side ([0-9]) .*/\1/')
  [ "$side" = 2 ] && continue
  rest2=${tag#*_}; seed=${rest2#*_}
  line=$(grep -m1 " conn7 ${tag} " jobs_conn.txt); envs=$(echo $line | cut -d' ' -f7-); f=$(echo $line | cut -d' ' -f5); sp=$(echo $line | cut -d' ' -f6)
  out=""
  for b in conn7 conn7_old; do
    a=$(env CELL=0.03 NOSEAM=1 $envs ./bin/$b $seed 50 $f $sp $sc | grep SUMMARY | grep -oE "severed [0-9]+" )
    c=$(env CELL=0.02 $envs ./bin/$b $seed 50 $f $sp $sc | grep SUMMARY | grep -oE "severed [0-9]+")
    out="$out | $b c0.03ns: $a c0.02: $c"
  done
  echo "$tag sc $sc side $side $out"
done
