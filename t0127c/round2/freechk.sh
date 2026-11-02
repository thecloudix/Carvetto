#!/bin/bash
# freechk.sh: every FREE scene in the new wsf4 logs -> is the loose piece reported (diag "not held by any kept tab")?
cd $(dirname $(readlink -f $0))
for f in sw4/wsf4_2*.log; do
  b=$(basename $f .log); seed=$(echo $b | cut -d_ -f2); mode=$(echo $b | cut -d_ -f3)
  grep "FREE" $f | while read -r line; do
    sc=$(echo "$line" | awk '{print $3}'); al=$(echo "$line" | grep -oE "allLost [0-9]+"); area=$(echo "$line" | grep -oE "biggest [0-9.]+")
    scn=d4new/m${mode}_${seed}_${sc}.scn
    rep=$(./bin/mvdump $scn 2>/dev/null | grep -c "not held by any kept tab")
    kept=$(echo "$line" | grep -oE "\| G [0-9]+" | grep -oE "[0-9]+")
    echo "m${mode}_${seed}_${sc} $area $al keptG $kept reported $rep"
  done
done
