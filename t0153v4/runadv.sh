#!/bin/sh
# usage: runadv.sh <bin_vc3> <dir of .txt> <out file> [jobs]
# each file x {chordal 0.1, 0.05, 0.01 with distance default and 1e-5; chordal 1.0 with distance 1e-5} x tool {V90 D12, ball D6};
# plus V30 D6 at chordal 0.1. vc3 single fixed case, one line per run, 600 s cap each (TIMEOUT marks a cap).
V=$1; D=$2; OUT=$3; J=${4:-4}
for f in $D/*.txt; do
  for c in 0.1 0.05 0.01; do for dd in def 1e-5; do for t in "0 12 90" "1 6 0"; do echo "$f|$c|$dd|$t"; done; done; done
  for t in "0 12 90" "1 6 0"; do echo "$f|1.0|1e-5|$t"; done; echo "$f|0.1|def|0 6 30"
done |
xargs -P $J -I{} sh -c 'f=$(echo "{}"|cut -d"|" -f1); c=$(echo "{}"|cut -d"|" -f2); dd=$(echo "{}"|cut -d"|" -f3); t=$(echo "{}"|cut -d"|" -f4);
  if [ $dd = def ]; then unset VC_DIST; else export VC_DIST=$dd; fi
  r=$(VCVIOL= VC_FILE=$f VC_FIX="$t $c 0 0" timeout 600 '$V' 1 1 v 2>&1; [ $? = 124 ] && echo TIMEOUT); r=$(echo "$r" | grep -E "^case|TIMEOUT|SUMMARY|OUTSIDE|VIOL move|diag" | head -n 8 | tr "\n" " ");
  echo "$(basename $f) ch=$c dist=$dd tool=[$t] :: $r"' > $OUT
echo done
