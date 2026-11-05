#!/bin/sh
# usage: advhash.sh <vc3 with VC_HASH> <dir of .txt> <out file> [jobs]: same run matrix as runadv.sh, hash only.
V=$1; D=$2; OUT=$3; J=${4:-4}
for f in $D/*.txt; do
  for c in 0.1 0.05 0.01; do for dd in def 1e-5; do for t in "0 12 90" "1 6 0"; do echo "$f|$c|$dd|$t"; done; done; done
  for t in "0 12 90" "1 6 0"; do echo "$f|1.0|1e-5|$t"; done; echo "$f|0.1|def|0 6 30"
done |
xargs -P $J -I{} sh -c 'f=$(echo "{}"|cut -d"|" -f1); c=$(echo "{}"|cut -d"|" -f2); dd=$(echo "{}"|cut -d"|" -f3); t=$(echo "{}"|cut -d"|" -f4);
  if [ $dd = def ]; then unset VC_DIST; else export VC_DIST=$dd; fi
  r=$(VC_HASH=1 VC_HASHONLY=1 VC_FILE=$f VC_FIX="$t $c 0 0" timeout 600 '$V' 1 1 2>&1 | grep HASH; [ $? = 124 ] && echo TIMEOUT)
  echo "$(basename $f) ch=$c dist=$dd tool=[$t] :: $r"' | sort > $OUT
