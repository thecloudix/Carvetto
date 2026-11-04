#!/bin/sh
# usage: repros.sh <dir with vc3> <out dir>: every round-1/2 repro through vc3 (random tools, 40 cases) and fixed params
H=$1; R=$2; mkdir -p $R
for f in repro2/*.txt repro1/*.txt; do b=$(basename $f .txt); d=$(basename $(dirname $f));
  case $b in *ball*|*moves*) ;; esac
  echo "$d/$b|$f"; done | xargs -P 2 -I{} sh -c 'n=$(echo "{}" | cut -d"|" -f1 | tr / _); f=$(echo "{}" | cut -d"|" -f2);
  VC_FILE=$f '$H'/vc3 77 40 > '$R'/$n.rand.txt 2>&1;
  for c in 0.1 0.05 0.01 0.001; do VC_FILE=$f VC_FIX="0 12 90 $c 0 0" '$H'/vc3 1 1 v; VC_FILE=$f VC_FIX="1 11.606 0 $c 0 0" '$H'/vc3 1 1 v; VC_FILE=$f VC_FIX="0 6 60 $c 0.3 0" '$H'/vc3 1 1 v; done > '$R'/$n.fix.txt 2>&1'
echo done
