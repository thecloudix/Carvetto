#!/bin/bash
# usage: time.sh <files...>; prints file br_user br_maxrssKB main_user main_maxrssKB br_msg | main_msg
BR=/home/chris/wt-T0134/build-verify3/app/cli/carvetto-cli
MN=/home/chris/harness/t0134v2/main-build/app/cli/carvetto-cli
BASE=/home/chris/harness/t0122v1/base.carv
O=/home/chris/wt-T0134/build-verify3/pj/out
mkdir -p $O
for f in "$@"; do
  b=$( { /usr/bin/time -f "%U %M" $BR toolpath profile $f $BASE $O/b.carv >/dev/null 2>$O/bm; } 2>&1 | tail -1); bt=$(tail -1 $O/bm); bm=$(head -1 $O/bm | cut -c1-70)
  m=$( { /usr/bin/time -f "%U %M" $MN toolpath profile $f $BASE $O/m.carv >/dev/null 2>$O/mm; } 2>&1 | tail -1); mt=$(tail -1 $O/mm); mm=$(head -1 $O/mm | cut -c1-50)
  echo "$(basename $f) BR[$bt] MAIN[$mt] | $bm | $mm"
done
