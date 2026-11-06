#!/bin/bash
# perfrun.sh LIST OUT : LIST lines "family n|ENV...|K"; CPU of profileToolpath, base (fa856ec) vs br (40ccd46), K interleaved pairs per page,
# order alternating per pair (base,br / br,base).
B=/home/chris/wt-T0140/build-verify2-rel/bin; L=$1; O=$2; echo $$ > $O.pid
while IFS='|' read f e k; do for i in $(seq $k); do if [ $((i % 2)) = 1 ]; then o="base br"; else o="br base"; fi
  for v in $o; do r=$(env $e timeout 1800 $B/perf_$v $f); echo "$v | $f | $e | $r"; done; done; done < $L > $O
echo ALLDONE >> $O
