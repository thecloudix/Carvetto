#!/bin/bash
# perfrun2.sh LIST OUT : LIST lines "family n|ENV...|K"; Release CPU of profileToolpath base (51c2dd6) vs br (334a87d), interleaved
# (base, br) K times per page.
B=/home/chris/wt-T0140/build-verify1-rel/bin; L=$1; O=$2; echo $$ > $O.pid
while IFS='|' read f e k; do for i in $(seq $k); do for v in base br; do r=$(env $e timeout 1800 $B/perf_$v $f); echo "$v | $f | $e | $r"; done; done; done < $L > $O
echo ALLDONE >> $O
