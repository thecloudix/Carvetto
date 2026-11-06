#!/bin/bash
# perfrun.sh OUT REPS : Release CPU time of profileToolpath, base (51c2dd6) vs br (334a87d) interleaved per page (base, br, base, br, ...).
B=/home/chris/wt-T0140/build-verify1-rel/bin; O=$1; K=${2:-3}; echo $$ > $O.pid
pages=(
"rect 2000|A=0 SEP=0" "rect 2000|A=0.3 SEP=1" "rect 2000|A=0 SEP=0 SIDE=1" "rect 2000|TL=200" "arc 2000|" "arc 2000|TL=200" "bez 1000|" "bez 1000|TL=200 SEP=1"
"mix 2000|" "mix 2000|TL=200" "plate 1000|" "plate 1000|SIDE=1 TL=200" "letters 500|" "letters 500|TL=200 V=90 D=6" "bezwave 500|" "flower1 2000|TL=200" "gear1 4000|"
"row 2000|" "row 4000|" "row 2000|TL=200" "row 4000|TL=200" "mix 1000|V=90 D=6 TL=200" "rect 1000|GAP=0.3 TL=200" "rect 1000|GAP=0.3")
for pg in "${pages[@]}"; do f=${pg%%|*}; e=${pg#*|}; for i in $(seq $K); do for v in base br; do r=$(env $e timeout 1800 $B/perf_$v $f); echo "$v | $f | $e | $r"; done; done; done > $O
echo ALLDONE >> $O
