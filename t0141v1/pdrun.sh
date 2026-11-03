#!/bin/bash
cd /home/chris/wt-T0141/build-verify/drv
for k in 8 9; do for s in $(seq 0 24); do
  KIND=$k DUMP=1 ./pcw_old 1 25 0 $s > pd/o_${k}_$s.txt; KIND=$k DUMP=1 ./pcw_new 1 25 0 $s > pd/n_${k}_$s.txt
  ch=$(grep -o "chordal [0-9.]*" pd/o_${k}_$s.txt | head -1)
  if cmp -s <(grep "^t" pd/o_${k}_$s.txt) <(grep "^t" pd/n_${k}_$s.txt); then echo "$k $s same"; else echo "$k $s $ch $(python3 mvcmp.py pd/o_${k}_$s.txt pd/n_${k}_$s.txt | awk '{for(i=1;i<=NF;i++) if($i=="old->new"||$i=="new->old") printf "%s ", $(i+1)}')"; fi
  rm -f pd/o_${k}_$s.txt pd/n_${k}_$s.txt
done; done > pd/summary.txt 2>&1
echo done >> pd/summary.txt
