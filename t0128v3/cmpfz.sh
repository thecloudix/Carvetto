#!/bin/bash
# exactJunction change: cmp6 (alternating order) branch cae3fa7 vs e87f549 (cmp6_old: every differing offset listed) and vs origin/main (cmp6_main),
# fresh seeds 0x7b3Fxxxx (vs old: 3000 inputs per fam = 18 000 offsets x 9 fams; vs main: 900 / 450 inputs per fam).
D=/home/chris/wt-T0128/build-verify3-drv; L=/home/chris/harness/t0128v3/logs/fz; mkdir -p $L; cd $D
: > $L/jobs.txt
for f in 0 2 3 4 5 21 22 23 25; do for k in 0 1; do
  s=$(printf "0x7b3%x%04x" $f $((k*1500)))
  echo "timeout 3000 ./cmp6_old $f 1500 $s > $L/old_f${f}_$k.log 2>&1" >> $L/jobs.txt
done; done
for f in 5 2 3 4 0 23 22 25 21; do for k in 0 1 2 3 4 5; do
  [ $f -ge 20 ] && [ $k -ge 3 ] && continue
  s=$(printf "0x7b3%x%04x" $f $((k*150 + 4000)))
  echo "timeout 3000 ./cmp6_main $f 150 $s > $L/main_f${f}_$k.log 2>&1" >> $L/jobs.txt
done; done
xargs -d "\n" -P ${P:-2} -I{} bash -c {} < $L/jobs.txt
echo done > $L/fz.done
