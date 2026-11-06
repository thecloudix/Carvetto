#!/bin/bash
# output identity: cmp6 branch vs origin/main (alternating call order), fresh seeds 0x9c1Fkkkk, chunks of 150 inputs, P parallel.
D=/home/chris/wt-T0172/build-verify2-drv; L=/home/chris/harness/t0172v2/logs/fz; mkdir -p $L; cd $D
J=$L/jobs_${K0:-0}.txt; : > $J
for k in $(seq ${K0:-0} ${K1:-7}); do
  for f in 5 2 3 4 0 23 22 25 21; do
    s=$(printf "${PFX:-0x9d2}%x%04x" $f $((k*150)))
    [ $f -ge 20 ] && [ $k -ge ${K2:-4} ] && continue
    echo "timeout 3000 ./cmp6 $f 150 $s > $L/f${f}_$k.log 2>&1" >> $J
  done
done
xargs -P ${P:-4} -I{} bash -c "{}" < $J
echo done > $L/fuzz_${K0:-0}.done
