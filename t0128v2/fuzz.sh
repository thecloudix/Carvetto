#!/bin/bash
# AC3: branch vs origin/main, cmp6 (xk + vchk; call order alternating), fresh seeds 0x7a2Fxxxx, chunks of 150 inputs, P parallel.
D=/home/chris/wt-T0128/build-verify2-drv; L=/home/chris/harness/t0128v2/logs/fz; cd $D
: > $L/jobs.txt
for k in 0 1 2 3 4 5; do
  for f in 5 2 3 4 0 23 22 25 21; do
    s=$(printf "0x7a2%x%04x" $f $((k*150)))
    [ $f -ge 20 ] && [ $k -ge 3 ] && continue
    echo "timeout 2400 ./cmp6 $f 150 $s > $L/f${f}_$k.log 2>&1" >> $L/jobs.txt
  done
done
xargs -P ${P:-3} -I{} bash -c "{}" < $L/jobs.txt
echo done > $L/fuzz.done
