#!/bin/bash
# AC3 (chunked, not niced): branch vs origin/main, cmp6 (xk + vchk), fresh seeds 0x7128xxxx, chunks of 150 inputs, P parallel.
D=/home/chris/wt-T0128/build-verify1-drv; L=/home/chris/harness/t0128v1/logs/fz; cd $D
: > $L/jobs.txt
for k in 0 1 2 3 4 5 6 7 8 9; do
  for f in 5 2 3 4 23 22 25 21 0; do
    s=$(printf "0x7128%x%03x" $f $((k*150)))
    [ $f -ge 20 ] && [ $k -ge 4 ] && continue
    [ $f -eq 0 ] && [ $k -ge 4 ] && continue
    echo "timeout 2400 ./cmp6 $f 150 $s > $L/f${f}_$k.log 2>&1" >> $L/jobs.txt
  done
done
xargs -P ${P:-4} -I{} bash -c "{}" < $L/jobs.txt
echo done > $L/fuzz.done
