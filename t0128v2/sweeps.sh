#!/bin/bash
# AC1 channel sweep (chan) and AC2 cusp sweeps (vdrv5 cusp, Round), branch vs origin/main; P parallel.
D=/home/chris/wt-T0128/build-verify2-drv; L=/home/chris/harness/t0128v2/logs; cd $D
{
for d in 0.1317 1 3.175; do echo "timeout 3000 ./chan 3.6 0 1 $d > $L/chan_$d.log 2>&1"; done
echo "timeout 3000 ./chan 20 0 0 3.175 > $L/chanL20_3.175.log 2>&1"
for a in "1 1.0328719931380388" "1 0.13" "1 3.175" "1 0.05" "1 4.2" "2 1.0328719931380388" "2 3.175" "3 1.0328719931380388" "0 1.0328719931380388" "0 0.13"; do
  set -- $a; echo "timeout 3000 ./vdrv5 cusp $1 $2 30 1e-6 1 1 > $L/cusp_$1_$2.log 2>&1"
done
} > $L/sweeps_jobs.txt
xargs -P ${P:-2} -I{} bash -c "{}" < $L/sweeps_jobs.txt
echo done > $L/sweeps.done
