#!/bin/bash
# AC2: cusp sweeps (t0017v3 cusp.inc) branch vs current origin/main, Round. Serial.
D=/home/chris/wt-T0128/build-verify1-drv; L=/home/chris/harness/t0128v1/logs; cd $D
for a in "1 1.0328719931380388" "1 0.13" "1 3.175" "1 0.05" "1 0.37" "1 4.2" "2 1.0328719931380388" "2 3.175" "3 1.0328719931380388" "3 3.175" "0 1.0328719931380388" "0 0.13"; do
  set -- $a; timeout 3000 ./vdrv5 cusp $1 $2 30 1e-6 1 1 > $L/cusp_$1_$2.log 2>&1
done
echo done > $L/cusp.done
