#!/bin/bash
cd /home/chris/wt-T0135/build-verify/drv
while [ ! -f cand_mut.txt.done ]; do sleep 15; done
for b in wsf8x wsf8x_base wsf8x_old; do ./runcand.sh $b cand2.txt cand2_$b.txt; done
./runcand.sh wsf8x_old cand3.txt cand_old.txt
