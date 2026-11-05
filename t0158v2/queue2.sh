#!/bin/bash
# queue2.sh: after queue.sh, flood-compare every changed stored run (tabs on), then dump + flood the changed random scenes
R=/home/chris/wt-T0158/build-verify2-rel/out; H=/home/chris/harness/t0158v2; echo $$ > $R/queue2.pid
until [ -f $R/queue.done ]; do sleep 20; done
timeout 12000 $H/floodcmp.sh $R/flood_stored 15 $H/results/flood_stored_changed_tabs.lst
timeout 3000 python3 $H/rnddump.py $R/rnd $R/rnddump > $H/results/flood_random_changed.lst
timeout 6000 $H/floodcmp.sh $R/flood_random 15 $H/results/flood_random_changed.lst
echo Q2DONE > $R/queue2.done
