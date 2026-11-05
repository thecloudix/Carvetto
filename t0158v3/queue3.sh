#!/bin/bash
# queue3.sh (replaces queue.sh/queue2.sh tail): after pert_gen, flood the changed stored runs; after rnd, dump + flood changed random scenes
R=/home/chris/wt-T0158/build-verify3-rel/out; H=/home/chris/harness/t0158v3; echo $$ > $R/queue3.pid
until grep -q ALLDONE $R/pert_gen/run.log 2>/dev/null; do sleep 20; done
timeout 12000 $H/floodcmp.sh $R/flood_stored 15 $H/results/flood_stored_changed_tabs.lst
until grep -q ALLDONE $R/rnd/run.log 2>/dev/null; do sleep 20; done
timeout 3000 python3 $H/rnddump.py $R/rnd $R/rnddump > $H/results/flood_random_changed.lst
timeout 6000 $H/floodcmp.sh $R/flood_random 15 $H/results/flood_random_changed.lst
echo Q3DONE > $R/queue3.done
