#!/bin/bash
# queue.sh: after pert_stored finishes, run the random sweep and the synthetic-family perturbation sweep
R=/home/chris/wt-T0158/build-verify2-rel/out; H=/home/chris/harness/t0158v2; echo $$ > $R/queue.pid
until grep -q ALLDONE $R/pert_stored/run.log 2>/dev/null; do sleep 20; done
timeout 10000 $H/rndsweep.sh $R/rnd 15
timeout 10000 $H/sweep.sh $R/pert_gen 15 /home/chris/wt-T0158/build-verify2-rel/gen.lst pert "log logoff logv3"
echo QDONE > $R/queue.done
