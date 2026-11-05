#!/bin/bash
# q3.sh: round-3 sweep chain (each step hard-timed): gen3 pert, gen1+gen2 pert, stored scn VARS (4 configs), stored pert, random.
R=/home/chris/wt-T0158/build-verify3-rel; H=/home/chris/harness/t0158v3; O=$R/out; P=${P:-8}; echo $$ > $O/q3.pid
timeout 7200 $H/sweep.sh $O/pert_gen3 $P $R/gen3.lst pert "log logoff logv3"; echo "gen3 done $(date)" >> $O/q3.log
timeout 14400 $H/sweep.sh $O/pert_gen12 $P $R/gen12.lst pert "log logoff logv3"; echo "gen12 done $(date)" >> $O/q3.log
VARS=1 timeout 14400 $H/sweep.sh $O/scn $P $R/stored_scenes.lst scn "log logoff logv3 logv3off"; echo "scn done $(date)" >> $O/q3.log
VARS=1 timeout 14400 $H/sweep.sh $O/pert_stored $P $R/stored_pert.lst pert "log logv3"; echo "pert_stored done $(date)" >> $O/q3.log
timeout 14400 $H/rndsweep.sh $O/rnd $P; echo "rnd done $(date)" >> $O/q3.log
echo Q3DONE >> $O/q3.log
