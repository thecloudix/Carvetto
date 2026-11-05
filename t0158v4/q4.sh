#!/bin/bash
# q4.sh: round-4 sweep chain (each step hard-timed). log = branch (instrumented), logoff = main, logfma(off) = FMA build.
R=/home/chris/wt-T0158/build-verify4-rel; H=/home/chris/harness/t0158v4; O=$R/out; P=${P:-8}; mkdir -p $O; echo $$ > $O/q4.pid
timeout 7200 $H/sweep.sh $O/pert_gen3 $P $H/gen3.lst pert "log logoff"; echo "gen3 done $(date)" >> $O/q4.log
timeout 14400 $H/sweep.sh $O/pert_gen12 $P $H/gen12.lst pert "log logoff"; echo "gen12 done $(date)" >> $O/q4.log
timeout 10000 $H/sweep.sh $O/genscn $P $H/genall.lst scn "logfma logfmaoff"; echo "genscn fma done $(date)" >> $O/q4.log
VARS=1 timeout 14400 $H/sweep.sh $O/scn $P $H/stored_scenes.lst scn "log logoff logfma logfmaoff"; echo "scn done $(date)" >> $O/q4.log
VARS=1 timeout 14400 $H/sweep.sh $O/pert_stored $P $H/stored_pert.lst pert "log logoff"; echo "pert_stored done $(date)" >> $O/q4.log
C="log logoff logfma logfmaoff" timeout 14400 $H/rndsweep.sh $O/rnd $P; echo "rnd done $(date)" >> $O/q4.log
echo Q4DONE >> $O/q4.log
