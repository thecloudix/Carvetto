#!/bin/bash
# q5.sh: round-5 sweep chain (each step hard-timed). log = branch 1156cad (instrumented, tfs_instr5.py), logoff = main
# (TFS_OFF=1, hash-checked == origin/main 34db4d0), logfma(off) = the same built -march=x86-64-v3 -ffp-contract=fast.
R=/home/chris/wt-T0158/build-verify5-rel; H=/home/chris/harness/t0158v5; O=$R/out; P=${P:-5}; mkdir -p $O; echo $$ > $O/q5.pid
VARS=1 timeout 14400 $H/sweep.sh $O/scn $P $H/stored_scenes.lst scn "log logoff logfma logfmaoff"; echo "scn done $(date)" >> $O/q5.log
timeout 7200 $H/sweep.sh $O/pert_gen3 $P $H/gen3.lst pert "log logoff"; echo "gen3 done $(date)" >> $O/q5.log
timeout 14400 $H/sweep.sh $O/pert_gen12 $P $H/gen12.lst pert "log logoff"; echo "gen12 done $(date)" >> $O/q5.log
timeout 10000 $H/sweep.sh $O/genscn $P $H/genall.lst scn "log logoff logfma logfmaoff"; echo "genscn done $(date)" >> $O/q5.log
VARS=1 timeout 14400 $H/sweep.sh $O/pert_stored $P $H/stored_pert.lst pert "log logoff"; echo "pert_stored done $(date)" >> $O/q5.log
C="log logoff logfma logfmaoff" timeout 14400 $H/rndsweep.sh $O/rnd $P; echo "rnd done $(date)" >> $O/q5.log
echo Q5DONE >> $O/q5.log
