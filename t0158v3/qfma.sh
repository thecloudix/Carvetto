#!/bin/bash
# qfma.sh: FMA-contracted variant (core + Clipper2 built WITHOUT -ffp-contract=off, -march=x86-64-v3 -ffp-contract=fast: a
# different rounding environment, stand-in for another compiler). Stored scn VARS=1 and generated families (scn) with
# logfma (branch) and logfmaoff (main rule off); compare against out/scn and out/genscn (plain) with xcomp.py.
R=/home/chris/wt-T0158/build-verify3-rel; H=/home/chris/harness/t0158v3; O=$R/out; P=${P:-6}; echo $$ > $O/qfma.pid
timeout 10000 $H/sweep.sh $O/genscn $P $R/genall.lst scn "log logoff logfma logfmaoff"; echo "genscn done $(date)" >> $O/qfma.log
VARS=1 timeout 14400 $H/sweep.sh $O/scn $P $R/stored_scenes.lst scn "logfma logfmaoff"; echo "scnfma done $(date)" >> $O/qfma.log
echo QFMADONE >> $O/qfma.log
