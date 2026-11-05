#!/bin/bash
# q4b.sh: round-3 (520ca28) hashes for the r4-vs-r3 delta: stored VARS=1 and generated families (st_r3, no instrumentation).
R=/home/chris/wt-T0158/build-verify4-rel; H=/home/chris/harness/t0158v4; O=$R/out; P=${P:-5}; echo $$ > $O/q4b.pid
VARS=1 timeout 14400 $H/sweep.sh $O/scn $P $H/stored_scenes.lst scn "r3"; echo "scn r3 done $(date)" >> $O/q4b.log
timeout 10000 $H/sweep.sh $O/genscn $P $H/genall.lst scn "r3"; echo "gen r3 done $(date)" >> $O/q4b.log
echo Q4BDONE >> $O/q4b.log
