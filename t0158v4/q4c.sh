#!/bin/bash
# q4c.sh: logd (log + harmful "most even feet" tie margin mLopH and TFS_DETAIL) over stored VARS=1, generated families, random.
R=/home/chris/wt-T0158/build-verify4-rel; H=/home/chris/harness/t0158v4; O=$R/out; P=${P:-4}; echo $$ > $O/q4c.pid
VARS=1 timeout 14400 $H/sweep.sh $O/scn $P $H/stored_scenes.lst scn "logd"; echo "scn logd done $(date)" >> $O/q4c.log
timeout 10000 $H/sweep.sh $O/genscn $P $H/genall.lst scn "logd"; echo "gen logd done $(date)" >> $O/q4c.log
C="logd" timeout 14400 $H/rndsweep.sh $O/rndd $P; echo "rnd logd done $(date)" >> $O/q4c.log
echo Q4CDONE >> $O/q4c.log
