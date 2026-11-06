#!/bin/bash
# q5b.sh: after q5 (sweeps) finishes: whole-scene flood oracle, generated 620 (br/main/r4), stored changed-vs-main sample 150
# (br/main), stored round-5-vs-round-4 delta (br/main/r4; list built by delta54.py once scn_r4 is complete).
H=/home/chris/harness/t0158v5; O=/home/chris/wt-T0158/build-verify5-rel/out; echo $$ > $O/q5b.pid
timeout 40000 bash -c "until grep -q Q5DONE $O/q5.log 2>/dev/null; do sleep 30; done"
timeout 20000 $H/floodcmp5.sh $O/flood_gen 5 $H/flood_gen.lst "br main r4"; echo "flood_gen done $(date)" >> $O/q5b.log
timeout 10000 $H/floodcmp5.sh $O/flood_chg 5 $H/flood_chg_sample.lst "br main"; echo "flood_chg done $(date)" >> $O/q5b.log
timeout 20000 bash -c "until grep -q ALLDONE $O/scn_r4/run.log 2>/dev/null; do sleep 30; done"
python3 $H/delta54.py > $H/flood_delta54.lst
timeout 20000 $H/floodcmp5.sh $O/flood_d54 5 $H/flood_delta54.lst "br main r4"; echo "flood_d54 done $(date)" >> $O/q5b.log
echo Q5BDONE >> $O/q5b.log
