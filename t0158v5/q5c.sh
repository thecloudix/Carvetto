#!/bin/bash
# q5c.sh: whole-scene flood oracle now (P=4), in parallel with the random sweep: round-5-vs-round-4 stored delta, generated 620,
# stored changed-vs-main sample 150.
H=/home/chris/harness/t0158v5; O=/home/chris/wt-T0158/build-verify5-rel/out; echo $$ > $O/q5c.pid
timeout 20000 $H/floodcmp5.sh $O/flood_d54 4 $H/flood_delta54.lst "br main r4"; echo "flood_d54 done $(date)" >> $O/q5c.log
timeout 30000 $H/floodcmp5.sh $O/flood_gen 4 $H/flood_gen.lst "br main r4"; echo "flood_gen done $(date)" >> $O/q5c.log
timeout 20000 $H/floodcmp5.sh $O/flood_chg 4 $H/flood_chg_sample.lst "br main"; echo "flood_chg done $(date)" >> $O/q5c.log
echo Q5CDONE >> $O/q5c.log
