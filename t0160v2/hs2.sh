#!/bin/bash
# t0160v2 hash sweeps (vc3, 26 groups x N cases): base 681327a vs branch 923d66c off/on, plain vs x86-64-v3, forced flat+clearance (FM2), INS
H=/home/chris/harness/t0160v2; B=/home/chris/scratch-t0160v2/bin; R=$H/results/hs; N=${1:-40}; J=${2:-3}
sh $H/hashsweep.sh $B/vc3_main $R/main $J $N
sh $H/hashsweep.sh $B/vc3_br $R/br_off $J $N VC_CS=0
sh $H/hashsweep.sh $B/vc3_br $R/br_on $J $N
sh $H/hashsweep.sh $B/vc3_br3 $R/br3_on $J $N
sh $H/hashsweep.sh $B/vc3_main $R/main_fm2 $J $N VC_FM=2
sh $H/hashsweep.sh $B/vc3_br $R/br_fm2_off $J $N VC_FM=2 VC_CS=0
sh $H/hashsweep.sh $B/vc3_br $R/br_fm2_on $J $N VC_FM=2
sh $H/hashsweep.sh $B/vc3_br3 $R/br3_fm2_on $J $N VC_FM=2
sh $H/hashsweep.sh $B/vc3_br $R/br_fm2_ins $J $N VC_FM=2 VC_INS=1 VC_INSONLY=1
echo done > $R/done
