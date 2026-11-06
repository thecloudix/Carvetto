#!/bin/sh
# usage: vsweep.sh <jobs> <hash|cone>: V-Carve vc3 (t0160v2) sweeps. hash: branch vs main (families 0-23 + 2 mixes, 40 cases),
# default mix and VC_FM=2 (flat depth + clearance end mills forced). cone: the cone/end-mill oracle on the branch, VC_FM=2, 12/family.
J=$1; H=/home/chris/harness/t0160v2; B=/home/chris/wt-T0169/build-verify1-bin; R=/home/chris/harness/t0169v1/results/vc
if [ "$2" = hash ]; then for v in br main; do sh $H/hashsweep.sh $B/vc3_$v $R/hs_$v $J 40; sh $H/hashsweep.sh $B/vc3_$v $R/hsfm2_$v $J 40 VC_FM=2; done; echo done > $R/DONE_hash; fi
if [ "$2" = cone ]; then sh $H/conesweep.sh $B/vc3_br $R/cone_br $J 12; echo done > $R/DONE_cone; fi
