#!/bin/sh
# t0160v2: t0153v4 adv/adv2 inputs x 5 configs (advhash.sh): base vs branch hashes, branch INS (on == off + insertions)
H=/home/chris/harness/t0160v2; B=/home/chris/scratch-t0160v2/bin; R=$H/results
sh $H/advhash.sh $B/vc3_main $R/adv_main.txt 2
sh $H/advhash.sh $B/vc3_br $R/adv_br.txt 2
sh $H/advhash.sh $B/vc3_br $R/adv_ins.txt 2 VC_INS=1
echo done > $R/adv.done
