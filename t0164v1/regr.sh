#!/bin/sh
# t0164v1 regression: family hash sweep and adv/adv2 hash runs, branch vs main (moves hash mh + diag list dl separately)
H=/home/chris/harness/t0164v1; S=/home/chris/scratch-t0164v1; cd /home/chris/harness/t0153v4
for v in br main; do
  timeout 5400 sh $H/hashsweep.sh $S/bin/vc3_$v $S/hash_$v 5; cat $S/hash_$v/*.txt | grep HASH | sort > $S/hashfam_$v.txt
done
echo fam > $S/regr.fam.done
for v in br main; do for d in adv adv2; do timeout 7200 sh $H/advhash.sh $S/bin/vc3_$v $d $S/advhash_${d}_$v.txt 5; done; done
echo done > $S/regr.done
