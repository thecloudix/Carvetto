#!/bin/sh
cd /home/chris/harness/t0153v4
timeout 5400 sh /home/chris/harness/t0153v6/hashsweep.sh /home/chris/scratch-t0153v6/bin/vc3_new /home/chris/scratch-t0153v6/hash_new 6; cat /home/chris/scratch-t0153v6/hash_new/*.txt | grep HASH | sort > /home/chris/harness/t0153v6/results/hash_bdc932c.txt; echo hash > /home/chris/scratch-t0153v6/hash.done
timeout 7200 sh /home/chris/harness/t0153v6/advhash.sh /home/chris/scratch-t0153v6/bin/vc3_new adv /home/chris/harness/t0153v6/results/advhash_bdc932c.txt 6; echo adv > /home/chris/scratch-t0153v6/advh.done
