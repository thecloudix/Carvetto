#!/bin/sh
cd /home/chris/harness/t0153v6
for f in fuzzgeo/nest.txt fuzzgeo/thinwalls.txt /home/chris/harness/t0153v4/repro4/framehole_isl_g0.05.txt /home/chris/harness/t0153v4/repro4/annulus_isl_g0.01.txt /home/chris/harness/t0153v5/repro5/stad_isl_sab4_c0.05.txt repro6/fullisl_c1.0_g0.3_d0.3.txt; do
 for c in 0.01 0.05 0.3 1.0; do for seed in 11 12 13; do timeout 600 /home/chris/scratch-t0153v6/bin/fuzz6 $f $c 20000 $seed 2>/dev/null; timeout 600 env BADEND=1 /home/chris/scratch-t0153v6/bin/fuzz6 $f $c 5000 $seed 2>/dev/null | sed 's/^/BADEND /'; done; done; done > /home/chris/harness/t0153v6/results/fuzz6.txt
echo done > /home/chris/scratch-t0153v6/fuzz.done
