#!/bin/sh
# t0164v2: after sab5 finishes run the fuzz6 matrix and the SABEND matrix
S=/home/chris/scratch-t0164v2; H=/home/chris/harness/t0164v2
until grep -q done $S/sab5.txt 2>/dev/null; do sleep 20; done
timeout 14000 sh $H/fuzzrun.sh; timeout 7200 sh $H/sabend.sh
