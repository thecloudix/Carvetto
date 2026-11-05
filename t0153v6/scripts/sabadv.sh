#!/bin/sh
cd /home/chris/harness/t0153v4
for sab in 2 3 4; do for f in adv/*.txt adv2/*.txt repro4/*.txt; do echo "$sab $f"; done; done | xargs -P 3 -I{} sh -c 'set -- {}; r=$(SAB=$1 VC_FILE=$2 VC_FIX="0 12 90 0.05 0 0" timeout 600 /home/chris/scratch-t0153v6/bin/vc3_sab 1 1 2>&1 | grep SUMMARY | grep -oE "ok=[0-9]+ refused=[0-9]+|realViolations=[0-9]+|worstCone=[-0-9.e+]+" | tr "\n" " "); echo "SAB=$1 $2 $r"' | sort > /home/chris/harness/t0153v6/results/sabadv.txt
echo done > /home/chris/scratch-t0153v6/sabadv.done
