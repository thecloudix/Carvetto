#!/bin/sh
# t0160v2: vc3 cone/end-mill oracle (VCVIOL) on t0153v4 adv/adv2 inputs with flat depth + clearance end mill configs (branch, sharpening on)
V=/home/chris/scratch-t0160v2/bin/vc3_br; OUT=/home/chris/harness/t0160v2/results/advcone.txt
for f in /home/chris/harness/t0153v4/adv/*.txt /home/chris/harness/t0153v4/adv2/*.txt; do
  for cfg in "0 12 90 0.05 1.0 0|2" "1 6 0 0.02 1.0 0.3|1" "0 12 60 0.01 0.8 0|0.6"; do echo "$f|$cfg"; done
done | xargs -P 2 -I{} sh -c 'f=$(echo "{}"|cut -d"|" -f1); t=$(echo "{}"|cut -d"|" -f2); em=$(echo "{}"|cut -d"|" -f3); export VC_EM=$em;
  r=$(VCVIOL=1 VC_FILE=$f VC_FIX="$t" timeout 600 '$V' 1 1 2>&1 | grep -E "^SUMMARY|VIOL|OUTSIDE"; [ $? = 124 ] && echo TIMEOUT)
  echo "$(basename $f) fix=[$t] em=$em :: $r"' | sort > $OUT
echo done >> $OUT
