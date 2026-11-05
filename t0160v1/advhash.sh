#!/bin/sh
# t0160v1: adv/adv2 (t0153v4) hole-hugging-wall inputs, 4 configs (no flat; flat no clearance; flat + 2 mm end mill; ball nose
# flat + 1 mm end mill), hash only (and INS lines when VC_INS is set). usage: advhash.sh <vc3> <out file> <jobs> [env]
V=$1; OUT=$2; J=$3; shift 3; E="$*"
for f in /home/chris/harness/t0153v4/adv/*.txt /home/chris/harness/t0153v4/adv2/*.txt; do
  for cfg in "0 12 90 0.05 0 0|" "0 12 90 0.05 1.5 0|" "0 12 90 0.05 1.0 0|2" "1 6 0 0.02 1.0 0.3|1" "0 12 60 0.01 0.8 0|0.6"; do echo "$f|$cfg"; done
done | xargs -P $J -I{} sh -c 'f=$(echo "{}"|cut -d"|" -f1); t=$(echo "{}"|cut -d"|" -f2); em=$(echo "{}"|cut -d"|" -f3);
  if [ -n "$em" ]; then export VC_EM=$em; else unset VC_EM; fi
  r=$(env '"$E"' VC_HASH=1 VC_HASHONLY=1 VC_FILE=$f VC_FIX="$t" timeout 600 '$V' 1 1 2>&1 | grep -E "^HASH|^INS"; [ $? = 124 ] && echo TIMEOUT)
  echo "$(basename $f) fix=[$t] em=$em :: $r"' | sort > $OUT
