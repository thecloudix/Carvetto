#!/bin/bash
# rep.sh build-suffix count outfile : runs the T-0171 test count times, logs ratios / large seconds / pass and wall time
T="${TNAME:-T-0171: outsets of a column whose holes are listed far from their outers stay near-linear}"
for i in $(seq 1 $2); do
  s=$(date +%s.%N)
  timeout 900 /home/chris/wt-T0171/$1/core/tests/carvetto_tests "$T" -s > /tmp/claude-1000/t0171_$(basename $1)_$$.log 2>&1; rc=$?
  e=$(date +%s.%N)
  r=$(grep -A2 "t.ratio() <= 2.5" /tmp/claude-1000/t0171_$(basename $1)_$$.log | grep -o "^  [0-9.]*" | tr -d ' ' | tr '\n' ' ')
  l=$(grep -A2 "t.largeSeconds <= absBound" /tmp/claude-1000/t0171_$(basename $1)_$$.log | grep -o "^  [0-9.]*" | tr -d ' ' | tr '\n' ' ')
  printf "%s run %d rc=%d wall=%.1fs ratios(rev shuf)=%s large=%s load=%s\n" $1 $i $rc $(echo "$e - $s" | bc) "$r" "$l" "$(cut -d' ' -f1 /proc/loadavg)" >> $3
done
rm -f /tmp/claude-1000/t0171_$(basename $1)_$$.log
