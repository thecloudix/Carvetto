#!/bin/bash
# runmut2.sh X...: run mutant test binaries: [t0172],[t0128] first; survivors also [offset] and [offset]~[t0172]~[t0128].
O=/home/chris/wt-T0172/build-verify2-mut; L=/home/chris/harness/t0172v2/logs/mut; mkdir -p $L; cd /home/chris/wt-T0172/build-verify2/core/tests
for X in "$@"; do
  T=$O/$X/tests; [ -x $T ] || { echo "$X: no binary"; continue; }
  timeout 1800 $T "[t0172],[t0128]" > $L/$X.a.log 2>&1; rc=$?
  if [ $rc -ne 0 ]; then echo "$X KILLED by [t0172],[t0128] rc $rc: $(grep -E '^T-0|^test cases' $L/$X.a.log | sort | uniq -c | sort -rn | head -4 | tr -s ' ' | tr '\n' ';')"; continue; fi
  timeout 3600 $T "[offset]~[t0172]~[t0128]" > $L/$X.b.log 2>&1; rc=$?
  if [ $rc -ne 0 ]; then echo "$X KILLED by [offset] rc $rc: $(grep -E '^T-0|^test cases' $L/$X.b.log | sort | uniq -c | sort -rn | head -4 | tr -s ' ' | tr '\n' ';')"; continue; fi
  echo "$X SURVIVES ([t0172] [t0128] [offset])"
done
