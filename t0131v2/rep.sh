#!/bin/bash
# Usage: rep.sh <carvetto_tests binary> <runs> <outfile> [test name substrings...]
# Runs each T-0131 timing test <runs> times with -s and logs the "CPU seconds" INFO line
# and pass/fail per run.
bin=$1; runs=$2; out=$3; shift 3
names=("$@")
[ ${#names[@]} -eq 0 ] && names=("T-0131 c:*" "T-0131 d: Profile*" "T-0131 d: tabs*" "T-0131 e:*" "T-0131 f:*")
for i in $(seq 1 "$runs"); do
  for n in "${names[@]}"; do
    res=$("$bin" "$n" -s 2>&1)
    rc=$?
    line=$(grep -m1 "CPU seconds" <<<"$res" | sed 's/^ *//')
    echo "run $i | $n | rc=$rc | load=$(cut -d' ' -f1 /proc/loadavg) | $line" | tee -a "$out"
  done
done
