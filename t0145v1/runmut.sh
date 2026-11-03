#!/bin/bash
# runmut.sh builddir factor
B=$1; F=$2
while IFS='|' read -r k name; do
  if [ $k = core ]; then bin=$B/core/tests/carvetto_tests; else bin=$B/app/cli/tests/carvetto_cli_tests; fi
  out=$(MUTANT_FACTOR=$F $bin "$name" 2>&1); rc=$?
  r=$(echo "$out" | grep -oE "(ratio|perOffsetRatio)[^0-9]*[0-9.e+-]+" | head -3 | tr '\n' ' ')
  f=$(echo "$out" | grep -E "^ +(CHECK|REQUIRE)\(" | head -2 | tr -s ' ' | tr '\n' ';')
  echo "rc=$rc | $name | $r | $f"
done < /home/chris/wt-T0145/build-verify/v145/tests.txt
