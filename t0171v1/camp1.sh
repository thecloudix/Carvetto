#!/bin/bash
H=/home/chris/harness/t0171v1
if [ "$1" = rel ]; then
  for k in 1 2 3; do $H/rep.sh build-verify1-rel 5 $H/results/rel_real.txt; $H/rep.sh build-verify1-mutC-Release 3 $H/results/rel_mutC.txt; done
else
  for k in 1 2 3; do $H/rep.sh build-verify1 3 $H/results/debug_real.txt; $H/rep.sh build-verify1-mutC-Debug 3 $H/results/debug_mutC.txt; done
fi
