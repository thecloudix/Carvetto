#!/bin/bash
cd /home/chris/wt-T0145/build-verify
mkdir -p flake; rm -f flake/*
( for i in 1 2 3 4 5; do [ -f flake/STOP ] && break; ctest --test-dir . -j8 > flake/bg_$i.log 2>&1; echo "bg $i rc=$?" >> flake/bg.sum; done ) &
BG=$!; echo $BG > flake/bg.pid
sleep 20
for i in $(seq 1 20); do
  ctest --test-dir . -j16 -I 0,0,0,420,455,456,489,491,557,558,586,711,712,831 --output-on-failure > flake/fg_$i.log 2>&1
  echo "fg $i rc=$? $(grep -E 'tests passed' flake/fg_$i.log)" >> flake/fg.sum
done
touch flake/STOP
echo "fg done; waiting bg" >> flake/fg.sum
wait $BG
echo ALLDONE >> flake/fg.sum
