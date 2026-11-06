#!/bin/bash
W=/home/chris/wt-T0171; H=/home/chris/harness/t0171v1
case $1 in
 relT171) timeout 2400 ctest --test-dir $W/build-verify1-rel -R 'T-0171' --repeat until-fail:20 --output-on-failure > $H/results/ctest_rep_rel_T0171.txt 2>&1; echo rc=$? >> $H/results/ctest_rep_rel_T0171.txt ;;
 relT142) timeout 2400 ctest --test-dir $W/build-verify1-rel -R 'T-0142 round 3: outsets of touching' --repeat until-fail:10 --output-on-failure > $H/results/ctest_rep_rel_T0142r3.txt 2>&1; echo rc=$? >> $H/results/ctest_rep_rel_T0142r3.txt ;;
 dbgT171) timeout 2400 ctest --test-dir $W/build-verify1 -R 'T-0171' --repeat until-fail:5 --output-on-failure > $H/results/ctest_rep_dbg_T0171.txt 2>&1; echo rc=$? >> $H/results/ctest_rep_dbg_T0171.txt ;;
esac
