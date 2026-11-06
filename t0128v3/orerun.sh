#!/bin/bash
# rerun failing perturbed [offset] runs (branch) with names of the failing tests
U=/home/chris/wt-T0128/build-verify3-ulp; L=/home/chris/harness/t0128v3/logs
while read k s; do
  ULP_K=$k ULP_SEED=$s timeout 900 $U/carvetto_tests_ulp "[offset]~[perf]" 2>&1 | awk -v k=$k -v s=$s '/^---/{getline t; if (t !~ /^\//) name=t} /FAILED:/{getline e; print "k"k" s"s" | "name" | "e}' >> $L/orerun_br.log
done < $L/orerun.txt
echo done >> $L/orerun_br.log
