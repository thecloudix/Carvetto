#!/bin/bash
# libm perturbation (ulpwrap.c): branch Debug tests relinked (build-verify2-ulp/tests): [t0128]~[perf] and [t0172]~[perf] K1-4 x seeds 1-50,
# [offset]~[perf] K1-4 x seeds 1-6.
U=/home/chris/wt-T0172/build-verify2-ulp/tests; L=/home/chris/harness/t0172v2/logs/ulp; mkdir -p $L; cd /home/chris/wt-T0172/build-verify2
J=$L/jobs.txt; : > $J
for t in t0128 t0172; do for k in 1 2 3 4; do
  echo "for s in \$(seq 1 50); do r=\$(ULP_K=$k ULP_SEED=\$s timeout 900 $U '[$t]~[perf]' 2>&1); echo \"\$s rc \$? \$(echo \"\$r\" | grep -E '^test cases|^All tests' | tr -s ' ') FAILS: \$(echo \"\$r\" | grep -B3 -E '^/.*FAILED' | grep -E '^T-0' | sort -u | tr '\n' ';')\"; done > $L/${t}_k$k.log 2>&1" >> $J
done; done
for k in 1 2 3 4; do for s in 1 2 3 4 5 6; do
  echo "r=\$(ULP_K=$k ULP_SEED=$s timeout 1800 $U '[offset]~[perf]' 2>&1); echo \"$s rc \$? \$(echo \"\$r\" | grep -E '^test cases|^All tests' | tr -s ' ') FAILS: \$(echo \"\$r\" | grep -B3 -E '^/.*FAILED' | grep -vE '^/|^\.|^--' | sort -u | tr '\n' ';')\" > $L/offset_k${k}_s$s.log 2>&1" >> $J
done; done
xargs -d "\n" -P ${P:-5} -I{} bash -c {} < $J
echo done > $L/ulp.done
