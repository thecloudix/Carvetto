#!/bin/bash
# libm perturbation (ulpwrap.c, -Wl,--wrap): Debug test binaries (br = cae3fa7, old = e87f549, oldlib = e87f549 offset.cpp + cae3fa7 test,
# oldtest = cae3fa7 lib + e87f549 test) on [t0128]~[perf] (K 1..4, seeds 1..50) and br/old on [offset]~[perf] (K 1..4, seeds 1..6);
# Release sx drivers (branch vs main / vs e87f549) on cusp (which 0..3, 3 d, 12 shifts) and channel sweeps.
U=/home/chris/wt-T0128/build-verify3-ulp; D=/home/chris/wt-T0128/build-verify3-drv; L=/home/chris/harness/t0128v3/logs/ulp; mkdir -p $L
J=$L/jobs.txt; : > $J
bin() { case $1 in br) echo $U/carvetto_tests_ulp;; *) echo $U/$1/tests;; esac; }
for v in br old oldlib oldtest; do for k in 1 2 3 4; do
  echo "for s in \$(seq 1 50); do r=\$(ULP_K=$k ULP_SEED=\$s timeout 600 $(bin $v) '[t0128]~[perf]' 2>&1); echo \"\$s rc \$? \$(echo \"\$r\" | grep -E '^test cases|^All tests' | tr -s ' ') FAILS: \$(echo \"\$r\" | grep -B3 -E '^/.*FAILED' | grep -E '^T-0128' | sort -u | tr '\n' ';')\"; done > $L/t_${v}_k$k.log 2>&1" >> $J
done; done
for v in br old; do for k in 1 2 3 4; do for s in 1 2 3 4 5 6; do
  echo "r=\$(ULP_K=$k ULP_SEED=$s timeout 900 $(bin $v) '[offset]~[perf]' 2>&1); echo \"$s rc \$? \$(echo \"\$r\" | grep -E '^test cases|^All tests' | tr -s ' ') FAILS: \$(echo \"\$r\" | grep -B3 -E '^/.*FAILED' | grep -vE '^/|^\.|^--' | sort -u | tr '\n' ';')\" > $L/o_${v}_k${k}_s$s.log 2>&1" >> $J
done; done; done
for ref in main old; do for k in 1 2 3 4; do
  echo "for s in \$(seq 1 25); do for w in 0 1 2 3; do for d in 0.13 1.0328719931380388 3.175; do echo \"s \$s \$(ULP_K=$k ULP_SEED=\$s timeout 300 $D/sxu_$ref sxcusp \$w \$d 30 1e-6 6 1)\"; done; done; done > $L/cusp_${ref}_k$k.log 2>&1" >> $J
  echo "for s in \$(seq 1 10); do for d in 0.1317 1 3.175; do echo \"s \$s \$(ULP_K=$k ULP_SEED=\$s timeout 300 $D/sxu_$ref sxchan 3.6 \$d)\"; done; done > $L/chan_${ref}_k$k.log 2>&1" >> $J
done; done
xargs -d "\n" -P ${P:-5} -I{} bash -c {} < $J
echo done > $L/ulp.done
