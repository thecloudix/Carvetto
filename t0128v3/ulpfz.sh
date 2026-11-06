#!/bin/bash
# Fuzz families 2-5 with the tests' rules: under libm perturbation (K 1..4, seeds 1..3) instrumented branch (I = cae3fa7 + EJNEW/EJGAP probes)
# vs e87f549 and branch vs origin/main; mutant F vs branch on glibc (fresh seeds, volume) and under perturbation.
D=/home/chris/wt-T0128/build-verify3-drv; L=/home/chris/harness/t0128v3/logs/ulpfz; mkdir -p $L; cd $D
J=$L/jobs.txt; : > $J
for k in 1 2 3 4; do for s in 1 2 3; do for f in 2 3 4 5; do
  sd=$(printf "0x7c%x%x%x0000" $k $s $f)
  echo "ULP_K=$k ULP_SEED=$s timeout 1800 ./sxIu sxfz $f 400 $sd > $L/I_k${k}_s${s}_f$f.log 2> $L/I_k${k}_s${s}_f$f.err" >> $J
  echo "ULP_K=$k ULP_SEED=$s timeout 1800 ./sxu_main sxfz $f 400 $sd > $L/M_k${k}_s${s}_f$f.log 2>&1" >> $J
done; done; done
for f in 2 3 4 5; do for c in 0 1 2; do
  sd=$(printf "0x7d%x%x0000" $f $c)
  echo "timeout 1800 ./sx_F_vs_br sxfz $f 3000 $sd > $L/F_f${f}_$c.log 2>&1" >> $J
done; done
xargs -d "\n" -P ${P:-5} -I{} bash -c {} < $J
echo done > $L/ulpfz.done
