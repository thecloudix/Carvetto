#!/bin/bash
# origin/main Debug tests with the libm wrapper on [offset]~[perf], K 1..4 x seeds 1..6: failing test names (compare with orerun_br.log)
U=/home/chris/wt-T0128/build-verify3-ulp; L=/home/chris/harness/t0128v3/logs/omain; mkdir -p $L
for k in 1 2 3 4; do for s in 1 2 3 4 5 6; do echo "$k $s"; done; done | xargs -P 2 -L 1 bash -c 'ULP_K=$0 ULP_SEED=$1 timeout 900 '$U'/main/tests "[offset]~[perf]" 2>&1 | awk -v k=$0 -v s=$1 "/^---/{getline t; if (t !~ /^\\//) name=t} /FAILED:/{print \"k\"k\" s\"s\" | \"name} /^(test cases|All tests)/{print \"k\"k\" s\"s\" END \"\$0}" > '$L'/k$0_s$1.log'
echo done > $L/done
