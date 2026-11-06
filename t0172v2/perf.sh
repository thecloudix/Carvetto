#!/bin/bash
# perf matrix: branch vs origin/main (perf2_main), vs 8cd3568 (perf2_pre), branch plain vs branch namespaced (perf2_brns, A/A control)
D=/home/chris/wt-T0172/build-verify2-drv; L=/home/chris/harness/t0172v2/logs/perf; mkdir -p $L; cd $D
R=${REPS:-5}
run() { for b in ${REFS:-main pre brns}; do timeout 900 ./perf2_$b "$@" >> $L/$b.log 2>&1; done; }
# clustered (card b)
run lhatch 1000 0.002 0 0 $R 0 1e5
run lhatch 2000 0.002 0 0 $R 0 1e5
run lhatch 4000 0.002 0 0 3 0 1e5
run lhatch 1000 0.05 0 0 $R 0 1e5
run lhatch 4000 0.05 0 0 3 0 1e4
run lhatch 1000 0.002 0 0 $R 0 50
run lhatch 1000 0.05 1e-7 2 $R 0 1e4
run fhatch 1000 0.002 0 0 $R 0 1e5
run fhatch 4000 0.002 0 0 3 0 1e5
run chatch 1000 0.01 0 0 $R 0 1e4
run chatch 4000 0.01 0 0 3 0 1e4
# elsewhere (no slowdown)
run hatch 3000 1 0 0 $R
run hatch 3000 1 1e-7 2 $R
run comb 3000 1 0 0 $R
run comb 3000 1 1e-7 1 $R
run comb 3000 0.05 0 2 $R
run rects 3000 1 0 2 $R
run ring 3000 1 0 0 $R
run hatch 1000 0.002 0 0 $R
run hatch 4000 0.05 0 0 $R
run longz 4000 1 0 0 $R
run long 10000 1 0 0 $R
run spiral 200 0.5 0 0 $R 0 1e5
run sslot 200 0.5 0 0 $R 0 0
run sslot 400 0.5 0 0 3 0 0
run spiral 400 0.5 0 0 $R 0 600
run spiral 700 0.5 0 0 3 0 1e3
run spiral 400 0.5 0 0 3 0 1e5
run rcomb 1000 0.05 0 0 $R 0 0
run rcomb 4000 0.05 0 0 3 0 1e5
run rhatch 1000 0.002 0 0 $R 0 1e5
run rhatch 1000 0.05 0 0 $R 0 0
echo done >> $L/done
