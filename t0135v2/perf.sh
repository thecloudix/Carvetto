#!/bin/bash
# perf.sh REPS -> interleaved branch/base/main CPU times
cd /home/chris/wt-T0135/build-verify2/drv
R=${1:-5}
run() { for rep in $(seq $R); do for b in perf2 perf2_base perf2_main; do echo "SIDE=$1 ALLOW=$2 $b $3 $4 $5: $(SIDE=$1 ALLOW=$2 ./bin/$b $3 $4 $5)"; done; done; }
run 0 -1.0 500 1 2
run 2 0 500 0 0
run 2 0 500 1 0
run 0 -0.5 500 0 1
run 0 0 500 1 2
run 1 0 500 0 0
run 2 -0.5 500 1 1
for rep in $(seq $R); do for b in perf3 perf3_base perf3_main; do for k in 3 4; do echo "SIDE=2 ALLOW=0 $b 500 0 $k: $(SIDE=2 ALLOW=0 ./bin/$b 500 0 $k)"; echo "SIDE=2 ALLOW=0 $b 200 1 $k: $(SIDE=2 ALLOW=0 ./bin/$b 200 1 $k)"; done; done; done
echo DONE
