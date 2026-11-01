#!/bin/bash
# psweep.sh P: pocket checks, new and old, seeds 5001..5020, 60 scenes each
cd $(dirname $(readlink -f $0))
for s in $(seq 5001 5020); do for v in new old; do echo "$v $s"; done; done | \
  xargs -P ${1:-1} -L 1 bash -c 'timeout 3600 ./pc_$0 $1 60 1 > out/c_$0_$1.txt 2>&1; echo "done $0 $1 rc=$?"'
