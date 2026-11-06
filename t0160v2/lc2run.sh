#!/bin/bash
# t0160v2: run all lc2 cases in N shards (results/lc2/shard_K.txt)
N=${1:-4}; B=/home/chris/scratch-t0160v2/bin/lc2; O=/home/chris/harness/t0160v2/results/lc2
for k in $(seq 0 $((N-1))); do LC_SHN=$N LC_SHK=$k timeout 5400 $B > $O/shard_$k.txt 2>&1 & done; wait
