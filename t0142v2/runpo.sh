#!/bin/bash
# runpo.sh {br|main} n reps layouts...  -> results/po2_<which>_<n>.txt
w=$1; n=$2; r=$3; shift 3
for L in "$@"; do timeout 900 bin/po2_$w $L $n $r; done
