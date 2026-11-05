#!/bin/bash
# runpo3.sh n reps layouts...  -> alternates branch and main per layout (po3_br, po3_main), each n -> 2n interleaved CPU
n=$1; r=$2; shift 2
for L in "$@"; do for w in br main br main; do timeout 900 bin/po3_$w $L $n $r | sed "s/^/$w /"; done; done
