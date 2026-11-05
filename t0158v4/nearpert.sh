#!/bin/bash
# nearpert.sh LIST OUT: PERT (21 transforms) runs of random scenes "rSEED SCENE" (scale/offset from rndsweep.sh) with st_logd.
B=/home/chris/wt-T0158/build-verify4-rel/bin; L=$1; O=$2; mkdir -p $O; echo $$ > $O/pid
so() { case $1 in 2[0-9][0-9]) case $1 in 201|202|203) echo "1 100000";; 221|222|223|224) echo "0.01 0";; 242) echo "37.5 -3.3e6";; 243|244) echo "0.001 0";; esac;; *) echo "1 0";; esac; }
while read r s; do seed=${r#r}; echo "$seed $s $(so $seed)"; done < $L | xargs -P ${P:-4} -L 1 bash -c 'PERT=1 TARGET=$1 timeout 1800 '$B'/st_logd $0 $(( $1 + 1 )) $2 $3 > '$O'/r$0_$1.txt 2>&1'
echo ALLDONE > $O/done
