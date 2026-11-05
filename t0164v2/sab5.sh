#!/bin/sh
# t0164v2 (g/B2): SAB=5 (drop own contour SABI silently) / SAB=6 (drop all but 0) / SAB=0 (control) through the patched builds
# (sab = branch cce938c + sabotage164.patch, sabmain = origin/main + patch), every geo/drop_* x chordal x tool x SABI.
S=/home/chris/scratch-t0164v2; B=$S/bin; G=/home/chris/harness/t0164v2/geo
for f in $G/drop_*.txt /home/chris/harness/t0153v6/repro6/sq_hole_diag.txt; do
  case $(basename $f) in drop_multi.txt) is="1 2 3 4 5";; drop_annulus_isl.txt|drop_ring_hole.txt) is="1 2";; *) is="1";; esac
  for c in 0.01 0.05 1.0; do for t in "0 12 90" "1 6 0" "0 6 30"; do
    for i in $is; do echo "$f|5|$i|$c|$t"; done; echo "$f|6|1|$c|$t"; echo "$f|0|1|$c|$t"; done; done; done |
 xargs -P 4 -I{} sh -c 'IFS="|"; set -- $(echo "{}"); f=$1; sab=$2; si=$3; c=$4; t=$5; l="$(basename $f) SAB=$sab SABI=$si c=$c [$t]";
  for v in sab sabmain; do l="$l | $v: $(env SAB=$sab SABI=$si VC_FILE=$f VC_FIX="$t $c 0 0" timeout 900 '$B'/vc3_$v 1 1 2>&1 | grep SUMMARY | grep -oE "refused=[0-9]+|realViolations=[0-9]+|worstCone=[-0-9.e+]+" | tr "\n" " ")"; done; echo "$l"' | sort > $S/sab5.txt
echo done >> $S/sab5.txt
