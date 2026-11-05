#!/bin/sh
# t0164v1: moved branch ends (SABEND=<d>, SABEND2 also moves starts) alone and with SAB=2/4 wrong skeletons; also SAB=2/3/4 matrix
S=/home/chris/scratch-t0164v1; B=$S/bin
for f in /home/chris/harness/t0153v4/repro4/annulus_isl_g0.01.txt /home/chris/harness/t0153v4/repro4/framehole_isl_g0.05.txt /home/chris/harness/t0153v5/repro5/stad_isl_sab2_c0.05.txt /home/chris/harness/t0153v5/repro5/stad_isl_sab4_c0.05.txt /home/chris/harness/t0153v6/repro6/sq_hole_diag.txt /home/chris/harness/t0164v1/geo/drop_rect_q.txt /home/chris/harness/t0153v6/fuzzgeo/nest.txt; do
 for sab in 0 2 3 4; do for e in "" "SABEND=0.3" "SABEND=2" "SABEND=0.3 SABEND2=1"; do for t in "0 12 90" "1 6 0"; do echo "$f|$sab|$e|$t"; done; done; done; done |
 xargs -P 3 -I{} sh -c 'f=$(echo "{}"|cut -d"|" -f1); sab=$(echo "{}"|cut -d"|" -f2); e=$(echo "{}"|cut -d"|" -f3); t=$(echo "{}"|cut -d"|" -f4); l="$(basename $f) SAB=$sab [$e] [$t]";
  for v in sab sabmain; do l="$l | $v: $(env SAB=$sab $e VC_FILE=$f VC_FIX="$t 0.05 0 0" timeout 600 '$B'/vc3_$v 1 1 2>&1 | grep SUMMARY | grep -oE "refused=[0-9]+|realViolations=[0-9]+|worstCone=[-0-9.e+]+" | tr "\n" " ")"; done; echo "$l"' | sort > $S/sabend.txt
echo done >> $S/sabend.txt
