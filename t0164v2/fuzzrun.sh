#!/bin/sh
# t0164v2 fuzz6 matrix: modes normal / BADEND (orig: end moved, radius stale) / BADEND_R (end moved, radius exact) /
# BADSTART (start moved) / DROP (silently dropped contours claimed own), branch vs main
S=/home/chris/scratch-t0164v2; G6=/home/chris/harness/t0153v6
for f in $G6/fuzzgeo/nest.txt $G6/fuzzgeo/thinwalls.txt /home/chris/harness/t0153v4/repro4/framehole_isl_g0.05.txt /home/chris/harness/t0153v4/repro4/annulus_isl_g0.01.txt /home/chris/harness/t0153v5/repro5/stad_isl_sab4_c0.05.txt $G6/repro6/fullisl_c1.0_g0.3_d0.3.txt $G6/repro6/sq_hole_diag.txt /home/chris/harness/t0164v2/geo/drop_rect_centre.txt /home/chris/harness/t0164v2/geo/drop_multi.txt; do
 for c in 0.01 0.05 0.3 1.0; do for v in br main; do
  echo "$v|$f|$c|20000|" ; echo "$v|$f|$c|5000|BADEND=1"; echo "$v|$f|$c|5000|BADEND=1 BADEND_R=1"; echo "$v|$f|$c|5000|BADSTART=1"; echo "$v|$f|$c|5000|DROP=1"
 done; done; done | xargs -P 3 -I{} sh -c 'v=$(echo "{}"|cut -d"|" -f1); f=$(echo "{}"|cut -d"|" -f2); c=$(echo "{}"|cut -d"|" -f3); n=$(echo "{}"|cut -d"|" -f4); e=$(echo "{}"|cut -d"|" -f5);
   r=$(env $e timeout 900 '$S'/bin/fuzz6_$v $f $c $n 11 2>/dev/null | tail -1); echo "$v [$e] $r"' > $S/fuzz.txt
echo done > $S/fuzz.done
