#!/bin/sh
# t0164v2: reduced fuzz6 on the long-strip drop geometries (1500 iterations; the full fuzzrun matrix took >10 min per run there)
S=/home/chris/scratch-t0164v2
for f in /home/chris/harness/t0164v2/geo/drop_rect_centre.txt /home/chris/harness/t0164v2/geo/drop_multi.txt /home/chris/harness/t0164v2/geo/drop_slot.txt; do
 for c in 0.01 0.05 0.3 1.0; do for v in br main; do echo "$v|$f|$c|1500|"; echo "$v|$f|$c|1500|DROP=1"; echo "$v|$f|$c|1500|BADEND=1"; done; done; done |
 xargs -P 3 -I{} sh -c 'v=$(echo "{}"|cut -d"|" -f1); f=$(echo "{}"|cut -d"|" -f2); c=$(echo "{}"|cut -d"|" -f3); n=$(echo "{}"|cut -d"|" -f4); e=$(echo "{}"|cut -d"|" -f5);
   r=$(env $e timeout 900 '$S'/bin/fuzz6_$v $f $c $n 11 2>/dev/null | tail -n 1); echo "$v [$e] $r"' > $S/fuzzdrop.txt
echo done > $S/fuzzdrop.done
