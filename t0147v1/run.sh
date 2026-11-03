#!/bin/bash
cd /home/chris/wt-T0147/build-verify/h; mkdir -p out
for v in new old; do
 ( for s in 47110 47111 47112; do ./pc2_$v $s 3000 0 | sed 's/ t [0-9.]*$//' > out/pc2_${v}_$s.txt; done ) &
 ( for s in 55501 55502 55503; do ./eg_$v $s 3000 > out/eg_${v}_$s.txt; done ) &
 wait
 ( for m in 0 1 2 3; do HASH=1 ./wsh2_$v 0x7a110$m 800 $m | grep '^H' > out/wsh2_${v}_$m.txt; done ) &
 ( ./dgen_$v out/dgen_$v.d none 200 > out/dgen_${v}.txt 2>&1 ) &
 wait
done
echo DONE
