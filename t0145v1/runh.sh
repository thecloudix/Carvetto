#!/bin/bash
cd /home/chris/wt-T0145/build-verify/v145
P=/home/chris/wt-T0145/posts
for v in base new; do
 ( for m in 0 1 2; do for s in 1 2 3 4; do HASH=1 ./bin/wsh_$v $((s*1000+m)) 200 $m; done; done > out/wsh_$v.txt 2>&1 ) &
 ( for s in 11 12 13 14; do ./bin/pc_$v $s 150 0; done | sed 's/ t [0-9.]*$//' > out/pc_$v.txt 2>&1 ) &
 ( for s in 21 22 23; do ./bin/eg_$v $s 200; done > out/eg_$v.txt 2>&1 ) &
 ( mkdir -p out/gen_$v out/eggen_$v; ./bin/gen_$v out/gen_$v $P 400 > out/gen_$v.log 2>&1; for s in 31 32 33; do ./bin/eggen_$v $P out/eggen_$v 300 $s; done > out/eggen_$v.log 2>&1 ) &
 wait
done
echo done > out/ALLDONE
