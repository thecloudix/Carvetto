#!/bin/bash
# AC2 run: branch vs main, every join x side. fam0 = T-0006 generator (makeCase); 2..5 later adversarial families; 20/21 dense zig-zag/hatch.
D=/home/chris/wt-T0016/build-verify1-drv; L=/home/chris/harness/t0016v1/logs; cd $D
cat > $L/jobs.txt <<EOT
timeout 5400 nice ./cmp6 0 4000 0x16000000 > $L/f0a.log 2>&1
timeout 5400 nice ./cmp6 0 4000 0x16100000 > $L/f0b.log 2>&1
timeout 5400 nice ./cmp6 2 2000 0x16200000 > $L/f2.log 2>&1
timeout 5400 nice ./cmp6 3 1500 0x16300000 > $L/f3.log 2>&1
timeout 5400 nice ./cmp6 4 1500 0x16400000 > $L/f4.log 2>&1
timeout 5400 nice ./cmp6 5 1500 0x16500000 > $L/f5.log 2>&1
timeout 5400 nice ./cmp6 20 1500 0x16600000 > $L/f20.log 2>&1
timeout 5400 nice ./cmp6 21 1500 0x16700000 > $L/f21.log 2>&1
EOT
xargs -P 6 -I{} bash -c "{}" < $L/jobs.txt
echo done > $L/fuzz.done
