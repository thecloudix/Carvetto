#!/bin/bash
# AC3: branch vs origin/main, every join x side, xk + vchk (cmp6), fresh seeds 0x7128xxxx. P parallel jobs (default 3).
D=/home/chris/wt-T0128/build-verify1-drv; L=/home/chris/harness/t0128v1/logs; cd $D
cat > $L/jobs.txt <<EOT
timeout 3600 nice ./cmp6 5 1500 0x71285000 > $L/f5a.log 2>&1
timeout 3600 nice ./cmp6 2 1500 0x71282000 > $L/f2a.log 2>&1
timeout 3600 nice ./cmp6 0 3000 0x71280000 > $L/f0a.log 2>&1
timeout 3600 nice ./cmp6 3 1500 0x71283000 > $L/f3a.log 2>&1
timeout 3600 nice ./cmp6 4 2000 0x71284000 > $L/f4a.log 2>&1
timeout 3600 nice ./cmp6 5 1500 0x71295000 > $L/f5b.log 2>&1
timeout 3600 nice ./cmp6 23 1500 0x712a3000 > $L/f23.log 2>&1
timeout 3600 nice ./cmp6 22 1500 0x712a2000 > $L/f22.log 2>&1
timeout 3600 nice ./cmp6 25 1500 0x712a5000 > $L/f25.log 2>&1
timeout 3600 nice ./cmp6 21 1500 0x712a1000 > $L/f21.log 2>&1
timeout 3600 nice ./cmp6 2 1500 0x71292000 > $L/f2b.log 2>&1
timeout 3600 nice ./cmp6 0 3000 0x71290000 > $L/f0b.log 2>&1
EOT
xargs -P ${P:-3} -I{} bash -c "{}" < $L/jobs.txt
echo done > $L/fuzz.done
