#!/bin/bash
cd /tmp/claude-0/-home-claude/dd097e91-8a7e-530c-818b-244e9548b908/scratchpad/t0125v2
{
DLO=5.0 DHI=5.15 ./r1_scan 3 301 600 0.01
./r1_scan 3 301 4000 0.01
./r1_scan 3 700 4000 0.01
./r1_scan 3 606 4000 0.5
./r1_scan 3 505 4000 0.01
./r1_scan 10 101 4000 0.01
./r1_scan 6 101 4000 0.01
./r1_lobe 3 700 40 0.01
./r1_lobe 3 300 40 0.1
./r1_lobe 8 300 40 0.01
./r1_lobe 10 1 1 0.01
} > R1.log 2>&1
echo DONE >> R1.log
