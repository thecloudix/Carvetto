#!/bin/bash
# phantom stress: diameters far past the largest inscribed disc
cd /tmp/claude-0/-home-claude/dd097e91-8a7e-530c-818b-244e9548b908/scratchpad/t0125v2
for k in 0 1 2 3 5 6 7 8 9 10; do DLO=0.2 DHI=70 NORAW=1 ./scan2 $k 88$k 4000 0.05 0; done > S2.log 2>&1
echo DONE >> S2.log
