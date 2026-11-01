#!/bin/bash
cd /tmp/claude-0/-home-claude/dd097e91-8a7e-530c-818b-244e9548b908/scratchpad/t0125v2
for ch in 0.01 0.1 0.5; do for k in 0 1 2 3 4 5 6 7 8 9 10; do
  al=0; if [ $k -eq 3 ] || [ $k -eq 9 ]; then al=0.3; fi
  ./scan2 $k 7$k$((${#ch}*13)) 4000 $ch $al
done; done > S1.log 2>&1
echo DONE >> S1.log
