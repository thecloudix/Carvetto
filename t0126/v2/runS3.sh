#!/bin/bash
cd /tmp/claude-0/-home-claude/dd097e91-8a7e-530c-818b-244e9548b908/scratchpad/t0125v2
for ch in 0.01 0.1 0.5; do for k in 12 13 14 8; do
  al=0; if [ $k -eq 13 ]; then al=0.2; fi
  INSTR=1 ./scan2_i_S3 $k 33$k${#ch} 4000 $ch $al
  INSTR=1 PD=0.02 ./scan2_i_S3 $(( k == 8 ? 2 : k - 12 )) 34$k${#ch} 4000 $ch 0
done; done > S3.log 2>&1
echo DONE >> S3.log
