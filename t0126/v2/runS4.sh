#!/bin/bash
cd /tmp/claude-0/-home-claude/dd097e91-8a7e-530c-818b-244e9548b908/scratchpad/t0125v2
{
for s in $(seq 1 30); do INSTR=1 NORAW=1 ./scan2_i_S4 12 $s 400 0.01 0; done
for s in $(seq 31 45); do INSTR=1 NORAW=1 ./scan2_i_S4 12 $s 400 0.1 0.1; done
for s in $(seq 46 55); do INSTR=1 NORAW=1 ./scan2_i_S4 12 $s 400 0.5 0; done
for ch in 0.01 0.1 0.5; do INSTR=1 ./scan2_i_S4 15 9${#ch} 4000 $ch 0; INSTR=1 ./scan2_i_S4 16 9${#ch} 4000 $ch 0.15; done
} > S4.log 2>&1
echo DONE >> S4.log
