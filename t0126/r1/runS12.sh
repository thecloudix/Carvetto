#!/bin/bash
cd /home/claude/vscratch/t0126/r1
for ch in 0.01 0.1 0.5; do for k in 0 1 2 3 4 5 6 7 8 9 10; do
  al=0; if [ $k -eq 3 ] || [ $k -eq 9 ]; then al=0.3; fi
  ./scan2 $k 7$k$((${#ch}*13)) 4000 $ch $al
done; done > S1.log 2>&1
echo DONE >> S1.log
for k in 0 1 2 3 5 6 7 8 9 10; do DLO=0.2 DHI=70 NORAW=1 ./scan2 $k 88$k 4000 0.05 0; done > S2.log 2>&1
echo DONE >> S2.log
