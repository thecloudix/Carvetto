#!/bin/bash
cd /home/claude/vscratch/t0126/final
ch=0.5; for k in 8 9 10; do al=0; if [ $k -eq 9 ]; then al=0.3; fi; ./scan2 $k 7$k$((${#ch}*13)) 4000 $ch $al; done >> S1.log 2>&1
for k in 0 1 2 3 5 6 7 8 9 10; do DLO=0.2 DHI=70 NORAW=1 ./scan2 $k 88$k 4000 0.05 0; done > S2.log 2>&1
for ch in 0.01 0.1 0.5; do for k in 12 13 14 8; do
  al=0; if [ $k -eq 13 ]; then al=0.2; fi
  ./scan2 $k 33$k${#ch} 4000 $ch $al
  PD=0.02 ./scan2 $(( k == 8 ? 2 : k - 12 )) 34$k${#ch} 4000 $ch 0
done; done > S3.log 2>&1
echo DONE > B3.done
