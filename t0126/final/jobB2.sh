#!/bin/bash
cd /home/claude/vscratch/t0126/final
{ for f in 0 1 2 3; do ./rawph $f 1 100 0.01; ./rawph $f 1 40 0.1; done; } > rawph.log 2>&1
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
for ch in 0.01 0.1 0.5; do for k in 0 1 2 3 4 5 6 7 8 9 10; do
  al=0; if [ $k -eq 3 ] || [ $k -eq 9 ]; then al=0.3; fi
  ./scan2 $k 7$k$((${#ch}*13)) 4000 $ch $al
done; done > S1.log 2>&1
for k in 0 1 2 3 5 6 7 8 9 10; do DLO=0.2 DHI=70 NORAW=1 ./scan2 $k 88$k 4000 0.05 0; done > S2.log 2>&1
for ch in 0.01 0.1 0.5; do for k in 12 13 14 8; do
  al=0; if [ $k -eq 13 ]; then al=0.2; fi
  ./scan2 $k 33$k${#ch} 4000 $ch $al
  PD=0.02 ./scan2 $(( k == 8 ? 2 : k - 12 )) 34$k${#ch} 4000 $ch 0
done; done > S3.log 2>&1
{
for s in $(seq 1 30); do NORAW=1 ./scan2 12 $s 400 0.01 0; done
for s in $(seq 31 45); do NORAW=1 ./scan2 12 $s 400 0.1 0.1; done
for s in $(seq 46 55); do NORAW=1 ./scan2 12 $s 400 0.5 0; done
for ch in 0.01 0.1 0.5; do ./scan2 15 9${#ch} 4000 $ch 0; ./scan2 16 9${#ch} 4000 $ch 0.15; done
} > S4.log 2>&1
echo DONE > B2.done
