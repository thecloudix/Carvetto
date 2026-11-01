#!/bin/bash
cd /home/claude/vscratch/t0124v2
export CELL=0.04
TLMIN=0.3 TLMAX=4 ./conn_sweep 41002 300 0 1 > sw_B.log 2>&1
./conn_sweep 41004 200 4 1 > sw_D.log 2>&1
./conn_sweep 41006 150 5 -1 > sw_F.log 2>&1
./conn_sweep 41008 150 7 -1 > sw_H.log 2>&1
./conn_sweep 41010 150 2 -1 > sw_J.log 2>&1
echo done > sweep2.done
