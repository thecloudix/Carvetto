#!/bin/bash
cd /home/claude/vscratch/t0124v2
export CELL=0.04
./conn_sweep 41001 400 0 -1 > sw_A.log 2>&1
TLMIN=0.3 TLMAX=4 ./conn_sweep 41003 250 1 1 > sw_C.log 2>&1
TLMIN=0.3 TLMAX=4 ./conn_sweep 41005 250 6 -1 > sw_E.log 2>&1
./conn_sweep 41007 150 3 -1 > sw_G.log 2>&1
./conn_sweep 41009 150 8 -1 > sw_I.log 2>&1
echo done > sweep1.done
