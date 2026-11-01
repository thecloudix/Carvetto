#!/bin/bash
cd /home/claude/cscratch-T0124
export CELL=0.04
(./conn 41001 400 0 -1 > sw_A.log 2>&1 &
 TLMIN=0.3 TLMAX=4 ./conn 41002 300 0 1 > sw_B.log 2>&1 &
 TLMIN=0.3 TLMAX=4 ./conn 41003 250 1 1 > sw_C.log 2>&1 &
 ./conn 41004 200 4 1 > sw_D.log 2>&1 &
 TLMIN=0.3 TLMAX=4 ./conn 41005 250 6 -1 > sw_E.log 2>&1 &
 wait)
(./conn 41006 150 5 -1 > sw_F.log 2>&1 &
 ./conn 41007 150 3 -1 > sw_G.log 2>&1 &
 ./conn 41008 150 7 -1 > sw_H.log 2>&1 &
 ./conn 41009 150 8 -1 > sw_I.log 2>&1 &
 ./conn 41010 150 2 -1 > sw_J.log 2>&1 &
 wait)
echo done > sweeps.done
