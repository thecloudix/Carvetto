#!/bin/bash
cd /home/claude/vscratch/t0126/final
timeout 1800 ./phantom 1 600 > phantom.log 2>&1; echo "rc=$?" >> phantom.log
./v3src sweep3 8800100 50 > sw3.log 2>&1
./v3src sweep2b 6600010 50 > sw2b.log 2>&1
echo DONE > C.done
