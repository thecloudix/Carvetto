#!/bin/bash
cd /home/claude/vscratch/t0126/final2
{ echo ===AC; ./ac; ./ac34; echo ===ROUTE; ./route; echo ===CUSP; ./cusp; echo ===CUSP2; ./cusp2; echo ===PHANTOM; ./phantom 1 1000; } > B1.log 2>&1
echo DONE >> B1.log
