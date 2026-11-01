#!/bin/bash
cd /home/claude/t17r3
B=$1
./$B fuzz4 8000 0xf5000000 5 > f_${B}_5a.log 2>&1 &
./$B fuzz4 4000 0xf5100000 5 > f_${B}_5b.log 2>&1 &
wait
./$B fuzz4 4000 0xf2000000 2 > f_${B}_2.log 2>&1 &
GEN3=1 ./$B fuzz4 4000 0xf3000000 3 > f_${B}_3.log 2>&1 &
wait
./$B fuzz4 4000 0xf4000000 4 > f_${B}_4.log 2>&1
echo done > run_${B}.done
