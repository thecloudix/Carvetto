#!/bin/bash
# usage: hb.sh <src.cpp> <out>
V=/home/claude/vscratch/t0017v3; W=/home/claude/wt-T-0017; B=${HB_B:-$V/b-RelLib}
cd $V/h && g++ -std=c++20 -O2 -Wall ${1:-vdrv4.cpp} main_wrap.o -I$W/core/include -I. -I$W/core/tests/geometry $B/core/libcarvetto_core.a $V/h/mainlib/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a ${3} -o ${2:-vdrv4} 2>&1 | grep -E "error|warning" | head -20
