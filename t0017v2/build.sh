#!/bin/bash
# usage: build.sh <src.cpp> <out>   (links branch Release lib + main cvmain lib)
V=/tmp/claude-0/-home-claude/dd097e91-8a7e-530c-818b-244e9548b908/scratchpad/t0017v2; W=/home/claude/wt-T-0017; B=$W/build-verify2-rel
cd $V && g++ -std=c++20 -O2 -Wall ${1:-vdrv4.cpp} main_wrap.o -I$W/core/include -I. -I$W/core/tests/geometry $B/core/libcarvetto_core.a $V/mainns2/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a -o ${2:-vdrv4} 2>&1 | grep -E "error|warning" | head -20
