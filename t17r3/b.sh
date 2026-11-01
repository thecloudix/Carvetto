#!/bin/bash
# b.sh out : verifier vdrv4 (round-3 harness) against the worktree build-rel and main
V=/tmp/claude-0/-home-claude/dd097e91-8a7e-530c-818b-244e9548b908/scratchpad/t0017v2; W=/home/claude/wt-T-0017; B=$W/build-rel
nice g++ -std=c++20 -O2 -Wall $V/vdrv4.cpp $V/main_wrap.o -I$W/core/include -I$V -I$W/core/tests/geometry $B/core/libcarvetto_core.a $V/mainns2/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a -o /home/claude/t17r3/$1 2>&1 | grep -E "error" | head
