#!/bin/bash
# usage: mk.sh src.cpp out [dbg|rel]
B=/home/claude/wt-T-0120/build-release; [ "$3" = rel ] && B=/home/claude/wt-T-0120/build-release
OPT="-g -O0"; [ "$3" = rel ] && OPT="-O2"
c++ -std=c++20 $OPT -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I/home/claude/wt-T-0120/core/include -I/home/claude/vscratch/t0124v2 -I/tmp/claude-0/-home-claude/dd097e91-8a7e-530c-818b-244e9548b908/scratchpad/t0124v -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include "$1" -o "$2" $B/core/libcarvetto_core.a $(find $B/_deps -name "libClipper2*.a") $B/core/libcarvetto_miniz.a
