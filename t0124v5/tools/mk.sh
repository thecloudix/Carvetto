#!/bin/bash
# usage: mk.sh src.cpp out [lib.a]   (includes: this dir first, then v2 headers)
B=/home/claude/vscratch/t0124v3/br; L=${3:-$B/core/libcarvetto_core.a}
c++ -std=c++20 -O2 -DCLIPPER2_MAX_DECIMAL_PRECISION=8 -I/home/claude/wt-T-0120/core/include -I/home/claude/vscratch/t0124v4/tools -I/home/claude/vscratch/t0124v2 -isystem $B/_deps/clipper2-src/CPP/Clipper2Lib/include -isystem $B/_deps/nlohmann_json-src/include "$1" -o "$2" $L $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a 2>&1 | grep -E "error" 
exit ${PIPESTATUS[0]}
