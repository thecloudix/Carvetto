#!/bin/bash
# usage: cc.sh src.cpp out   (links against the worktree Release lib)
W=/home/claude/wt-T-0126; B=$W/build-rel; S=/home/claude/vscratch/t0126
g++ -std=c++20 -O2 -I$W/core/include -I$W/core/src/toolpath -I$S/deps/clipper2-src/CPP/Clipper2Lib/include "$1" -o "$2" $B/core/libcarvetto_core.a $(find $B/_deps -name 'libClipper2.a' | head -1) -lpthread
