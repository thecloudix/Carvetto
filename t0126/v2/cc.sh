#!/bin/bash
# usage: cc.sh src.cpp out [builddir]
B=${3:-/home/claude/wt-T-0121/build-verify5-rel}
W=/home/claude/wt-T-0121
g++ -std=c++20 -O2 -I$W/core/include -I$B/_deps/clipper2-src/CPP/Clipper2Lib/include "$1" -o "$2" $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a -lpthread
