#!/bin/bash
# dbg.sh main.cpp out : instrumented dbg_src.cpp linked ahead of the library
W=/home/claude/wt-T-0017
g++ -std=c++20 -O1 -c /home/claude/t17r3/dbg_src.cpp -I$W/core/include -I$W/core/src/geometry -I$W/core/src -I$W/build-rel/_deps/clipper2-src/CPP/Clipper2Lib/include -o /home/claude/t17r3/dbg.o || exit 1
g++ -std=c++20 -O1 "$1" /home/claude/t17r3/dbg.o -I$W/core/include -I$W/core/tests/geometry $W/build-rel/core/libcarvetto_core.a $W/build-rel/_deps/clipper2-build/libClipper2.a -o $2
