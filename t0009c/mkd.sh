#!/bin/bash
# t9 against the debug copy of offset.cpp
W=/home/chris/wt-T0009; B=$W/build-rel; R=$W/build-h/ref-ns
cd $W/build-h/drv && g++ -std=c++20 -O2 -w t9.cpp ../dbg/offset_dbg.cpp main_wrap.o -I$W/core/include -I$W/core/src/geometry -I. -I$W/core/tests/geometry -I$B/_deps/clipper2-src/CPP/Clipper2Lib/include $B/core/libcarvetto_core.a $R/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a -o t9d 2>&1 | grep error | head
