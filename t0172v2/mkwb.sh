#!/bin/bash
# build wb.cpp against the branch sources (W) and Release lib (B).
W=${W:-/home/chris/wt-T0172}; B=${B:-$W/build-verify2-rel}; H=/home/chris/harness/t0172v2; O=$W/build-verify2-drv; mkdir -p $O
g++ -std=c++20 -O2 -g $H/${SRC:-wb.cpp} -I$W/core/src/geometry -I$W/core/src -I$W/core/include -I$B/_deps/clipper2-src/CPP/Clipper2Lib/include \
  $B/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a -o $O/${OUT:-wb} 2>&1 | grep -E "error|undefined" | head -20
