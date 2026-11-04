#!/bin/sh
# usage: build.sh <worktree> <builddir-name> src.cpp out
W=$1; B=$W/$2
g++ -std=c++20 -O0 -g -DCARVETTO_POSTS_DIR=\"$W/posts\" -DCARVETTO_GOLDEN_DIR=\"$W/core/tests/golden\" -I$W/core/include -I$W/core/tests/post -I$W/core/src/document -I$W/core/src/toolpath "$3" -o "$4" $B/core/libcarvetto_core.a $B/core/libcarvetto_miniz.a $(find $B/_deps -name "libClipper2*.a" | head -1)
