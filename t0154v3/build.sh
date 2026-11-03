#!/bin/sh
# usage: build.sh src.cpp out
W=/home/chris/wt-T0154
g++ -std=c++20 -O0 -g -DCARVETTO_POSTS_DIR=\"$W/posts\" -DCARVETTO_GOLDEN_DIR=\"$W/core/tests/golden\" -I$W/core/include -I$W/core/tests/post -I$W/core/src/document -I$W/core/src/toolpath "$1" -o "$2" $W/build-verify3/core/libcarvetto_core.a $W/build-verify3/core/libcarvetto_miniz.a $(find $W/build-verify3/_deps -name "libClipper2*.a" | head -1)
