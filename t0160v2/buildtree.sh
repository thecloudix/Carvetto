#!/bin/bash
# builds: $1 = srcdir, $2 = builddir, $3 = type, $4 = extra flags, $5 = jobs
set -o pipefail
cmake -S "$1" -B "$2" -G Ninja -DCMAKE_BUILD_TYPE=$3 -DCARVETTO_BUILD_APP=OFF -DCMAKE_CXX_FLAGS="$4" > "$2.cfg.log" 2>&1 && timeout 3000 ninja -C "$2" -j${5:-3} > "$2.build.log" 2>&1; echo "exit $?" >> "$2.build.log"
