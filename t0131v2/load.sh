#!/bin/bash
# Usage: load.sh <src> <builddir> <jobs>  -- rebuilds <src> in <builddir> from clean, forever,
# as a compile-load generator. Stop with: kill $(cat <builddir>.pid)
src=$1; bd=$2; j=${3:-6}
echo $$ > "$bd.pid"
cmake -S "$src" -B "$bd" -G Ninja -DCMAKE_BUILD_TYPE=Release -DCARVETTO_BUILD_APP=OFF >/dev/null
while :; do cmake --build "$bd" --target clean >/dev/null; cmake --build "$bd" -j"$j" >/dev/null 2>&1; done
