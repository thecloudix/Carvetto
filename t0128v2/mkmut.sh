#!/bin/bash
# mkmut.sh X : Release lib with offset_X.cpp in place of offset.cpp -> X-rel/lib.a
M=/home/chris/wt-T0128/build-verify2-mut; B=/home/chris/wt-T0128/build-verify2-rel; cd $B
CC=$(sed -e 's/-Werror//g' -e "s#-MD -MT.*##" $M/ccrel.txt)
for X in "$@"; do
  mkdir -p $M/$X-rel
  $CC -I/home/chris/wt-T0128/core/src/geometry -I/home/chris/wt-T0128/core/src -o $M/$X-rel/offset.cpp.o -c $M/offset_$X.cpp > $M/$X-rel/cc.log 2>&1 || { echo "$X compile failed"; continue; }
  cp core/libcarvetto_core.a $M/$X-rel/lib.a && ar r $M/$X-rel/lib.a $M/$X-rel/offset.cpp.o && echo "$X-rel built"
done
