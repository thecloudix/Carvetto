#!/bin/bash
# Release test binaries with mutant offset.cpp: mutrel.sh H ...  -> X-relt/tests
M=/home/chris/wt-T0128/build-verify2-mut; B=/home/chris/wt-T0128/build-verify2-rel; cd $B
CC=$(sed -e 's/-Werror//g' -e "s#-MD -MT.*##" $M/ccrel.txt)
LD=$(cat $M/ldrel.txt)
for X in "$@"; do
  mkdir -p $M/$X-relt
  eval "$CC -I/home/chris/wt-T0128/core/src/geometry -I/home/chris/wt-T0128/core/src -o $M/$X-relt/offset.cpp.o -c $M/offset_$X.cpp" > $M/$X-relt/cc.log 2>&1 || { echo "$X compile failed"; continue; }
  cp core/libcarvetto_core.a $M/$X-relt/lib.a && ar r $M/$X-relt/lib.a $M/$X-relt/offset.cpp.o
  L2=$(echo "$LD" | sed -e "s#core/libcarvetto_core.a#$M/$X-relt/lib.a#g" -e "s#-o core/tests/carvetto_tests #-o $M/$X-relt/tests #" -e 's/-Werror//g' -e 's#-Wl,--dependency-file=[^ ]*##')
  bash -c "$L2" > $M/$X-relt/ld.log 2>&1 || { echo "$X link failed"; continue; }
  echo "$X-relt built"
done
