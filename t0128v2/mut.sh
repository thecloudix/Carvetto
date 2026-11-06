#!/bin/bash
# Mutants of offset.cpp linked into the Debug test binary (build-verify2): mut.sh A B C ...
M=/home/chris/wt-T0128/build-verify2-mut; B=/home/chris/wt-T0128/build-verify2; cd $B
CC=$(sed -e 's/-Werror//g' -e "s#-MD -MT.*##" $M/cc.txt)
LD=$(cat $M/ld.txt)
for X in "$@"; do
  mkdir -p $M/$X
  eval "$CC -I/home/chris/wt-T0128/core/src/geometry -I/home/chris/wt-T0128/core/src -o $M/$X/offset.cpp.o -c $M/offset_$X.cpp" > $M/$X/cc.log 2>&1 || { echo "$X compile failed"; continue; }
  cp core/libcarvetto_core.a $M/$X/lib.a && ar r $M/$X/lib.a $M/$X/offset.cpp.o
  L2=$(echo "$LD" | sed -e "s#core/libcarvetto_core.a#$M/$X/lib.a#g" -e "s#-o core/tests/carvetto_tests #-o $M/$X/tests #" -e 's/-Werror//g' -e 's#-Wl,--dependency-file=[^ ]*##')
  bash -c "$L2" > $M/$X/ld.log 2>&1 || { echo "$X link failed"; continue; }
  echo "$X built"
done
