#!/bin/bash
# Debug test binaries (cae3fa7 tests) with round-2 mutants applied to cae3fa7 offset.cpp; then [t0128]~[perf] and, for survivors, [offset]~[perf].
W=/home/chris/wt-T0128; B=$W/build-verify3; U=$W/build-verify3-ulp; H=/home/chris/harness/t0128v3; L=$H/logs/mut; mkdir -p $L; cd $B
LIBCMD=$(ninja -t commands core/tests/carvetto_tests | grep -E " -o core/CMakeFiles/carvetto_core.dir/src/geometry/offset.cpp.o -c" | sed -e 's# -MD -MT [^ ]* -MF [^ ]*##' -e 's#-Werror##')
LINK=$(cat $U/link.sh)
for X in "$@"; do
  n=m$X; mkdir -p $U/$n; cp $W/core/src/geometry/offset.cpp $U/$n/offset_$X.cpp
  sed "s#^--- .*#--- offset_$X.cpp#; s#^+++ .*#+++ offset_$X.cpp#" $H/mutants/$X.diff | (cd $U/$n && patch -p0 -s offset_$X.cpp) || { echo "$X patch failed" >> $L/summary.txt; continue; }
  cp core/libcarvetto_core.a $U/$n/lib.a
  c=${LIBCMD/-o core\/CMakeFiles\/carvetto_core.dir\/src\/geometry\/offset.cpp.o -c */-o $U/$n/offset.cpp.o -c $U/$n/offset_$X.cpp}
  (cd $B && eval "$c" -I$W/core/src/geometry -I$W/core/src) > $L/$X.cc.log 2>&1 || { echo "$X compile failed" >> $L/summary.txt; continue; }
  (cd $U/$n && ar r lib.a offset.cpp.o)
  Lk="$LINK"; Lk=${Lk//core\/libcarvetto_core.a/$U/$n/lib.a}; Lk=${Lk//..\/build-verify3-ulp\/carvetto_tests_ulp/$U/$n/tests}
  (cd $B && eval "$Lk") || { echo "$X link failed" >> $L/summary.txt; continue; }
  ULP_K=0 timeout 900 $U/$n/tests "[t0128]~[perf]" > $L/$X.t0128.log 2>&1; r1=$?
  if [ $r1 = 0 ]; then ULP_K=0 timeout 1500 $U/$n/tests "[offset]~[perf]" > $L/$X.offset.log 2>&1; r2=$?; else r2=skip; fi
  echo "$X t0128 rc $r1 offset rc $r2" >> $L/summary.txt
done
