#!/bin/bash
# Debug test binary per mutant: mutant offset.cpp compiled with build-verify1's flags (no -Werror), lib copy, relinked -> MU/<X>/tests
W=/home/chris/wt-T0172; B=$W/build-verify1; MU=$W/build-verify1-mut; H=/home/chris/harness/t0172v1; mkdir -p $MU; cd $B
CC=$(ninja -t commands core/tests/carvetto_tests | grep -E "src/geometry/offset.cpp.o -c" | sed -e 's# -MD -MT [^ ]* -MF [^ ]*##' -e 's#-Werror##g')
LINK=$(ninja -t commands core/tests/carvetto_tests | /usr/bin/tail -n 1 | sed -e 's#^: && ##' -e 's# && :$##' -e 's#-Wl,--dependency-file=[^ ]*##')
for X in "$@"; do
  mkdir -p $MU/$X; cp core/libcarvetto_core.a $MU/$X/lib.a
  c=${CC/-o core\/CMakeFiles\/carvetto_core.dir\/src\/geometry\/offset.cpp.o -c */-o $MU/$X/offset.cpp.o -c $H/mutants/offset_$X.cpp}
  (eval "$c -I$W/core/src/geometry -I$W/core/src") > $MU/$X/cc.log 2>&1 || { echo "$X compile failed"; continue; }
  (cd $MU/$X && ar r lib.a offset.cpp.o)
  L=${LINK//core\/libcarvetto_core.a/$MU/$X/lib.a}; L=${L/-o core\/tests\/carvetto_tests/-o $MU/$X/tests}
  (eval "$L") > $MU/$X/ld.log 2>&1 && echo "$X built" || echo "$X link failed"
done
