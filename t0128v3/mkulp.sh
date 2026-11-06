#!/bin/bash
# Debug test binaries with the libm wrapper and variant objects: br (cae3fa7), oldlib (e87f549 offset.cpp), oldtest (e87f549 test), old (both), F (mutant F).
W=/home/chris/wt-T0128; B=$W/build-verify3; U=$W/build-verify3-ulp; cd $B
LIBCMD=$(ninja -t commands core/tests/carvetto_tests | grep -E " -o core/CMakeFiles/carvetto_core.dir/src/geometry/offset.cpp.o -c" | sed -e 's# -MD -MT [^ ]* -MF [^ ]*##' -e 's#-Werror##')
TSTCMD=$(ninja -t commands core/tests/carvetto_tests | grep -E "offset_narrow_loops_test.cpp.o -c" | sed -e 's# -MD -MT [^ ]* -MF [^ ]*##' -e 's#-Werror##')
LINK=$(cat $U/link.sh)
mk() { # name libsrc testsrc
  local n=$1; mkdir -p $U/$n; cp core/libcarvetto_core.a $U/$n/lib.a
  local L="$LINK"; L=${L//core\/libcarvetto_core.a/$U/$n/lib.a}
  if [ "$2" != - ]; then c=${LIBCMD/-o core\/CMakeFiles\/carvetto_core.dir\/src\/geometry\/offset.cpp.o -c */-o $U/$n/offset.cpp.o -c $2}; (cd $B && eval "$c" -I$W/core/src/geometry -I$W/core/src) || return 1; (cd $U/$n && ar r lib.a offset.cpp.o); fi
  if [ "$3" != - ]; then c=${TSTCMD/-o core\/tests\/CMakeFiles\/carvetto_tests.dir\/geometry\/offset_narrow_loops_test.cpp.o -c */-o $U/$n/t.o -c $3}; (cd $B && eval "$c" -I$W/core/tests/geometry -I$W/core/tests) || return 1; L=${L//core\/tests\/CMakeFiles\/carvetto_tests.dir\/geometry\/offset_narrow_loops_test.cpp.o/$U/$n/t.o}; fi
  L=${L//..\/build-verify3-ulp\/carvetto_tests_ulp/$U/$n/tests}; (cd $B && eval "$L") && echo "$n built"
}
cd $B; mk oldtest - $U/test_old.cpp; mk old $U/offset_old.cpp $U/test_old.cpp; mk F $U/offset_F.cpp -; mk Fold $U/offset_F.cpp $U/test_old.cpp
