#!/bin/bash
# mkmutbin2.sh X...: Debug test binary per mutant at build-verify2-mut/X/tests (offset.cpp + box_tree_test.cpp from mutants/X).
W=/home/chris/wt-T0172; B=$W/build-verify2; O=$W/build-verify2-mut; H=/home/chris/harness/t0172v2/mutants; cd $B
LIBCMD=$(ninja -t commands core/tests/carvetto_tests | grep -E " -o core/CMakeFiles/carvetto_core.dir/src/geometry/offset.cpp.o -c" | sed -e 's# -MD -MT [^ ]* -MF [^ ]*##' -e 's# -o .*$##' -e 's#-Werror##g')
TSTCMD=$(ninja -t commands core/tests/carvetto_tests | grep -E "box_tree_test.cpp.o -c" | sed -e 's# -MD -MT [^ ]* -MF [^ ]*##' -e 's# -o .*$##' -e 's#-Werror##g')
LINK=$(ninja -t commands core/tests/carvetto_tests | tail -n 1 | sed -e 's#^: && ##' -e 's# && cd .*$##' -e 's#-Wl,--dependency-file=[^ ]*##')
for X in "$@"; do
  D=$O/$X; mkdir -p $D; cp core/libcarvetto_core.a $D/lib.a
  $LIBCMD -I$W/core/src/geometry -I$W/core/src -o $D/offset.o -c $H/$X/offset.cpp > $D/build.log 2>&1 || { echo "$X lib compile failed"; continue; }
  $TSTCMD -o $D/bt.o -c $H/$X/box_tree_test.cpp >> $D/build.log 2>&1 || { echo "$X test compile failed"; continue; }
  (cd $D && ar r lib.a offset.o 2>/dev/null) 
  # ar r replaces member 'offset.o'? the archive member is offset.cpp.o: rename
  (cd $D && cp offset.o offset.cpp.o && ar r lib.a offset.cpp.o && ar d lib.a offset.o)
  L=${LINK//core\/libcarvetto_core.a/$D/lib.a}
  L=${L//core\/tests\/CMakeFiles\/carvetto_tests.dir\/geometry\/box_tree_test.cpp.o/$D/bt.o}
  L=${L//-o core\/tests\/carvetto_tests/-o $D/tests}
  eval "$L" >> $D/build.log 2>&1 && echo "$X built" || echo "$X link failed"
done
