#!/bin/bash
# Release lib variant: mutants/offset_<X>.cpp compiled with build-verify1-rel's flags -> build-verify1-var<X> (lib + _deps symlink)
W=/home/chris/wt-T0172; B=$W/build-verify1-rel; H=/home/chris/harness/t0172v1; cd $B
CC=$(ninja -t commands core/libcarvetto_core.a | grep -E "src/geometry/offset.cpp.o -c" | sed -e 's# -MD -MT [^ ]* -MF [^ ]*##' -e 's#-Werror##g')
for X in "$@"; do O=$W/build-verify1-var$X; mkdir -p $O/core; ln -sfn $B/_deps $O/_deps; cp core/libcarvetto_miniz.a $O/core/; cp core/libcarvetto_core.a $O/core/libcarvetto_core.a
  c=${CC/-o core\/CMakeFiles\/carvetto_core.dir\/src\/geometry\/offset.cpp.o -c */-o $O/offset.cpp.o -c $H/mutants/offset_$X.cpp}
  (eval "$c -I$W/core/src/geometry -I$W/core/src") > $O/cc.log 2>&1 || { echo "$X failed"; continue; }
  (cd $O && ar r core/libcarvetto_core.a offset.cpp.o) && echo "$X rel built"; done
