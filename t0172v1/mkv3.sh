#!/bin/bash
# t0128v3 vdrv5 (sx3.inc) vs branch Release lib, REF namespaced; WRAP=1 links ulpwrap (libm perturbation) -> build-verify1-drv/<out>
W=/home/chris/wt-T0172; H=/home/chris/harness/t0172v1; H3=/home/chris/harness/t0009v3; D=$W/build-verify1-drv; cd $D
REF=${REF:-main}; B=${BLIB:-$W/build-verify1-rel}; R=$W/build-verify1-${REF}ns
[ -f ulpwrap.o ] || gcc -O2 -c $H/ulpwrap.c -o ulpwrap.o
X=""; [ -n "$WRAP" ] && X="$D/ulpwrap.o -Wl,--wrap=sin,--wrap=cos,--wrap=atan2,--wrap=hypot,--wrap=acos,--wrap=asin,--wrap=tan,--wrap=sincos,--wrap=atan"
g++ -std=c++20 -O2 -w $H/v3/vdrv5.cpp mw_$REF.o -I$H/v3 -I$H3 -I$W/core/include -I$W/core/tests/geometry $B/core/libcarvetto_core.a $R/core/libcarvetto_core.a $B/_deps/clipper2-build/libClipper2.a $B/core/libcarvetto_miniz.a $X -o $1 2>&1 | grep -E "error" | head
