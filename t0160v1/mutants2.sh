#!/bin/bash
# t0160v1 mutants round 2 (replacements for the ones that did not build under -Werror)
source <(sed -n '/^M=/,/^}/p' /home/chris/harness/t0160v1/mutants.sh)
run groove_only2 $C 'const double ay = std::fabs(y);' 'if (k != 0) continue; const double ay = std::fabs(y);'
run eps_big2 $V 'const detail::RingRegion wallRegion(std::move(ringPoints), 1e-9 * scale);' 'const detail::RingRegion wallRegion(std::move(ringPoints), 1e-2 + 0.0 * scale);'
run no_degenerate_skip2 $C 'if (spanSize(c.spans[i]) > spec.distance) {' 'if (spanSize(c.spans[i]) > -1.0) {'
cp $O/corners.orig $C; cp $O/vcarve.orig $V; ninja -C $M/b corner_only > /dev/null 2>&1; echo restored
