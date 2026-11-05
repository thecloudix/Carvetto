#!/bin/bash
source <(sed -n '/^M=/,/^}/p' /home/chris/harness/t0160v1/mutants.sh)
run groove_only3 $C '        const double ay = std::fabs(y);' '        if (k != 0) {
            continue;
        }
        const double ay = std::fabs(y);'
cp $O/corners.orig $C; cp $O/vcarve.orig $V; ninja -C $M/b corner_only > /dev/null 2>&1; echo restored
