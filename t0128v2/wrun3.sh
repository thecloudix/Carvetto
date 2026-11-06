#!/bin/bash
# wrun.sh fam N seed tag: run wfz (W lib), keep WVIOL lines and count WADD by kind.
export WBIN=wfz3; D=/home/chris/wt-T0128/build-verify2-drv; L=${WLOG:-/home/chris/harness/t0128v2/logs/w}
cd $D; timeout 2400 ./${WBIN:-wfz} $1 $2 $3 2>&1 >$L/$4.out | grep -E "WVIOL|WADD" | awk '/WVIOL/{print; v++} /WADD/{a[$3]++} END{for (k in a) print "WADD kind", k, a[k]; print "WVIOL total", v+0}' > $L/$4.err
