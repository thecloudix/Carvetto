#!/bin/bash
# cntcheck.sh mode seed sc : contours, loose diags, NOLEAD oracle verdict
cd /home/chris/wt-T0127/build-verify3/drv
f=d/wsf8/m$1_$2_$3.scn
echo "== $1 $2 $3 contours $(grep -c '^C' $f) open $(grep -c '^C 0' $f)"
./bin/diag $f | grep "comes free" | sed 's/^DIAG 10 /  /'
echo -n "  NOLEAD: "; NOLEAD=1 SCENE=$f ./bin/wsf8 0 1 99 | head -1 | sed 's/.*| G/G/'
