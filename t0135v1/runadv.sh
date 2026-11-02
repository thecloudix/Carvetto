#!/bin/bash
# runadv.sh dir bin out : run oracle on each scene
cd /home/chris/wt-T0135/build-verify/drv
for f in $1/*.scn; do echo "F $f $(TABS=1 SCENE=$f timeout 300 ./bin/$2 0 1 99 | grep -v SUMMARY | tr '\n' ' ')"; done > $3.tmp; mv $3.tmp $3
