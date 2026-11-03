#!/bin/bash
# advone.sh scene bin -> advo/<scene>_<bin>.txt
cd /home/chris/wt-T0135/build-verify3/drv
f=$1; b=$2; o=advo/$(basename $f .scn)_$b.txt; [ -s $o ] && exit 0
echo "$(TABS=1 V=1 SCENE=$f timeout 900 ./bin/$b 0 1 99 | grep -v SUMMARY)" > $o.tmp && mv $o.tmp $o
