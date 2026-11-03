#!/bin/bash
cd /home/chris/wt-T0135/build-verify2/drv
f=$1; o=advo/$(basename $f .scn).txt; [ -s $o ] && exit 0
echo "F $f $(TABS=1 SCENE=$f timeout 300 ./bin/wsf8x 0 1 99 | grep -v SUMMARY | tr '\n' ' ')" > $o.tmp && mv $o.tmp $o
