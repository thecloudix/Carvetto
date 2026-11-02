#!/bin/bash
# dumpone.sh seed sc mode -> d1/m<mode>_<seed>_<sc>.scn via DUMPALL
cd /home/chris/wt-T0127/build-verify2/drv; mkdir -p d1
DUMPALL=1 DUMP=d1 SC=$2 ./bin/wsf7 $1 $(($2+1)) $3 > /dev/null
