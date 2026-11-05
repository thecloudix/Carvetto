#!/bin/bash
# ulpflood.sh scene var : main (TFS_OFF) and branch whole-scene flood under ULPS=-4..4 scale perturbations
f=$1; v=$2; VV=""; [ $v != orig ] && VV="VAR=$v"
for u in -4 -3 -2 -1 0 1 2 3 4; do for c in br main; do echo "$c $u"; done; done | xargs -P 6 -L 1 bash -c 'E=X=1; [ $0 = main ] && E=TFS_OFF=1; echo "$0 ulps $1 $(env $E '$VV' ULPS=$1 SCENE='$f' timeout 1800 /home/chris/wt-T0158/build-verify1-rel/bin/wsf_log 0 1 99 | grep "^S" | sed "s/.*| G/G/" | cut -c1-150)"'
