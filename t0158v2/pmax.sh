#!/bin/bash
# pmax.sh file [variant]: max start move of the first tieFreeStart call under st pert (21 transforms), branch (rule on) and main (TFS_OFF)
B=/home/chris/wt-T0158/build-verify2-rel/bin; v=${2:-log}
A='$3==0{n=split($2,a,":"); if(a[n]=="id"){x0=$10;y0=$11;r0=$4} else if(ENVIRON["ALLX"]!="" || a[n] ~ /^[ur]/) {d=sqrt(($10-x0)^2+($11-y0)^2); if(d>mx){mx=d; w=a[n]}}} END{printf "%.4g(%s,rule%s) ", mx, w, r0}'
echo "$(basename $1) br $($B/st_$v pert $1 | grep '^T' | awk "$A") main $(TFS_OFF=1 $B/st_$v pert $1 | grep '^T' | awk "$A")"
