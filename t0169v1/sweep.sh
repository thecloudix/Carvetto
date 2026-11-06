#!/bin/sh
# usage: sweep.sh <bin dir> <out dir> <cases> <jobs> "<cfg list>" "<families>" [binary prefix, default pk]:
# pk_br / pk_main per family x config (own oracle + hashes). cfg list entries name|ENV=.. separated by spaces in quotes, ';' between entries
B=$1; R=$2; N=$3; J=$4; CFGS=$5; FAMS=$6; PFX=${7:-pk}; mkdir -p $R
echo "$CFGS" | tr ';' '\n' | while read cfg; do [ -z "$cfg" ] && continue; for f in $FAMS; do for v in br main; do echo "$cfg|$f|$v"; done; done; done |
 xargs -P $J -I{} sh -c 'n=$(echo "{}" | cut -d"|" -f1); e=$(echo "{}" | cut -d"|" -f2); f=$(echo "{}" | cut -d"|" -f3); v=$(echo "{}" | cut -d"|" -f4);
   env $e timeout 2400 '$B'/'$PFX'_$v $f 1000 '$N' > '$R'/${n}_f${f}_$v.txt 2>&1; echo "exit $?" >> '$R'/${n}_f${f}_$v.txt'
echo done > $R/DONE
