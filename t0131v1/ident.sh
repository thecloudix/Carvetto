#!/bin/bash
# ident.sh BINDIR P: card workloads, main vs branch hashes (pfh_*), P at a time -> BINDIR/ident_<v>.txt
D=$1
{
for a in 0 17 30 45 90; do for n in 250 1000; do echo "ANG=$a|comb $n"; done; done
echo "RD=0.1 TOOL=1|comb 500"; echo "OFFSET=1|comb 500"; echo "PP=1 ANG=90|comb 300"; echo "PP=0 ANG=45 RD=0.12|comb 200"
for v in 0 1 2; do echo "X=1|sep 1250 $v"; echo "TABS=1|sep 1250 $v"; echo "X=1|rest 1250 $v"; echo "X=1|tabs 1250 $v"; echo "X=1|rest 5000 $v"; done
echo "X=1|tabs 5000 0"; echo "REV=1 SIDE=1|sep 1250 0"; echo "TABS=1 LEADS=1 ORDER=1 LPA=0.05|sep 1250 1"; echo "RASTER=1 ANG=30|rest 1250 0"; echo "RASTER=1 ANG=90 RD=0.5|rest 1250 2"; echo "RASTER=1 ANG=45|pockpage 2000 1"; echo "SIDE=2|tabs 1250 2"
} | while IFS='|' read e a; do for v in br main; do echo "$v|$e|$a"; done; done | \
 xargs -P ${2:-3} -I{} bash -c 'IFS="|" read v e a <<< "{}"; r=$(env $e '$D'/pfh_$v $a 2>&1); echo "$e | $r" >> '$D'/ident_$v.txt'
echo DONE >> $D/ident_done
