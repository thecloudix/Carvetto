#!/bin/bash
# scn.sh BINDIR P: stored-scene hash runs, main and branch, one process per scene (a crash loses one scene).
# All harness .scn originals (ORIGONLY=1) plus 37 side/allowance variants on every 18th scene.
# HSB = binary prefix (default hs -> BINDIR/hs_{br,main}). Output: BINDIR/scn/{all,var}/{br,main}_<scene>.txt; compare with cmp.py BINDIR/scn/all (and /var)
D=$1; P=${2:-4}; mkdir -p $D/scn/all $D/scn/var; export HSB=${HSB:-hs} D
find /home/chris/harness -name '*.scn' -not -path "*/t0163v1/*" | sort > $D/scn/all.lst
awk 'NR % 18 == 0' $D/scn/all.lst > $D/scn/var.lst
{ for f in $(cat $D/scn/all.lst); do echo "all $f"; done; for f in $(cat $D/scn/var.lst); do echo "var $f"; done; } | \
 while read k f; do for v in br main; do echo "$v $k $f"; done; done | \
 xargs -P $P -L 1 bash -c 'if [ $1 = all ]; then E=ORIGONLY=1; else E=X=1; fi; n=$(echo $2 | sed "s|/home/chris/harness/||; s|/|_|g"); env $E timeout 900 $D/${HSB}_$0 scn $2 > $D/scn/$1/$0_$n.txt 2>&1; rc=$?; [ $rc = 0 ] || echo "rc=$rc $0 $1 $2"' > $D/scn/run.log 2>&1
echo ALLDONE >> $D/scn/run.log
