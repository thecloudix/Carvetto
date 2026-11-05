#!/bin/sh
# t0164v2 (c): byte identity branch vs main over cluster attack patterns x seeds x sizes x chordal; instr stats alongside
B=/home/chris/scratch-t0164v2/bin; O=/home/chris/scratch-t0164v2/ident.txt
for m in pairs pairsrev pairsmix star rand nest nestshuf bridge chain revchain meet; do for N in 20 60 150; do for c in 0.05 0.5; do for s in 1 2 3 4 5; do
  case $m in pairs|pairsrev|chain|revchain|meet|nest) [ $s -gt 1 ] && continue;; esac
  echo "$m|$N|$c|$s"; done; done; done; done |
xargs -P 4 -I{} sh -c 'IFS="|"; set -- $(echo "{}"); a=$(timeout 900 '$B'/grow2_br $1 $2 $3 $4 | grep -o "hash=[0-9a-f]*"); b=$(timeout 900 '$B'/grow2_main $1 $2 $3 $4 | grep -o "hash=[0-9a-f]*"); i=$(timeout 900 '$B'/grow2_instr $1 $2 $3 $4 2>&1 >/dev/null | grep INSTR);
  [ -n "$a" ] && [ "$a" = "$b" ] && r=SAME || r=DIFF; echo "$r $1 $2 c=$3 s=$4 br:$a main:$b $i"' > $O
echo done >> $O
