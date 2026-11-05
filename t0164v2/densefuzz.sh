#!/bin/sh
# t0164v2 (c): byte identity branch vs main on dense absorption cascades (union-find parent overwrites happen: see instr ovw)
B=/home/chris/scratch-t0164v2/bin; O=/home/chris/scratch-t0164v2/densefuzz.txt
for s in $(seq 1 600); do for N in 10 30; do for c in 0.05 0.5; do echo "$N|$c|$s"; done; done; done |
xargs -P 4 -I{} sh -c 'IFS="|"; set -- $(echo "{}"); a=$(timeout 600 '$B'/grow2_br dense $1 $2 $3 | grep -o "hash=[0-9a-f]*"); b=$(timeout 600 '$B'/grow2_main dense $1 $2 $3 | grep -o "hash=[0-9a-f]*");
  [ -n "$a" ] && [ "$a" = "$b" ] && r=SAME || r=DIFF; echo "$r dense N=$1 c=$2 s=$3 br:$a main:$b"' > $O
echo done >> $O
