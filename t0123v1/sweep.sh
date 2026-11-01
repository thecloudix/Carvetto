#!/bin/bash
# sweep.sh: run profile hashing jobs, 4 at a time
cd $(dirname $(readlink -f $0))
jobs=()
for s in 1001 1002 1003 1004 1005 1006; do jobs+=("$s 150 0"); done
for s in 2001 2002 2003 2004 2005 2006; do jobs+=("$s 120 1"); done
for s in 3001 3002 3003; do jobs+=("$s 40 2"); done
printf '%s\n' "${jobs[@]}" | while read s n m; do for v in new old; do echo "$v $s $n $m"; done; done | \
  xargs -P 4 -L 1 bash -c 'timeout 3000 ./pm_$0 $1 $2 $3 > out/p_$0_$1_$3.txt 2>&1; echo "done $0 $1 rc=$?"'
