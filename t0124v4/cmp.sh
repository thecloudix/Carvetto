#!/bin/bash
cd /home/claude/vscratch/t0124v4/cmp
for c in "91001 0 -1" "91002 0 1" "91003 8 -1" "91004 5 -1" "91005 3 -1" "91006 6 -1" "91007 2 -1" "91008 8 1"; do
  set -- $c
  for v in new old; do b=../tools/nontab; [ $v = old ] && b=../tools/nontab_old
    ONP=4 TABS=${TABS:-0} $b $1 120 $2 $3 > ${v}_t${TABS:-0}_$1.txt 2>&1; done
done
echo done > cmp_t${TABS:-0}.done
