#!/bin/bash
# classify.sh: every UNREP/FALSEREP scene of the new wsf7 logs -> rerun with leads off (P field 32 = 0);
# "lead" when the flag disappears without leads, otherwise print the no-lead verdict.
cd $(dirname $(readlink -f $0))
grep -h "UNREP\|FALSEREP" sw/wsf7_30*.log | while read -r line; do
  seed=$(echo "$line" | awk '{print $2}'); sc=$(echo "$line" | awk '{print $3}'); mode=$(ls sw/wsf7_${seed}_*.log | head -1 | sed -E 's/.*_([0-9]).log/\1/')
  f=d/wsf7/m${mode}_${seed}_${sc}.scn; n=nl/m${mode}_${seed}_${sc}.scn
  python3 - $f $n <<'PY'
import sys
out=[]
for l in open(sys.argv[1]):
    t=l.split()
    if t and t[0]=='P': t[32]='0'; l=' '.join(t)+'\n'
    out.append(l)
open(sys.argv[2],'w').write(''.join(out))
PY
  flag=$(echo "$line" | grep -oE "UNREP|FALSEREP" | head -1)
  r=$(SCENE=$n ./bin/wsf7 0 1 99 2>&1 | head -1 | grep -oE "\[(FREE|REP)[^]]*\]|\[UNREP\]|\[FALSEREP\]" | tr '\n' ' ')
  if echo "$r" | grep -q "$flag"; then echo "REAL m${mode}_${seed}_${sc} $flag | no-lead: $r"; else echo "lead m${mode}_${seed}_${sc} $flag"; fi
done
