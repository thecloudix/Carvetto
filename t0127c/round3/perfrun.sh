#!/bin/bash
cd $(dirname $(readlink -f $0))
while read -r line; do
  set -- $line; e1=$1; e2=$2; shift 2; args="$*"
  for v in perf2 perf2_old; do
    t=$( { env $e1 $e2 /usr/bin/time -f "%U %e" ./bin/$v $args >/dev/null; } 2>&1 | tail -1)
    echo "$e1 $e2 perf2 $args | $v user/wall $t"
  done
done <<'L'
SIDE=0 ALLOW=-1.0 500 1 2
SIDE=0 ALLOW=-1.0 500 1 2
SIDE=2 ALLOW=0 500 0 0
SIDE=2 ALLOW=0 500 0 1
SIDE=2 ALLOW=0.3 500 0 0
SIDE=2 ALLOW=0 500 1 0
SIDE=0 ALLOW=-0.5 500 0 0
SIDE=0 ALLOW=-0.5 500 0 1
SIDE=0 ALLOW=0 500 1 2
SIDE=1 ALLOW=0 500 0 0
L
