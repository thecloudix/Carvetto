#!/bin/bash
# each mutant's Debug test binary: [t0172], [t0128], then [offset] (all run); summary in logs/runmut.log
MU=/home/chris/wt-T0172/build-verify1-mut; L=/home/chris/harness/t0172v1/logs; cd /home/chris/wt-T0172/build-verify1
run() { X=$1
  for t in t0172 t0128 offset; do timeout 2400 $MU/$X/tests "[$t]" > $MU/$X/$t.log 2>&1; eval r_$t=$?; done
  f=$(grep -hE "^/home.*FAILED|^-------" -A0 $MU/$X/t0172.log $MU/$X/t0128.log $MU/$X/offset.log | true)
  names=$(grep -h -B1 -E "^-{70,}$" $MU/$X/*.log | true)
  echo "$X t0172 rc=$r_t0172 t0128 rc=$r_t0128 offset rc=$r_offset | $(grep -hE '^test cases' $MU/$X/t0172.log $MU/$X/t0128.log $MU/$X/offset.log | tr '\n' ';')" >> $L/runmut.log
}
export -f run; export MU L
: > $L/runmut.log
printf "%s\n" "$@" | xargs -P ${P:-3} -I{} bash -c "run {}"
echo done >> $L/runmut.log
