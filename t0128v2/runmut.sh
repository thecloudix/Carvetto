#!/bin/bash
# run each mutant's Debug test binary: [t0128] then (if it passes) [offset]; summary in runmut.log
M=/home/chris/wt-T0128/build-verify2-mut; cd /home/chris/wt-T0128/build-verify2
run() { X=$1
  timeout 1500 $M/$X/tests "[t0128]" > $M/$X/t0128.log 2>&1; r1=$?
  if [ $r1 -eq 0 ]; then timeout 2400 $M/$X/tests "[offset]" > $M/$X/offset.log 2>&1; r2=$?; else r2=skip; fi
  echo "$X t0128 rc=$r1 offset rc=$r2 | $(grep -E '^test cases' $M/$X/t0128.log | head -1) | $(grep -E '^test cases' $M/$X/offset.log 2>/dev/null | head -1)" >> $M/runmut.log
}
export -f run; export M
: > $M/runmut.log
printf "%s\n" "$@" | xargs -P ${P:-2} -I{} bash -c "run {}"
echo done >> $M/runmut.log
