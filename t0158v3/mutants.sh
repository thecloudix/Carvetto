#!/bin/bash
# mutants.sh: test-quality check. Builds carvetto_tests (Release, /home/chris/wt-T0158/build-verify3-rel/mut from a 5649b2b export)
# with profile.cpp mutated, runs the T-0158 tests ("[start]") and the T-0135 m1_402002_27 / m2_800016_13 tests.
#  M0 tieFreeStart -> plain projection (full revert)   M1 merged region compared at d, no slack (round-2 behaviour)
#  M2 pick = first junction of the run (round-2)       M3 kTipTurnTie = 0      M4 slack 0 only
R=/home/chris/wt-T0158/build-verify3-rel; S=$R/mutsrc; O=$S/profile.orig.cpp; T=$S/core/src/toolpath/profile.cpp
mut() { python3 - "$O" "$T" "$1" <<'PY'
import sys; s = open(sys.argv[1]).read(); m = sys.argv[3]
def rep(a, b):
    global s
    assert a in s, (m, a); s = s.replace(a, b)
if m == 'M0': rep('    const Contour& c = walk.contour();\n    const std::size_t n = c.spans.size();\n    if (n == 0) {\n        return 0.0;\n    }\n    std::vector<double> dist(n);',
                  '    const Contour& c = walk.contour();\n    const std::size_t n = c.spans.size();\n    if (n == 0 || eps >= 0.0) {\n        return walk.project(target);\n    }\n    std::vector<double> dist(n);')
if m == 'M1': rep('cut.offsetBy = std::fabs(dd);', 'cut.offsetBy = d;'); rep('cut.offsetSlack = offTol_.chordal;', 'cut.offsetSlack = 0.0;')
if m == 'M2': rep('while (pick + 1 < turns.size() && turns[pick] < sharpest - kTipTurnTie)', 'while (sharpest < 0.0 && pick + 1 < turns.size() && turns[pick] < sharpest - kTipTurnTie)')
if m == 'M3': rep('constexpr double kTipTurnTie = 1e-3;', 'constexpr double kTipTurnTie = 0.0;')
if m == 'M4': rep('cut.offsetSlack = offTol_.chordal;', 'cut.offsetSlack = 0.0;')
open(sys.argv[2], 'w').write(s)
PY
}
for m in ${MUTS:-M0 M1 M2 M3 M4}; do
  mut $m || { echo "$m: patch failed"; continue; }
  timeout 2400 cmake --build $R/mut --target carvetto_tests > $R/mut_$m.build.log 2>&1 || { echo "$m: build failed"; continue; }
  timeout 2400 $R/mut/core/tests/carvetto_tests "[start]" > $R/mut_$m.start.log 2>&1; a=$?
  timeout 2400 $R/mut/core/tests/carvetto_tests "Separate last pass\\, negative allowance: tabs that hold are matched on the roughing cut,T-0135 round 2: exact loose pieces; add-back and fitting never cost a holding tab" > $R/mut_$m.t0135.log 2>&1; b=$?
  echo "$m: [start] rc=$a ($(grep -E '^test cases:' $R/mut_$m.start.log)) | T-0135 rc=$b ($(grep -E '^test cases:' $R/mut_$m.t0135.log)) | failing: $(grep -B1 -h '^-------' $R/mut_$m.start.log | grep -v '^---' | grep -v '^--$' | sort -u | head -8 | tr '\n' ';')"
done
cp $O $T
