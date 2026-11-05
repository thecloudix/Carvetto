#!/bin/bash
# mutants.sh (round 4): test-quality check. carvetto_tests (Release, /home/chris/wt-T0158/build-verify4-rel/mut from a 999a90e export)
# with profile.cpp mutated; runs the T-0158 tests ("[start]") and the two T-0135 tests (m1_402002_27 / m2_800016_13).
#  M0 tieFreeStart -> plain projection (full revert)      M1 no walk back to the first equidistant junction (chamfer first corner)
#  M2 no round-join branch (first point on the circle)    M3 no back-up over the join's short last chord
#  M4 equidistance tolerance 4x -> 0 (exact equality)      M5 merged-region slack 0
#  M6 pick = first equidistant junction (ignore "most even feet")   M7 rule 1 only (no sharp-corner rule)
R=/home/chris/wt-T0158/build-verify4-rel; S=$R/mutsrc; O=$S/profile.orig.cpp; T=$S/core/src/toolpath/profile.cpp
mut() { python3 - "$O" "$T" "$1" <<'PY'
import sys; s = open(sys.argv[1]).read(); m = sys.argv[3]
def rep(a, b):
    global s
    assert a in s, (m, a); s = s.replace(a, b)
if m == 'M0': rep('    if (n == 0) {\n        return 0.0;\n    }\n    std::vector<double> dist(n);', '    if (n == 0 || eps >= 0.0) {\n        return walk.project(target);\n    }\n    std::vector<double> dist(n);')
if m == 'M1': rep('while (equi[pick] != 0 && pick > 0 && equi[pick - 1] != 0) {', 'while (false && equi[pick] != 0 && pick > 0 && equi[pick - 1] != 0) {')
if m == 'M2': rep('            if (roundJoins) {\n                // A region', '            if (false && roundJoins) {\n                // A region')
if m == 'M3': rep('for (std::size_t back = 0; back + 1 < n && onJoin((i + n - 1) % n); ++back) {', 'for (std::size_t back = 0; false && back + 1 < n && onJoin((i + n - 1) % n); ++back) {')
if m == 'M4': rep('equi[k] = std::fabs(ga - gb) <= 4.0 * (na + nb) ? 1 : 0;', 'equi[k] = std::fabs(ga - gb) <= 0.0 * (na + nb) ? 1 : 0;')
if m == 'M5': rep('cut.offsetSlack = offTol_.chordal;', 'cut.offsetSlack = 0.0;')
if m == 'M6': rep('(pick == junctions || lopsided[k] < lopsided[pick])', '(pick == junctions)')
if m == 'M7': rep('    if (walk.closed() && n >= 2 && nearest <= cutDistance + eps', '    if (false && walk.closed() && n >= 2 && nearest <= cutDistance + eps')
open(sys.argv[2], 'w').write(s)
PY
}
for m in ${MUTS:-M0 M1 M2 M3 M4 M5 M6 M7}; do
  mut $m || { echo "$m: patch failed"; continue; }
  timeout 2400 cmake --build $R/mut --target carvetto_tests > $R/mut_$m.build.log 2>&1 || { echo "$m: build failed"; continue; }
  timeout 2400 $R/mut/core/tests/carvetto_tests "[start]" > $R/mut_$m.start.log 2>&1; a=$?
  timeout 2400 $R/mut/core/tests/carvetto_tests "Separate last pass\\, negative allowance: tabs that hold are matched on the roughing cut,T-0135 round 2: exact loose pieces; add-back and fitting never cost a holding tab" > $R/mut_$m.t0135.log 2>&1; b=$?
  echo "$m: [start] rc=$a ($(grep -E '^test cases:' $R/mut_$m.start.log)) | T-0135 rc=$b ($(grep -E '^test cases:' $R/mut_$m.t0135.log)) | failing: $(grep -B1 -h '^-------' $R/mut_$m.start.log | grep -v '^---' | grep -v '^--$' | sort -u | head -8 | tr '\n' ';')"
done
cp $O $T
