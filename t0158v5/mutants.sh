#!/bin/bash
# mutants.sh (round 5): test-quality check. carvetto_tests (Release, build-verify5-rel/mut from a 1156cad export) with profile.cpp
# mutated; runs the T-0158 tests ("[start]") and the two T-0135 tests (m1_402002_27 / m2_800016_13).
#  N0 tieFreeStart -> projection (full revert)          N1 pieces on both offsets count as neither (label 3 -> 0)  [coder: survives]
#  N2 no "contains a nearest span" check  [coder: survives]   N3 count >= 1 accepted (last place wins)
#  N4 label-3 pieces never on the circle (always after)  N5 travel direction inverted
#  N6 labelling band x0.01                               N7 labelling band x100
#  N8 Bezier edges not replaced by their chord           N9 no corner path (tipEdges -> nullopt; fallback only)
#  N10 arc edges never labelled                          N11 no region slack on arc edges
#  N12 label-3 run never preceded by first-edge span (from = k0+1)   N13 onCircle (corner path) x0.01
R=/home/chris/wt-T0158/build-verify5-rel; S=$R/mutsrc; O=$S/profile.orig.cpp; T=$S/core/src/toolpath/profile.cpp
echo $$ > $R/mutants.pid
mut() { python3 - "$O" "$T" "$1" <<'PY'
import sys; s = open(sys.argv[1]).read(); m = sys.argv[3]
def rep(a, b):
    global s
    assert a in s, (m, a); s = s.replace(a, b, 1)
if m == 'N0': rep('    if (n == 0) {\n        return 0.0;\n    }\n    std::vector<double> dist(n);', '    if (n == 0 || eps >= 0.0) {\n        return walk.project(target);\n    }\n    std::vector<double> dist(n);')
if m == 'N1': rep("label[i] = (onOffsetOf(c.spans[i], tip->in) ? 1 : 0) + (onOffsetOf(c.spans[i], tip->out) ? 2 : 0);", "label[i] = (onOffsetOf(c.spans[i], tip->in) ? 1 : 0) + (onOffsetOf(c.spans[i], tip->out) ? 2 : 0); if (label[i] == 3) label[i] = 0;")
if m == 'N2': rep("if (!near && !isNear(j)) {\n                        continue;", "if (false && !near && !isNear(j)) {\n                        continue;")
if m == 'N3': rep("if (count == 1) {\n                    return startAt(walk.spanStart(pick));", "if (count >= 1) {\n                    return startAt(walk.spanStart(pick));")
if m == 'N4': rep("std::size_t at = (i + 1) % n;\n                    for (std::size_t k = from;", "std::size_t at = (i + 1) % n;\n                    for (std::size_t k = from; false &&")
if m == 'N5': rep("const int m = along.dot(edgeDir) > 0.0 ? 1 : 2;", "const int m = along.dot(edgeDir) > 0.0 ? 2 : 1;")
if m == 'N6': rep("const double noise = 4.0 * coord * (2.0 + 2.0 * beyond / len);", "const double noise = 0.04 * coord * (2.0 + 2.0 * beyond / len);")
if m == 'N7': rep("const double noise = 4.0 * coord * (2.0 + 2.0 * beyond / len);", "const double noise = 400.0 * coord * (2.0 + 2.0 * beyond / len);")
if m == 'N8': rep("if (!std::holds_alternative<geometry::BezierSpan>(sp)) {\n                    return sp;", "if (true) {\n                    return sp;")
if m == 'N9': rep("if (const std::optional<TipEdges> tip = tipEdges()) {", "if (const std::optional<TipEdges> tip = (tipEdges(), std::optional<TipEdges>{})) {")
if m == 'N10': rep("if (const auto* a = std::get_if<geometry::ArcSpan>(&edge)) {\n                    const double noise", "if (const auto* a = (const geometry::ArcSpan*)nullptr) {\n                    const double noise")
if m == 'N11': rep("const double noise = 8.0 * coord + 4e-12 * a->radius + slack;", "const double noise = 8.0 * coord + 4e-12 * a->radius;")
if m == 'N12': rep("from = (label[(k0 + n - 1) % n] & met) != 0 ? k0 : (k0 + 1) % n;", "from = (k0 + 1) % n;")
if m == 'N13': rep("const double onCircle = slack > 0.0 ? 3.0 / geometry::kFixedPointScale : 4e-10 * scale;", "const double onCircle = slack > 0.0 ? 0.03 / geometry::kFixedPointScale : 4e-12 * scale;")
open(sys.argv[2], 'w').write(s)
PY
}
for m in ${MUTS:-N0 N1 N2 N3 N4 N5 N6 N7 N8 N9 N10 N11 N12 N13}; do
  mut $m || { echo "$m: patch failed"; continue; }
  timeout 2400 nice cmake --build $R/mut -j2 --target carvetto_tests > $R/mut_$m.build.log 2>&1 || { echo "$m: build failed"; continue; }
  timeout 2400 $R/mut/core/tests/carvetto_tests "[start]" > $R/mut_$m.start.log 2>&1; a=$?
  timeout 2400 $R/mut/core/tests/carvetto_tests "Separate last pass\\, negative allowance: tabs that hold are matched on the roughing cut,T-0135 round 2: exact loose pieces; add-back and fitting never cost a holding tab" > $R/mut_$m.t0135.log 2>&1; b=$?
  echo "$m: [start] rc=$a ($(grep -E '^test cases:' $R/mut_$m.start.log)) | T-0135 rc=$b ($(grep -E '^test cases:' $R/mut_$m.t0135.log)) | failing: $(grep -B1 -h '^-------' $R/mut_$m.start.log | grep -v '^---' | grep -v '^--$' | sort -u | head -8 | tr '\n' ';')"
done
cp $O $T
echo MUTDONE
