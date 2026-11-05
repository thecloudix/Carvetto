#!/bin/bash
# t0160v1 test-quality mutants: each edits the scratch copy of the branch (git archive of 3fcccae), rebuilds the corner
# + post tests (corner_only: v_carve_corner_test.cpp + v_carve_post_test.cpp) and reports whether they fail.
M=/home/chris/scratch-t0160v1/mut; O=/home/chris/scratch-t0160v1; C=$M/core/src/toolpath/v_carve_corners.cpp; V=$M/core/src/toolpath/v_carve.cpp
run() { name=$1; shift; cp $O/corners.orig $C; cp $O/vcarve.orig $V; python3 - "$@" <<'PY'
import sys
f, old, new = sys.argv[1], sys.argv[2], sys.argv[3]
s = open(f).read(); assert old in s, (f, old); open(f, 'w').write(s.replace(old, new, 1))
PY
  [ $? -ne 0 ] && { echo "$name: PATCH FAILED"; return; }
  timeout 1200 ninja -C $M/b corner_only > $O/mut_$name.build 2>&1 || { echo "$name: BUILD FAILED"; return; }
  timeout 1200 $M/b/core/tests/corner_only > $O/mut_$name.log 2>&1; rc=$?
  echo "$name: rc=$rc $(grep -E 'test cases|All tests passed' $O/mut_$name.log | tail -1) failing: $(grep -B1 -E '^  REQUIRE|^  CHECK' $O/mut_$name.log | grep -oE 'v_carve_[a-z_]+test.cpp:[0-9]+' | sort -u | tr '\n' ' ')"
}
run off        $V 'const bool sharpen = params.cornerSharpen && multi && params.flatDepth > 0.0;' 'const bool sharpen = false;'
run groove_only $C 'for (long k = -sides; k <= sides; ++k) {' 'for (long k = 0; k <= 0; ++k) {'
run no_overlap $C 'const double ov = std::min(spec.chordal, 0.5 * r);' 'const double ov = 0.0;'
run neg_overlap $C 'const double ov = std::min(spec.chordal, 0.5 * r);' 'const double ov = -spec.chordal;'
run no_clip    $C 'for (const auto& [t0, t1] : walls.clip(p0, p1)) {' 'for (const auto& [t0, t1] : std::vector<std::pair<double, double>>{{0.0, 1.0}}) {'
run always_join $C 'if (!cur.empty() && !walls.segmentInside(cur.back(), p.from)) {' 'if (false) {'
run never_join $C 'if (!cur.empty() && !walls.segmentInside(cur.back(), p.from)) {' 'if (!cur.empty()) {'
run no_reach   $C 'if (!(mill > 0.0)) {' 'if (!(mill > 0.0)) { mill = radii.front(); } if (false) {'
run largest_mill $C 'for (const double r : radii) {' 'for (const double r : std::vector<double>(radii.rbegin(), radii.rend())) {'
run thr_zero   $C 'if (!(predicted > spec.chordal)) {' 'if (!(predicted > 0.0)) {'
run thr_2x     $C 'if (!(predicted > spec.chordal)) {' 'if (!(predicted > 2.0 * spec.chordal)) {'
run no_flatcheck $C 'if (!walls.inside(v + u * (spec.ringInset / sh + spec.chordal))) {' 'if (false) {'
run hole_side  $C 'const bool insideLeft = (area > 0.0) == (k == 0);' 'const bool insideLeft = (area > 0.0);'
run stepover_x2 $C 'const double s = spec.stepover;' 'const double s = 2.0 * spec.stepover;'
run no_ext     $C 'const double ext = (3.0 * spec.pocketChordal + spec.distance) / sh;' 'const double ext = 0.0;'
run big_ext    $C 'const double ext = (3.0 * spec.pocketChordal + spec.distance) / sh;' 'const double ext = (30.0 * spec.pocketChordal + spec.distance) / sh;'
run eps_big    $V 'const detail::RingRegion wallRegion(std::move(ringPoints), 1e-9 * scale);' 'const detail::RingRegion wallRegion(std::move(ringPoints), 1e-2);'
run pos_reset  $V '                pos = cornerItems[c].end;' '                (void)0;'
run first_corner_only $C 'out.push_back(SharpCorner{v, u, angle, mill, predicted});' 'if (out.empty()) out.push_back(SharpCorner{v, u, angle, mill, predicted});'
run no_degenerate_skip $C 'if (spanSize(c.spans[i]) > spec.distance) {' 'if (true) {'
cp $O/corners.orig $C; cp $O/vcarve.orig $V; ninja -C $M/b corner_only > /dev/null 2>&1; echo restored
