#!/bin/bash
# t0160v2 test-quality mutants on a git-archive copy of 923d66c (/home/chris/scratch-t0160v2/mut, Release, target corner_only =
# v_carve_corner_test.cpp + v_carve_post_test.cpp). Each mutant: patch, rebuild, run, report rc + failing lines.
M=/home/chris/scratch-t0160v2/mut; O=/home/chris/scratch-t0160v2/mutout; mkdir -p $O
C=$M/core/src/toolpath/v_carve_corners.cpp; V=$M/core/src/toolpath/v_carve.cpp
[ -f $O/corners.orig ] || cp $C $O/corners.orig; [ -f $O/vcarve.orig ] || cp $V $O/vcarve.orig
run() { name=$1; shift; cp $O/corners.orig $C; cp $O/vcarve.orig $V; python3 - "$@" <<'PY'
import sys
f, old, new = sys.argv[1], sys.argv[2], sys.argv[3]
s = open(f).read(); assert old in s, (f, old); open(f, 'w').write(s.replace(old, new, 1))
PY
  [ $? -ne 0 ] && { echo "$name: PATCH FAILED"; return; }
  timeout 1800 ninja -j2 -C $M/b corner_only > $O/mut_$name.build 2>&1 || { echo "$name: BUILD FAILED"; return; }
  timeout 1800 $M/b/core/tests/corner_only > $O/mut_$name.log 2>&1; rc=$?
  echo "$name: rc=$rc $(grep -E 'test cases|All tests passed' $O/mut_$name.log | tail -1) failing: $(grep -oE 'v_carve_[a-z_]+test.cpp:[0-9]+' $O/mut_$name.log | sort -u | tr '\n' ' ')"
}
if [ -n "$1" ]; then ONLY="$1"; fi
want() { [ -z "$ONLY" ] || [[ ",$ONLY," == *",$1,"* ]]; }
want off && run off $V 'const bool sharpen = params.cornerSharpen && multi && params.flatDepth > 0.0;' 'const bool sharpen = false;'
want own_line && run own_line $C 'const double lo = y - half;
        const double hi = y + half;' 'const double lo = y;
        const double hi = y;'
want narrow_strip && run narrow_strip $C 'const double half = 0.5 * s;' 'const double half = 0.3 * s;'
want no_overlap && run no_overlap $C 'const double xe = x1 + spec.chordal;' 'const double xe = x1;'
want no_back && run no_back $C 'const double xs = x0 - backReach;' 'const double xs = x0 - 0.0 * backReach;'
want no_clip && run no_clip $C 'for (const auto& [t0, t1] : walls.clip(p0, p1)) {' 'for (const auto& [t0, t1] : std::vector<std::pair<double, double>>{{0.0, 1.0}}) {'
want always_join && run always_join $C 'if (!cur.empty() && !walls.segmentInside(cur.back(), p.from)) {' 'if (false) {'
want never_join && run never_join $C 'if (!cur.empty() && !walls.segmentInside(cur.back(), p.from)) {' 'if (!cur.empty()) {'
want thr_none && run thr_none $C 'bool deep = allDeep;' 'bool deep = true;'
want thr_never_alldeep && run thr_never_alldeep $C 'const bool allDeep = !(delta < rc);' 'const bool allDeep = false;'
want thr_2x && run thr_2x $C '-(rc + margin)' '-(2.0 * rc + margin)'
want no_area_check && run no_area_check $C 'if (!area || !sweptArea[*area]) {' 'if (false && (!area || !sweptArea[*area])) {'
want swept_bigger && run swept_bigger $C 'geometry::offsetContours(open, radius - margin,' 'geometry::offsetContours(open, radius + 0.05,'
want delta_half_s && run delta_half_s $C 'const double delta = std::min(rc, 0.5 * spec.stepover);' 'const double delta = 0.5 * spec.stepover;'
want delta_2x && run delta_2x $C 'const double delta = std::min(rc, 0.5 * spec.stepover);' 'const double delta = 2.0 * std::min(rc, 0.5 * spec.stepover);'
want stepover_x2 && run stepover_x2 $C 'const double s = spec.stepover;' 'const double s = 2.0 * spec.stepover;'
want eps_big && run eps_big $V 'std::min(kRingEps * scale, inner.chordal / 16.0)' 'std::max(kRingEps * scale, inner.chordal)'
want pos_update && run pos_update $V 'emitCut(gr.cornerCuts[c], params.startDepth, params.safeZ, result.toolpath.moves);' 'emitCut(gr.cornerCuts[c], params.startDepth, params.safeZ, result.toolpath.moves); pos = cornerItems[c].end;'
want ramps_included && run ramps_included $V '&& atFloor(a.z) &&' '&& (true || atFloor(a.z)) &&'
want maxpasses && run maxpasses $C 'constexpr double kMaxPasses = 200000.0;' 'constexpr double kMaxPasses = 2.0;'
want dist_filter && run dist_filter $C 'if (geometry::distance(a, b) > spec.distance) {' 'if (geometry::distance(a, b) > 50.0 * spec.distance) {'
want first_patch_only && run first_patch_only $C 'out.push_back(SharpStretch{std::move(poly), root, dir, farthest, predicted});' 'if (out.empty()) out.push_back(SharpStretch{std::move(poly), root, dir, farthest, predicted});'
want root_nearest && run root_nearest $C 'if (d > deepest) {' 'if (deepest < 0.0 || d < deepest) {'
want no_zigzag && run no_zigzag $C 'if (i % 2 == 1) {' 'if (false) {'
want margin_neg && run margin_neg $C 'constexpr double kGridMargin = 4e-6;' 'constexpr double kGridMargin = -0.02;'
want radius_first && run radius_first $V 'const double radius = 0.5 * clearanceTools[t].tool.diameter;' 'const double radius = 0.5 * clearanceTools[0].tool.diameter;'
want coarse_E && run coarse_E $C 'constexpr double kFineDivisor = 64.0;' 'constexpr double kFineDivisor = 0.5;'
cp $O/corners.orig $C; cp $O/vcarve.orig $V; ninja -j2 -C $M/b corner_only > /dev/null 2>&1; echo restored
