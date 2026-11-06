#!/usr/bin/env python3
# mutants2.py SRCROOT ORIGROOT NAME : writes mutant NAME into SRCROOT (from pristine ORIGROOT copies of enclosed_pieces.cpp / profile.cpp).
import sys, shutil
src, orig, name = sys.argv[1], sys.argv[2], sys.argv[3]
E = 'core/src/toolpath/enclosed_pieces.cpp'; P = 'core/src/toolpath/profile.cpp'
for f in (E, P): shutil.copy(f'{orig}/{f}', f'{src}/{f}')
M = {
 'E1_nojoin':     (E, 'constexpr std::int64_t kMinOverlap = 10;', 'constexpr std::int64_t kMinOverlap = 1000000000;'),
 'E2_touchjoin':  (E, 'constexpr std::int64_t kMinOverlap = 10;', 'constexpr std::int64_t kMinOverlap = 0;'),
 'E3_nowinding':  (E, 'winding += windingAt(rings[i], rect.centre());', 'winding += 0 * windingAt(rings[i], rect.centre());'),
 'E4_nocrop':     (E, 'constexpr double kCropMargin = 1e-3;', 'constexpr double kCropMargin = 0.0;'),
 'E5_noislands':  (E, '            parts[best].area += sa[k];', '            (void)0;'),
 'E6_noframe':    (E, '                        parts[j].frame = true;', '                        (void)0;'),
 'E7_nofallback': (E, 'if (!piece.inner && classOf[r].size() > 1) {', 'if (false) {'),
 'E8_firstpart':  (E, '            piece.inner = eroded(region(j));\n            if (piece.inner) break;', '            piece.inner = eroded(region(j));\n            break;'),
 'E9_largest':    (E, 'sa[parts[j].outer] >= sa[parts[best].outer]', 'sa[parts[j].outer] <= sa[parts[best].outer]'),
 'E10_no45rule':  (E, '    if (5 * std::max(nBelow, nAbove) > 4 * count) return std::nullopt;', ''),
 'E11_minside0':  (E, 'const double minSide = std::max(0.01, 10.0 * opt.tol.distance);', 'const double minSide = 1e-6;'),
 'E12_walkflip':  (E, '            if (a.hi < b.hi) {\n                ++i;', '            if (a.hi > b.hi) {\n                ++i;'),
 'E13_windsign':  (E, 'winding += windingAt(rings[i], rect.centre());', 'winding -= windingAt(rings[i], rect.centre());'),
 'E14_enters0':   (E, '        if (hit) return true;\n    }\n    return false;', '        (void)hit;\n    }\n    return false;'),
 'E15_nocentre':  (E, 'for (const Vec2 q : {reg.front().bounds().centre(), centroid(reg.front())}) {', 'for (const Vec2 q : {Vec2{1e300, 1e300}, centroid(reg.front()) + Vec2{1e300, 0.0}}) {'),
 'E16_sideonly1': (E, '                if (uy == vy && uy == y1) add(1, y1, ux, vx, -1, y1 == fy1);', '                (void)y1; (void)fy1;'),
 'P1_gate':       (P, '    if (!sidedTabs_) {\n        std::vector<const Cut*> cuts;', '    if (false) {\n        std::vector<const Cut*> cuts;'),
 'P2_closedopen': (P, 'const bool whole = o->walk.closed() && o->tabs.empty();', 'const bool whole = false;'),
 'P3_nothick':    (P, '        opt.clearance = 0.025;', '        opt.clearance = 0.0;'),
 'P4_attrib':     (P, 'if (k >= primaryCuts || !o->walk.closed()) continue;', 'if (k >= primaryCuts || !o->walk.closed() || true) continue;'),
 'P5_area':       (P, '        opt.minArea = 0.04;', '        opt.minArea = 0.1;'),
 'P6_reach':      (P, '        const double reach = rt + 2e-3;', '        const double reach = rt - 2e-3;'),
 'P7_covered':    (P, '                if (!covered) pts.push_back(q.b);', '                (void)covered;'),
 'P8_nowedge':    (P, '                arcPoints(q.b, w, angleOf(tin.perp() * side), turn);', '                pts.push_back(q.b);'),
 'P9_closing':    (P, '            if (closing && !closedPath) break;', '            if (closing) break;'),
}
f, a, b = M[name]
p = f'{src}/{f}'; s = open(p).read(); assert s.count(a) == 1, (name, s.count(a)); open(p, 'w').write(s.replace(a, b))
