#!/usr/bin/env python3
# mutants.py ORIG OUT NAME: writes profile.cpp mutant NAME (T-0140 test-quality mutants) from ORIG to OUT.
import sys
o, out, name = sys.argv[1:4]
s = open(o).read()
M = {
 'M1_gate': [("    if (!p_.tabs.enabled) {\n        return;\n    }\n    const double rt = tabLayerRadius();", "    if (!sidedTabs_ || !p_.tabs.enabled) {\n        return;\n    }\n    const double rt = tabLayerRadius();")],
 'M2_clipperonly': [("    if (ps.empty()) return std::nullopt;\n    std::vector<Vec2> pts;", "    if (ps.empty() || true) return std::nullopt;\n    std::vector<Vec2> pts;")],
 'M3_arcinner': [("const double r = inner ? q.rho - w : q.rho + w;", "const double r = inner ? q.rho - 0.5 * w : q.rho + w;")],
 'M4_covered': [("const bool covered = !q.arc && !next.arc && std::fabs(turn) < 0.5 * std::numbers::pi &&", "const bool covered = true || (!q.arc && !next.arc && std::fabs(turn) < 0.5 * std::numbers::pi &&"), ("std::min(q.len, next.len) >= w * std::sin(std::fabs(turn));", "std::min(q.len, next.len) >= w * std::sin(std::fabs(turn)));")],
 'M5_nothick': [("            if (thick.empty()) continue;\n            // Attributed", "            if (thick.empty() && false) continue;\n            // Attributed")],
 'M6_attrib': [("                if (k >= primaryCuts || !o->walk.closed()) continue;", "                if (true) { (void)primaryCuts; continue; }"), ("                    if (best <= rad || nearby.size() == cuts.size()) break;", "                    break;")],
 'M7_islands': [("                    pieceArea -= all.area[k];\n                    region.push_back", "                    pieceArea -= 0.0;\n                    region.push_back")],
 'M8_lowered': [("                for (Contour& path : stretches(*o, true)) {\n                    if (std::optional<Contour> band", "                for (Contour& path : stretches(*o, false)) {\n                    if (std::optional<Contour> band")],
 'M9_nowedge': [("                arcPoints(q.b, w, angleOf(tin.perp() * side), turn);", "                pts.push_back(q.b + tout.perp() * (side * w));")],
 'M10_nocap': [("    arcPoints(last.b, w, angleOf(-last.tb.perp()), std::numbers::pi);", "    pts.push_back(last.b);")],
 'M11_reach': [("        const double reach = rt + 2e-3;", "        const double reach = rt - 0.02;")],
 'M12_area': [("            if (pieceArea < 0.04) continue;\n            const std::vector<Contour> thick", "            if (pieceArea < 0.4) continue;\n            const std::vector<Contour> thick")],
}
for a, b in M[name]:
    assert s.count(a) == 1, (name, a[:60], s.count(a))
    s = s.replace(a, b)
open(out, 'w').write(s)
