// r2 verifier: dump one cmp6 case (fam seed join side): input, branch and main chains, vchk notes.
#include "v9lib.cpp"
#include "xchk.hpp"
static void dumpC(const char* tag, const std::vector<Contour>& cs) {
    for (size_t k = 0; k < cs.size(); ++k) { std::printf("%s chain %zu closed %d spans %zu\n", tag, k, (int)cs[k].closed, cs[k].spans.size());
        for (auto& s : cs[k].spans) { if (auto* l = std::get_if<LineSpan>(&s)) std::printf("   L (%.17g, %.17g) -> (%.17g, %.17g) len %.3g\n", l->start.x, l->start.y, l->end.x, l->end.y, length(s));
            else if (auto* a = std::get_if<ArcSpan>(&s)) std::printf("   A c(%.17g, %.17g) r %.17g %.17g..%.17g %s len %.3g\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, a->direction == ArcDirection::CCW ? "CCW" : "CW", length(s)); } }
}
int main(int argc, char** argv) {
    int fam = atoi(argv[1]); std::uint64_t seed = strtoull(argv[2], nullptr, 0); int j = atoi(argv[3]), sd = atoi(argv[4]); int dumpAll = argc > 5;
    Case c; if (fam == 0) c = makeCase(seed); else if (fam == 2) c = makeCase2(seed).c; else if (fam == 3) c = makeCase3(seed).c; else if (fam == 4) c = makeCase4(seed).c; else c = makeCase5(seed).c;
    std::printf("d %.17g mitre %.17g tol %g spans %zu closed %d\n", c.d, c.mitre, c.tol, c.in.spans.size(), (int)c.in.closed);
    if (dumpAll) { std::vector<Contour> in{c.in}; dumpC("IN", in); }
    auto ob = offsetOpenContour(c.in, c.d, (OffsetSide)sd, (JoinType)j, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
    auto mp = mainOffset(toP(c.in), c.d, sd, j, c.tol, c.mitre); std::vector<Contour> om; for (auto& p : mp) om.push_back(fromP(p));
    Metrics mb, mm; check(c.in, ob, c.d, (OffsetSide)sd, (JoinType)j, c.tol, c.mitre, mb, true); g_tiePts.clear(); check(c.in, om, c.d, (OffsetSide)sd, (JoinType)j, c.tol, c.mitre, mm, true);
    std::printf("branch first issue: %s\nmain first issue: %s\n", mb.firstIssue.c_str(), mm.firstIssue.c_str());
    dumpC("BR", ob); if (dumpAll) dumpC("MAIN", om);
    auto ovl = [&](const char* tag, const std::vector<Contour>& cs) {
        for (size_t a = 0; a < cs.size(); ++a) for (size_t i = 0; i < cs[a].spans.size(); ++i) for (size_t b = a; b < cs.size(); ++b) for (size_t k = (a == b ? i + 1 : 0); k < cs[b].spans.size(); ++k) {
            auto* A = std::get_if<LineSpan>(&cs[a].spans[i]); auto* B = std::get_if<LineSpan>(&cs[b].spans[k]); if (!A || !B) continue;
            double sc = std::max({1.0, std::fabs(A->start.x), std::fabs(A->start.y)}), e = 1e-9 * sc; double LA = distance(A->start, A->end); if (LA <= 0) continue;
            Vec2 u = (A->end - A->start) * (1.0 / LA), nn{-u.y, u.x};
            if (std::fabs((B->start - A->start).dot(nn)) > e || std::fabs((B->end - A->start).dot(nn)) > e) continue;
            double s0 = (B->start - A->start).dot(u), s1 = (B->end - A->start).dot(u); double ov = std::min(LA, std::max(s0, s1)) - std::max(0.0, std::min(s0, s1));
            if (ov > e) std::printf("%s OVERLAP ch%zu sp%zu / ch%zu sp%zu ov %.3g\n", tag, a, i, b, k, ov);
        }
    };
    ovl("BR", ob); ovl("MAIN", om);
    std::printf("vchk overlapIn br %d main %d\n", mb.overlapIn, mm.overlapIn);
    for (size_t drop = 0; drop < ob.size(); ++drop) { auto o2 = ob; o2.erase(o2.begin() + drop); Metrics m2; g_tiePts.clear(); check(c.in, o2, c.d, (OffsetSide)sd, (JoinType)j, c.tol, c.mitre, m2, true);
        if (m2.overlapIn != mb.overlapIn) std::printf("  without br chain %zu: overlapIn %d\n", drop, m2.overlapIn); }
    { auto o3 = om; Metrics m3; std::reverse(o3.begin(), o3.end()); g_tiePts.clear(); check(c.in, o3, c.d, (OffsetSide)sd, (JoinType)j, c.tol, c.mitre, m3, true); std::printf("  main reversed order: overlapIn %d\n", m3.overlapIn); }
}
