// T-0172 verifier (a): putBackStrays' "cut into consecutive non-crossing parts" fallback, driven directly (white-box) with
// strays that cross themselves and no chains: parts must be consecutive, non-crossing (old pairwise), no end span <= tol,
// exact junctions, and together cover the stray except dropped sub-tol end spans.
#include "offset.cpp"
#include <cstdio>
using namespace carvetto::geometry;
namespace cg = carvetto::geometry;
static bool pairMeets(const Span& a, const Span& b) {
    for (const Vec2 p : cg::carrierIntersections(a, b))
        if (cg::inUnit(cg::carrierParam(a, p)) && cg::inUnit(cg::carrierParam(b, p))) return true;
    return false;
}
static bool crosses(const std::vector<Span>& s) { for (size_t i = 0; i < s.size(); ++i) for (size_t j = i + 2; j < s.size(); ++j) if (pairMeets(s[i], s[j])) return true; return false; }
static void run(const char* name, std::vector<Span> st) {
    Tolerance tol{1e-3, 1e-9, 0.01};
    std::vector<Contour> result; std::vector<double> keys;
    std::vector<cg::Stray> strays{cg::Stray{0.0, st}};
    cg::putBackStrays(result, keys, strays, tol);
    double lin = 0, lout = 0; for (auto& s : st) lin += length(s);
    int bad = 0; std::printf("%s: stray %zu spans len %.4f -> %zu chains:", name, st.size(), lin, result.size());
    for (size_t c = 0; c < result.size(); ++c) { auto& sp = result[c].spans; double l = 0; for (auto& s : sp) l += length(s); lout += l;
        std::printf(" [%zu spans %.4f%s%s]", sp.size(), l, crosses(sp) ? " CROSSES" : "", (length(sp.front()) <= tol.distance || length(sp.back()) <= tol.distance) ? " TINYEND" : "");
        bad += crosses(sp) || length(sp.front()) <= tol.distance || length(sp.back()) <= tol.distance;
        for (size_t k = 1; k < sp.size(); ++k) if (!(endPoint(sp[k - 1]) == startPoint(sp[k]))) ++bad;
        if (c) { double g = distance(endPoint(result[c - 1].spans.back()), startPoint(sp.front())); if (g > 2.5e-3) { std::printf(" (gap %.3g)", g); } } }
    std::printf(" | total %.4f lost %.4g bad %d\n", lout, lin - lout, bad);
}
int main() {
    // figure 8 of lines
    run("figure8", {LineSpan{{0, 0}, {2, 2}}, LineSpan{{2, 2}, {2, 0}}, LineSpan{{2, 0}, {0, 2}}, LineSpan{{0, 2}, {0, 0.5}}});
    // a long strand that crosses itself many times (zig-zag over a line), with sub-tol spans at the crossings
    std::vector<Span> z; Vec2 p{0, 0};
    for (int i = 0; i < 40; ++i) { Vec2 q{p.x + 1.0, (i % 2) ? -1.0 : 1.0}; z.emplace_back(LineSpan{p, q}); p = q; Vec2 r{p.x + 0.0005, p.y}; z.emplace_back(LineSpan{p, r}); p = r; }
    z.emplace_back(LineSpan{p, {0, 0.3}});  // back across all of it
    run("zigzag+chord", z);
    // spiral then a chord with a tiny span right at the cut
    std::vector<Span> s; Vec2 pr{1, 0};
    for (int i = 1; i <= 600; ++i) { double a = 0.05 * i; Vec2 q = Vec2::fromAngle(a, 1 + 0.02 * a); s.emplace_back(LineSpan{pr, q}); pr = q; }
    s.emplace_back(LineSpan{pr, pr + Vec2{0.0004, 0}}); pr = pr + Vec2{0.0004, 0};
    s.emplace_back(LineSpan{pr, {-0.3, 0.2}}); s.emplace_back(LineSpan{{-0.3, 0.2}, {3, 3.5}});
    run("spiral+chords", s);
    // loop of arcs crossing: two arcs forming a lens crossing, then back
    run("arcs", {ArcSpan{{0, 0}, 1, 0, 3, ArcDirection::CCW}, LineSpan{Vec2::fromAngle(3, 1), {0.5, -2}}, LineSpan{{0.5, -2}, {0.5, 2}}});
}
