#include "../t0121v2/common.hpp"
#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/fill_rule.hpp"
int main() {
    double nan = std::nan("");
    std::vector<std::vector<Contour>> cases;
    { Contour c; c.append(ArcSpan{{0, 0}, 12, nan, nan, ArcDirection::CCW}); c.append(ArcSpan{{2.1, 0}, 9, nan, nan, ArcDirection::CCW}); c.closed = true; cases.push_back({c}); }
    { Contour c; c.append(LineSpan{{0, 0}, {nan, 0}}); c.append(LineSpan{{nan, 0}, {0, 5}}); c.append(LineSpan{{0, 5}, {0, 0}}); c.closed = true; cases.push_back({c}); }
    { Contour c; c.append(LineSpan{{0, 0}, {1e300, 0}}); c.append(LineSpan{{1e300, 0}, {0, 1e300}}); c.append(LineSpan{{0, 1e300}, {0, 0}}); c.closed = true; cases.push_back({c}); }
    { cases.push_back({Contour::circle({0, 0}, std::numeric_limits<double>::infinity())}); }
    for (size_t i = 0; i < cases.size(); ++i) {
        int rc = forked([&] { PocketParams p; p.cutDepth = 2; try { auto r = pocketToolpath(cases[i], {tool(6, 2.4, 1)}, p); printf("case %zu: moves %zu msgs:", i, r.toolpaths[0].moves.size()); for (auto& m : r.messages) printf(" [%s]", m.c_str()); printf("\n"); } catch (std::exception& e) { printf("case %zu: EXCEPTION %s\n", i, e.what()); return 3; } return 0; }, 120);
        printf("  rc=%d\n", rc);
    }
    // same through geometry directly
    int rc = forked([&] { try { auto u = geometry::unionContours(cases[0], geometry::Tolerance::defaults(), geometry::FillRule::EvenOdd); printf("union ok %zu\n", u.size()); } catch (std::exception& e) { printf("union EXCEPTION %s\n", e.what()); } return 0; }, 60);
    printf("  union rc=%d\n", rc);
}
