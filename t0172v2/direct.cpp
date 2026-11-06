// T-0172 verifier: hand-built near-tangent pairs where the replaced pairwise test (carrierIntersections + inUnit) reports a
// meeting but the boxes, grown by 4 kParamEps len + 1e-12 scale, are apart, so selfCrossingCuts misses it.
#include "offset.cpp"
#include <cstdio>
using namespace carvetto::geometry;
namespace cg = carvetto::geometry;
static bool pairMeets(const Span& a, const Span& b) {
    for (const Vec2 p : cg::carrierIntersections(a, b))
        if (cg::inUnit(cg::carrierParam(a, p)) && cg::inUnit(cg::carrierParam(b, p))) return true;
    return false;
}
static void run(const char* name, const Span& a, const Span& b) {
    std::vector<Span> s{a, LineSpan{{100, 100}, {101, 100}}, b};
    std::vector<Span> s3{a, LineSpan{endPoint(a), {0, 50}}, LineSpan{{0, 50}, startPoint(b)}, b};  // continuous strand
    std::printf("%-34s pairwise meets %d | selfCrossingCuts(3 spans) %zu cuts, continuous 4-span strand: old %d new %zu cuts\n", name,
                pairMeets(a, b), cg::selfCrossingCuts(s).size(), pairMeets(a, b) || pairMeets(s3[0], s3[2]) || pairMeets(s3[1], s3[3]),
                cg::selfCrossingCuts(s3).size());
}
int main() {
    const double kPi = 3.14159265358979323846;
    for (double gap : {0.0, 1e-11, 1e-10, 5e-10, 1e-9, 1.2e-9}) {
        char n[64];
        std::snprintf(n, sizeof n, "line tangent r10 gap %.2g", gap);
        run(n, ArcSpan{{0, 0}, 10, kPi / 2 - 1e-3, kPi / 2 + 1e-3, ArcDirection::CCW}, LineSpan{{-0.005, 10 + gap}, {0.005, 10 + gap}});
        std::snprintf(n, sizeof n, "arc-arc external r1 gap %.2g", gap);
        run(n, ArcSpan{{0, 0}, 1, kPi / 2 - 1e-3, kPi / 2 + 1e-3, ArcDirection::CCW},
            ArcSpan{{0, 2 + gap}, 1, -kPi / 2 - 1e-3, -kPi / 2 + 1e-3, ArcDirection::CCW});
        std::snprintf(n, sizeof n, "arc-arc internal gap %.2g", gap);
        run(n, ArcSpan{{0, 0}, 1, kPi / 2 - 1e-3, kPi / 2 + 1e-3, ArcDirection::CCW},
            ArcSpan{{0, 0.5 - gap}, 0.5, kPi / 2 - 1e-3, kPi / 2 + 1e-3, ArcDirection::CCW});
    }
}
