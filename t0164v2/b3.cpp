// t0164v2 (d/B3): StepoverLimited at stepover == width, within the distance tolerance of it, beyond it.
// Square 10x10, end mill 20 (too large: carver does rest clearing). Width computed independently.
#include <cmath>
#include <cstdio>
#include "carvetto/toolpath/v_carve.hpp"
#include "../../wt-T0164/core/tests/toolpath/v_carve_test_util.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using namespace carvetto::geometry;
int main() {
    const std::vector<Contour> sq{Contour::rectangle(Box2{Vec2{0, 0}, Vec2{10, 10}})};
    const std::vector<PocketTool> big{{test::v_carve::endMill(20.0, "EM 20", 4), std::nullopt}};
    struct T { const char* n; tooling::Tool t; double limit; double w; };
    auto V = [](double a, double d) { return test::v_carve::vBit(a, d, "V"); };
    std::vector<T> ts;
    ts.push_back({"V90 l1", V(90, 12), 1.0, 2.0 * 1.0 * std::tan(kPi / 4)});
    ts.push_back({"V60 l1.3", V(60, 12), 1.3, 2.0 * 1.3 * std::tan(kPi / 6)});
    ts.push_back({"V30 l0.77", V(30, 6), 0.77, 2.0 * 0.77 * std::tan(kPi / 12)});
    { auto b = test::v_carve::ballNose(6, "B"); ts.push_back({"ball6 l0.7", b, 0.7, 2.0 * std::sqrt(9.0 - 2.3 * 2.3)}); }
    int bad = 0;
    for (double dist : {0.001, 1e-6, 0.01}) for (auto& t : ts) {
        const double w = t.w;
        struct S { const char* n; double s; int expect; };  // expect: 0 no notice, 1 notice, -1 either (inside tolerance boundary)
        std::vector<S> ss{{"w-1e-3", w - 1e-3, 0}, {"w(1-2ulp)", w * (1 - 4.4e-16), 0}, {"w", w, 0}, {"nextafter(w)", std::nextafter(w, 9.0), 0},
            {"w(1+1e-12)", w * (1 + 1e-12), 0}, {"w+0.5d", w + 0.5 * dist, 0}, {"w+0.99d", w + 0.99 * dist, 0},
            {"w+1.01d", w + 1.01 * dist, 1}, {"w+2d", w + 2 * dist, 1}, {"w+0.01", w + 0.01, dist < 0.01 ? 1 : -1}, {"w+0.1", w + 0.1, 1}, {"2.4w", 2.4 * w, 1}};
        for (auto& s : ss) {
            VCarvingParams p; p.flatDepth = t.limit; p.tolerance.chordal = 0.05; p.tolerance.distance = dist;
            tooling::Tool tool = t.t; tool.defaultCuttingData.stepover = s.s;
            auto r = vCarvingToolpath(sq, tool, std::nullopt, big, p);
            const bool has = r.has(ToolpathIssue::StepoverLimited);
            const bool ok = s.expect < 0 || (s.expect == 1) == has; bad += !ok;
            std::string msg; for (auto& d : r.diagnostics) if (d.issue == ToolpathIssue::StepoverLimited) msg = d.message;
            std::printf("%s d=%g %-12s s=%.17g w=%.17g ok=%d notice=%d %s %s\n", t.n, dist, s.n, s.s, w, (int)r.ok(), (int)has, ok ? "" : "UNEXPECTED", msg.c_str());
        }
    }
    std::printf("b3 unexpected=%d\n", bad); return bad != 0;
}
