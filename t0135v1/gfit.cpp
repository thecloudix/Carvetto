// gfit seed N : random fitTabs inputs, prints result line per case (diff branch vs base)
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/path_walk.hpp"
#include "carvetto/toolpath/tab_placement.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
int main(int argc, char** argv) {
    std::mt19937_64 rng(std::strtoull(argv[1], 0, 0)); int N = std::atoi(argv[2]);
    auto U = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); };
    auto I = [&](int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); };
    Contour line; line.append(geometry::LineSpan{{0, 0}, {137.3, 0}});
    Contour sq = Contour::rectangle(geometry::Box2{{-3.1, 7.7}, {40.13, 29.9}});
    ArcLengthPath lp(line, geometry::Tolerance::defaults()), cp(sq, geometry::Tolerance::defaults());
    long moved = 0;
    for (int k = 0; k < N; ++k) {
        const ArcLengthPath& p = I(0, 1) ? lp : cp;
        double L = p.length(); double fp = U(0.1, 12);
        double reach = fp / 2 + kMinTabGap;
        std::vector<double> kc, cs; int nk = I(0, 3), nc = I(1, 6);
        for (int i = 0; i < nk; ++i) kc.push_back(U(0, L));
        for (int i = 0; i < nc; ++i) {
            if (!kc.empty() && I(0, 1)) { double f = kc[(size_t)I(0, nk - 1)]; int t = I(0, 2); cs.push_back(t == 0 ? f + reach : t == 1 ? f - reach : f + U(-reach, reach)); }
            else cs.push_back(U(0, L));
        }
        double gap = I(0, 1) ? kMinTabGap : U(0, 3);
        TabPlacement r = fitTabs(p, cs, fp, kc, gap);
        std::printf("%d k%zu d%zu", k, r.centres.size(), r.dropped);
        for (double c : r.centres) std::printf(" %a", c);
        // kept footprints must clear every keep point by kMinTabGap (minus 1e-9)
        int bad = 0; for (double c : r.centres) for (double f : kc) if (std::fabs(c - f) < reach - 1e-9 - 1e-12) bad++;
        std::printf(" bad%d\n", bad);
    }
}
