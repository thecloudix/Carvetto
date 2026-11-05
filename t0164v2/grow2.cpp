// t0164v2 (c): cluster-resolution attack patterns. Prints CPU time and an FNV hash of moves + diagnostics
// (byte identity branch vs main). usage: grow2 <mode> <N> [chordal] [seed]
//  pairs    N crossing circle pairs, each with a hole 0.003 inside one circle (as grow.cpp)
//  chain    N near-touching circles (gap ch/16), crossing pair at the low-index end (as grow.cpp)
//  revchain same chain but contour order reversed (crossing pair has the highest indices: every
//           absorption merges into a lower root, root moves down; the k > rj branch)
//  shufchain chain with contour indices shuffled (members change order, roots jump)
//  meet     chain with a crossing pair at both ends (two clusters grow and re-merge in the middle)
//  bridge   row of N crossing pairs; between neighbours a small circle 0.003 inside... a hole circle
//           cut by both pairs' polygons (cascade: clusters merge through shared absorbed vectors)
//  star     one crossing pair; N small hole circles 0.003 inside its circles all round (one
//           resolution absorbs many vectors at once; then they are stable)
//  rand     N circles on a jittered grid, random gaps {0,1e-4,ch/32,ch/16,ch/8,ch/4} to a neighbour,
//           random holes 0.003 inside, shuffled order
//  pairsrev pairs with the page order reversed and the hole listed first (absorbed vector < root)
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <random>
#include "gen3.hpp"
#include "carvetto/toolpath/v_carve.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
int main(int argc, char** argv) {
    const char* mode = argv[1]; int N = atoi(argv[2]); double ch = argc > 3 ? atof(argv[3]) : 0.05; unsigned seed = argc > 4 ? (unsigned)atoi(argv[4]) : 1;
    std::mt19937_64 rg(seed); auto U = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(rg); };
    std::vector<Contour> cs; double g = ch / 16;
    auto C = [](Vec2 o, double r) { return g2::circleAt(o, r, 0); };
    auto H = [](Vec2 o, double r) { return vs::rev(g2::circleAt(o, r, 0)); };
    auto chain = [&](int n, bool bothEnds) { std::vector<Contour> v; double x = 0; for (int i = 0; i < n; ++i) { v.push_back(C({x, 0}, 1)); x += (i == 0 || (bothEnds && i == n - 2)) ? 1.5 : 2 + g; } return v; };
    if (!strcmp(mode, "pairs") || !strcmp(mode, "pairsrev")) { int side = (int)std::ceil(std::sqrt((double)N));
        for (int i = 0; i < N; ++i) { Vec2 o{(i % side) * 30.0, (i / side) * 20.0}; std::vector<Contour> t{C(o, 6), C({o.x + 8, o.y}, 4), H({o.x, o.y + 4.997}, 1)};
            if (!strcmp(mode, "pairsrev")) std::reverse(t.begin(), t.end()); for (auto& c : t) cs.push_back(c); }
        }
    else if (!strcmp(mode, "chain")) cs = chain(N, false);
    else if (!strcmp(mode, "nest") || !strcmp(mode, "nestshuf")) { // crossing pair; N rings 0.003 apart inside the big circle (alternating hole/island): cascade, one ring per resolution
        cs.push_back(C({0, 0}, 10)); cs.push_back(C({14, 0}, 6)); double ng = getenv("NEST_G") ? atof(getenv("NEST_G")) : 0.003; for (int i = 1; i <= N; ++i) { double rr = 1.0 * (1.0 - 0.9 * i / (N + 1)); Vec2 o{0, 10 - ng * i - rr}; cs.push_back(i % 2 ? H(o, rr) : C(o, rr)); }
        if (!strcmp(mode, "nestshuf")) std::shuffle(cs.begin(), cs.end(), rg); }
    else if (!strcmp(mode, "pairsmix")) { // pairs page, per pair a random order of (circle, circle, hole) and random global order of pairs
        int side = (int)std::ceil(std::sqrt((double)N)); std::vector<std::vector<Contour>> ps;
        for (int i = 0; i < N; ++i) { Vec2 o{(i % side) * 30.0, (i / side) * 20.0}; std::vector<Contour> t{C(o, 6), C({o.x + 8, o.y}, 4), H({o.x, o.y + 4.997}, 1)}; if (U(0, 1) < 0.5) t.push_back(H({o.x - 4.997, o.y}, 0.7)); std::shuffle(t.begin(), t.end(), rg); ps.push_back(t); }
        std::shuffle(ps.begin(), ps.end(), rg); for (auto& t : ps) for (auto& c : t) cs.push_back(c); }
    else if (!strcmp(mode, "revchain")) { cs = chain(N, false); std::reverse(cs.begin(), cs.end()); }
    else if (!strcmp(mode, "shufchain")) { cs = chain(N, false); std::shuffle(cs.begin(), cs.end(), rg); }
    else if (!strcmp(mode, "meet")) cs = chain(N, true);
    else if (!strcmp(mode, "bridge")) { // pairs along x at pitch 20; bridge: a long thin hole-ish vector near both? use a circle between the
        // pairs, 0.003 outside... use one hole inside each big circle near the right wall, and big circle of next pair overlapping that region
        for (int i = 0; i < N; ++i) { double x = i * 13.0; cs.push_back(C({x, 0}, 6)); cs.push_back(C({x + 5, 5}, 3));
            cs.push_back(H({x + 4.997, 0}, 1)); }  // hole 0.003 inside circle i at its right wall; circle i+1 (centre x+13, r 6) spans x+7..: no
        for (int i = 0; i + 1 < N; ++i) { double x = i * 13.0; cs.push_back(C({x + 6.5, -5.5}, 1.5)); }  // touches circle i and i+1 polygons? near
        std::shuffle(cs.begin(), cs.end(), rg); }
    else if (!strcmp(mode, "star")) { cs.push_back(C({0, 0}, 40)); cs.push_back(C({45, 0}, 10));
        for (int i = 0; i < N; ++i) { double a = 6.283185307179586 * i / N; double rr = std::min(0.5, 40 * 3.0 / N); Vec2 o{(40 - 0.003 - rr) * std::cos(a), (40 - 0.003 - rr) * std::sin(a)}; cs.push_back(H(o, rr)); }
        std::shuffle(cs.begin() + 2, cs.end(), rg); }
    else if (!strcmp(mode, "rand")) { int side = (int)std::ceil(std::sqrt((double)N)); double gaps[] = {0.0, 1e-4, ch / 32, ch / 16, ch / 8, ch / 4};
        for (int i = 0; i < N; ++i) { Vec2 o{(i % side) * 9.0 + U(-0.5, 0.5), (i / side) * 9.0 + U(-0.5, 0.5)}; double r = U(1.5, 3.0); cs.push_back(C(o, r));
            int m = (int)U(0, 6); double gp = gaps[m]; double a = U(0, 6.28); double r2 = U(0.8, 2.0);
            double d = r + r2 + gp - (m == 0 ? 0.3 : 0.0); cs.push_back(C({o.x + d * std::cos(a), o.y + d * std::sin(a)}, r2));
            if (U(0, 1) < 0.5) { double rh = U(0.2, 0.5); double ah = U(0, 6.28); double dh = r - rh - 0.003; cs.push_back(H({o.x + dh * std::cos(ah), o.y + dh * std::sin(ah)}, rh)); } }
        std::shuffle(cs.begin(), cs.end(), rg); }
    else if (!strcmp(mode, "dense")) { // N crossing pairs; each big circle has 0..6 holes 0.003 inside its wall, each hole maybe an island
        // 0.003 inside it (cascade), pairs sometimes overlapping a neighbour (initial merges); global shuffle (absorptions up and down)
        int side = (int)std::ceil(std::sqrt((double)N));
        for (int i = 0; i < N; ++i) { Vec2 o{(i % side) * (U(0, 1) < 0.2 ? 15.0 : 22.0), (i / side) * 22.0}; double R = U(4, 7); cs.push_back(C(o, R)); double a0 = U(0, 6.28); double r2 = U(2, 4);
            cs.push_back(C({o.x + (R + r2 - 1) * std::cos(a0), o.y + (R + r2 - 1) * std::sin(a0)}, r2));
            int m = (int)U(0, 7); for (int h = 0; h < m; ++h) { double a = a0 + 0.6 + h * 0.75 + U(-0.1, 0.1); double rh = U(0.3, 1.0); double gh = U(0.001, 0.004);
                Vec2 oh{o.x + (R - gh - rh) * std::cos(a), o.y + (R - gh - rh) * std::sin(a)}; cs.push_back(H(oh, rh));
                if (U(0, 1) < 0.5) { double ri = rh * U(0.4, 0.8); double gi = U(0.001, 0.004); cs.push_back(C({oh.x + (rh - gi - ri) * std::cos(a), oh.y + (rh - gi - ri) * std::sin(a)}, ri)); } } }
        std::shuffle(cs.begin(), cs.end(), rg); }
    else if (!strcmp(mode, "gear") || !strcmp(mode, "gearh")) { // one part: N-tooth gear polyline (R 50, teeth 1 mm deep), gearh adds a central hole r 5
        std::vector<Vec2> pts; for (int i = 0; i < 2 * N; ++i) { double a = 6.283185307179586 * i / (2 * N); double r = i % 2 ? 49.0 : 50.0; pts.push_back({r * std::cos(a), r * std::sin(a)}); }
        cs.push_back(Contour::fromPolyline(geometry::Polyline{pts, true})); if (!strcmp(mode, "gearh")) cs.push_back(H({0.3, 0.2}, 5)); }
    else if (!strcmp(mode, "dpoly")) { // one part: circle R 50 as an N-vertex polyline, N small holes on a ring (many branches with large discs)
        std::vector<Vec2> pts; for (int i = 0; i < N * 20; ++i) { double a = 6.283185307179586 * i / (N * 20); pts.push_back({50 * std::cos(a), 50 * std::sin(a)}); }
        cs.push_back(Contour::fromPolyline(geometry::Polyline{pts, true})); for (int i = 0; i < N; ++i) { double a = 6.283185307179586 * i / N; cs.push_back(H({25 * std::cos(a), 25 * std::sin(a)}, 0.3)); } }
    else { std::fprintf(stderr, "mode?\n"); return 2; }
    tooling::Tool t; t.id = t.name = "V"; t.type = tooling::ToolType::VBit; t.includedAngle = 3.14159265358979 / 2; t.diameter = 12;
    auto& c = t.defaultCuttingData; c.feedRate = 1000; c.plungeRate = 300; c.passDepth = 1; c.stepover = 1; c.toolNumber = 1; c.spindleSpeed = 10000;
    VCarvingParams p; p.tolerance.chordal = ch; p.tolerance.distance = std::min(p.tolerance.distance, ch);
    double t0 = double(std::clock()) / CLOCKS_PER_SEC; auto r = vCarvingToolpath(cs, t, std::nullopt, {}, p); double t1 = double(std::clock()) / CLOCKS_PER_SEC;
    unsigned long long h = 1469598103934665603ull; auto mix = [&](const void* d, size_t n) { auto* b = (const unsigned char*)d; for (size_t q = 0; q < n; ++q) { h ^= b[q]; h *= 1099511628211ull; } };
    for (auto& m : r.toolpath.moves) { int k = (int)m.kind; mix(&k, sizeof k); mix(&m.to.x, 8); mix(&m.to.y, 8); mix(&m.to.z, 8); mix(&m.centre.x, 8); mix(&m.centre.y, 8); }
    for (auto& d : r.diagnostics) { int k = (int)d.issue; mix(&k, sizeof k); mix(d.message.data(), d.message.size()); }
    std::printf("%s %d seed=%u contours=%zu ch=%g cpu=%.2fs ok=%d moves=%zu regions=%zu hash=%016llx", mode, N, seed, cs.size(), ch, t1 - t0, (int)r.ok(), r.toolpath.moves.size(), r.regions, h);
    for (auto& d : r.diagnostics) std::printf(" [%d %s]", (int)d.issue, d.message.substr(0, 50).c_str()); std::printf("\n");
}
