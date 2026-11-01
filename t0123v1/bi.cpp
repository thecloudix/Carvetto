// T-0123 verifier (throwaway): BoxIndex and ClearanceField vs brute force, incl. degenerate/huge inputs.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/toolpath/box_index.hpp"
#include "carvetto/toolpath/clearance.hpp"
using namespace carvetto;
using geometry::Box2; using geometry::Vec2; using geometry::Contour;
static std::mt19937_64 rng(12345);
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static long fails = 0;
static void checkSet(const char* name, const std::vector<Box2>& boxes, const std::vector<Box2>& queries) {
    toolpath::BoxIndex idx(boxes);
    long bad = 0;
    for (auto& q : queries) {
        std::vector<size_t> brute;
        for (size_t i = 0; i < boxes.size(); ++i) if (boxes[i].intersects(q)) brute.push_back(i);
        auto got = idx.query(q);
        if (got != brute) { if (bad < 3) std::printf("  %s MISMATCH q=[%g,%g]-[%g,%g] got %zu brute %zu\n", name, q.min.x, q.min.y, q.max.x, q.max.y, got.size(), brute.size()); ++bad; }
    }
    std::printf("%s: n=%zu queries=%zu mismatches=%ld\n", name, boxes.size(), queries.size(), bad);
    fails += bad;
}
static std::vector<Box2> randQueries(double lo, double hi, double maxw, int n) {
    std::vector<Box2> q;
    for (int i = 0; i < n; ++i) { Vec2 a{U(lo, hi), U(lo, hi)}; double w = i % 3 == 0 ? 0 : U(0, maxw), h = i % 3 == 0 ? 0 : U(0, maxw); q.push_back(Box2{a, a + Vec2{w, h}}); }
    q.push_back(Box2{}); q.push_back(Box2{{-1e300, -1e300}, {1e300, 1e300}});
    const double inf = std::numeric_limits<double>::infinity(), nan = std::numeric_limits<double>::quiet_NaN();
    Box2 qi; qi.min = {-inf, -inf}; qi.max = {inf, inf}; q.push_back(qi);
    Box2 qn; qn.min = {nan, 0}; qn.max = {1, 1}; q.push_back(qn);
    Box2 qh; qh.min = {lo - 1, -inf}; qh.max = {lo - 1, inf}; q.push_back(qh);
    return q;
}
int main() {
    const double inf = std::numeric_limits<double>::infinity();
    { std::vector<Box2> b; checkSet("empty-index", b, randQueries(-10, 10, 5, 50)); }
    { std::vector<Box2> b{Box2{{1, 1}, {1, 1}}}; checkSet("single-point", b, randQueries(0, 2, 1, 200)); }
    { std::vector<Box2> b(500, Box2{{0, 0}, {1, 1}}); checkSet("identical", b, randQueries(-1, 2, 1, 200)); }
    { std::vector<Box2> b; for (int i = 0; i < 3000; ++i) { Vec2 a{U(0, 100), U(0, 100)}; b.push_back(Box2{a, a}); } checkSet("points", b, randQueries(-5, 105, 3, 500)); }
    { std::vector<Box2> b; for (int i = 0; i < 3000; ++i) { double x = U(0, 1000); b.push_back(Box2{{x, 5}, {x + U(0, 2), 5}}); } checkSet("collinear-zero-height", b, randQueries(-5, 1005, 10, 500)); }
    { std::vector<Box2> b; for (int i = 0; i < 4000; ++i) { Vec2 a{U(0, 100), U(0, 100)}; double w = i % 50 == 0 ? U(30, 300) : U(0, 3); b.push_back(i % 77 == 0 ? Box2{} : Box2{a, a + Vec2{w, U(0, 3)}}); } checkSet("mixed-frames-empties", b, randQueries(-50, 150, 40, 1000)); }
    { std::vector<Box2> b; for (int i = 0; i < 3000; ++i) { Vec2 a{1e12 + U(0, 1e-3), -1e12 + U(0, 1e-3)}; b.push_back(Box2{a, a + Vec2{U(0, 1e-5), U(0, 1e-5)}}); } checkSet("huge-offset-tiny", b, randQueries(-1e12, 1e12, 1, 50)); std::vector<Box2> q; for (int i = 0; i < 500; ++i) { Vec2 a{1e12 + U(0, 1e-3), -1e12 + U(0, 1e-3)}; q.push_back(Box2{a, a + Vec2{U(0, 2e-5), U(0, 2e-5)}}); } checkSet("huge-offset-tiny-local", b, q); }
    { std::vector<Box2> b; for (int i = 0; i < 3000; ++i) { Vec2 a{U(-1e6, 1e6), U(-1e6, 1e6)}; b.push_back(Box2{a, a + Vec2{U(0, 1e4), U(0, 1e4)}}); } checkSet("huge-extent", b, randQueries(-1.1e6, 1.1e6, 1e5, 1000)); }
    { std::vector<Box2> b; for (int i = 0; i < 3000; ++i) { Vec2 a{U(0, 1), U(0, 1)}; b.push_back(Box2{a, a + Vec2{0.01, 0.01}}); } b.push_back(Box2{{1e6, 1e6}, {1e6 + 1, 1e6 + 1}}); checkSet("cluster+outlier", b, randQueries(-1, 2, 0.1, 500)); }
    { std::vector<Box2> b; for (int i = 0; i < 1000; ++i) { Vec2 a{U(0, 10), U(0, 10)}; b.push_back(Box2{a, a + Vec2{1, 1}}); } Box2 bi; bi.min = {-inf, 0}; bi.max = {inf, 1}; b.push_back(bi); Box2 bn; bn.min = {std::nan(""), 0}; bn.max = {1, 1}; b.push_back(bn); checkSet("inf-and-nan-boxes", b, randQueries(-20, 20, 5, 500)); }
    { std::vector<Box2> b; for (int i = 0; i < 100; ++i) { Vec2 a{U(0, 10), U(0, 10)}; b.push_back(Box2{a, a + Vec2{1, 1}}); } b.push_back(Box2{{-1e308, 0}, {1e308, 1}}); checkSet("overflowing-width-box", b, randQueries(-20, 20, 5, 500)); std::vector<Box2> q{Box2{{1e300, 0.5}, {1e300, 0.5}}, Box2{{-5e307, 0}, {-5e307, 1}}}; checkSet("overflowing-width-box-farq", b, q); }
    { std::vector<Box2> b{Box2{{-1e308, -1e308}, {-1e308, -1e308}}, Box2{{1e308, 1e308}, {1e308, 1e308}}, Box2{{0, 0}, {1, 1}}}; std::fflush(stdout); checkSet("overflowing-extent", b, randQueries(-20, 20, 5, 100)); }
    // big n timing
    { std::vector<Box2> b; for (int i = 0; i < 200000; ++i) { Vec2 a{U(0, 2000), U(0, 2000)}; b.push_back(Box2{a, a + Vec2{U(0, 4), U(0, 4)}}); }
      auto t0 = std::chrono::steady_clock::now(); toolpath::BoxIndex idx(b); size_t tot = 0; for (int i = 0; i < 200000; ++i) { Vec2 a{U(0, 2000), U(0, 2000)}; tot += idx.query(Box2{a, a + Vec2{5, 5}}).size(); }
      std::printf("200k boxes build+200k queries %.3f s (hits %zu)\n", std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(), tot); }
    // ClearanceField
    auto fieldCheck = [&](const char* name, std::vector<Contour> obs, double lo, double hi, int nq, std::vector<size_t> skip) {
        toolpath::ClearanceField f(obs); auto g = f.excluding(skip);
        long bad = 0; double worst = 0;
        for (int q = 0; q < nq; ++q) {
            Vec2 p{U(lo, hi), U(lo, hi)};
            double all = inf, some = inf;
            for (size_t i = 0; i < obs.size(); ++i) { double d = geometry::distanceTo(obs[i], p); all = std::min(all, d); if (std::find(skip.begin(), skip.end(), i) == skip.end()) some = std::min(some, d); }
            double a = f.distance(p), s = g.distance(p);
            if (!(a == all) || !(s == some)) { ++bad; worst = std::max({worst, std::fabs(a - all), std::fabs(s - some)}); if (bad < 3) std::printf("  %s MISMATCH p=(%g,%g) all %.17g got %.17g some %.17g got %.17g\n", name, p.x, p.y, all, a, some, s); }
        }
        std::printf("field %s: n=%zu q=%d mismatches=%ld worst %g\n", name, obs.size(), nq, bad, worst); fails += bad;
    };
    { std::vector<Contour> o; for (int i = 0; i < 2000; ++i) { Vec2 c{U(0, 300), U(0, 300)}; int t = i % 4; if (t == 0) o.push_back(Contour::circle(c, U(0.1, 3))); else if (t == 1) o.push_back(Contour::rectangle(Box2{c, c + Vec2{U(0.1, 5), U(0.1, 5)}})); else if (t == 2) { Contour l = Contour::fromPolyline(geometry::Polyline{{c, c + Vec2{U(-20, 20), U(-20, 20)}}, false}); o.push_back(l); } else o.push_back(Contour::fromPolyline(geometry::Polyline{{c, c + Vec2{U(-3, 3), U(-3, 3)}, c + Vec2{U(-3, 3), U(-3, 3)}}, true})); }
      o.push_back(Contour::rectangle(Box2{{-50, -50}, {350, 350}})); o.push_back(Contour{});
      std::vector<size_t> skip; for (int i = 0; i < 300; ++i) skip.push_back(size_t(U(0, 2100)));
      fieldCheck("mixed", o, -100, 400, 3000, skip); fieldCheck("mixed-far", o, -1e5, 1e5, 300, skip);
      std::vector<size_t> allButOne; for (size_t i = 0; i + 1 < o.size(); ++i) allButOne.push_back(i); fieldCheck("all-but-empty-excluded", o, -100, 400, 200, allButOne); }
    { std::vector<Contour> o; for (int i = 0; i < 500; ++i) o.push_back(Contour::circle({5, 5}, 1 + i * 1e-3)); fieldCheck("concentric", o, 0, 10, 500, {0, 1, 2}); }
    { std::vector<Contour> o; for (int i = 0; i < 400; ++i) { double x = U(0, 1000); o.push_back(Contour::fromPolyline(geometry::Polyline{{{x, 0}, {x + 1, 0}}, false})); } fieldCheck("collinear", o, -10, 1010, 1000, {}); }
    { std::vector<Contour> o; for (int i = 0; i < 300; ++i) { Vec2 c{1e9 + U(0, 1), 1e9 + U(0, 1)}; o.push_back(Contour::circle(c, U(1e-4, 1e-2))); } std::vector<Contour> o2 = o; toolpath::ClearanceField f(o2); long bad = 0; for (int q = 0; q < 500; ++q) { Vec2 p{1e9 + U(-0.5, 1.5), 1e9 + U(-0.5, 1.5)}; double all = inf; for (auto& c : o) all = std::min(all, geometry::distanceTo(c, p)); if (!(f.distance(p) == all)) ++bad; } std::printf("field huge-offset: mismatches %ld\n", bad); fails += bad; }
    { std::vector<Contour> o; for (int i = 0; i < 3000; ++i) { Vec2 c{U(0, 1), U(0, 1)}; o.push_back(Contour::circle(c, 0.001)); } o.push_back(Contour::circle({1e6, 1e6}, 1)); fieldCheck("cluster+outlier", o, -1, 2, 300, {}); }
    std::printf("TOTAL FAILS %ld\n", fails);
}
