// t0164v2 (g): SkeletonCheck::closer with 16-segment run boxes vs brute force over every segment.
// Random rings (circles, polygons with collinear runs, degenerate: repeated points, 1-2 point rings,
// zero-length segments, axis-parallel runs whose box has zero width) and adversarial query points:
// near box placed to miss a run box by 1 ulp / touch it exactly, points at exactly `limit` from a segment.
#include <cmath>
#include <cstdio>
#include <random>
#define private public
#include "v_carve_check.hpp"
#undef private
using namespace carvetto; using namespace carvetto::geometry; using carvetto::toolpath::detail::SkeletonCheck;
static bool brute(const SkeletonCheck& s, Vec2 p, double limit) {
    const double l2 = limit * limit;
    for (const auto& r : s.rings_) { for (std::size_t i = 0, n = r.size(); i < n; ++i) { Vec2 a = r[i]; Vec2 ab = r[(i + 1) % n] - a; double len2 = ab.lengthSquared();
        double t = len2 > 0.0 ? std::clamp((p - a).dot(ab) / len2, 0.0, 1.0) : 0.0; if ((p - (a + ab * t)).lengthSquared() < l2) return true; } }
    return false;
}
int main(int argc, char** argv) {
    int iters = argc > 1 ? atoi(argv[1]) : 200; std::mt19937_64 rg(7); auto U = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(rg); };
    long long q = 0, mism = 0, trueCount = 0;
    for (int it = 0; it < iters; ++it) {
        std::vector<Contour> cs; int nr = 1 + (int)U(0, 4);
        for (int k = 0; k < nr; ++k) { int m = (int)U(0, 5); Vec2 o{U(-50, 50), U(-50, 50)};
            if (m == 0) cs.push_back(Contour::circle(o, U(0.01, 30)));
            else if (m == 1) { std::vector<Vec2> pts; int n = 3 + (int)U(0, 80); for (int i = 0; i < n; ++i) pts.push_back({o.x + U(-20, 20), o.y + U(-20, 20)}); cs.push_back(Contour::fromPolyline(Polyline{pts, true})); }
            else if (m == 2) { std::vector<Vec2> pts; double w = U(0.1, 40); int n = 2 + (int)U(0, 60);  // long collinear axis-parallel runs
                for (int i = 0; i <= n; ++i) pts.push_back({o.x + w * i / n, o.y}); for (int i = n; i >= 0; --i) pts.push_back({o.x + w * i / n, o.y + (it % 3 == 0 ? 0.0 : 1e-9)}); cs.push_back(Contour::fromPolyline(Polyline{pts, true})); }
            else if (m == 3) { std::vector<Vec2> pts; int n = 1 + (int)U(0, 40); for (int i = 0; i < n; ++i) pts.push_back(i % 5 == 0 ? o : Vec2{o.x + U(-1, 1), o.y}); pts.push_back(o); cs.push_back(Contour::fromPolyline(Polyline{pts, true})); }
            else { cs.push_back(Contour::rectangle(Box2{o, Vec2{o.x + U(0, 30), o.y + U(0, 0.001)}})); } }
        Tolerance tol{U(0.0005, 0.5), 1e-9, 0.01}; tol.chordal = U(0.0005, 0.5);
        SkeletonCheck s(cs, tol);
        for (std::size_t k = 0; k < s.rings_.size(); ++k) {
            const auto& r = s.rings_[k];
            for (std::size_t c = 0; c < s.chunkBoxes_[k].size(); ++c) {
                const Box2 b = s.chunkBoxes_[k][c];
                for (int j = 0; j < 40; ++j) {
                    double limit = std::pow(10.0, U(-6, 1.5)); Vec2 p; int a = (int)U(0, 6);
                    if (a == 0) { p = {b.max.x + limit, U(b.min.y, b.max.y)}; p.x = std::nextafter(p.x, 1e300); }  // misses by an ulp-ish
                    else if (a == 1) { p = {U(b.min.x, b.max.x), b.min.y - limit}; }                                  // touches exactly
                    else if (a == 2) { p = {b.max.x + limit * 0.7071, b.max.y + limit * 0.7071}; }                    // diagonal corner
                    else if (a == 3) { std::size_t i = c * 16 + (std::size_t)U(0, 16); if (i >= r.size()) i = r.size() - 1; Vec2 A = r[i], B = r[(i + 1) % r.size()];
                        Vec2 m{(A.x + B.x) / 2, (A.y + B.y) / 2}; Vec2 d = B - A; double L = std::sqrt(d.lengthSquared()); Vec2 nrm = L > 0 ? Vec2{-d.y / L, d.x / L} : Vec2{1, 0};
                        double f = U(0.999999, 1.000001); p = m + nrm * (limit * f); }                                       // ~limit from a segment
                    else if (a == 4) { p = {U(b.min.x - 2 * limit, b.max.x + 2 * limit), U(b.min.y - 2 * limit, b.max.y + 2 * limit)}; }
                    else { p = {U(-80, 80), U(-80, 80)}; limit = U(0, 40); }
                    bool x = s.closer(p, limit, {}), y = brute(s, p, limit); ++q; trueCount += y;
                    if (x != y) { if (getenv("DBG") && mism == 0) { const auto& rr = s.rings_[k]; std::printf("ring n=%zu chunks=%zu ncs=%zu\n", rr.size(), s.chunkBoxes_[k].size(), cs.size()); for (std::size_t q = 0; q < rr.size() && q < 40; ++q) std::printf("  pt %zu (%.17g,%.17g)\n", q, rr[q].x, rr[q].y); for (std::size_t q = 0; q < s.chunkBoxes_[k].size() && q < 5; ++q) { auto bb = s.chunkBoxes_[k][q]; std::printf("  box %zu [%.17g,%.17g]-[%.17g,%.17g]\n", q, bb.min.x, bb.min.y, bb.max.x, bb.max.y); } for (std::size_t kk = 0; kk < s.rings_.size(); ++kk) { std::printf(" ring %zu n=%zu box [%.6g,%.6g]-[%.6g,%.6g]\n", kk, s.rings_[kk].size(), s.ringBoxes_[kk].min.x, s.ringBoxes_[kk].min.y, s.ringBoxes_[kk].max.x, s.ringBoxes_[kk].max.y); } }
                        if (++mism < 10) std::printf("MISMATCH it=%d ring=%zu chunk=%zu p=(%.17g,%.17g) limit=%.17g closer=%d brute=%d\n", it, k, c, p.x, p.y, limit, x, y); }
                }
            }
        }
    }
    std::printf("chunk: queries=%lld brute_true=%lld mismatches=%lld\n", q, trueCount, mism);
    return mism != 0;
}
