// round 4: per-glyph medial axis of scale.cpp's text glyphs (h=5): samples, branches, points, ignored, and skeleton
// points inconsistent with the region (centre outside, or radius > exact distance + 2c + d). usage: glyphs
#include <cstdio>
#include "acc.hpp"
#include "shapes.hpp"
#include "carvetto/geometry/intersect.hpp"
using namespace carvetto::geometry;
static std::vector<Contour> glyph(int k, Vec2 o, double h) {
    double s = h / 20.0; Transform2 m{s, 0, 0, s, o.x, o.y}; std::vector<Contour> g;
    switch (k % 5) {
    case 0: g = {vs::bezEllipse({8, 11}, 8, 11), vs::rev(vs::bezEllipse({8, 11}, 5, 8.5))}; break;
    case 1: g = {vs::poly({{0, 0}, {3, 0}, {4, 4}, {8, 4}, {9, 0}, {12, 0}, {7.5, 16}, {4.5, 16}}), vs::rev(vs::poly({{4.8, 6.5}, {7.2, 6.5}, {6, 11.5}}))}; break;
    case 2: g = {vs::sGlyph({0, 0}, 20)}; break;
    case 3: g = {vs::cGlyph({10, 10}, 10, 3)}; break;
    default: g = {vs::poly({{0, 0}, {3, 0}, {3, 8}, {9, 8}, {9, 0}, {12, 0}, {12, 20}, {9, 20}, {9, 11}, {3, 11}, {3, 20}, {0, 20}})}; break; }
    for (auto& c : g) c.transform(m); return g;
}
int main(int argc, char**) { Tolerance tol;
    if (argc > 1) { std::vector<Contour> cs; size_t ns = 0; int k = 0; double h = 5; while (ns < 1562) { int r = k / 80, c = k % 80; auto g = glyph(k * 7 + r, {c * h * 0.8, -r * h * 1.4}, h); for (auto& q : g) { ns += q.spans.size(); cs.push_back(q); } ++k; }
        { size_t x = 0; for (size_t i = 0; i < cs.size(); ++i) for (size_t j = i + 1; j < cs.size(); ++j) if (cs[i].bounds().inflated(0.01).intersects(cs[j].bounds()) && !intersect(cs[i], cs[j], tol).isEmpty()) ++x; std::printf("crossing/touching pairs: %zu\n", x); }
        MedialAxis ax = medialAxis(cs, tol); va::Acc A; A.build(cs); size_t pts = 0, bad = 0, out = 0; double worst = 0; Vec2 wp{};
        for (auto& b : ax.branches) for (auto& m : b.points) { ++pts; bool in = A.inside(m.point); double d = A.dist(m.point); if (!in) ++out; double e = !in ? m.radius + d : std::max(0.0, std::fabs(m.radius - d) - 2 * tol.chordal - tol.distance); if (e > 1e-9) { ++bad; if (e > worst) { worst = e; wp = m.point; } } }
        std::printf("page: contours=%zu samples=%zu branches=%zu pts=%zu ign=%zu outside=%zu bad=%zu worst=%.2e at (%.3f,%.3f)\n", cs.size(), ax.boundarySamples, ax.branches.size(), pts, ax.ignoredContours, out, bad, worst, wp.x, wp.y); return 0; }
    for (int k = 0; k < 5; ++k) { auto cs = glyph(k, {0, 0}, 5); MedialAxis ax = medialAxis(cs, tol); va::Acc A; A.build(cs); size_t pts = 0, bad = 0; double worst = 0, rmax = 0;
        for (auto& b : ax.branches) for (auto& m : b.points) { ++pts; rmax = std::max(rmax, m.radius); bool in = A.inside(m.point); double d = A.dist(m.point); double e = !in ? m.radius + d : std::max(0.0, std::fabs(m.radius - d) - 2 * tol.chordal - tol.distance); if (e > 1e-9) { ++bad; worst = std::max(worst, e); } }
        std::printf("glyph %d: samples=%zu branches=%zu pts=%zu rmax=%.4f ign=%zu bad=%zu worst=%.2e\n", k, ax.boundarySamples, ax.branches.size(), pts, rmax, ax.ignoredContours, bad, worst); } }
