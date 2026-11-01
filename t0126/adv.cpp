// Adversarial raw insets fed into the (scratch-copied) Strict insetBy via a test hook: exercises the
// reflex inset repair and pull-in on shapes Clipper itself has not been seen to produce.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/tolerance.hpp"
using namespace carvetto;
using geometry::Contour; using geometry::Vec2; using Poly = std::vector<Vec2>;
#include "pocket_guard.hpp"
namespace carvetto::toolpath {
static std::vector<Contour> advInset(const std::vector<Contour>& walls, const std::vector<Contour>& raw, double d,
                              const geometry::Tolerance& tol, std::size_t* dropped, std::size_t* trimmed) {
    detail::InsetReport rep; auto out = detail::strictInset(walls, raw, d, tol, &rep); *dropped = rep.dropped; *trimmed = rep.trimmed; return out; }
}
static double segPt(Vec2 p, Vec2 a, Vec2 b) { double dx = b.x - a.x, dy = b.y - a.y, L = dx * dx + dy * dy; double t = L > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / L : 0; t = std::clamp(t, 0.0, 1.0); return std::hypot(p.x - a.x - t * dx, p.y - a.y - t * dy); }
static double orient(Vec2 a, Vec2 b, Vec2 c) { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); }
static bool cross(Vec2 a, Vec2 b, Vec2 c, Vec2 d) { double o1 = orient(a, b, c), o2 = orient(a, b, d), o3 = orient(c, d, a), o4 = orient(c, d, b); return ((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) && ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0)); }
static double segSeg(Vec2 a, Vec2 b, Vec2 c, Vec2 d) { if (cross(a, b, c, d)) return 0; return std::min({segPt(a, c, d), segPt(b, c, d), segPt(c, a, b), segPt(d, a, b)}); }
static double area(const Poly& p) { double A = 0; for (size_t i = 0; i < p.size(); ++i) A += p[i].x * p[(i + 1) % p.size()].y - p[(i + 1) % p.size()].x * p[i].y; return A / 2; }

static void run(const char* name, const Poly& wall, const std::vector<Poly>& rawPolys, double d, double ch) {
    geometry::Tolerance tol = geometry::Tolerance::defaults(); tol.chordal = ch / 2;
    std::vector<Contour> walls{Contour::fromPolyline(geometry::Polyline{wall, true})}, raw;
    double rawA = 0;
    for (auto& p : rawPolys) { raw.push_back(Contour::fromPolyline(geometry::Polyline{p, true})); rawA += area(p); }
    std::size_t dropped = 0, trimmed = 0;
    auto t0 = std::chrono::steady_clock::now();
    auto out = toolpath::advInset(walls, raw, d, tol, &dropped, &trimmed);
    double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    double need = d - tol.chordal - 1e-6, minV = 1e9, minE = 1e9, A = 0; long xs = 0; size_t nv = 0;
    std::vector<std::pair<Vec2, Vec2>> E;
    for (auto& c : out) { auto p = c.polygonise(geometry::Tolerance::defaults()).points; A += area(p); nv += p.size();
        for (size_t i = 0; i < p.size(); ++i) { Vec2 a = p[i], b = p[(i + 1) % p.size()]; E.push_back({a, b});
            for (size_t k = 0; k < wall.size(); ++k) { minV = std::min(minV, segPt(a, wall[k], wall[(k + 1) % wall.size()])); minE = std::min(minE, segSeg(a, b, wall[k], wall[(k + 1) % wall.size()])); } } }
    for (size_t i = 0; i < E.size(); ++i) for (size_t j = i + 1; j < E.size(); ++j) xs += cross(E[i].first, E[i].second, E[j].first, E[j].second);
    printf("%-34s d=%.3f: parts %zu verts %zu dropped %zu trimmed %zu | area %.3f of raw %.3f (lost %.3f) | minVertex-need %+.2e minEdge-need %+.2e | selfX %ld | %.1f ms\n",
           name, d, out.size(), nv, dropped, trimmed, A, rawA, rawA - A, minV - need, minE - need, xs, ms);
}

int main() {
    const double ch = getenv("CH") ? atof(getenv("CH")) : 0.01;
    // A: pocket with a tooth poking in; one reflex arc vertex of the true inset pushed towards the tooth tip.
    Poly wall{{0, 0}, {18, 0}, {20, 12}, {22, 0}, {40, 0}, {40, 40}, {0, 40}};
    geometry::Tolerance tol = geometry::Tolerance::defaults(); tol.chordal = ch / 2;
    for (double d : {3.0, 5.0}) {
        auto raw = geometry::offsetContours({Contour::fromPolyline(geometry::Polyline{wall, true})}, -d, geometry::JoinType::Round, tol);
        Poly rp = raw[0].polygonise(geometry::Tolerance::defaults()).points;
        // find the arc vertex closest to the tooth tip (reflex for the region)
        size_t best = 0; double bd = 1e9; for (size_t i = 0; i < rp.size(); ++i) { double q = std::hypot(rp[i].x - 20, rp[i].y - 12); if (q < bd && rp[i].y > 12) { bd = q; best = i; } }
        run("A0 tooth, raw unchanged", wall, {rp}, d, ch);
        for (double delta : {1e-5, 1e-3, 0.05, 0.5, 2.0}) {
            Poly p = rp; Vec2 v = p[best]; double L = std::hypot(v.x - 20, v.y - 12);
            p[best] = {20 + (v.x - 20) * (L - delta) / L, 12 + (v.y - 12) * (L - delta) / L};
            char nm[64]; snprintf(nm, sizeof nm, "A reflex arc vertex -%.0e", delta); run(nm, wall, {p}, d, ch);
        }
    }
    // C: square pocket; true inset [d, 40-d]^2 plus a bulge towards the bottom wall.
    Poly sq{{0, 0}, {40, 0}, {40, 40}, {0, 40}};
    for (double d : {5.0}) {
        double lo = d, hi = 40 - d;
        // C1 convex-only bulge (junctions on the true boundary)
        run("C1 convex bulge", sq, {{{lo, lo}, {18, lo}, {19, 2}, {20, 1.5}, {21, 2}, {22, lo}, {hi, lo}, {hi, hi}, {lo, hi}}}, d, ch);
        // C2 bulge whose junction vertices are reflex and slightly too close
        run("C2 bulge, reflex junctions -0.1", sq, {{{lo, lo}, {18, lo - 0.1}, {19, 2}, {20, 1.5}, {21, 2}, {22, lo - 0.1}, {hi, lo}, {hi, hi}, {lo, hi}}}, d, ch);
        // C3 mushroom bulge: neck then wider head (reflex vertices deep in the failing zone)
        run("C3 mushroom bulge", sq, {{{lo, lo}, {19.5, lo}, {19.5, 3}, {17, 2.5}, {17, 1}, {23, 1}, {23, 2.5}, {20.5, 3}, {20.5, lo}, {hi, lo}, {hi, hi}, {lo, hi}}}, d, ch);
        // C4 bulge that is most of the ring's vertices but small (minority by count is what matters)
        Poly c4{{lo, lo}};
        for (int i = 0; i <= 8; ++i) { double t = 3.14159265 * i / 8; c4.push_back({20 - 2 * std::cos(t), lo - 0.2 - 3 * std::sin(t)}); }
        c4.insert(c4.end(), {{hi, lo}, {hi, hi}, {lo, hi}});
        run("C4 round bulge 9 of 13 verts", sq, {c4}, d, ch);
    }
}
