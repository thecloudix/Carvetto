// T-0140 round-2 verifier: enclosedPieces (leaf by leaf) vs one global Clipper2 union (holes less directly nested outers)
// and vs enclosedPieces with a single leaf, on adversarial ring families.
// usage: ep FAMILY SEED COUNT [leafsizes...]   (FAMILY -1 = all). env SHIFT (add to coordinates), V=1 verbose, NECK (fam 5 width nm)
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>
#include <string>
#include <vector>
#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/offset.hpp"
#include "enclosed_pieces.hpp"
#include "epdbg.inc"
using namespace carvetto; using geometry::Contour; using geometry::Vec2; using geometry::Box2; using geometry::Tolerance;
using toolpath::detail::PointRing; using toolpath::detail::EnclosedOptions; using toolpath::detail::EnclosedPiece;
namespace geo = carvetto::geometry;
constexpr double PI = 3.14159265358979323846;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
static double SH = 0;
static PointRing ngon(Vec2 c, double r, int n, bool cw, double a0 = 0) { PointRing p; for (int k = 0; k < n; ++k) { double a = a0 + (cw ? -1 : 1) * 2 * PI * k / n; p.push_back(c + Vec2{r * std::cos(a), r * std::sin(a)}); } return p; }
static PointRing rect(Box2 b, bool cw) { PointRing p{b.min, {b.max.x, b.min.y}, b.max, {b.min.x, b.max.y}}; if (cw) std::reverse(p.begin(), p.end()); return p; }
static Contour contourOf(const PointRing& r) { Contour c; c.closed = true; for (size_t k = 0; k < r.size(); ++k) c.spans.push_back(geo::LineSpan{r[k], r[(k + 1) % r.size()]}); return c; }
static Tolerance tolr() { Tolerance t; t.distance = 0.001; t.chordal = 0.005; return t; }
static void annulus(std::vector<PointRing>& R, Vec2 c, double r, double w, int n, double a0) { R.push_back(ngon(c, r + w, n, false, a0)); if (r - w > 0) R.push_back(ngon(c, r - w, n, true, a0)); }
static void openBand(std::vector<PointRing>& R, const std::vector<Vec2>& pts, double w) {
    Contour path; path.closed = false; for (size_t k = 0; k + 1 < pts.size(); ++k) path.spans.push_back(geo::LineSpan{pts[k], pts[k + 1]});
    for (const Contour& b : geo::offsetContours({path}, w, geo::JoinType::Round, tolr(), 2.0)) R.push_back(b.polygonise(tolr()).points);
}
static std::vector<PointRing> gen(int fam, double& slack) {
    std::vector<PointRing> R; slack = 1e-4;
    switch (fam) {
    case 0: { // test-like random mix
        int n = I(4, 30);
        for (int i = 0; i < n; ++i) { Vec2 c{U(0, 40), U(0, 40)}; int k = I(0, 3);
            if (k == 0) annulus(R, c, U(1, 9), U(0.2, 1.5), I(8, 48), U(0, 6));
            else if (k == 1) { std::vector<Vec2> p{c}; for (int j = 0; j < I(2, 6); ++j) p.push_back(p.back() + Vec2{U(-12, 12), U(-12, 12)}); openBand(R, p, U(0.2, 1.2)); }
            else if (k == 2) { double w = U(2, 14), h = U(2, 14), t = U(0.3, 2); Box2 o{c, c + Vec2{w, h}}; R.push_back(rect(o, false)); if (w > 2 * t && h > 2 * t) R.push_back(rect(o.inflated(-t), true)); }
            else { double r = U(1, 6); PointRing p; for (int j = 0; j < 5; ++j) { double a = 2 * PI * 2 * j / 5; p.push_back(c + Vec2{r * std::cos(a), r * std::sin(a)}); } R.push_back(p); } }
        break; }
    case 1: { // combs / zigzags: long bands crossing many cuts, plus a closing frame band
        int n = I(1, 5);
        for (int i = 0; i < n; ++i) { std::vector<Vec2> p; double y0 = U(0, 30), amp = U(0.5, 6), step = U(0.3, 3); int m = I(10, 120); bool vert = I(0, 1);
            for (int j = 0; j < m; ++j) { Vec2 q{j * step, y0 + ((j & 1) ? amp : 0)}; p.push_back(vert ? Vec2{q.y, q.x} : q); } openBand(R, p, U(0.05, 0.6)); }
        double W = U(20, 80); annulus(R, {W / 2, W / 2}, W * 0.7, U(0.3, 2), I(40, 400), U(0, 1));
        // a few square frames
        for (int i = 0; i < I(0, 4); ++i) { Vec2 c{U(0, 60), U(0, 60)}; double s = U(3, 30), t = U(0.2, 1); R.push_back(rect({c, c + Vec2{s, s}}, false)); R.push_back(rect({c + Vec2{t, t}, c + Vec2{s - t, s - t}}, true)); }
        break; }
    case 2: { // axis-aligned lattice rectangles, coordinates on integer mm (many shared x/y), frames and bars
        int n = I(10, 80);
        for (int i = 0; i < n; ++i) { int x = I(0, 30), y = I(0, 30), w = I(1, 10), h = I(1, 10); Box2 b{{double(x), double(y)}, {double(x + w), double(y + h)}};
            if (I(0, 2) == 0 && w > 2 && h > 2) { R.push_back(rect(b, false)); R.push_back(rect(b.inflated(-1), true)); }
            else R.push_back(rect(b, I(0, 9) == 0)); }
        break; }
    case 3: { // dense nanometre-cluster coordinates straddling the frame centre: cuts forced onto vertices
        // symmetric scene: rectangles with x edges at k nm (k in -6..6) around x=0, y similar, plus symmetric frame rings
        int n = I(6, 40);
        for (int i = 0; i < n; ++i) { double a = I(-6, 6) * 1e-6, b = a + I(1, 20) * 1e-6 + (I(0, 1) ? U(0.5, 8) : 0);
            double ya = U(-10, 10), yb = ya + U(0.3, 5); R.push_back(rect({{a, ya}, {b, yb}}, false));
            double c = I(-6, 6) * 1e-6, d = c - I(1, 20) * 1e-6 - (I(0, 1) ? U(0.5, 8) : 0); R.push_back(rect({{d, ya}, {c, yb}}, false)); }
        for (int i = 0; i < I(1, 4); ++i) { double s = U(4, 12), t = U(0.2, 1.5); Box2 o{{-s, -s}, {s, s}}; R.push_back(rect(o, false)); R.push_back(rect(o.inflated(-t), true)); }
        R.push_back(rect({{-12, -12}, {12, -11}}, false)); R.push_back(rect({{-12, 11}, {12, 12}}, false));
        break; }
    case 4: { // surrounding rings: big frames (few vertices) enclosing dense clusters, double-wound, CW-alone, nested
        int lv = I(1, 4); double s = 50;
        for (int l = 0; l < lv; ++l) { double t = U(0.3, 2); Box2 o{{-s, -s}, {s, s}}; int wnd = I(1, 2);
            for (int k = 0; k < wnd; ++k) { R.push_back(rect(o, false)); R.push_back(rect(o.inflated(-t), true)); } s -= U(3, 10); }
        for (int i = 0; i < I(5, 40); ++i) { Vec2 c{U(-s, s) * 0.5, U(-s, s) * 0.5}; annulus(R, c, U(0.3, 3), U(0.05, 0.8), I(16, 200), U(0, 6)); }
        if (I(0, 1)) R.push_back(ngon({0, 0}, s * 0.9, 3, true));            // a CW ring alone: winding -1, filled
        if (I(0, 1)) { PointRing p = ngon({0, 0}, s * 0.4, 7, false); for (int k = 0; k < 7; ++k) p.push_back(p[k]); R.push_back(p); } // wound twice
        break; }
    case 5: { // necks: an annulus-like square frame with a gap of width NECK nm leading to the outside, at random place/orientation
        double neck = std::getenv("NECK") ? std::atof(std::getenv("NECK")) * 1e-6 : std::pow(10.0, U(-6, -2.5));
        slack = 1e-4;
        for (int i = 0; i < I(1, 6); ++i) {
            Vec2 c{U(0, 40), U(0, 40)}; double s = U(2, 10), t = U(0.2, 1.0);
            // frame from 4 bars; the bottom bar split by a gap of `neck` at x = c.x + gx
            double gx = U(t + 0.1, s - t - 0.1);
            R.push_back(rect({c, {c.x + gx, c.y + t}}, false)); R.push_back(rect({{c.x + gx + neck, c.y}, {c.x + s, c.y + t}}, false));
            R.push_back(rect({{c.x, c.y + s - t}, {c.x + s, c.y + s}}, false));
            R.push_back(rect({{c.x, c.y}, {c.x + t, c.y + s}}, false)); R.push_back(rect({{c.x + s - t, c.y}, {c.x + s, c.y + s}}, false));
        }
        // fillers so leaves are small around
        for (int i = 0; i < I(0, 30); ++i) { Vec2 c{U(0, 50), U(0, 50)}; R.push_back(ngon(c, U(0.05, 0.4), I(6, 40), false)); }
        break; }
    case 6: { // thin leaves: many vertices packed in a 10-40 um wide column, long thin rings
        double x0 = U(0, 10);
        for (int i = 0; i < I(5, 40); ++i) { double a = x0 + U(0, 0.04), b = a + U(0.001, 0.05); double ya = U(0, 30); R.push_back(rect({{a, ya}, {b, ya + U(0.5, 20)}}, I(0, 5) == 0)); }
        for (int i = 0; i < I(2, 10); ++i) annulus(R, {x0 + U(-5, 5), U(0, 30)}, U(1, 8), U(0.1, 1), I(16, 300), U(0, 6));
        for (int i = 0; i < I(0, 5); ++i) { double y = U(0, 30); R.push_back(rect({{x0 - 20, y}, {x0 + 20, y + U(0.01, 0.5)}}, false)); }
        break; }
    case 7: { // nested concentric annuli with islands and pieces inside islands
        Vec2 c{U(-5, 5), U(-5, 5)}; int n = I(3, 30); double r = 0.5;
        for (int i = 0; i < n; ++i) { double w = U(0.05, 0.6); r += U(0.15, 1.5); annulus(R, c, r, w, I(12, 300), U(0, 6)); r += w; }
        for (int i = 0; i < I(0, 30); ++i) { double a = U(0, 2 * PI), rr = U(0, r); annulus(R, c + Vec2{rr * std::cos(a), rr * std::sin(a)}, U(0.1, 1), U(0.05, 0.3), I(8, 60), 0); }
        break; }
    case 8: { // random polygon bands (Clipper offset outlines) of random open polylines, dense: overlapping kerfs grid
        int g = I(3, 12); double sp = U(2, 6);
        for (int i = 0; i < g; ++i) for (int j = 0; j < g; ++j) { Vec2 c{i * sp, j * sp}; double s = sp * U(0.4, 0.9);
            std::vector<Vec2> p{c, c + Vec2{s, 0}, c + Vec2{s, s}, c + Vec2{0, s}, c}; openBand(R, p, U(0.3, 1.2) * (sp - s) * 0.5 + U(0, 0.3)); }
        break; }
    case 9: { // rings exactly on leaf-cut-like coordinates: dyadic/grid coordinates; tangent rectangles sharing edges
        int n = I(10, 60);
        for (int i = 0; i < n; ++i) { double x = I(0, 40) * 0.5, y = I(0, 40) * 0.5, w = I(1, 8) * 0.5, h = I(1, 8) * 0.5;
            R.push_back(rect({{x, y}, {x + w, y + h}}, false)); if (I(0, 3) == 0) R.push_back(rect({{x + w, y}, {x + 2 * w, y + h}}, false)); }
        for (int i = 0; i < 3; ++i) { double s = I(4, 20), t = 0.5; R.push_back(rect({{0, 0}, {s, s}}, false)); R.push_back(rect({{t, t}, {s - t, s - t}}, true)); }
        break; }
    case 10: { // held corridor: region RA (left) reaches the outside only through a corridor of slanted walls to RA' (right), which is open
        auto bar = [&](Vec2 a, Vec2 b, double w) { Vec2 d = b - a; double L = d.length(); Vec2 n{-d.y / L * w * 0.5, d.x / L * w * 0.5}; return PointRing{a - n, b - n, b + n, a + n}; };
        double t = U(0.2, 0.6), H = U(3, 8), W = U(4, 12), gy = U(-H + 1, H - 2), gw = U(0.3, 1.2), x0 = U(0.3, 2.0);
        double s1 = U(-0.3, 0.3), s2 = s1 + U(-0.05, 0.05);
        for (int side : {-1, 1}) {
            double xn = side * x0, xf = side * (x0 + W);
            R.push_back(rect({{std::min(xn, xf), H}, {std::max(xn, xf), H + t}}, false)); R.push_back(rect({{std::min(xn, xf), -H - t}, {std::max(xn, xf), -H}}, false));
            if (side < 0) R.push_back(rect({{xf - t, -H - t}, {xf, H + t}}, false)); else { R.push_back(rect({{xf, -H - t}, {xf + t, gy}}, false)); R.push_back(rect({{xf, gy + gw}, {xf + t, H + t}}, false)); }
            // near wall with the corridor gap [gy, gy+gw]
            R.push_back(rect({{std::min(xn, xn - side * t), -H - t}, {std::max(xn, xn - side * t), gy}}, false)); R.push_back(rect({{std::min(xn, xn - side * t), gy + gw}, {std::max(xn, xn - side * t), H + t}}, false));
        }
        // corridor walls from (-x0, gy) to (x0, gy + 2 x0 s1) etc. (slanted)
        double wt = U(0.05, 0.4);
        { double e = U(-0.6, 0.6) * gw; R.push_back(bar({-x0 - t * 0.5, gy - wt * 0.5}, {x0 + t * 0.5, gy - wt * 0.5 + e + U(-0.01, 0.01)}, wt));
          R.push_back(bar({-x0 - t * 0.5, gy + gw + wt * 0.5}, {x0 + t * 0.5, gy + gw + wt * 0.5 + e + U(-0.01, 0.01)}, wt)); }
        (void)s2;
        // fillers: many-vertex circles outside, to force leaf cuts (count env FILL, default random)
        int nf = std::getenv("FILL") ? std::atoi(std::getenv("FILL")) : I(0, 20);
        for (int i = 0; i < nf; ++i) { Vec2 c{U(-x0 - W, x0 + W), (I(0, 1) ? 1 : -1) * U(H + 2, H + 10)}; R.push_back(ngon(c, U(0.2, 1.0), I(20, 200), false, U(0, 6))); }
        break; }
    case 11: { // nested C-bands (polyline arcs, long chords) with aligned gaps and a radial wall opposite: every region is held (joined to the outside through the gap corridor)
        Vec2 c{U(-3, 3), U(-3, 3)}; int n = I(4, 25); double r = U(0.5, 2), th = U(0, 2 * PI), gap = U(0.15, 0.5); double rmax = 0;
        for (int i = 0; i < n; ++i) { double w = U(0.05, 0.4); r += U(0.2, 1.5) + w; int m = I(16, 300) * (std::getenv("MULT") ? std::atoi(std::getenv("MULT")) : 1); std::vector<Vec2> p;
            double a0 = th + gap / r * 0.5 + w / r, a1 = th + 2 * PI - gap / r * 0.5 - w / r;
            for (int k = 0; k <= m; ++k) { double a = a0 + (a1 - a0) * k / m; p.push_back(c + Vec2{r * std::cos(a), r * std::sin(a)}); }
            openBand(R, p, w); r += w; rmax = r; }
        double tw = th + PI; openBand(R, {c, c + Vec2{(rmax + 0.5) * std::cos(tw), (rmax + 0.5) * std::sin(tw)}}, U(0.05, 0.3));
        break; }
    }
    if (SH != 0) for (auto& r : R) for (auto& p : r) { p.x += SH; p.y += SH * 0.7; }
    return R;
}
struct OP { double area; bool thick, thickLess, thickMore; std::vector<Contour> region; };
static std::vector<OP> oracle(const std::vector<PointRing>& rings, const EnclosedOptions& opt) {
    std::vector<Contour> cs; for (auto& r : rings) cs.push_back(contourOf(r));
    auto u = geo::unionContours(cs, opt.tol, geo::FillRule::NonZero);
    std::vector<double> sa; for (auto& c : u) sa.push_back(geo::signedArea(c));
    auto enclosing = [&](size_t k) { size_t best = u.size(); double ba = 1e300;
        for (size_t m = 0; m < u.size(); ++m) { if (m == k || std::fabs(sa[m]) >= ba || !u[m].bounds().contains(u[k].bounds())) continue;
            for (auto& sp : u[k].spans) { auto at = geo::locatePoint(u[m], geo::startPoint(sp), geo::FillRule::NonZero, Tolerance{}); if (at == geo::PointLocation::OnBoundary) continue; if (at == geo::PointLocation::Inside) { best = m; ba = std::fabs(sa[m]); } break; } }
        return best; };
    std::vector<OP> out;
    for (size_t h = 0; h < u.size(); ++h) { if (sa[h] >= 0) continue; OP p; p.area = -sa[h]; p.region.push_back(u[h]); geo::setOrientation(p.region.back(), geo::Orientation::CCW, opt.tol);
        for (size_t k = 0; k < u.size(); ++k) { if (sa[k] <= 0 || !u[h].bounds().contains(u[k].bounds())) continue; if (enclosing(k) != h) continue; p.area -= sa[k]; p.region.push_back(u[k]); geo::setOrientation(p.region.back(), geo::Orientation::CW, opt.tol); }
        if (p.area < opt.minArea) continue;
        auto th = [&](double c) { return !geo::offsetContours(p.region, -c, geo::JoinType::Round, opt.tol, opt.mitreLimit).empty(); };
        p.thick = th(opt.clearance); p.thickLess = th(0.98 * opt.clearance); p.thickMore = th(1.02 * opt.clearance); out.push_back(std::move(p)); }
    return out;
}
static int V = 0;
// returns number of problems
static int compare(const char* tag, const std::vector<EnclosedPiece>& got, const std::vector<OP>& want, double slack, const EnclosedOptions& opt) {
    int bad = 0; std::vector<bool> used(want.size(), false);
    for (auto& g : got) { size_t match = want.size();
        for (size_t k = 0; k < want.size(); ++k) { if (used[k] || std::fabs(want[k].area - g.area) > 1e-5 * want[k].area + slack) continue;
            if (g.inner && geo::locatePoint(want[k].region, *g.inner, geo::FillRule::NonZero, Tolerance{}) != geo::PointLocation::Inside) continue; match = k; break; }
        if (match == want.size()) { ++bad; std::printf("  %s EXTRA piece area %.6f box %.4f,%.4f-%.4f,%.4f inner %d\n", tag, g.area, g.bounds.min.x, g.bounds.min.y, g.bounds.max.x, g.bounds.max.y, (int)g.inner.has_value()); continue; }
        used[match] = true; const OP& w = want[match];
        if (w.thickLess == w.thickMore && g.inner.has_value() != w.thick) { ++bad; std::printf("  %s THICK mismatch area %.6f got %d want %d\n", tag, g.area, (int)g.inner.has_value(), (int)w.thick); }
        if (g.inner) { double d = 1e300; for (auto& c : w.region) d = std::min(d, geo::distanceTo(c, *g.inner)); if (d < opt.clearance - 2 * opt.tol.chordal) { ++bad; std::printf("  %s INNER too close %.5f\n", tag, d); } }
    }
    for (size_t k = 0; k < want.size(); ++k) if (!used[k]) { ++bad; auto b = want[k].region[0].bounds(); std::printf("  %s MISSING piece area %.6f box %.4f,%.4f-%.4f,%.4f thick %d\n", tag, want[k].area, b.min.x, b.min.y, b.max.x, b.max.y, (int)want[k].thick); }
    return bad;
}
static int compare2(const char* tag, const std::vector<EnclosedPiece>& got, const std::vector<EnclosedPiece>& ref, double slack) {
    int bad = 0; std::vector<bool> used(ref.size(), false);
    auto near = [](const Box2& a, const Box2& b) { double e = 2e-3; return std::fabs(a.min.x - b.min.x) < e && std::fabs(a.min.y - b.min.y) < e && std::fabs(a.max.x - b.max.x) < e && std::fabs(a.max.y - b.max.y) < e; };
    for (auto& g : got) { size_t m = ref.size();
        for (size_t k = 0; k < ref.size(); ++k) if (!used[k] && std::fabs(ref[k].area - g.area) <= 1e-5 * ref[k].area + slack && near(ref[k].bounds, g.bounds)) { m = k; break; }
        if (m == ref.size()) { ++bad; std::printf("  %s EXTRA-vs-single area %.6f box %.4f,%.4f-%.4f,%.4f inner %d\n", tag, g.area, g.bounds.min.x, g.bounds.min.y, g.bounds.max.x, g.bounds.max.y, (int)g.inner.has_value()); continue; }
        used[m] = true; if (g.inner.has_value() != ref[m].inner.has_value()) { ++bad; std::printf("  %s THICK-vs-single area %.6f got %d single %d\n", tag, g.area, (int)g.inner.has_value(), (int)ref[m].inner.has_value()); } }
    for (size_t k = 0; k < ref.size(); ++k) if (!used[k]) { ++bad; auto b = ref[k].bounds; std::printf("  %s MISSING-vs-single area %.6f box %.4f,%.4f-%.4f,%.4f inner %d\n", tag, ref[k].area, b.min.x, b.min.y, b.max.x, b.max.y, (int)ref[k].inner.has_value()); }
    return bad;
}
int main(int argc, char** argv) {
    int fam = std::atoi(argv[1]); unsigned long seed = std::strtoul(argv[2], nullptr, 0); int count = std::atoi(argv[3]);
    std::vector<size_t> leaves; for (int i = 4; i < argc; ++i) leaves.push_back(std::strtoul(argv[i], nullptr, 0));
    if (leaves.empty()) leaves = {4, 5, 6, 8, 13, 24, 64, 100, 256, 512};
    if (std::getenv("SHIFT")) SH = std::atof(std::getenv("SHIFT")); V = std::getenv("V") ? 1 : 0;
    long unionDiff = 0, scenes = 0, problems = 0, pieces = 0, badScenes = 0;
    for (int s = 0; s < count; ++s) {
        rng.seed(seed * 1000003ULL + s); int f = fam >= 0 ? fam : I(0, 9); double slack; auto R = gen(f, slack);
        if (std::getenv("ONLY") && std::atoi(std::getenv("ONLY")) != s) continue;
        EnclosedOptions opt; opt.minArea = 0.04; opt.clearance = 0.025; opt.tol = tolr();
        if (std::getenv("DUMPBOX")) { double bx0, by0, bx1, by1; std::sscanf(std::getenv("DUMPBOX"), "%lf,%lf,%lf,%lf", &bx0, &by0, &bx1, &by1);
            for (auto& r : R) { Box2 b; for (auto p : r) b.expand(p); if (b.max.x < bx0 || b.min.x > bx1 || b.max.y < by0 || b.min.y > by1) continue;
                std::printf("RING n=%zu area %.6f:", r.size(), geo::signedArea(contourOf(r))); for (size_t k = 0; k < r.size() && k < 12; ++k) std::printf(" (%.6f,%.6f)", r[k].x, r[k].y); std::printf("\n"); } }
        auto want = oracle(R, opt); pieces += want.size();
        if (std::getenv("PA")) { double pa = std::atof(std::getenv("PA")); for (auto& w : want) if (std::fabs(w.area - pa) < 1e-3) { auto b = w.region[0].bounds(); std::printf("ORACLE area %.6f thick %d less %d more %d box %.4f,%.4f-%.4f,%.4f\n", w.area, w.thick, w.thickLess, w.thickMore, b.min.x, b.min.y, b.max.x, b.max.y);
            for (auto c : {0.02, 0.022, 0.024, 0.025, 0.026, 0.028}) std::printf("   erode %.3f -> %zu contours\n", c, geo::offsetContours(w.region, -c, geo::JoinType::Round, opt.tol, opt.mitreLimit).size()); } }
        EnclosedOptions o1 = opt; o1.leafVertices = 1u << 30; auto one = toolpath::detail::enclosedPieces(R, o1);
        if (std::getenv("EPDBG")) { for (size_t L : leaves) { EnclosedOptions o = opt; o.leafVertices = L; std::printf("== L%zu\n", L); epdbg::enclosedPieces(R, o); } continue; }
        char tag[64]; int pb = 0;
        std::snprintf(tag, sizeof tag, "f%d s%lu_%d single", f, seed, s); int ob = compare(tag, one, want, slack, opt); unionDiff += ob ? 1 : 0;
        for (size_t L : leaves) { EnclosedOptions o = opt; o.leafVertices = L; auto got = toolpath::detail::enclosedPieces(R, o);
            std::snprintf(tag, sizeof tag, "f%d s%lu_%d L%zu", f, seed, s, L); pb += compare2(tag, got, one, slack); }
        ++scenes; problems += pb; if (pb) { ++badScenes; std::printf("BAD f%d seed %lu s %d rings %zu oracle pieces %zu single %zu\n", f, seed, s, R.size(), want.size(), one.size()); }
        else if (V) std::printf("ok f%d s%d pieces %zu\n", f, s, want.size());
        std::fflush(stdout);
    }
    if (std::getenv("EPDBG")) std::printf("SIDEEDGES exact %ld off %ld\n", epdbg::g_tot, epdbg::g_off);
    std::printf("SUMMARY fam %d seed %lu scenes %ld oraclePieces %ld badScenes %ld problems %ld singleVsUnionDiffScenes %ld\n", fam, seed, scenes, pieces, badScenes, problems, unionDiff);
}
