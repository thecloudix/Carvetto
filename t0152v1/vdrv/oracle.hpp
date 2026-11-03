// Verifier's own oracle (independent of the coder's medial_axis_oracle.hpp and of closestPoint).
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <variant>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/medial_axis.hpp"
#include "carvetto/geometry/span.hpp"
namespace vo {
using namespace carvetto::geometry;
constexpr double PI = 3.14159265358979323846;
inline double sweepOf(const ArcSpan& a) {
    double s = a.endAngle - a.startAngle;
    if (a.direction == ArcDirection::CCW) { s = std::fmod(s, 2 * PI); if (s <= 1e-12) s += 2 * PI; }
    else { s = std::fmod(s, 2 * PI); if (s >= -1e-12) s -= 2 * PI; }
    return s;
}
inline Vec2 bez(const BezierSpan& b, double t) {
    double u = 1 - t;
    return b.p0 * (u * u * u) + b.p1 * (3 * u * u * t) + b.p2 * (3 * u * t * t) + b.p3 * (t * t * t);
}
inline Vec2 evalSpan(const Span& s, double t) {
    if (auto* l = std::get_if<LineSpan>(&s)) return l->start + (l->end - l->start) * t;
    if (auto* a = std::get_if<ArcSpan>(&s)) { double an = a->startAngle + t * sweepOf(*a); return {a->centre.x + a->radius * std::cos(an), a->centre.y + a->radius * std::sin(an)}; }
    return bez(std::get<BezierSpan>(s), t);
}
inline double dist(Vec2 a, Vec2 b) { return std::hypot(a.x - b.x, a.y - b.y); }
struct Foot { double d; Vec2 q; double t; };
inline Foot spanDist(const Span& s, Vec2 p) {
    if (auto* l = std::get_if<LineSpan>(&s)) {
        Vec2 d = l->end - l->start; double L2 = d.dot(d);
        double t = L2 > 0 ? std::clamp((p - l->start).dot(d) / L2, 0.0, 1.0) : 0.0;
        Vec2 q = l->start + d * t; return {dist(p, q), q, t};
    }
    if (auto* a = std::get_if<ArcSpan>(&s)) {
        double sw = sweepOf(*a);
        Foot best{dist(p, evalSpan(s, 0)), evalSpan(s, 0), 0};
        Foot e1{dist(p, evalSpan(s, 1)), evalSpan(s, 1), 1};
        if (e1.d < best.d) best = e1;
        Vec2 v = p - a->centre; double L = std::hypot(v.x, v.y);
        if (L > 0) {
            double ang = std::atan2(v.y, v.x);
            double rel = (ang - a->startAngle) / sw; // fraction, mod period 2pi/|sw|
            double per = 2 * PI / std::fabs(sw);
            rel = std::fmod(rel, per); if (rel < 0) rel += per;
            if (rel <= 1.0) { Vec2 q = a->centre + v * (a->radius / L); double d = std::fabs(L - a->radius); if (d < best.d) best = {d, q, rel}; }
        }
        return best;
    }
    const auto& b = std::get<BezierSpan>(s);
    const int N = 400; Foot best{1e300, {}, 0};
    std::vector<double> ds(N + 1);
    for (int i = 0; i <= N; ++i) ds[i] = dist(p, bez(b, double(i) / N));
    for (int i = 0; i <= N; ++i) {
        bool lm = (i == 0 || ds[i] <= ds[i - 1]) && (i == N || ds[i] <= ds[i + 1]);
        if (!lm) continue;
        double lo = std::max(0.0, double(i - 1) / N), hi = std::min(1.0, double(i + 1) / N);
        for (int it = 0; it < 100; ++it) { // golden-ish ternary
            double m1 = lo + (hi - lo) / 3, m2 = hi - (hi - lo) / 3;
            if (dist(p, bez(b, m1)) < dist(p, bez(b, m2))) hi = m2; else lo = m1;
        }
        double t = 0.5 * (lo + hi); double d = dist(p, bez(b, t));
        if (d < best.d) best = {d, bez(b, t), t};
    }
    return best;
}
struct Boundary {
    const std::vector<Contour>* cs;
    struct S { Vec2 p; int c, s; double t; };
    std::vector<S> dense;          // dense samples (step h)
    std::vector<std::vector<Vec2>> poly; // for point-in-region
    double h;
    double minx = 1e300, miny = 1e300, maxx = -1e300, maxy = -1e300;
};
inline Boundary makeBoundary(const std::vector<Contour>& cs, double h) {
    Boundary B; B.cs = &cs; B.h = h;
    for (int c = 0; c < (int)cs.size(); ++c) {
        std::vector<Vec2> pl;
        for (int s = 0; s < (int)cs[c].spans.size(); ++s) {
            const Span& sp = cs[c].spans[s];
            // length estimate
            double L = 0; Vec2 prev = evalSpan(sp, 0);
            for (int i = 1; i <= 256; ++i) { Vec2 q = evalSpan(sp, i / 256.0); L += dist(prev, q); prev = q; }
            int n = std::max(1, (int)std::ceil(L / h) * (std::holds_alternative<BezierSpan>(sp) ? 3 : 1));
            for (int i = 0; i < n; ++i) { double t = double(i) / n; Vec2 q = evalSpan(sp, t); B.dense.push_back({q, c, s, t}); pl.push_back(q);
                B.minx = std::min(B.minx, q.x); B.maxx = std::max(B.maxx, q.x); B.miny = std::min(B.miny, q.y); B.maxy = std::max(B.maxy, q.y);}
        }
        B.poly.push_back(pl);
    }
    return B;
}
inline Foot exactDist(const Boundary& B, Vec2 p, int* bc = nullptr, int* bs = nullptr) {
    Foot best{1e300, {}, 0};
    for (int c = 0; c < (int)B.cs->size(); ++c) for (int s = 0; s < (int)(*B.cs)[c].spans.size(); ++s) {
        Foot f = spanDist((*B.cs)[c].spans[s], p); if (f.d < best.d) { best = f; if (bc) *bc = c; if (bs) *bs = s; }
    }
    return best;
}
inline bool insideEO(const Boundary& B, Vec2 p) {
    bool in = false;
    for (auto& pl : B.poly) { size_t n = pl.size();
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            const Vec2 &a = pl[i], &b = pl[j];
            if ((a.y > p.y) != (b.y > p.y)) { double x = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y); if (x > p.x) in = !in; }
        } }
    return in;
}
// delta(p): (distance to nearest dense sample whose direction differs from the foot direction by >= theta) - r
inline double medialDefect(const Boundary& B, Vec2 p, double r, double chordal) {
    Foot f = exactDist(B, p);
    Vec2 u = f.q - p; double ul = std::hypot(u.x, u.y); if (ul == 0) return 0; u = u / ul;
    double theta = std::min(PI / 2, 2.0 * std::sqrt(2.0 * chordal / std::max(r, 1e-12)));
    double ct = std::cos(theta); double best = 1e300;
    for (auto& s : B.dense) { Vec2 v = s.p - p; double vl = std::hypot(v.x, v.y); if (vl == 0) continue;
        if (v.dot(u) / vl <= ct) best = std::min(best, vl); }
    return best - f.d;
}

// number of runs of dense samples within lim of p (per contour, cyclic)
inline int nearRuns(const Boundary& B, Vec2 p, double lim) {
    int runs = 0; size_t i = 0, n = B.dense.size();
    while (i < n) {
        size_t j = i; int c = B.dense[i].c; while (j < n && B.dense[j].c == c) ++j;
        size_t m = j - i; size_t start = m;
        for (size_t k = 0; k < m; ++k) if (dist(B.dense[i + k].p, p) > lim) { start = k; break; }
        if (start == m) { ++runs; i = j; continue; }
        bool in = false;
        for (size_t k = 1; k <= m; ++k) { bool nr = dist(B.dense[i + (start + k) % m].p, p) <= lim; if (nr && !in) ++runs; in = nr; }
        i = j;
    }
    return runs;
}
inline double medialDefect2(const Boundary& B, Vec2 p, double r, double chordal) {
    double d = medialDefect(B, p, r, chordal);
    if (d <= chordal) return d;
    double rr = exactDist(B, p).d;
    if (nearRuns(B, p, rr + chordal) >= 2) return 0.0;
    return d;
}
}  // namespace vo
