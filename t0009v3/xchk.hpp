// T-0009 round-3 verifier checker (from the round-2 xchk.hpp; round-3 change: owner decision (c)).
//  (c): a run = maximal consecutive spans <= tol around a corner that turns > 135 deg at a span <= tol,
//  all within a box of diagonal 2 tol. Loci of run spans and of corners touching a run are not required;
//  instead the collapsed corner (end of the longer span A before the run, tangents of A and of the
//  longer span B after it) is a required locus (if convex on the requested side), judged with the
//  tolerance-sized slack of (c): valid if >= d - 2 tol from the input and >= d from every span but
//  A, B and the run; covered if the output passes within 3 tol. No run at a chain's open end gets a
//  collapsed corner (the offset ends with A's). Kites: excused only at corners not touching a run and
//  not between two sub-tol spans, and at collapsed corners. A, B offsets are checked as usual.
// (round-2 header follows)
//  distance : exact min distance output-span -> input (analytic critical points + crossing test)
//  round    : rigorous max distance over each output span (Lipschitz-1 branch and bound)
//  coverage : locus of every input span (and convex-corner arcs) sampled; a valid point (>= d from the
//             whole input) uncovered by more than tol is a miss; stretch length measured at tol/16.
//             Sharp/Square: excused only inside the mitre/chamfer kite of a corner that keeps the
//             requested join (owner decision (a): corners between two sub-tol spans are Round).
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <variant>
#include <vector>
#include "carvetto/geometry/offset.hpp"
namespace xk {
using namespace carvetto::geometry;
constexpr double PI2 = 6.283185307179586;
struct S { int k; Vec2 a, b, c; double r = 0, sa = 0, sw = 0, len = 0; double x0, y0, x1, y1; };
inline double wrap(double x) { x = std::fmod(x, PI2); if (x < 0) x += PI2; return x; }
inline Vec2 P(const S& s, double t) { return s.k == 0 ? s.a + (s.b - s.a) * t : s.c + Vec2{std::cos(s.sa + s.sw * t), std::sin(s.sa + s.sw * t)} * s.r; }
inline Vec2 T(const S& s, double t) { if (s.k == 0) return (s.b - s.a).normalized(); double an = s.sa + s.sw * t; Vec2 u{-std::sin(an), std::cos(an)}; return s.sw >= 0 ? u : u * -1.0; }
inline S mk(const Span& sp) {
    S s{};
    if (auto* l = std::get_if<LineSpan>(&sp)) { s.k = 0; s.a = l->start; s.b = l->end; s.len = distance(s.a, s.b); }
    else { auto& a = std::get<ArcSpan>(sp); s.k = 1; s.c = a.centre; s.r = a.radius; s.sa = a.startAngle; s.sw = a.sweepAngle(); s.a = a.startPoint(); s.b = a.endPoint(); s.len = s.r * std::fabs(s.sw); }
    // conservative box
    if (s.k == 0) { s.x0 = std::min(s.a.x, s.b.x); s.x1 = std::max(s.a.x, s.b.x); s.y0 = std::min(s.a.y, s.b.y); s.y1 = std::max(s.a.y, s.b.y); }
    else {
        s.x0 = std::min(s.a.x, s.b.x); s.x1 = std::max(s.a.x, s.b.x); s.y0 = std::min(s.a.y, s.b.y); s.y1 = std::max(s.a.y, s.b.y);
        for (int q = 0; q < 4; ++q) { double an = q * PI2 / 4; Vec2 u{std::cos(an), std::sin(an)}; if (std::fabs(s.sw) >= PI2 - 1e-15 || true) { double rel = s.sw >= 0 ? wrap(an - s.sa) : wrap(s.sa - an); if (rel <= std::fabs(s.sw)) { Vec2 p = s.c + u * s.r; s.x0 = std::min(s.x0, p.x); s.x1 = std::max(s.x1, p.x); s.y0 = std::min(s.y0, p.y); s.y1 = std::max(s.y1, p.y); } } }
        double e = 1e-12 * std::max({1.0, std::fabs(s.c.x), std::fabs(s.c.y), s.r}); s.x0 -= e; s.y0 -= e; s.x1 += e; s.y1 += e;
    }
    return s;
}
inline bool inArc(const S& s, Vec2 dir) {  // direction from centre within sweep (inclusive)
    if (std::fabs(s.sw) >= PI2) return true;
    double an = std::atan2(dir.y, dir.x);
    double rel = s.sw >= 0 ? wrap(an - s.sa) : wrap(s.sa - an);
    return rel <= std::fabs(s.sw) || rel >= PI2 - 1e-15;
}
inline double dPt(const S& s, Vec2 p) {
    if (s.k == 0) { Vec2 ab = s.b - s.a; double L2 = ab.lengthSquared(); double t = L2 > 0 ? std::clamp((p - s.a).dot(ab) / L2, 0.0, 1.0) : 0.0; return distance(p, s.a + ab * t); }
    Vec2 v = p - s.c; double m = v.length();
    double e = std::min(distance(p, s.a), distance(p, s.b));
    if (m > 0 && inArc(s, v)) return std::min(e, std::fabs(m - s.r));
    if (m == 0) return s.r;
    return e;
}
// crossing tests (float, generous: only used to return 0 for real crossings)
inline bool crossLL(const S& A, const S& B) {
    auto o = [](Vec2 a, Vec2 b, Vec2 c) { return (b - a).cross(c - a); };
    double d1 = o(A.a, A.b, B.a), d2 = o(A.a, A.b, B.b), d3 = o(B.a, B.b, A.a), d4 = o(B.a, B.b, A.b);
    return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
}
inline bool crossLA(const S& L, const S& A) {
    Vec2 u = L.b - L.a; double a = u.lengthSquared(); if (a <= 0) return false;
    Vec2 f = L.a - A.c; double b = 2 * u.dot(f), c = f.lengthSquared() - A.r * A.r; double disc = b * b - 4 * a * c;
    if (disc <= 0) return false;
    double rt = std::sqrt(disc);
    for (double t : {(-b - rt) / (2 * a), (-b + rt) / (2 * a)}) if (t > 0 && t < 1) { Vec2 p = L.a + u * t; if (inArc(A, p - A.c)) return true; }
    return false;
}
inline bool crossAA(const S& A, const S& B) {
    Vec2 u = B.c - A.c; double D = u.length(); if (D <= 0) return false;
    if (D >= A.r + B.r || D <= std::fabs(A.r - B.r)) return false;
    double x = (D * D + A.r * A.r - B.r * B.r) / (2 * D); double h2 = A.r * A.r - x * x; if (h2 <= 0) return false;
    double h = std::sqrt(h2); Vec2 e = u / D; Vec2 m = A.c + e * x;
    for (double sg : {-1.0, 1.0}) { Vec2 p = m + e.perp() * (h * sg); if (inArc(A, p - A.c) && inArc(B, p - B.c)) return true; }
    return false;
}
inline double dSS(const S& A, const S& B) {
    double m = std::min({dPt(A, B.a), dPt(A, B.b), dPt(B, A.a), dPt(B, A.b)});
    if (m == 0) return 0;
    if (A.k == 0 && B.k == 0) { if (crossLL(A, B)) return 0; return m; }
    const S& L = A.k == 0 ? A : B; const S& C = A.k == 0 ? B : A;
    if (A.k != B.k) {
        if (crossLA(L, C)) return 0;
        Vec2 u = L.b - L.a; double a = u.lengthSquared();
        if (a > 0) {
            double t = (C.c - L.a).dot(u) / a;
            if (t >= 0 && t <= 1) { Vec2 F = L.a + u * t; Vec2 v = F - C.c; double vl = v.length();
                if (vl > 0) { for (double sg : {-1.0, 1.0}) if (inArc(C, v * sg)) { Vec2 Q = C.c + v * (sg * C.r / vl); m = std::min(m, distance(Q, F)); } }
                else { m = std::min(m, C.r); } }
        }
        return m;
    }
    if (crossAA(A, B)) return 0;
    Vec2 u = B.c - A.c; double D = u.length();
    if (D > 0) { Vec2 e = u / D; for (double sa : {-1.0, 1.0}) for (double sb : {-1.0, 1.0}) if (inArc(A, e * sa) && inArc(B, e * sb)) m = std::min(m, distance(A.c + e * (sa * A.r), B.c + e * (sb * B.r))); }
    return m;
}
struct Grid {
    double cell = 1, ox = 0, oy = 0; int nx = 1, ny = 1; std::vector<std::vector<int>> b; const std::vector<S>* s = nullptr;
    void build(const std::vector<S>& v, double c) {
        s = &v; cell = c; if (v.empty()) return;
        double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
        for (auto& g : v) { x0 = std::min(x0, g.x0); y0 = std::min(y0, g.y0); x1 = std::max(x1, g.x1); y1 = std::max(y1, g.y1); }
        double span = std::max(x1 - x0, y1 - y0); if (span / cell > 2000) cell = span / 2000;
        ox = x0; oy = y0; nx = (int)((x1 - x0) / cell) + 1; ny = (int)((y1 - y0) / cell) + 1;
        b.assign((size_t)nx * ny, {});
        for (int i = 0; i < (int)v.size(); ++i) { auto& g = v[i]; int ax = cx(g.x0), bx = cx(g.x1), ay = cy(g.y0), by = cy(g.y1); for (int y = ay; y <= by; ++y) for (int x = ax; x <= bx; ++x) b[(size_t)y * nx + x].push_back(i); }
    }
    int cx(double x) const { return std::clamp((int)std::floor((x - ox) / cell), 0, nx - 1); }
    int cy(double y) const { return std::clamp((int)std::floor((y - oy) / cell), 0, ny - 1); }
    template <class F> void query(double x0, double y0, double x1, double y1, F f, std::vector<int>& seen, int& stamp) const {
        if (!s || s->empty()) return;
        ++stamp; int ax = cx(x0), bx = cx(x1), ay = cy(y0), by = cy(y1);
        for (int y = ay; y <= by; ++y) for (int x = ax; x <= bx; ++x) for (int i : b[(size_t)y * nx + x]) { if (seen[i] == stamp) continue; seen[i] = stamp; const S& g = (*s)[i]; if (g.x0 > x1 || g.x1 < x0 || g.y0 > y1 || g.y1 < y0) continue; f(i); }
    }
};
struct Res {
    long excess = 0; double worstExcess = 0; long cmissThin = 0; double thinLen = 0; std::string thinFirst, covFirst; long dv = 0, rv = 0, cmiss = 0, cmissTie = 0, cmissLong = 0, unres = 0, covPts = 0;
    double under = 0, over = 0, miss = 0, missLen = 0, missTieLen = 0; std::string first;
    long cSideMiss = 0, cOpenEndMiss = 0; double cSideDout = 0, cOpenEndDout = 0; std::string cSideFirst; Vec2 sideP{0, 0}, covP{0, 0}; bool hasCovP = false; long cExemptLoci = 0, cRuns = 0, cExemptMiss = 0, cCollMiss = 0, cCollPts = 0; double cExemptDout = 0, cCollDout = 0; std::string cFirst;
    void note(const char* s) { if (first.empty()) first = s; }
};
inline double allowance(double d, double tol) { return std::max(tol * tol / (8 * d), std::min(5e-7 * std::max(1.0, d), 0.1 * tol * tol / d)); }
struct Checker {
    struct Cn { Vec2 V, n1, n2; double sw; bool excuse, arcs; };
    mutable std::vector<Cn> cs_; mutable bool csDone_ = false;
    std::vector<S> in, out; Grid gi, go; mutable std::vector<int> si, so; mutable int ti = 0, to = 0;
    struct Run { int A = -1, B = -1; std::vector<int> spans; };
    std::vector<Run> runs_; std::vector<int> runOf_;
    void computeRuns() {
        int n = (int)in.size(); runOf_.assign(n, -1); if (n < 2) return;
        auto nxt = [&](int k) { return k + 1 < n ? k + 1 : (closed ? 0 : -1); };
        auto prv = [&](int k) { return k > 0 ? k - 1 : (closed ? n - 1 : -1); };
        auto tiny = [&](int k) { return in[k].len <= tol; };
        const double lim = 2 * tol * (1 + 1e-9);
        for (int k = 0; k < n; ++k) {
            int nx = nxt(k); if (nx < 0) break;
            if (!tiny(k) && !tiny(nx)) continue;
            if (T(in[k], 1).dot(T(in[nx], 0)) >= -0.70710678118654752) continue;  // turn <= 135 deg
            // grow a run of consecutive tiny spans, box diagonal <= 2 tol
            double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
            auto add = [&](int j, bool commit) { double a0 = std::min(x0, in[j].x0), a1 = std::max(x1, in[j].x1), b0 = std::min(y0, in[j].y0), b1 = std::max(y1, in[j].y1);
                if (std::hypot(a1 - a0, b1 - b0) > lim) return false; if (commit) { x0 = a0; x1 = a1; y0 = b0; y1 = b1; } return true; };
            std::vector<int> sp; int lo = -1, hi = -1;
            if (tiny(k) && add(k, true)) { sp.push_back(k); lo = hi = k; }
            if (tiny(nx) && add(nx, true)) { sp.push_back(nx); if (lo < 0) lo = nx; hi = nx; }
            if (sp.empty()) continue;
            bool grew = true;
            while (grew && (int)sp.size() < n) { grew = false;
                int b = prv(lo); if (b >= 0 && tiny(b) && std::find(sp.begin(), sp.end(), b) == sp.end() && add(b, true)) { sp.insert(sp.begin(), b); lo = b; grew = true; }
                int f = nxt(hi); if (f >= 0 && tiny(f) && std::find(sp.begin(), sp.end(), f) == sp.end() && add(f, true)) { sp.push_back(f); hi = f; grew = true; } }
            // merge with an existing run sharing spans or adjacent
            int target = -1; for (int j : sp) if (runOf_[j] >= 0) target = runOf_[j];
            if (target < 0) { int b = prv(lo), f = nxt(hi); if (b >= 0 && runOf_[b] >= 0) target = runOf_[b]; else if (f >= 0 && runOf_[f] >= 0) target = runOf_[f]; }
            if (target < 0) { target = (int)runs_.size(); runs_.push_back({}); }
            for (int j : sp) if (runOf_[j] != target) { runOf_[j] = target; runs_[target].spans.push_back(j); }
        }
        // A and B of each run: walk to the first span outside the run on either side
        for (int r = 0; r < (int)runs_.size(); ++r) { auto& R = runs_[r]; if (R.spans.empty()) continue; if ((int)R.spans.size() >= n) { R.A = R.B = -1; continue; }
            int s0 = R.spans[0]; int a = s0; while (true) { int b = prv(a); if (b < 0) { a = -1; break; } if (runOf_[b] != r) { a = b; break; } a = b; } 
            int z = s0; while (true) { int f = nxt(z); if (f < 0) { z = -1; break; } if (runOf_[f] != r) { z = f; break; } z = f; }
            R.A = a; R.B = z; }
    }
    bool collapsedCorner(const Run& R, Vec2& V, double& a1, double& sw) const {
        if (R.A < 0 || R.B < 0 || R.A == R.B) return false;
        Vec2 t1 = T(in[R.A], 1), t2 = T(in[R.B], 0); double cr = t1.cross(t2); bool convex = sgn > 0 ? cr < 0 : cr > 0; bool rev = t1.dot(t2) < -0.999999;
        if (!convex) return false;
        Vec2 n1 = t1.perp() * (double)sgn, n2 = t2.perp() * (double)sgn; a1 = std::atan2(n1.y, n1.x);
        sw = std::remainder(std::atan2(n2.y, n2.x) - a1, PI2);
        if (rev) { double ahead = std::remainder(std::atan2(t1.y, t1.x) - a1, PI2); sw = ahead >= 0 ? std::fabs(sw) : -std::fabs(sw); }
        V = in[R.A].b; return true;
    }
    bool runRev(int r) const { const Run& R = runs_[r]; if (R.A < 0 || R.B < 0 || R.A == R.B) return false; return T(in[R.A], 1).dot(T(in[R.B], 0)) < -0.70710678118654752; }
    // decision (c) side test: is q on the requested side of the run collapsed to V = end of A, i.e. in the
    // wedge at V between the ray back along A and the ray forward along B that contains A's offset normal?
    int requestedSide(int r, Vec2 q) const {  // 1 yes, 0 no, -1 undefined (open end / whole path)
        const Run& R = runs_[r]; if (R.A < 0 || R.B < 0 || R.A == R.B) return -1;
        Vec2 V = in[R.A].b, ub = T(in[R.A], 1) * -1.0, uf = T(in[R.B], 0), nA = T(in[R.A], 1).perp() * (double)sgn;
        double sg = ub.cross(nA) > 0 ? 1.0 : -1.0;
        auto ang = [&](Vec2 v) { double a = std::atan2(ub.cross(v), ub.dot(v)) * sg; if (a < 0) a += PI2; return a; };
        return ang(q - V) < ang(uf) ? 1 : 0;
    }
    bool inRun(int k) const { return k >= 0 && k < (int)runOf_.size() && runOf_[k] >= 0; }
    double d, tol, mitre; int sgn; JoinType join; bool closed;
    Checker(const Contour& input, const std::vector<Contour>& o, double d_, OffsetSide side, JoinType j, double tol_, double mitre_) : d(d_), tol(tol_), mitre(mitre_), join(j) {
        sgn = side == OffsetSide::Left ? 1 : -1; closed = input.closed;
        for (auto& sp : input.spans) if (length(sp) > 0) in.push_back(mk(sp));
        for (auto& c : o) for (auto& sp : c.spans) out.push_back(mk(sp));
        gi.build(in, std::max(d / 4, 0.02)); go.build(out, std::max(d / 4, 0.02));
        si.assign(in.size(), 0); so.assign(out.size(), 0);
        computeRuns();
    }
    double fInMask(Vec2 p, double cap, const std::vector<char>& mask) const {
        double best = cap;
        gi.query(p.x - cap, p.y - cap, p.x + cap, p.y + cap, [&](int i) { if (mask[i]) return; best = std::min(best, dPt(in[i], p)); }, si, ti);
        return best;
    }
    double fIn(Vec2 p, double cap, int ex1 = -1, int ex2 = -1) const {
        double best = cap;
        gi.query(p.x - cap, p.y - cap, p.x + cap, p.y + cap, [&](int i) { if (i == ex1 || i == ex2) return; best = std::min(best, dPt(in[i], p)); }, si, ti);
        return best;
    }
    double fOut(Vec2 p, double cap) const {
        double best = cap;
        go.query(p.x - cap, p.y - cap, p.x + cap, p.y + cap, [&](int i) { best = std::min(best, dPt(out[i], p)); }, so, to);
        return best;
    }
    // max over the sub-arc/sub-segment [t0,t1] of o of the distance to point V (exact)
    static double maxToPt(const S& o, double t0, double t1, Vec2 V) {
        double m = std::max(distance(P(o, t0), V), distance(P(o, t1), V));
        if (o.k == 1) { Vec2 w = o.c - V; double wl = w.length(); if (wl > 0) { double an = std::atan2(w.y, w.x); double a0 = o.sa + o.sw * t0, a1 = o.sa + o.sw * t1;
            double rel = o.sw >= 0 ? wrap(an - a0) : wrap(a0 - an); if (rel <= std::fabs(a1 - a0)) m = std::max(m, wl + o.r); } }
        return m;
    }
    // rigorous upper bound of min_i dist(p, in_i) over p in o[t0,t1]
    double ubInterval(const S& o, double t0, double t1, double lim) const {
        Vec2 p0 = P(o, t0), p1 = P(o, t1); double h = (t1 - t0) * o.len;
        S sub = o; if (o.k == 0) { sub.a = p0; sub.b = p1; } else { sub.sa = o.sa + o.sw * t0; sub.sw = o.sw * (t1 - t0); sub.a = p0; sub.b = p1; }
        double x0 = std::min(p0.x, p1.x) - h - lim, x1 = std::max(p0.x, p1.x) + h + lim, y0 = std::min(p0.y, p1.y) - h - lim, y1 = std::max(p0.y, p1.y) + h + lim;
        double best = 1e300;
        gi.query(x0, y0, x1, y1, [&](int i) {
            const S& g = in[i]; double ub;
            double g0 = dPt(g, p0), g1 = dPt(g, p1);
            ub = (g0 + g1 + h) / 2;                                   // Lipschitz
            ub = std::min(ub, maxToPt(o, t0, t1, g.a)); ub = std::min(ub, maxToPt(o, t0, t1, g.b));   // via end points
            if (o.k == 0 && g.k == 0) ub = std::min(ub, std::max(g0, g1));   // convex along a line
            if (o.k == 1 && g.k == 1) {
                double cd = distance(o.c, g.c);
                if (cd <= 1e-9 * std::max({1.0, std::fabs(o.c.x), std::fabs(o.c.y)})) {
                    bool inside = std::fabs(sub.sw) < 3.0;
                    for (int q = 0; q <= 4 && inside; ++q) { Vec2 pp = P(sub, q / 4.0); inside = inArc(g, pp - g.c); }
                    // interval endpoints + 3 interior samples in sweep and sub-sweep < 3 rad: a sub-arc of the same circle direction set lies within g's sweep iff its end angles do and it does not wrap past g's gap; check gap
                    if (inside) { double ga = std::fabs(g.sw); if (ga >= PI2 - 1e-12) ub = std::min(ub, std::fabs(o.r - g.r) + cd);
                        else { Vec2 gm = g.c + Vec2{std::cos(g.sa + g.sw + (PI2 - ga) / 2 * (g.sw >= 0 ? 1 : -1)), std::sin(g.sa + g.sw + (PI2 - ga) / 2 * (g.sw >= 0 ? 1 : -1))};  // middle of g's gap
                               if (!inArc(sub, gm - g.c)) ub = std::min(ub, std::fabs(o.r - g.r) + cd); } }
                }
            }
            best = std::min(best, ub);
        }, si, ti);
        return best * (1 + 1e-15) + 1e-15;
    }
    void run(Res& r) const {
        const double A = allowance(d, tol); char buf[300];
        for (size_t k = 0; k < out.size(); ++k) {
            const S& o = out[k];
            double sl = 1e-12 * std::max({1.0, std::fabs(o.a.x), std::fabs(o.a.y), d});
            // exact min
            double mn = d + 1;
            gi.query(o.x0 - d, o.y0 - d, o.x1 + d, o.y1 + d, [&](int i) { mn = std::min(mn, dSS(o, in[i])); }, si, ti);
            if (d - mn > r.under) r.under = d - mn;
            if (d - mn > A + sl) { r.dv++; snprintf(buf, sizeof buf, "X-dist out %zu under %.3g (A %.3g) k%d len %.3g", k, d - mn, A, o.k, o.len); r.note(buf); }
            if (join != JoinType::Round) {
                // Sharp/Square: every output point beyond d must lie in the mitre/chamfer region of a corner that
                // keeps the requested join (owner decision (a): none at corners between two sub-tol spans)
                if (cs_.empty() && !csDone_) { cs_ = corners(); csDone_ = true; }
                int N = (int)std::clamp(std::ceil(o.len / (0.01 * d)), 8.0, 400.0);
                for (int q = 0; q <= N; ++q) { Vec2 pp = P(o, (double)q / N); double f = fIn(pp, d + 2.0 * std::max(mitre, 1.5) * d);
                    if (f <= d + 1e-6 * std::max(1.0, d)) continue;
                    if (excused(cs_, pp)) continue;
                    r.excess++; r.worstExcess = std::max(r.worstExcess, f - d);
                    if (r.excess == 1) { snprintf(buf, sizeof buf, "X-excess out %zu at (%.17g,%.17g) %.3g beyond d outside every corner region", k, pp.x, pp.y, f - d); r.note(buf); }
                    break; }
                continue;
            }
            // rigorous max (branch and bound, Lipschitz 1 in arclength)
            const double lim = d + A + sl, cap = d + 1;
            int N = (int)std::clamp(std::ceil(o.len / (0.02 * d)), 8.0, 4000.0);
            std::vector<double> fv(N + 1); for (int q = 0; q <= N; ++q) fv[q] = fIn(P(o, (double)q / N), cap);
            double mx = *std::max_element(fv.begin(), fv.end());
            struct I { double t0, t1, f0, f1; }; std::vector<I> st; long evals = 0; bool unres = false;
            for (int q = 0; q < N; ++q) st.push_back({(double)q / N, (double)(q + 1) / N, fv[q], fv[q + 1]});
            while (!st.empty()) {
                I iv = st.back(); st.pop_back();
                double h = (iv.t1 - iv.t0) * o.len;
                if ((iv.f0 + iv.f1 + h) / 2 <= lim) continue;
                if (ubInterval(o, iv.t0, iv.t1, lim) <= lim) continue;
                if (std::max(iv.f0, iv.f1) > lim) continue;  // already recorded via mx
                if (h < 1e-13 * std::max(1.0, d)) continue;
                if (++evals > 300000) { unres = true; break; }
                double tm = (iv.t0 + iv.t1) / 2, fm = fIn(P(o, tm), cap); mx = std::max(mx, fm);
                if (fm > lim) continue;
                st.push_back({iv.t0, tm, iv.f0, fm}); st.push_back({tm, iv.t1, fm, iv.f1});
            }
            if (unres) r.unres++;
            if (mx - d > r.over) r.over = mx - d;
            if (mx > lim) { r.rv++; snprintf(buf, sizeof buf, "X-round out %zu over %.3g k%d len %.3g", k, mx - d, o.k, o.len); r.note(buf); }
        }
        coverage(r);
    }
    // corners: (index k = corner between in[k] and in[k+1])
    std::vector<Cn> corners() const {
        std::vector<Cn> cs; int n = (int)in.size();
        for (int k = 0; k < n; ++k) {
            int nx = k + 1; if (nx == n) { if (!closed) break; nx = 0; }
            Vec2 t1 = T(in[k], 1), t2 = T(in[nx], 0);
            double cr = t1.cross(t2); bool convex = sgn > 0 ? cr < 0 : cr > 0;
            bool rev = t1.dot(t2) < -0.999999;
            if (!convex && !rev) continue;
            Vec2 n1 = t1.perp() * (double)sgn, n2 = t2.perp() * (double)sgn;
            double sw = std::remainder(std::atan2(n2.y, n2.x) - std::atan2(n1.y, n1.x), PI2);
            if (rev) { double ahead = std::remainder(std::atan2(t1.y, t1.x) - std::atan2(n1.y, n1.x), PI2); sw = ahead >= 0 ? std::fabs(sw) : -std::fabs(sw); }
            bool bothTiny = in[k].len <= tol && in[nx].len <= tol;
            // a corner keeps its join unless it lies between two sub-tol spans (decision (a)), is itself a
            // sub-tol near-reversal (decision (c): no join), or touches a run that reverses as a whole
            // (decision (c): the run is joined as one corner, the collapsed corner below)
            bool nearRev = (in[k].len <= tol || in[nx].len <= tol) && t1.dot(t2) < -0.70710678118654752;
            bool inRevRun = (inRun(k) && runRev(runOf_[k])) || (inRun(nx) && runRev(runOf_[nx]));
            cs.push_back({in[k].b, n1, n2, sw, join != JoinType::Round && !bothTiny && !nearRev && !inRevRun, in[k].k == 1 || in[nx].k == 1});
        }
        for (auto& R : runs_) { Vec2 V; double a1, sw; if (!collapsedCorner(R, V, a1, sw)) continue; Vec2 n1{std::cos(a1), std::sin(a1)}, n2{std::cos(a1 + sw), std::sin(a1 + sw)};
            cs.push_back({V, n1, n2, sw, join != JoinType::Round, in[R.A].k == 1 || in[R.B].k == 1}); }
        return cs;
    }
    bool excused(const std::vector<Cn>& cs, Vec2 q) const {
        if (join == JoinType::Round) return false;
        const double lim = std::max(mitre, 1.5);
        for (auto& c : cs) {
            if (!c.excuse) continue;
            Vec2 v = q - c.V; double L = v.length();
            if (L < 1e-15 || L > lim * d * 1.000001 + 1e-9) continue;
            double a1 = std::atan2(c.n1.y, c.n1.x);
            double rel = std::remainder(std::atan2(v.y, v.x) - a1, PI2);
            double sl = c.arcs ? 0.2 : 1e-9;
            bool inW = c.sw >= 0 ? (rel >= -sl && rel <= c.sw + sl) : (rel <= sl && rel >= c.sw - sl);
            if (!inW) continue;
            if (c.arcs) return true;
            double e = 1e-9 * std::max(1.0, d);
            if (v.dot(c.n1) > d + e || v.dot(c.n2) > d + e) continue;   // beyond the offset lines: not in the kite
            Vec2 bis = (c.n1 + c.n2); double bl = bis.length();
            if (bl > 1e-12) { bis = bis / bl; double cap = join == JoinType::Square ? 1.5 * d : mitre * d; /* Square: a chamfer shorter than tol becomes the meeting point (<= 1.5 d) */ if (v.dot(bis) > cap + e) continue; }
            return true;
        }
        return false;
    }
    void coverage(Res& r) const {
        auto cs = corners(); char buf[300];
        const double epsV = 1e-9 * std::max(1.0, d);
        // locus generator: returns q for parameter u in [0,1] of locus j (span or corner) ; ex indices
        struct L { int kind; int k, nx; double len; bool ex = false; };
        std::vector<L> loci; int n = (int)in.size();
        for (int k = 0; k < n; ++k) {
            const S& g = in[k]; double ol = g.len;
            if (g.k == 1) { double rr = g.r + (g.sw >= 0 ? -sgn : sgn) * d; if (rr <= 0) continue; ol = rr * std::fabs(g.sw); }
            if (inRun(k)) { r.cExemptLoci++; loci.push_back({0, k, -1, ol, true}); continue; }
            loci.push_back({0, k, -1, ol});
        }
        std::vector<int> cIdx;
        { int ci = 0; for (int k = 0; k < n; ++k) { int nx = k + 1; if (nx == n) { if (!closed) break; nx = 0; } (void)ci; } }
        auto cornerOf = [&](int k, int nx, Vec2& V, double& a1, double& sw) -> bool {
            Vec2 t1 = T(in[k], 1), t2 = T(in[nx], 0); double cr = t1.cross(t2); bool convex = sgn > 0 ? cr < 0 : cr > 0; bool rev = t1.dot(t2) < -0.999999;
            // coverage loci: only corners convex on the requested side (ADR-0012: a corner arc belongs to the side
            // that produced it, even where it wraps round); an exact reversal (cross 0) is no side's corner.
            if (!convex) return false;
            Vec2 n1 = t1.perp() * (double)sgn, n2 = t2.perp() * (double)sgn; a1 = std::atan2(n1.y, n1.x);
            sw = std::remainder(std::atan2(n2.y, n2.x) - a1, PI2);
            if (rev) { double ahead = std::remainder(std::atan2(t1.y, t1.x) - a1, PI2); sw = ahead >= 0 ? std::fabs(sw) : -std::fabs(sw); }
            V = in[k].b; return true;
        };
        for (int k = 0; k < n; ++k) { int nx = k + 1; if (nx == n) { if (!closed) break; nx = 0; } Vec2 V; double a1, sw; if (cornerOf(k, nx, V, a1, sw)) { if (inRun(k) || inRun(nx)) { r.cExemptLoci++; loci.push_back({1, k, nx, std::fabs(sw) * d, true}); continue; } loci.push_back({1, k, nx, std::fabs(sw) * d}); } }
        r.cRuns += (long)runs_.size();
        for (int ri = 0; ri < (int)runs_.size(); ++ri) { Vec2 V; double a1, sw; if (collapsedCorner(runs_[ri], V, a1, sw)) loci.push_back({2, ri, -1, std::fabs(sw) * d}); }
        auto locPt = [&](const L& l, double u) -> Vec2 {
            if (l.kind == 2) { Vec2 V; double a1, sw; collapsedCorner(runs_[l.k], V, a1, sw); return V + Vec2{std::cos(a1 + sw * u), std::sin(a1 + sw * u)} * d; }
            if (l.kind == 0) { const S& g = in[l.k]; return P(g, u) + T(g, u).perp() * ((double)sgn * d); }
            Vec2 V; double a1, sw; cornerOf(l.k, l.nx, V, a1, sw); return V + Vec2{std::cos(a1 + sw * u), std::sin(a1 + sw * u)} * d;
        };
        // state of a locus point: 0 invalid, 1 covered/excused, 2 miss(tie), 3 miss(core)
        auto state = [&](const L& l, double u, double* dout = nullptr) -> int {
            Vec2 q = locPt(l, u);
            if (l.kind == 2) {  // collapsed corner of a run (decision (c)): tolerance-sized slack
                const Run& R = runs_[l.k];
                if (fIn(q, d + tol) < d - 2 * tol - epsV) return 0;
                std::vector<char> mask(in.size(), 0); mask[R.A] = mask[R.B] = 1; for (int j : R.spans) mask[j] = 1;
                if (fInMask(q, d + tol, mask) < d + epsV) return 0;
                double o = fOut(q, 4 * tol); if (dout) *dout = o;
                if (o <= 3 * tol) return 1;
                if (excused(cs, q)) return 1;
                return 3;
            }
            int e1 = l.k, e2 = l.kind == 1 ? l.nx : -1;
            double all = fIn(q, d + tol);
            if (all < d - epsV) return 0;
            double oth = fIn(q, d + tol, e1, e2);
            if (oth < d + epsV) return 0;
            double o = fOut(q, l.ex ? d + 1.0 : 2 * tol);
            if (dout) *dout = o;
            if (o <= tol) return 1;
            if (excused(cs, q)) return 1;
            return oth < d + tol / 2 ? 2 : 3;
        };
        for (const L& l : loci) {
            int N = (int)std::clamp(std::ceil(l.len / std::max(tol / 2, std::min(d / 8, l.len / 64))), 2.0, 1500.0);
            if (l.kind == 0 && in[l.k].len <= tol) N = 2;
            int prevMiss = -10;
            for (int q = 0; q <= N; ++q) {
                double u = (double)q / N; double o = 0;
                int s = state(l, u, &o); if (s == 0) continue;
                if (l.ex && s >= 2) {
                    int rr = l.kind == 0 ? runOf_[l.k] : (inRun(l.k) ? runOf_[l.k] : runOf_[l.nx]);
                    Vec2 qq = locPt(l, u); int rs = requestedSide(rr, qq);
                    if (rs == 1 && o > 3 * tol) { r.cSideMiss++; if (o > r.cSideDout) { r.cSideDout = o; r.sideP = qq; snprintf(buf, sizeof buf, "(c)-SIDE miss: valid offset point on the requested side of run %d (A %d B %d) at (%.17g,%.17g) dout %.3g locus %s %d", rr, runs_[rr].A, runs_[rr].B, qq.x, qq.y, o, l.kind ? "corner" : "span", l.k); r.cSideFirst = buf; } }
                    if (rs == -1) { r.cOpenEndMiss++; r.cOpenEndDout = std::max(r.cOpenEndDout, o); }
                }
                if (l.ex) { if (s >= 2) { r.cExemptMiss++; if (o > r.cExemptDout) { r.cExemptDout = o; Vec2 qq = locPt(l, u); snprintf(buf, sizeof buf, "(c)-exempt miss at (%.17g,%.17g) dout %.3g locus %s %d", qq.x, qq.y, o, l.kind ? "corner" : "span", l.k); r.cFirst = buf; } } continue; }
                r.covPts++; if (l.kind == 2) r.cCollPts++;
                if (s == 1) continue;
                if (q - prevMiss == 1) { prevMiss = q; continue; }  // same stretch
                prevMiss = q;
                // measure the uncovered stretch around u (step tol/16 in locus length)
                double du = l.len > 0 ? (tol / 16) / l.len : 1; double len = 0;
                for (int dir = -1; dir <= 1; dir += 2) for (double v = u + dir * du; v >= 0 && v <= 1; v += dir * du) { int z = state(l, v); if (z < 2) break; len += tol / 16; if (len > 50) break; }
                // thickness of the region outside the d-neighbourhood at q, along the outward normal
                Vec2 qq = locPt(l, u);
                Vec2 nn; if (l.kind == 2) { Vec2 V; double a1, sw; collapsedCorner(runs_[l.k], V, a1, sw); nn = (qq - V).normalized(); } else if (l.kind == 0) nn = T(in[l.k], u).perp() * (double)sgn; else { Vec2 V; double a1, sw; cornerOf(l.k, l.nx, V, a1, sw); nn = (qq - V).normalized(); }
                // (collapsed loci may lie up to 2 tol inside the neighbourhood: measure from 2 tol outward)
                Vec2 q0 = l.kind == 2 ? qq + nn * (2 * tol) : qq;
                double th = 0; { const int M = 128; for (int z = 1; z <= M; ++z) { double sd = 2 * tol * z / M; if (fIn(q0 + nn * sd, d + 5 * tol) < d - epsV) break; th = sd; } }
                bool thin = th < tol;
                bool tie = s == 2;
                if (thin) { r.cmissThin++; r.thinLen = std::max(r.thinLen, len); }
                else if (tie) { r.cmissTie++; r.missTieLen = std::max(r.missTieLen, len); }
                else { r.cmiss++; r.miss = std::max(r.miss, o); r.missLen = std::max(r.missLen, len); }
                if (!thin && len > tol) r.cmissLong++;
                snprintf(buf, sizeof buf, "X-cov%s at (%.17g,%.17g) dout %.3g stretch %.3g thick %.3g locus %s %d", thin ? "THIN" : tie ? "TIE" : "", qq.x, qq.y, o, len, th, l.kind == 2 ? "collapsed" : l.kind ? "corner" : "span", l.k);
                if (l.kind == 2 && !thin) { r.cCollMiss++; r.cCollDout = std::max(r.cCollDout, o); }
                if (!thin) { r.note(buf); if (r.covFirst.empty()) { r.covFirst = buf; r.covP = qq; r.hasCovP = true; } } else if (r.thinFirst.empty()) r.thinFirst = buf;
            }
        }
    }
};
}  // namespace xk
