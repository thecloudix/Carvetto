// Independent T-0006 verification driver (throwaway; not committed).
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <functional>
#include <string>
#include <variant>
#include <vector>

#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/intersect.hpp"
#include "pw.hpp"
#include "offset_chain_checks.hpp"

using namespace carvetto::geometry;

// ---------------------------------------------------------------- RNG (PCG-ish, own seeds)
struct Rng {
    std::uint64_t s;
    explicit Rng(std::uint64_t seed) : s(seed * 0x9E3779B97F4A7C15ULL + 0xD1B54A32D192ED03ULL) {}
    std::uint64_t next() {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        std::uint64_t x = s;
        x ^= x >> 31; x *= 0xBF58476D1CE4E5B9ULL; x ^= x >> 29; x *= 0x94D049BB133111EBULL; x ^= x >> 32;
        return x;
    }
    double uni() { return static_cast<double>(next() >> 11) * 0x1.0p-53; }
    double range(double a, double b) { return a + (b - a) * uni(); }
    int irange(int a, int b) { return a + static_cast<int>(next() % static_cast<std::uint64_t>(b - a + 1)); }
};

// ---------------------------------------------------------------- generators
struct Builder {
    Contour c;
    Vec2 p{0, 0};
    double h = 0;
    void line(double len) {
        Vec2 e = p + Vec2::fromAngle(h, len);
        c.spans.push_back(LineSpan{p, e});
        p = e;
    }
    // sweep signed: >0 CCW (left turn)
    void arc(double r, double sweep) {
        Vec2 left{-std::sin(h), std::cos(h)};
        Vec2 ctr = sweep > 0 ? p + left * r : p - left * r;
        double sa = (p - ctr).angle();
        ArcSpan a{ctr, r, sa, sa + sweep, sweep > 0 ? ArcDirection::CCW : ArcDirection::CW};
        // make start exactly p for continuity: p is approximately on it; use arc's own start
        if (!c.spans.empty()) {
            // replace previous end by arc start to make input exactly continuous
            Vec2 st = a.startPoint();
            if (auto* l = std::get_if<LineSpan>(&c.spans.back())) l->end = st;
            else {
                // previous is an arc: fall back to inserting nothing; tiny mismatch (~1e-16)
            }
        }
        c.spans.push_back(a);
        p = a.endPoint();
        h += sweep;
    }
};

Contour walk(Rng& g, int n, double step, double maxTurn) {
    Builder b;
    b.h = g.range(-kPi, kPi);
    for (int i = 0; i < n; ++i) {
        b.line(step);
        b.h += g.range(-maxTurn, maxTurn);
    }
    return b.c;
}

Contour mixed(Rng& g, int n, double scale) {
    Builder b;
    b.h = g.range(-kPi, kPi);
    for (int i = 0; i < n; ++i) {
        if (i > 0 && g.uni() > 0.4) b.h += g.range(-3.0, 3.0);
        if (g.uni() < 0.5) b.line(scale * g.range(0.05, 3.0));
        else {
            double r = scale * std::exp(g.range(std::log(0.03), std::log(5.0)));
            double sw = g.range(0.1, 4.5) * (g.uni() < 0.5 ? -1 : 1);
            b.arc(r, sw);
        }
    }
    return b.c;
}

Contour hypotrochoid(int n) {
    std::vector<Vec2> pts;
    const double R = 5, r = 3, dd = 5, T = 6 * kPi;
    for (int i = 0; i < n; ++i) {
        double t = T * i / n;
        pts.push_back(Vec2{(R - r) * std::cos(t) + dd * std::cos((R - r) / r * t),
                           (R - r) * std::sin(t) - dd * std::sin((R - r) / r * t)} * 4.0);
    }
    Contour c;
    for (int i = 0; i < n; ++i) c.spans.push_back(LineSpan{pts[i], pts[(i + 1) % n]});
    c.closed = true;
    return c;
}

Contour textLoops(int n) {  // cursive-like prolate cycloid, open
    Contour c;
    const double T = 80 * kPi;
    Vec2 prev;
    for (int i = 0; i <= n; ++i) {
        double t = T * i / n;
        Vec2 q{0.6 * t - 1.5 * std::sin(t), 1.5 * std::cos(t) + 0.3 * std::sin(0.37 * t)};
        if (i > 0) c.spans.push_back(LineSpan{prev, q});
        prev = q;
    }
    return c;
}

Contour zigzag(int n, double pitch, double amp) {
    Contour c;
    Vec2 prev{0, 0};
    for (int i = 1; i <= n; ++i) {
        Vec2 q{i * pitch, (i % 2) * amp};
        c.spans.push_back(LineSpan{prev, q});
        prev = q;
    }
    return c;
}

Contour arcSpiral(int n, double pitch) {  // semicircles alternating centres
    Builder b;
    b.h = kPi / 2;  // heading up at (0,0)... start
    double r = pitch / 2;
    for (int i = 0; i < n; ++i) {
        b.arc(r, kPi);
        r += pitch / 2;
    }
    return b.c;
}

Contour lineSpiral(int n, double pitch) {
    Contour c;
    Vec2 prev{0, 0};
    for (int i = 1; i <= n; ++i) {
        double t = 0.15 * i;
        double rr = pitch * t / kTwoPi;
        Vec2 q{rr * std::cos(t), rr * std::sin(t)};
        c.spans.push_back(LineSpan{prev, q});
        prev = q;
    }
    return c;
}

void translate(Contour& c, Vec2 t) {
    for (Span& s : c.spans) {
        if (auto* l = std::get_if<LineSpan>(&s)) { l->start += t; l->end += t; }
        else if (auto* a = std::get_if<ArcSpan>(&s)) {
            Vec2 st = a->startPoint();
            a->centre += t;
            (void)st;
        }
    }
    // re-stitch line ends to arc points so the input is exactly continuous where lines are involved
    for (std::size_t i = 1; i < c.spans.size(); ++i) {
        Vec2 e = endPoint(c.spans[i - 1]);
        if (auto* l = std::get_if<LineSpan>(&c.spans[i])) l->start = e;
        else if (auto* l2 = std::get_if<LineSpan>(&c.spans[i - 1])) l2->end = startPoint(c.spans[i]);
    }
}

// ---------------------------------------------------------------- exact primitives
struct Seg {
    int kind;  // 0 line, 1 arc
    Vec2 a, b;  // endpoints
    Vec2 c;
    double r = 0, sa = 0, sw = 0;
    Box2 box;
};
Seg toSeg(const Span& s) {
    Seg g{};
    if (auto* l = std::get_if<LineSpan>(&s)) { g.kind = 0; g.a = l->start; g.b = l->end; }
    else if (auto* a = std::get_if<ArcSpan>(&s)) {
        g.kind = 1; g.c = a->centre; g.r = a->radius; g.sa = a->startAngle; g.sw = a->sweepAngle();
        g.a = a->startPoint(); g.b = a->endPoint();
    } else { g.kind = 2; g.a = startPoint(s); g.b = endPoint(s); }
    g.box = bounds(s);
    return g;
}
static double wrap2pi(double x) { x = std::fmod(x, kTwoPi); if (x < 0) x += kTwoPi; return x; }
bool inSweep(const Seg& g, double ang) {
    if (g.sw > 0) return wrap2pi(ang - g.sa) <= g.sw;
    return wrap2pi(g.sa - ang) <= -g.sw;
}
Vec2 segPoint(const Seg& g, double t) {
    if (g.kind == 0) return lerp(g.a, g.b, t);
    return g.c + Vec2::fromAngle(g.sa + t * g.sw, g.r);
}
Vec2 segTan(const Seg& g, double t) {
    if (g.kind == 0) return (g.b - g.a).normalized();
    Vec2 u = Vec2::fromAngle(g.sa + t * g.sw);
    return g.sw > 0 ? u.perp() : -u.perp();
}
double segLen(const Seg& g) { return g.kind == 0 ? distance(g.a, g.b) : g.r * std::fabs(g.sw); }
double distPt(const Seg& g, Vec2 p) {
    if (g.kind == 0) {
        Vec2 ab = g.b - g.a;
        double L2 = ab.lengthSquared();
        double t = L2 > 0 ? std::clamp((p - g.a).dot(ab) / L2, 0.0, 1.0) : 0.0;
        return distance(p, g.a + ab * t);
    }
    Vec2 v = p - g.c;
    double ang = std::atan2(v.y, v.x);
    if (inSweep(g, ang)) return std::fabs(v.length() - g.r);
    return std::min(distance(p, g.a), distance(p, g.b));
}

struct Grid {
    std::vector<Seg> segs;
    double h = 1;
    Vec2 o;
    long nx = 1, ny = 1;
    std::vector<std::vector<int>> cells;
    void build(std::vector<Seg> s, double cell) {
        segs = std::move(s);
        Box2 bb;
        bool first = true;
        double lox = 0, loy = 0, hix = 0, hiy = 0;
        for (auto& g : segs) {
            if (first) { lox = g.box.min.x; loy = g.box.min.y; hix = g.box.max.x; hiy = g.box.max.y; first = false; }
            lox = std::min(lox, g.box.min.x); loy = std::min(loy, g.box.min.y);
            hix = std::max(hix, g.box.max.x); hiy = std::max(hiy, g.box.max.y);
        }
        (void)bb;
        double diag = std::hypot(hix - lox, hiy - loy);
        h = std::max(cell, diag / 700.0);
        if (h <= 0) h = 1;
        o = {lox - h, loy - h};
        nx = static_cast<long>((hix - lox) / h) + 3;
        ny = static_cast<long>((hiy - loy) / h) + 3;
        cells.assign(static_cast<std::size_t>(nx * ny), {});
        for (int i = 0; i < static_cast<int>(segs.size()); ++i) {
            const Box2& b = segs[i].box;
            long x0 = cx(b.min.x), x1 = cx(b.max.x), y0 = cy(b.min.y), y1 = cy(b.max.y);
            for (long y = y0; y <= y1; ++y)
                for (long x = x0; x <= x1; ++x) cells[static_cast<std::size_t>(y * nx + x)].push_back(i);
        }
    }
    long cx(double x) const { return std::clamp(static_cast<long>(std::floor((x - o.x) / h)), 0L, nx - 1); }
    long cy(double y) const { return std::clamp(static_cast<long>(std::floor((y - o.y) / h)), 0L, ny - 1); }
    template <class F>
    void forBox(double lx, double ly, double hx, double hy, F f) const {
        long x0 = cx(lx), x1 = cx(hx), y0 = cy(ly), y1 = cy(hy);
        for (long y = y0; y <= y1; ++y)
            for (long x = x0; x <= x1; ++x)
                for (int i : cells[static_cast<std::size_t>(y * nx + x)]) f(i);
    }
    double minDist(Vec2 p, double R, int ex1 = -1, int ex2 = -1) const {
        double best = R;
        forBox(p.x - R, p.y - R, p.x + R, p.y + R, [&](int i) {
            if (i == ex1 || i == ex2) return;
            const Box2& b = segs[i].box;
            if (p.x < b.min.x - best || p.x > b.max.x + best || p.y < b.min.y - best || p.y > b.max.y + best) return;
            best = std::min(best, distPt(segs[i], p));
        });
        return best;
    }
};

// ---------------------------------------------------------------- intersections (exact-ish, analytic)
// returns points of intersection/contact (within eps) between two segs
void intersect(const Seg& A, const Seg& B, double eps, std::vector<Vec2>& out) {
    if (A.kind == 0 && B.kind == 0) {
        Vec2 r = A.b - A.a, s = B.b - B.a;
        double den = r.cross(s);
        Vec2 qp = B.a - A.a;
        double lr = r.length(), ls = s.length();
        if (std::fabs(den) <= 1e-14 * lr * ls) {
            // parallel: collinear?
            if (lr == 0 || std::fabs(qp.cross(r)) / lr > eps) return;
            double t0 = qp.dot(r) / (lr * lr), t1 = (B.b - A.a).dot(r) / (lr * lr);
            double lo = std::max(0.0, std::min(t0, t1)), hi = std::min(1.0, std::max(t0, t1));
            if (hi >= lo - eps / lr) { out.push_back(A.a + r * lo); if (hi > lo) out.push_back(A.a + r * hi); }
            return;
        }
        double t = qp.cross(s) / den, u = qp.cross(r) / den;
        double et = eps / lr, eu = eps / ls;
        if (t >= -et && t <= 1 + et && u >= -eu && u <= 1 + eu) out.push_back(A.a + r * std::clamp(t, 0.0, 1.0));
        return;
    }
    if (A.kind == 1 && B.kind == 0) { intersect(B, A, eps, out); return; }
    if (A.kind == 0 && B.kind == 1) {
        Vec2 dvec = A.b - A.a, f = A.a - B.c;
        double a = dvec.dot(dvec), bq = 2 * f.dot(dvec), c = f.dot(f) - B.r * B.r;
        double disc = bq * bq - 4 * a * c;
        double L = std::sqrt(a);
        std::vector<double> ts;
        // closest approach of the line to the centre
        double tm = -bq / (2 * a);
        Vec2 pm = A.a + dvec * tm;
        double dm = distance(pm, B.c);
        if (disc < 0) {
            if (B.r - dm <= eps && dm - B.r <= eps) ts.push_back(tm);
        } else {
            double sq = std::sqrt(disc);
            ts.push_back((-bq - sq) / (2 * a));
            ts.push_back((-bq + sq) / (2 * a));
        }
        for (double t : ts) {
            if (t < -eps / L || t > 1 + eps / L) continue;
            Vec2 p = A.a + dvec * std::clamp(t, 0.0, 1.0);
            Vec2 v = p - B.c;
            if (inSweep(B, std::atan2(v.y, v.x)) || distance(p, B.a) <= eps || distance(p, B.b) <= eps) out.push_back(p);
        }
        return;
    }
    // arc-arc
    Vec2 dc = B.c - A.c;
    double D = dc.length();
    if (D <= 1e-12 * std::max(1.0, A.c.length()) && std::fabs(A.r - B.r) <= eps) {
        // same circle: overlap if any endpoint of one lies in the other
        Vec2 pts[4] = {A.a, A.b, B.a, B.b};
        for (int k = 0; k < 2; ++k) { Vec2 v = pts[k] - B.c; if (inSweep(B, std::atan2(v.y, v.x))) out.push_back(pts[k]); }
        for (int k = 2; k < 4; ++k) { Vec2 v = pts[k] - A.c; if (inSweep(A, std::atan2(v.y, v.x))) out.push_back(pts[k]); }
        // also full overlap of interiors: midpoint test
        Vec2 m = segPoint(A, 0.5); Vec2 v = m - B.c;
        if (inSweep(B, std::atan2(v.y, v.x))) out.push_back(m);
        return;
    }
    if (D == 0) return;
    if (D > A.r + B.r + eps || D < std::fabs(A.r - B.r) - eps) return;
    double a = (A.r * A.r - B.r * B.r + D * D) / (2 * D);
    double h2 = A.r * A.r - a * a;
    double hh = h2 > 0 ? std::sqrt(h2) : 0.0;
    Vec2 u = dc / D;
    Vec2 base = A.c + u * a;
    Vec2 cands[2] = {base + u.perp() * hh, base - u.perp() * hh};
    int nc = hh > 0 ? 2 : 1;
    for (int k = 0; k < nc; ++k) {
        Vec2 p = cands[k];
        Vec2 va = p - A.c, vb = p - B.c;
        bool ina = inSweep(A, std::atan2(va.y, va.x)) || distance(p, A.a) <= eps || distance(p, A.b) <= eps;
        bool inb = inSweep(B, std::atan2(vb.y, vb.x)) || distance(p, B.a) <= eps || distance(p, B.b) <= eps;
        if (ina && inb) out.push_back(p);
    }
}

// Ray q + s n, s in (smin, L]; true if hits a seg
bool rayHits(const Grid& G, Vec2 q, Vec2 n, double L) {
    Vec2 e = q + n * L;
    Seg ray{};
    ray.kind = 0;
    ray.a = q + n * 1e-9;
    ray.b = e;
    bool hit = false;
    std::vector<Vec2> pts;
    G.forBox(std::min(q.x, e.x), std::min(q.y, e.y), std::max(q.x, e.x), std::max(q.y, e.y), [&](int i) {
        if (hit) return;
        pts.clear();
        intersect(ray, G.segs[i], 1e-12, pts);
        if (!pts.empty()) hit = true;
    });
    return hit;
}

// ---------------------------------------------------------------- checker
struct Metrics {
    int chains = 0, spans = 0;
    int lineJunctionNotBitwise = 0;  // line involved, not bit-identical
    int arcGapBeyond = 0;            // arc-arc gap > 1e-12*scale
    double maxArcGapRel = 0;         // max gap / scale
    int reversals = 0;
    int zeroLen = 0;      // length <= 1e-12
    int subTolAtEnd = 0;  // sub-tol span at an open chain end
    int badSpan = 0;      // NaN, radius <= 0, bezier
    int distViol = 0;     // < d - A
    double worstUnder = 0;  // max (d - dist)
    int roundViol = 0;    // Round: > d + A
    double worstOver = 0;
    int selfX = 0, selfXNearJ = 0;
    int covSamples = 0, covMiss = 0, covMissTol = 0;
    double worstMiss = 0;
    double cpu = 0;
    int overlapIn = 0, overlapX = 0, shortBack = 0, coderBack = 0, hardRev = 0;
    std::string firstIssue;
};

double allowance(double d, double tol) {
    return std::max(tol * tol / (8 * d), std::min(5e-7 * std::max(1.0, d), 0.1 * tol * tol / d));
}

void note(Metrics& m, const std::string& s) { if (m.firstIssue.empty()) m.firstIssue = s; }

void check(const Contour& in, const std::vector<Contour>& out, double d, OffsetSide side, JoinType join, double tol,
           double mitre, Metrics& m, bool doCoverage = true, bool doDist = true) {
    char buf[256];
    std::vector<Seg> inSegs;
    for (auto& s : in.spans) inSegs.push_back(toSeg(s));
    Grid IG;
    IG.build(inSegs, std::max(d / 2, 0.05));
    const double A = allowance(d, tol);
    std::vector<Seg> outSegs;
    m.chains = static_cast<int>(out.size());
    for (std::size_t ci = 0; ci < out.size(); ++ci) {
        const Contour& c = out[ci];
        const std::size_t n = c.spans.size();
        m.spans += static_cast<int>(n);
        std::vector<Seg> cs;
        for (std::size_t i = 0; i < n; ++i) {
            const Span& s = c.spans[i];
            Seg g = toSeg(s);
            if (g.kind == 2 || !std::isfinite(g.a.x) || !std::isfinite(g.b.x) || (g.kind == 1 && !(g.r > 0))) {
                m.badSpan++; note(m, "bad span");
            }
            double len = segLen(g);
            if (len <= 1e-12) { m.zeroLen++; snprintf(buf, sizeof buf, "zero-length span chain %zu span %zu len %.3g", ci, i, len); note(m, buf); }
            if (len <= tol && !c.closed && (i == 0 || i + 1 == n) && n > 1) {
                m.subTolAtEnd++; snprintf(buf, sizeof buf, "sub-tol span at open chain end ch %zu sp %zu len %.3g", ci, i, len); note(m, buf);
            }
            cs.push_back(g);
        }
        // junctions
        for (std::size_t j = 0; j < n; ++j) {
            if (j == 0 && !c.closed) continue;
            const std::size_t pi = (j + n - 1) % n;
            if (n == 1 && !c.closed) continue;
            Vec2 P = endPoint(c.spans[pi]), Q = startPoint(c.spans[j]);
            bool lineInv = cs[pi].kind == 0 || cs[j].kind == 0;
            if (lineInv) {
                if (!(P == Q)) { m.lineJunctionNotBitwise++; snprintf(buf, sizeof buf, "line junction not bitwise ch %zu j %zu gap %.3g", ci, j, distance(P, Q)); note(m, buf); }
            } else {
                double scale = std::max({1.0, std::fabs(P.x), std::fabs(P.y), cs[pi].r, cs[j].r});
                double g = distance(P, Q) / scale;
                m.maxArcGapRel = std::max(m.maxArcGapRel, g);
                if (g > 1e-12) { m.arcGapBeyond++; snprintf(buf, sizeof buf, "arc-arc gap ch %zu j %zu rel %.3g", ci, j, g); note(m, buf); }
            }
            double dt = segTan(cs[pi], 1).dot(segTan(cs[j], 0));
            if (dt < -0.999999) m.hardRev++;
            if (dt < -0.9999) { m.reversals++; snprintf(buf, sizeof buf, "reversal ch %zu j %zu dot %.6f", ci, j, dt); note(m, buf); }
        }
        // self intersection
        std::vector<int> ord(n);
        for (std::size_t i = 0; i < n; ++i) ord[i] = static_cast<int>(i);
        std::sort(ord.begin(), ord.end(), [&](int a, int b) { return cs[a].box.min.x < cs[b].box.min.x; });
        std::vector<Vec2> pts;
        for (std::size_t ia = 0; ia < n; ++ia) {
            int i = ord[ia];
            for (std::size_t ib = ia + 1; ib < n; ++ib) {
                int k = ord[ib];
                if (cs[k].box.min.x > cs[i].box.max.x + 1e-9) break;
                if (cs[k].box.min.y > cs[i].box.max.y + 1e-9 || cs[k].box.max.y < cs[i].box.min.y - 1e-9) continue;
                int lo = std::min(i, k), hi = std::max(i, k);
                bool adj = (hi == lo + 1) || (c.closed && lo == 0 && hi == static_cast<int>(n) - 1);
                pts.clear();
                intersect(cs[lo], cs[hi], 1e-10, pts);
                for (Vec2 p : pts) {
                    // ignore the shared junction(s) of adjacent spans
                    if (adj) {
                        Vec2 J = (hi == lo + 1) ? cs[lo].b : cs[hi].b;
                        if (distance(p, J) <= 1e-9 * std::max(1.0, J.length())) continue;
                        if (n == 2 && c.closed && (distance(p, cs[lo].a) <= 1e-9 * std::max(1.0, p.length()))) continue;
                    }
                    // near a junction separated by short spans?
                    bool nearJ = false;
                    if (hi - lo <= 3 || (c.closed && static_cast<int>(n) - hi + lo <= 3)) {
                        for (int q = 0; q < static_cast<int>(n); ++q)
                            if (distance(p, cs[q].a) <= 1e-9 * std::max(1.0, p.length())) nearJ = true;
                    }
                    if (nearJ) m.selfXNearJ++;
                    else { m.selfX++; snprintf(buf, sizeof buf, "self-x ch %zu spans %d,%d at (%.9g,%.9g)", ci, lo, hi, p.x, p.y); note(m, buf);
                        if (std::getenv("SXV")) { auto r = carvetto::geometry::intersect(c.spans[lo], c.spans[hi], Tolerance{1e-12, 1e-9, 0.01}); std::printf("SX %s n=%zu closed=%d | lib intersect (tol 1e-12): %zu hits", buf, n, (int)c.closed, r.points.size()); for (auto& h : r.points) std::printf(" (%.17g,%.17g tA %.3g tB %.3g tan %d)", h.point.x, h.point.y, h.tA, h.tB, h.tangent); std::printf(" | spans:"); for (int q : {lo, hi}) { auto& sp = c.spans[q]; if (auto* l = std::get_if<LineSpan>(&sp)) std::printf(" L(%.17g,%.17g)->(%.17g,%.17g)", l->start.x, l->start.y, l->end.x, l->end.y); else if (auto* a = std::get_if<ArcSpan>(&sp)) std::printf(" A c(%.17g,%.17g) r %.17g sa %.17g ea %.17g %s", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, a->direction == ArcDirection::CCW ? "CCW" : "CW"); } std::printf("\n"); } }
                }
            }
        }
        for (auto& g : cs) outSegs.push_back(g);
    }
    for (const Contour& c : out) { std::string w; int k = (int)carvetto::geometry::testing::countBackwardSteps(c, Tolerance{tol, 1e-9, 0.01}, &w); if (k) { m.coderBack += k; note(m, "coderChecker: " + w); } }
    // ---- overlap check (any two output spans sharing positive length on one carrier) and short-span-back
    {
        struct OS { Seg g; int ch, sp; };
        std::vector<OS> all;
        for (std::size_t ci = 0; ci < out.size(); ++ci)
            for (std::size_t i = 0; i < out[ci].spans.size(); ++i) all.push_back({toSeg(out[ci].spans[i]), (int)ci, (int)i});
        std::vector<int> ord(all.size());
        for (std::size_t i = 0; i < all.size(); ++i) ord[i] = (int)i;
        std::sort(ord.begin(), ord.end(), [&](int a, int b) { return all[a].g.box.min.x < all[b].g.box.min.x; });
        for (std::size_t ia = 0; ia < ord.size(); ++ia) {
            const OS& A = all[ord[ia]];
            const double sc = std::max({1.0, std::fabs(A.g.a.x), std::fabs(A.g.a.y), d});
            const double e = 1e-9 * sc;
            for (std::size_t ib = ia + 1; ib < ord.size(); ++ib) {
                const OS& B = all[ord[ib]];
                if (B.g.box.min.x > A.g.box.max.x + e) break;
                if (B.g.box.min.y > A.g.box.max.y + e || B.g.box.max.y < A.g.box.min.y - e) continue;
                if (A.g.kind != B.g.kind) continue;
                double ov = 0;
                if (A.g.kind == 0) {
                    double LA = distance(A.g.a, A.g.b);
                    if (LA <= 0) continue;
                    Vec2 u = (A.g.b - A.g.a) / LA, nn = u.perp();
                    if (std::fabs((B.g.a - A.g.a).dot(nn)) > e || std::fabs((B.g.b - A.g.a).dot(nn)) > e) continue;
                    double s0 = (B.g.a - A.g.a).dot(u), s1 = (B.g.b - A.g.a).dot(u);
                    ov = std::min(LA, std::max(s0, s1)) - std::max(0.0, std::min(s0, s1));
                } else if (A.g.kind == 1) {
                    if (distance(A.g.c, B.g.c) > e || std::fabs(A.g.r - B.g.r) > e) continue;
                    int cnt = 0;
                    for (int k = 0; k < 32; ++k) {
                        Vec2 q = segPoint(B.g, (k + 0.5) / 32);
                        double ang = (q - A.g.c).angle();
                        // strict inside with margin
                        double m = 2e-9 * sc / A.g.r;
                        double rel = A.g.sw > 0 ? wrap2pi(ang - A.g.sa) : wrap2pi(A.g.sa - ang);
                        if (rel > m && rel < std::fabs(A.g.sw) - m) cnt++;
                    }
                    for (int k = 0; k < 32 && cnt == 0; ++k) {
                        Vec2 q = segPoint(A.g, (k + 0.5) / 32);
                        double ang = (q - B.g.c).angle();
                        double m = 2e-9 * sc / B.g.r;
                        double rel = B.g.sw > 0 ? wrap2pi(ang - B.g.sa) : wrap2pi(B.g.sa - ang);
                        if (rel > m && rel < std::fabs(B.g.sw) - m) cnt++;
                    }
                    ov = cnt ? 1.0 : 0.0;
                }
                if (ov > e) {
                    if (A.ch == B.ch) m.overlapIn++; else m.overlapX++;
                    snprintf(buf, sizeof buf, "overlap ch%d sp%d / ch%d sp%d kind %d ov %.3g", A.ch, A.sp, B.ch, B.sp, A.g.kind, ov);
                    note(m, buf);
                }
            }
        }
        for (std::size_t ci = 0; ci < out.size(); ++ci) {
            const Contour& c = out[ci];
            const std::size_t n = c.spans.size();
            for (std::size_t i = 0; i < n; ++i) {
                bool hp = c.closed ? n > 2 : i > 0, hn = c.closed ? n > 2 : i + 1 < n;
                if (!hp || !hn) continue;
                Seg P = toSeg(c.spans[(i + n - 1) % n]), S = toSeg(c.spans[i]), N = toSeg(c.spans[(i + 1) % n]);
                if (segLen(S) > tol) continue;
                Vec2 ch = S.b - S.a, ti = segTan(P, 1), to = segTan(N, 0);
                if (ti.dot(to) > 0.5 && ch.dot(ti) < 0 && ch.dot(to) < 0) {
                    m.shortBack++;
                    snprintf(buf, sizeof buf, "short span runs back ch %zu sp %zu len %.3g", ci, i, segLen(S));
                    note(m, buf);
                }
            }
        }
    }
    // distance property
    if (doDist) {
        const double R = d + 1.0;
        for (std::size_t si = 0; si < outSegs.size(); ++si) {
            const Seg& g = outSegs[si];
            if (g.kind == 2) continue;
            double len = segLen(g);
            int ns = static_cast<int>(std::clamp(std::ceil(len / (0.05 * d)), 8.0, 200.0));
            auto f = [&](double t) { return IG.minDist(segPoint(g, t), R); };
            std::vector<double> v(static_cast<std::size_t>(ns + 1));
            for (int k = 0; k <= ns; ++k) v[static_cast<std::size_t>(k)] = f(static_cast<double>(k) / ns);
            auto refine = [&](int k, bool minimise) {
                double a = std::max(0, k - 1) / static_cast<double>(ns), b = std::min(ns, k + 1) / static_cast<double>(ns);
                const double gr = 0.6180339887498949;
                double x1 = b - gr * (b - a), x2 = a + gr * (b - a);
                double f1 = f(x1), f2 = f(x2);
                for (int it = 0; it < 60; ++it) {
                    bool left = minimise ? (f1 < f2) : (f1 > f2);
                    if (left) { b = x2; x2 = x1; f2 = f1; x1 = b - gr * (b - a); f1 = f(x1); }
                    else { a = x1; x1 = x2; f1 = f2; x2 = a + gr * (b - a); f2 = f(x2); }
                }
                return minimise ? std::min({f1, f2, v[static_cast<std::size_t>(k)]}) : std::max({f1, f2, v[static_cast<std::size_t>(k)]});
            };
            int kmin = static_cast<int>(std::min_element(v.begin(), v.end()) - v.begin());
            double dmin = refine(kmin, true);
            double under = d - dmin;
            if (under > m.worstUnder) m.worstUnder = under;
            double slack = 1e-12 * std::max({1.0, std::fabs(g.a.x), std::fabs(g.a.y), d});
            if (under > A + slack) {
                m.distViol++;
                snprintf(buf, sizeof buf, "distance violation span %zu under %.3g (allow %.3g) kind %d len %.3g", si, under, A, g.kind, len);
                note(m, buf);
            }
            if (join == JoinType::Round) {
                int kmax = static_cast<int>(std::max_element(v.begin(), v.end()) - v.begin());
                double dmax = refine(kmax, false);
                double over = dmax - d;
                if (over > m.worstOver) m.worstOver = over;
                if (over > A + slack) {
                    m.roundViol++;
                    snprintf(buf, sizeof buf, "round over-distance span %zu over %.3g kind %d len %.3g", si, over, g.kind, len);
                    note(m, buf);
                }
            }
        }
    }
    // coverage
    if (doCoverage) {
        Grid OG;
        if (outSegs.empty()) outSegs.push_back(Seg{0, {1e300, 1e300}, {1e300, 1e300}, {}, 0, 0, 0, Box2{{1e300, 1e300}, {1e300, 1e300}}});
        OG.build(outSegs, std::max(d / 2, 0.05));
        const double sgn = side == OffsetSide::Left ? 1.0 : -1.0;
        struct Corner { Vec2 V; double a1, sweep; bool arcs; };
        std::vector<Corner> corners;
        {
            const int ni0 = static_cast<int>(inSegs.size());
            for (int k = 0; k < ni0; ++k) {
                int nx = k + 1;
                if (nx == ni0) { if (!in.closed) break; nx = 0; }
                Vec2 t1 = segTan(inSegs[k], 1), t2 = segTan(inSegs[nx], 0);
                double cr = t1.cross(t2);
                bool convex = side == OffsetSide::Left ? cr < -1e-12 : cr > 1e-12;
                if (!convex && t1.dot(t2) > -0.999) continue;
                Vec2 n1 = t1.perp() * sgn, n2 = t2.perp() * sgn;
                double a1 = n1.angle();
                double sw = std::remainder(n2.angle() - a1, kTwoPi);
                corners.push_back({inSegs[k].b, a1, sw, inSegs[k].kind == 1 || inSegs[nx].kind == 1});
            }
        }
        auto test = [&](Vec2 q, Vec2 nrm, int ex1, int ex2) {
            // valid locus point?
            double other = IG.minDist(q, d + 1e-5, ex1, ex2);
            if (other < d + 1e-6) return;
            m.covSamples++;
            double dout = OG.minDist(q, 1.0);
            if (dout <= 1e-7) return;
            if (join != JoinType::Round) {
                for (const Corner& cn : corners) {
                    Vec2 v = q - cn.V;
                    double L = v.length();
                    if (L > std::max(mitre, join == JoinType::Square || mitre < 1.5 ? 1.5 : mitre) * d + 1e-9 || L < 1e-12) continue;
                    double rel = std::remainder(v.angle() - cn.a1, kTwoPi);
                    double sl = cn.arcs ? 0.2 : 1e-9;
                    if (cn.sweep >= 0 ? (rel >= -sl && rel <= cn.sweep + sl) : (rel <= sl && rel >= cn.sweep - sl)) return;
                }
            }
            (void)nrm;
            if (dout <= tol) { m.covMissTol++; return; }  // here: count of sub-tolerance misses (allowed)
            m.covMiss++;
            m.worstMiss = std::max(m.worstMiss, dout);
            snprintf(buf, sizeof buf, "coverage miss at (%.9g,%.9g) dist-to-output %.3g", q.x, q.y, dout);
            note(m, buf);
        };
        const int ni = static_cast<int>(inSegs.size());
        for (int k = 0; k < ni; ++k) {
            const Seg& g = inSegs[k];
            for (int s = 1; s <= 9; ++s) {
                double t = s / 10.0;
                Vec2 p = segPoint(g, t), tg = segTan(g, t);
                Vec2 nrm = tg.perp() * sgn;
                Vec2 q = p + nrm * d;
                if (std::fabs(distPt(g, q) - d) > 1e-9 * std::max(1.0, d)) continue;
                if (g.kind == 1 && (q - g.c).dot(p - g.c) <= 0) continue;  // offset radius negative: not this side
                test(q, nrm, k, -1);
            }
        }
        for (int k = 0; k < ni; ++k) {
            int nx = k + 1;
            if (nx == ni) { if (!in.closed) break; nx = 0; }
            Vec2 t1 = segTan(inSegs[k], 1), t2 = segTan(inSegs[nx], 0);
            double cr = t1.cross(t2);
            bool convex = side == OffsetSide::Left ? cr < -1e-9 : cr > 1e-9;
            if (!convex) continue;
            Vec2 n1 = t1.perp() * sgn, n2 = t2.perp() * sgn;
            double a1 = n1.angle(), a2 = n2.angle();
            double sweep = std::remainder(a2 - a1, kTwoPi);
            Vec2 V = inSegs[k].b;
            for (int s = 1; s <= 4; ++s) {
                Vec2 u = Vec2::fromAngle(a1 + sweep * s / 5.0);
                Vec2 q = V + u * d;
                if (IG.minDist(q, d) < d - 1e-9) continue;
                test(q, u, k, nx);
            }
        }
    }
}

// ---------------------------------------------------------------- conversions to/from main
PContour toP(const Contour& c) {
    PContour p;
    p.closed = c.closed;
    for (auto& s : c.spans) {
        PSpan q{};
        if (auto* l = std::get_if<LineSpan>(&s)) { q.kind = 0; q.a[0] = l->start.x; q.a[1] = l->start.y; q.a[2] = l->end.x; q.a[3] = l->end.y; }
        else if (auto* a = std::get_if<ArcSpan>(&s)) {
            q.kind = 1; q.a[0] = a->centre.x; q.a[1] = a->centre.y; q.a[2] = a->radius; q.a[3] = a->startAngle; q.a[4] = a->endAngle;
            q.a[5] = a->direction == ArcDirection::CCW ? 0 : 1;
        }
        p.s.push_back(q);
    }
    return p;
}
Contour fromP(const PContour& p) {
    Contour c;
    c.closed = p.closed;
    for (auto& q : p.s) {
        if (q.kind == 0) c.spans.push_back(LineSpan{{q.a[0], q.a[1]}, {q.a[2], q.a[3]}});
        else if (q.kind == 1) c.spans.push_back(ArcSpan{{q.a[0], q.a[1]}, q.a[2], q.a[3], q.a[4], q.a[5] == 0 ? ArcDirection::CCW : ArcDirection::CW});
        else c.spans.push_back(BezierSpan{});
    }
    return c;
}

double cpuNow() { return static_cast<double>(std::clock()) / CLOCKS_PER_SEC; }

const char* jn(JoinType j) { return j == JoinType::Round ? "Round" : j == JoinType::Sharp ? "Sharp" : "Square"; }

// ---------------------------------------------------------------- fuzz case
struct Case {
    Contour in;
    double d, mitre, tol;
    OffsetSide side;
    JoinType join;
    int gen;
};
Case makeCase(std::uint64_t seed) {
    Rng g(seed);
    Case c;
    c.gen = g.irange(0, 5);
    int n = g.irange(2, 24);
    switch (c.gen) {
        case 0: c.in = walk(g, n, g.range(0.2, 2.0), g.range(0.5, 3.1)); break;
        case 1: case 2: c.in = mixed(g, n, g.range(0.3, 2.0)); break;
        case 3: c.in = zigzag(n, g.range(0.05, 0.6), g.range(0.05, 1.0)); break;
        case 4: c.in = mixed(g, n, g.range(0.05, 0.3)); break;  // small features vs d
        default: c.in = walk(g, n, g.range(0.01, 0.5), g.range(0.2, 3.1)); break;
    }
    if (g.uni() < 0.15 && c.in.spans.size() >= 3) {
        // close it if the gap is short: add closing line
        Vec2 e = c.in.endPoint(), s = c.in.startPoint();
        if (distance(e, s) > 1e-3) { c.in.spans.push_back(LineSpan{e, s}); c.in.closed = true; }
    }
    if (g.uni() < 0.3) translate(c.in, Vec2{g.range(-150, 150), g.range(-150, 150)});
    c.d = std::exp(g.range(std::log(0.05), std::log(7.0)));
    c.side = g.uni() < 0.5 ? OffsetSide::Left : OffsetSide::Right;
    c.join = static_cast<JoinType>(g.irange(0, 2));
    c.mitre = g.uni() < 0.6 ? 2.0 : g.range(1.0, 4.0);
    c.tol = 0.001;
    return c;
}

#include "mitre.inc"
int subTolIn(const Case& c) { for (auto& s : c.in.spans) if (length(s) <= c.tol) return 1; return 0; }


// ================================================================ round-2 adversarial generators
Contour fromPts(const std::vector<Vec2>& pts, bool closed) {
    Contour c;
    for (std::size_t i = 1; i < pts.size(); ++i) c.spans.push_back(LineSpan{pts[i - 1], pts[i]});
    if (closed && pts.size() >= 3) c.spans.push_back(LineSpan{pts.back(), pts.front()});
    c.closed = closed;
    return c;
}
void rotateSpans(Contour& c, int k) {  // closed contour: change the seam
    if (!c.closed || c.spans.empty()) return;
    k %= static_cast<int>(c.spans.size());
    std::rotate(c.spans.begin(), c.spans.begin() + k, c.spans.end());
}
void transformC(Contour& c, double th, Vec2 t) {
    const double cs = std::cos(th), sn = std::sin(th);
    auto R = [&](Vec2 p) { return Vec2{cs * p.x - sn * p.y + t.x, sn * p.x + cs * p.y + t.y}; };
    for (Span& s : c.spans) {
        if (auto* l = std::get_if<LineSpan>(&s)) { l->start = R(l->start); l->end = R(l->end); }
        else if (auto* a = std::get_if<ArcSpan>(&s)) { a->centre = R(a->centre); a->startAngle += th; a->endAngle += th; }
    }
    const std::size_t n = c.spans.size();
    for (std::size_t i = 0; i < n; ++i) {
        std::size_t j = i + 1;
        if (j == n) { if (!c.closed) break; j = 0; }
        if (auto* l = std::get_if<LineSpan>(&c.spans[j])) l->start = endPoint(c.spans[i]);
        else if (auto* l2 = std::get_if<LineSpan>(&c.spans[i])) l2->end = startPoint(c.spans[j]);
    }
}
// Many collinear horizontal runs on y = 0 and y = 2d (offsets coincide on y = d), both directions,
// equal numbers each way when `tie`; joined by detours far above/below.
Contour collinearRuns(Rng& g, double d, bool tie) {
    std::vector<Vec2> pts;
    int k = g.irange(1, 4);
    std::vector<std::pair<double, double>> iv;
    for (int i = 0; i < k; ++i) { double a = g.range(0, 6), b = a + g.range(0.5, 6); iv.push_back({a, b}); }
    struct Run { double y, x0, x1; };
    std::vector<Run> runs;
    for (int i = 0; i < k; ++i) {
        auto [a, b] = iv[i];
        // forward on y=0 (left offset at y=+d), backward on y=2d (left offset at y=+d)
        runs.push_back({0.0, a, b});
        if (tie) runs.push_back({2 * d, b, a});
        else if (g.uni() < 0.5) runs.push_back({2 * d, g.range(0, 6), g.range(0, 6)});
        if (g.uni() < 0.3) runs.push_back({0.0, b, a});
        if (g.uni() < 0.3) runs.push_back({2 * d, a, b});
    }
    for (std::size_t i = runs.size(); i > 1; --i) std::swap(runs[i - 1], runs[static_cast<std::size_t>(g.irange(0, static_cast<int>(i) - 1))]);
    double up = 30 + 4 * d;
    for (std::size_t i = 0; i < runs.size(); ++i) {
        const Run& r = runs[i];
        if (i > 0) {  // detour from previous end, far away
            Vec2 pe = pts.back();
            double yy = (g.uni() < 0.5 ? up : -up) + g.range(-3, 3);
            pts.push_back({pe.x + g.range(-2, 2), yy});
            pts.push_back({r.x0 + g.range(-2, 2), yy + g.range(-1, 1)});
        }
        pts.push_back({r.x0, r.y});
        pts.push_back({r.x1, r.y});
    }
    // dedupe consecutive equal points
    std::vector<Vec2> q;
    for (Vec2 p : pts) if (q.empty() || distance(q.back(), p) > 1e-6) q.push_back(p);
    return fromPts(q, false);
}
// Arcs on circles r0 and r0+2d around one centre (offsets coincide on r0+d), random directions.
Contour coCircular(Rng& g, double d, bool tie) {
    Contour c;
    Vec2 C{0, 0};
    double r0 = g.range(0.2, 3.0);
    int k = g.irange(1, 4);
    struct A { double r, a0, sw; };
    std::vector<A> arcs;
    for (int i = 0; i < k; ++i) {
        double a0 = g.range(-kPi, kPi), sw = g.range(0.3, 2.5);
        // Right side of a CCW arc of radius r0 -> r0+d; Right side of a CW arc of radius r0+2d -> r0+d
        arcs.push_back({r0, a0, sw});
        if (tie) arcs.push_back({r0 + 2 * d, a0 + sw, -sw});
        else if (g.uni() < 0.5) arcs.push_back({r0 + 2 * d, g.range(-kPi, kPi), -g.range(0.3, 2.5)});
        if (g.uni() < 0.3) arcs.push_back({r0, a0 + sw, -sw});
    }
    for (std::size_t i = arcs.size(); i > 1; --i) std::swap(arcs[i - 1], arcs[static_cast<std::size_t>(g.irange(0, static_cast<int>(i) - 1))]);
    double far = r0 + 2 * d + 20 + 3 * d;
    for (std::size_t i = 0; i < arcs.size(); ++i) {
        const A& a = arcs[i];
        ArcSpan s{C, a.r, a.a0, a.a0 + a.sw, a.sw > 0 ? ArcDirection::CCW : ArcDirection::CW};
        if (i > 0) {
            Vec2 pe = endPoint(c.spans.back());
            Vec2 st = s.startPoint();
            Vec2 m1 = Vec2::fromAngle(pe.angle(), far + g.range(0, 3));
            Vec2 m2 = Vec2::fromAngle(st.angle() + g.range(-0.2, 0.2), far + g.range(0, 3));
            c.spans.push_back(LineSpan{pe, m1});
            c.spans.push_back(LineSpan{m1, m2});
            c.spans.push_back(LineSpan{m2, st});
        }
        c.spans.push_back(s);
    }
    return c;
}
// Steep zig-zag whose peak chamfers (Square) touch end to end: d = pitch / tan(alpha/2) * (1+delta).
Contour zigzagPts(int n, double p, double a, Rng* jit, double jy, double jx) {
    std::vector<Vec2> pts{{0, 0}};
    for (int i = 1; i <= n; ++i) {
        double x = i * p, y = (i % 2) * a;
        if (jit) { x += jit->range(-jx, jx); y += jit->range(-jy, jy); }
        pts.push_back({x, y});
    }
    return fromPts(pts, false);
}
Contour closedZigzag(Rng& g, int n, double p, double a, double jy) {
    std::vector<Vec2> pts{{0, 0}};
    for (int i = 1; i <= n; ++i) pts.push_back({i * p, (i % 2) * a + (jy > 0 ? g.range(-jy, jy) : 0.0)});
    double B = g.range(0.3, 5);
    pts.push_back({n * p, -B});
    pts.push_back({0, -B});
    Contour c = fromPts(pts, true);
    return c;
}
struct Case2 { Case c; const char* name; bool noisy = false; };
Case2 makeCase2(std::uint64_t seed) {
    Rng g(seed ^ 0x2B992DDFA23249D6ULL);
    Case c;
    c.tol = 0.001;
    c.mitre = g.uni() < 0.5 ? 2.0 : g.range(1.0, 4.2);
    c.join = static_cast<JoinType>(g.irange(0, 2));
    if (g.uni() < 0.5 && c.join == JoinType::Round) c.join = g.uni() < 0.5 ? JoinType::Square : JoinType::Sharp;
    c.side = g.uni() < 0.5 ? OffsetSide::Left : OffsetSide::Right;
    c.d = std::exp(g.range(std::log(0.05), std::log(7.0)));
    c.gen = g.irange(10, 17);
    const char* nm = "?";
    bool noisy = false;
    switch (c.gen) {
        case 10: nm = "collinearRuns-tie"; c.in = collinearRuns(g, c.d, true); if (g.uni() < 0.7) c.side = OffsetSide::Left; break;
        case 11: nm = "collinearRuns"; c.in = collinearRuns(g, c.d, false); break;
        case 12: nm = "coCircular-tie"; c.in = coCircular(g, c.d, g.uni() < 0.6); if (g.uni() < 0.7) c.side = OffsetSide::Right; break;
        case 13: {  // chamfers touching end to end
            nm = "zigzag-touch";
            double p = g.range(0.02, 0.5), ratio = g.range(0.5, 25.0), a = p * ratio;
            double al = std::atan2(a, p);
            double deltas[] = {0, 1e-15, -1e-15, 1e-12, -1e-12, 1e-9, -1e-9, 1e-6, -1e-6, 1e-3, -1e-3, 0.05, -0.05};
            double dl = deltas[g.irange(0, 12)];
            c.d = p / std::tan(al / 2) * (1 + dl);
            if (c.d > 40) c.d = 40;
            c.join = g.uni() < 0.7 ? JoinType::Square : JoinType::Sharp;
            if (c.join == JoinType::Sharp) c.mitre = g.range(1.0, 1.5);  // force Sharp->Square fallbacks
            c.in = zigzagPts(g.irange(3, 30), p, a, nullptr, 0, 0);
            break;
        }
        case 14: {  // near-collinear carriers
            nm = "zigzag-nearcollinear"; noisy = true;
            double p = g.range(0.02, 0.5), a = p * g.range(0.5, 25.0);
            double jy = std::pow(10.0, g.range(-14, -5));
            Rng j2(seed * 7 + 1);
            c.in = zigzagPts(g.irange(3, 30), p, a, &j2, jy, g.uni() < 0.5 ? 0.0 : jy);
            c.d = a * g.range(0.5, 12);
            break;
        }
        case 15: {  // closed zig-zag with the seam rotated into a merged stretch
            nm = "closedZigzag-seam";
            double p = g.range(0.02, 0.5), a = p * g.range(0.5, 25.0);
            int n = 2 * g.irange(2, 14);
            double jyz = g.uni() < 0.3 ? std::pow(10.0, g.range(-13, -6)) : 0.0;
            noisy = jyz > 0;
            c.in = closedZigzag(g, n, p, a, jyz);
            rotateSpans(c.in, g.irange(0, n));
            c.d = a * g.range(0.3, 8);
            break;
        }
        case 16: {  // steep zigzag at random d, all joins
            nm = "zigzag-steep";
            double p = g.range(0.01, 0.6), a = p * g.range(0.3, 30.0);
            c.in = zigzagPts(g.irange(3, 40), p, a, nullptr, 0, 0);
            c.d = a * std::exp(g.range(std::log(0.2), std::log(15.0)));
            break;
        }
        default: {  // old generator families with new seeds
            nm = "legacy";
            Case o = makeCase(seed ^ 0x51ED2700ULL);
            c.in = o.in; c.d = o.d; c.join = o.join; c.side = o.side; c.mitre = o.mitre;
            break;
        }
    }
    // transforms: rotation and/or large translation
    double u = g.uni();
    if (u < 0.25) transformC(c.in, g.range(-kPi, kPi), Vec2{0, 0});
    else if (u < 0.5) transformC(c.in, g.uni() < 0.5 ? 0.0 : g.range(-kPi, kPi), Vec2{g.range(-1000, 1000), g.range(-1000, 1000)});
    else if (u < 0.6) transformC(c.in, 0.0, Vec2{g.range(-1000, 1000), g.range(-1000, 1000)});
    return {c, nm, noisy};
}

#include "gen3.inc"
// main-vs-branch geometric coverage: fraction of A's valid points (>= d - 1e-7 from input) farther than lim from B
struct XCov { long samples = 0, miss = 0; double worst = 0; Vec2 at; };
XCov crossCover(const Contour& in, const std::vector<Contour>& A, const std::vector<Contour>& B, double d, double lim) {
    XCov r;
    std::vector<Seg> inS, bS;
    for (auto& s : in.spans) inS.push_back(toSeg(s));
    for (auto& c : B) for (auto& s : c.spans) bS.push_back(toSeg(s));
    if (bS.empty()) bS.push_back(Seg{0, {1e300, 1e300}, {1e300, 1e300}, {}, 0, 0, 0, Box2{{1e300, 1e300}, {1e300, 1e300}}});
    Grid IG, BG;
    IG.build(inS, std::max(d / 2, 0.05));
    BG.build(bS, std::max(d / 2, 0.05));
    for (auto& c : A)
        for (auto& s : c.spans) {
            Seg g = toSeg(s);
            double len = segLen(g);
            int ns = static_cast<int>(std::clamp(std::ceil(len / (0.25 * lim)), 4.0, 400.0));
            for (int k = 0; k <= ns; ++k) {
                Vec2 q = segPoint(g, static_cast<double>(k) / ns);
                if (IG.minDist(q, d + 1) < d - 1e-7) continue;
                const double ddFirst = BG.minDist(q, 10 * lim + 1);
                if (ddFirst <= lim) { r.samples++; continue; }
                {   // exclude tie points: equidistant (~d) from two non-adjacent input elements
                    int kn = -1; double best = 1e300;
                    for (int ii = 0; ii < (int)inS.size(); ++ii) { double dd0 = distPt(inS[ii], q); if (dd0 < best) { best = dd0; kn = ii; } }
                    const int ni = (int)inS.size();
                    int e1 = kn > 0 ? kn - 1 : (in.closed ? ni - 1 : -1), e2 = kn + 1 < ni ? kn + 1 : (in.closed ? 0 : -1);
                    double other = 1e300;
                    for (int ii = 0; ii < ni; ++ii) if (ii != kn && ii != e1 && ii != e2) other = std::min(other, distPt(inS[ii], q));
                    if (best < d + 1e-6 && other < d + 1e-6) continue;
                }
                r.samples++;
                double dd = BG.minDist(q, 10 * lim + 1);
                if (dd > lim) { r.miss++; if (dd > r.worst) { r.worst = dd; r.at = q; } }
            }
        }
    return r;
}
int badCount(const Metrics& m) {
    return m.lineJunctionNotBitwise + m.arcGapBeyond + m.reversals + m.zeroLen + m.subTolAtEnd + m.badSpan + m.distViol +
           m.roundViol + m.selfX + m.covMiss + m.overlapIn + m.overlapX + m.shortBack;
}
int fuzz2(long N, std::uint64_t seed0, bool cmpMain, int onlyGen) {
    long exSx = 0, exDv = 0, exPrinted2 = 0; double exU = 0; long exCoder = 0, exOv = 0, exHard = 0, exPrinted = 0, failsNoisy = 0, failsExactAC3 = 0, fails = 0, failsSub = 0, regress = 0, same = 0, differ = 0, issuesMain = 0, n = 0;
    long gaps = 0, revs = 0, ovl = 0, sb = 0, cm = 0, dv = 0, rv = 0, sx = 0, xMissB = 0, xMissM = 0;
    double worstX = 0, cpuB = 0, cpuM = 0, worstUnder = 0, maxArcRel = 0;
    long perGenCases[40] = {0}, perGenFail[40] = {0}, perGenReg[40] = {0};
    TipStats tsE, tsN, tsM;
    for (long k = 0; k < N; ++k) {
        std::uint64_t seed = seed0 + static_cast<std::uint64_t>(k);
        Case2 cc = makeCaseX(seed);
        Case& c = cc.c;
        if (onlyGen >= 0 && c.gen != onlyGen) continue;
        if (c.in.spans.empty()) continue;
        n++;
        perGenCases[c.gen]++;
        double t0 = cpuNow();
        auto out = offsetOpenContour(c.in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
        cpuB += cpuNow() - t0;
        Metrics m;
        check(c.in, out, c.d, c.side, c.join, c.tol, c.mitre, m, true);
        tipCheck(c.in, out, c.d, c.tol, cc.noisy ? tsN : tsE, cc.name, seed);
        maxArcRel = std::max(maxArcRel, m.maxArcGapRel);
        int st = subTolIn(c);
        if (!st) worstUnder = std::max(worstUnder, m.worstUnder);
        gaps += m.lineJunctionNotBitwise + m.arcGapBeyond; revs += m.reversals; ovl += m.overlapIn + m.overlapX; sb += m.shortBack;
        cm += m.covMiss; dv += m.distViol; rv += m.roundViol; sx += m.selfX;
        int bad = badCount(m);
        if (!cc.noisy) { exSx += m.selfX > 0; exDv += m.distViol > 0; if (!st) exU = std::max(exU, m.worstUnder);
            if ((m.selfX || m.distViol) && exPrinted2 < 10) { exPrinted2++; std::printf("EXACT-SXDV seed=0x%llx %s subtol=%d: %s (selfX %d distViol %d under %.3g)\n", (unsigned long long)seed, cc.name, st, m.firstIssue.c_str(), m.selfX, m.distViol, m.worstUnder); }
            exCoder += m.coderBack > 0; exOv += (m.overlapIn + m.overlapX) > 0; exHard += m.hardRev > 0;
            if ((m.coderBack || m.overlapIn || m.overlapX) && exPrinted < 12 && std::getenv("EXDUMP")) { exPrinted++; std::printf("EXACT seed=0x%llx %s coderBack %d overlaps %d/%d hardRev %d d=%.9g %s side %d\n", (unsigned long long)seed, cc.name, m.coderBack, m.overlapIn, m.overlapX, m.hardRev, c.d, jn(c.join), (int)c.side); } }
        if (bad) {
            fails++; if (st) failsSub++; if (cc.noisy) failsNoisy++; else if (m.reversals + m.shortBack + m.overlapIn + m.overlapX + m.lineJunctionNotBitwise + m.arcGapBeyond) failsExactAC3++;
            perGenFail[c.gen]++;
            if (fails <= 25)
                std::printf("ISSUE%s %s subtol=%d seed=0x%llx gen=%d n=%zu closed=%d d=%.9g %s side=%d mitre=%.4g: %s [gap%d rev%d ov%d/%d sb%d dv%d rv%d sx%d cm%d z%d se%d]\n",
                            cc.noisy ? "(noisy)" : "", cc.name, st, (unsigned long long)seed, c.gen, c.in.spans.size(), c.in.closed, c.d, jn(c.join), (int)c.side, c.mitre,
                            m.firstIssue.c_str(), m.lineJunctionNotBitwise + m.arcGapBeyond, m.reversals, m.overlapIn, m.overlapX, m.shortBack,
                            m.distViol, m.roundViol, m.selfX, m.covMiss, m.zeroLen, m.subTolAtEnd);
        }
        if (cmpMain) {
            double t1 = cpuNow();
            auto mp = mainOffset(toP(c.in), c.d, c.side == OffsetSide::Left ? 0 : 1, static_cast<int>(c.join), c.tol, c.mitre);
            cpuM += cpuNow() - t1;
            std::vector<Contour> mo;
            for (auto& p : mp) mo.push_back(fromP(p));
            if (mo == out) { same++; continue; }
            differ++;
            Metrics mm;
            check(c.in, mo, c.d, c.side, c.join, c.tol, c.mitre, mm, true);
            { long pr = tsM.printed; tsM.printed = 1000; tipCheck(c.in, mo, c.d, c.tol, tsM, "main", seed); tsM.printed = pr; }
            if (badCount(mm)) issuesMain++;
            XCov bx = crossCover(c.in, mo, out, c.d, 2 * c.tol);  // main's valid points missing from branch
            XCov mx = crossCover(c.in, out, mo, c.d, 2 * c.tol);
            if (bx.miss) xMissB++;
            if (mx.miss) xMissM++;
            worstX = std::max(worstX, bx.worst);
            bool reg = (m.distViol > 0 && m.worstUnder > mm.worstUnder + 1e-12) || m.covMiss > mm.covMiss || m.selfX > mm.selfX ||
                       m.roundViol > mm.roundViol || m.reversals > mm.reversals || (m.overlapIn + m.overlapX) > (mm.overlapIn + mm.overlapX) ||
                       m.shortBack > mm.shortBack || bx.miss > 0;
            if (reg) {
                regress++;
                perGenReg[c.gen]++;
                if (regress <= 25)
                    std::printf("REGRESSION %s subtol=%d seed=0x%llx d=%.9g %s side=%d mitre=%.4g: br[%s] main[%s] mainPtsMissingInBranch %ld/%ld worst %.3g at (%.9g,%.9g)\n",
                                cc.name, st, (unsigned long long)seed, c.d, jn(c.join), (int)c.side, c.mitre, m.firstIssue.c_str(),
                                mm.firstIssue.c_str(), bx.miss, bx.samples, bx.worst, bx.at.x, bx.at.y);
            }
        }
    }
    std::printf("fuzz2 N=%ld (ran %ld) seed0=0x%llx: cases with issues %ld (%ld with sub-tol input); gaps %ld reversals %ld overlaps %ld shortBack %ld distViol %ld roundViol %ld selfX %ld covMiss %ld; worstUnder(no subtol) %.3g maxArcGapRel %.3g; cpu %.2fs\n",
                N, n, (unsigned long long)seed0, fails, failsSub, gaps, revs, ovl, sb, dv, rv, sx, cm, worstUnder, maxArcRel, cpuB);
    std::printf("  exact-input cases: coder-checker backward steps %ld, overlapping spans %ld, junction dot < -0.999999 %ld, selfX %ld, distViol %ld (worst under, no sub-tol %.3g)\n", exCoder, exOv, exHard, exSx, exDv, exU);
    std::printf("  near-reversal junctions (dot<-0.9999): exact %ld (genuine %ld, suspicious %ld, at input reversal %ld, min ratio %.3g); noisy %ld (genuine %ld, suspicious %ld, min ratio %.3g); main (differing cases) %ld\n", tsE.tips, tsE.genuine, tsE.suspicious, tsE.inputRev, tsE.minRatio, tsN.tips, tsN.genuine, tsN.suspicious, tsN.minRatio, tsM.tips);
    std::printf("  failing cases with noisy input %ld; exact-input cases with an AC3-type defect (reversal/shortBack/overlap/gap) %ld\n", failsNoisy, failsExactAC3);
    for (int gidx = 0; gidx < 40; ++gidx)
        if (perGenCases[gidx]) std::printf("  gen %d: cases %ld fail %ld regress %ld\n", gidx, perGenCases[gidx], perGenFail[gidx], perGenReg[gidx]);
    if (cmpMain)
        std::printf("vs main: identical %ld differ %ld (main had issues in %ld), regressions %ld; cases where main has valid pts >2tol from branch %ld (worst %.3g); branch pts >2tol from main %ld; cpu main %.2fs\n",
                    same, differ, issuesMain, regress, xMissB, worstX, xMissM, cpuM);
    return 0;
}
int one2(std::uint64_t seed) {
    Case2 cc = makeCaseX(seed);
    Case& c = cc.c;
    auto dump = [](const Contour& k) {
        for (auto& s : k.spans) {
            if (auto* l = std::get_if<LineSpan>(&s)) std::printf("  L (%.17g,%.17g)->(%.17g,%.17g)\n", l->start.x, l->start.y, l->end.x, l->end.y);
            else if (auto* a = std::get_if<ArcSpan>(&s)) std::printf("  A c(%.17g,%.17g) r %.17g sa %.17g ea %.17g %s\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, a->direction == ArcDirection::CCW ? "CCW" : "CW");
        }
    };
    std::printf("%s closed=%d d=%.17g %s side=%d mitre=%.17g\n", cc.name, c.in.closed, c.d, jn(c.join), (int)c.side, c.mitre);
    dump(c.in);
    auto out = offsetOpenContour(c.in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
    for (std::size_t i = 0; i < out.size(); ++i) { std::printf("branch chain %zu closed=%d\n", i, out[i].closed); dump(out[i]); }
    Metrics m; check(c.in, out, c.d, c.side, c.join, c.tol, c.mitre, m);
    std::printf("branch: %s [coderBack %d rev %d overlaps %d/%d]\n", m.firstIssue.c_str(), m.coderBack, m.reversals, m.overlapIn, m.overlapX);
    auto mp = mainOffset(toP(c.in), c.d, c.side == OffsetSide::Left ? 0 : 1, static_cast<int>(c.join), c.tol, c.mitre);
    std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p));
    for (std::size_t i = 0; i < mo.size(); ++i) { std::printf("main chain %zu closed=%d\n", i, mo[i].closed); dump(mo[i]); }
    Metrics mm; check(c.in, mo, c.d, c.side, c.join, c.tol, c.mitre, mm);
    std::printf("main: %s [coderBack %d rev %d overlaps %d/%d]\n", mm.firstIssue.c_str(), mm.coderBack, mm.reversals, mm.overlapIn, mm.overlapX);
    XCov bx = crossCover(c.in, mo, out, c.d, 2 * c.tol);
    std::printf("main valid pts missing in branch: %ld/%ld worst %.3g at (%.12g,%.12g)\n", bx.miss, bx.samples, bx.worst, bx.at.x, bx.at.y);
    return 0;
}

int ncMode(int trials, std::uint64_t seed0, double tx) {
    std::printf("near-collinear zig-zag sweep, translation %g\n", tx);
    for (double jy : {0.0, 1e-13, 1e-12, 1e-11, 1e-10, 1e-9, 1e-8, 1e-7, 1e-6, 1e-5, 1e-4}) {
        long dvM = 0, cases = 0, bad = 0, badM = 0, rev = 0, revM = 0, ov = 0, ovM = 0, sb = 0, sbM = 0, gp = 0, dvB = 0;
        double worstUnder = 0;
        std::uint64_t firstBad = 0;
        for (int t = 0; t < trials; ++t) {
            Rng g(seed0 + static_cast<std::uint64_t>(t) * 7919);
            double p = g.range(0.02, 0.5), a = p * g.range(0.5, 25.0);
            Rng j2(seed0 * 3 + static_cast<std::uint64_t>(t));
            Contour in = zigzagPts(g.irange(5, 30), p, a, jy > 0 ? &j2 : nullptr, jy, 0.0);
            if (tx != 0) transformC(in, 0.0, Vec2{tx, -tx * 0.37});
            double d = a * g.range(0.5, 12);
            JoinType j = g.uni() < 0.5 ? JoinType::Square : JoinType::Sharp;
            double mitre = g.range(1.0, 4.0);
            OffsetSide side = g.uni() < 0.5 ? OffsetSide::Left : OffsetSide::Right;
            auto out = offsetOpenContour(in, d, side, j, Tolerance{0.001, 1e-9, 0.01}, mitre);
            Metrics m; check(in, out, d, side, j, 0.001, mitre, m, false, true);
            auto mp = mainOffset(toP(in), d, side == OffsetSide::Left ? 0 : 1, static_cast<int>(j), 0.001, mitre);
            std::vector<Contour> mo; for (auto& q : mp) mo.push_back(fromP(q));
            Metrics mm; check(in, mo, d, side, j, 0.001, mitre, mm, false, true);
            cases++;
            int b = m.reversals + m.overlapIn + m.overlapX + m.shortBack + m.lineJunctionNotBitwise + m.arcGapBeyond;
            int bm = mm.reversals + mm.overlapIn + mm.overlapX + mm.shortBack + mm.lineJunctionNotBitwise + mm.arcGapBeyond;
            if (b) { bad++; if (!firstBad) firstBad = static_cast<std::uint64_t>(t) + 1; }
            if (m.distViol && std::getenv("NCDV")) std::printf("  DV t=%d d=%.6g %s: worstUnder %.3g allow %.3g (main worstUnder %.3g)\n", t, d, jn(j), m.worstUnder, allowance(d, 0.001), mm.worstUnder);
            if (m.reversals && std::getenv("NCDUMP")) {
                std::printf("  REV t=%d n=%zu p=%.17g a=%.17g d=%.17g %s mitre=%.17g side=%d: %s | main dv %d rev %d\n", t, in.spans.size(), p, a, d, jn(j), mitre, (int)side, m.firstIssue.c_str(), mm.distViol, mm.reversals);
            }
            if (bm) badM++;
            rev += m.reversals; revM += mm.reversals; ov += m.overlapIn + m.overlapX; ovM += mm.overlapIn + mm.overlapX;
            sb += m.shortBack; sbM += mm.shortBack; gp += m.lineJunctionNotBitwise + m.arcGapBeyond; dvB += m.distViol; dvM += mm.distViol;
            worstUnder = std::max(worstUnder, m.worstUnder);
        }
        std::printf("jy %-7.0e cases %ld | branch bad %ld (rev %ld, overlaps %ld, shortBack %ld, gaps %ld, distViol %ld, worstUnder %.2g) first t=%llu | main bad %ld (rev %ld ov %ld sb %ld dv %ld)\n",
                    jy, cases, bad, rev, ov, sb, gp, dvB, worstUnder, (unsigned long long)(firstBad ? firstBad - 1 : 0), badM, revM, ovM, sbM, dvM);
        std::fflush(stdout);
    }
    return 0;
}

int ncOne(std::uint64_t seed0, int t, double jy) {
    Rng g(seed0 + static_cast<std::uint64_t>(t) * 7919);
    double p = g.range(0.02, 0.5), a = p * g.range(0.5, 25.0);
    Rng j2(seed0 * 3 + static_cast<std::uint64_t>(t));
    Contour in = zigzagPts(g.irange(5, 30), p, a, jy > 0 ? &j2 : nullptr, jy, 0.0);
    double d = a * g.range(0.5, 12);
    JoinType j = g.uni() < 0.5 ? JoinType::Square : JoinType::Sharp;
    double mitre = g.range(1.0, 4.0);
    OffsetSide side = g.uni() < 0.5 ? OffsetSide::Left : OffsetSide::Right;
    std::printf("// d=%.17g join=%s mitre=%.17g side=%s\nconst std::vector<Vec2> pts{", d, jn(j), mitre, side == OffsetSide::Left ? "Left" : "Right");
    std::printf("{%.17g, %.17g}", in.spans[0].index() == 0 ? std::get<LineSpan>(in.spans[0]).start.x : 0.0, std::get<LineSpan>(in.spans[0]).start.y);
    for (auto& sp : in.spans) std::printf(", {%.17g, %.17g}", std::get<LineSpan>(sp).end.x, std::get<LineSpan>(sp).end.y);
    std::printf("};\n");
    return 0;
}

int subtolMode(long N, std::uint64_t seed0) {
    long n = 0, bDv = 0, mDv = 0, bRv = 0, mRv = 0, bCm = 0, mCm = 0, bAny = 0, mAny = 0, worse = 0, better = 0;
    double bU = 0, mU = 0, bO = 0, mO = 0, bW = 0, mW = 0;
    for (long k = 0; k < N; ++k) {
        Case c = makeCase(seed0 + static_cast<std::uint64_t>(k));
        if (!subTolIn(c)) {  // inject sub-tolerance spans: jogs after some line spans
            Rng g(seed0 * 31 + static_cast<std::uint64_t>(k));
            Contour o; o.closed = c.in.closed;
            for (std::size_t i = 0; i < c.in.spans.size(); ++i) {
                const Span& sp = c.in.spans[i];
                const auto* l = std::get_if<LineSpan>(&sp);
                bool last = i + 1 == c.in.spans.size();
                if (l && g.uni() < 0.3 && !(last && c.in.closed)) {
                    Vec2 dir = (l->end - l->start).normalized();
                    double L = distance(l->start, l->end);
                    double tl = g.range(0.05, 1.0) * c.tol;
                    if (L > 3 * tl) {
                        Vec2 jog = Vec2::fromAngle(dir.angle() + g.range(-3.0, 3.0), tl);
                        Vec2 mid = l->end - jog;
                        o.spans.push_back(LineSpan{l->start, mid});
                        o.spans.push_back(LineSpan{mid, l->end});
                        continue;
                    }
                }
                o.spans.push_back(sp);
            }
            c.in = o;
            if (!subTolIn(c)) continue;
        }
        n++;
        auto out = offsetOpenContour(c.in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
        auto mp = mainOffset(toP(c.in), c.d, c.side == OffsetSide::Left ? 0 : 1, static_cast<int>(c.join), c.tol, c.mitre);
        std::vector<Contour> mo; for (auto& q : mp) mo.push_back(fromP(q));
        Metrics m, mm;
        check(c.in, out, c.d, c.side, c.join, c.tol, c.mitre, m);
        check(c.in, mo, c.d, c.side, c.join, c.tol, c.mitre, mm);
        bDv += m.distViol > 0; mDv += mm.distViol > 0; bRv += m.roundViol > 0; mRv += mm.roundViol > 0; bCm += m.covMiss > 0; mCm += mm.covMiss > 0;
        bool ba = m.distViol || m.roundViol || m.covMiss, ma = mm.distViol || mm.roundViol || mm.covMiss;
        bAny += ba; mAny += ma; worse += ba && !ma; better += ma && !ba;
        bU = std::max(bU, m.worstUnder); mU = std::max(mU, mm.worstUnder); bO = std::max(bO, m.worstOver); mO = std::max(mO, mm.worstOver);
        bW = std::max(bW, m.worstMiss); mW = std::max(mW, mm.worstMiss);
    }
    std::printf("sub-tol inputs %ld: cases with distViol branch %ld main %ld; roundViol branch %ld main %ld; covMiss branch %ld main %ld; any branch %ld main %ld (branch-only %ld, main-only %ld)\n", n, bDv, mDv, bRv, mRv, bCm, mCm, bAny, mAny, worse, better);
    std::printf("   worst under branch %.3g main %.3g; worst over(Round) branch %.3g main %.3g; worst coverage miss branch %.3g main %.3g\n", bU, mU, bO, mO, bW, mW);
    return 0;
}

#include "reg.inc"
#include "dv.inc"
#include "shrink.inc"
bool i_dvWorse(const Metrics& m, const Metrics& mm, int st) { return !st && m.distViol && m.worstUnder > mm.worstUnder + 1e-12; }
#include "scan.inc"
#include "gen4.inc"
#include "gen5.inc"
#include "cs.inc"
#include "ac.inc"
#include "fuzz4.inc"
#include "lens.inc"
#include "hp.inc"
int main(int argc, char** argv) {
    if (argc > 1 && std::string(argv[1]) == "ac") return acMode();
    if (argc > 1 && std::string(argv[1]) == "cs") return csMode(argc > 2 ? std::atoi(argv[2]) : -1);
    if (argc > 1 && std::string(argv[1]) == "hpone") return hpOne(std::atoi(argv[2]), std::atoi(argv[3]), std::atof(argv[4]), std::atof(argv[5]), std::atoi(argv[6]));
    if (argc > 1 && std::string(argv[1]) == "lensone") return lensOne(std::atoi(argv[2]), std::atof(argv[3]), std::atof(argv[4]), std::atof(argv[5]), std::atoi(argv[6]));
    if (argc > 1 && std::string(argv[1]) == "hp") return hpMode(argc > 2 ? std::atoi(argv[2]) : 0, argc > 3 ? std::atoi(argv[3]) : 0);
    if (argc > 1 && std::string(argv[1]) == "lens") return lensMode(std::atoi(argv[2]), argc > 3 ? std::atof(argv[3]) : 0.0);
    if (argc > 1 && std::string(argv[1]) == "fuzz4") return fuzz4(std::atol(argv[2]), std::strtoull(argv[3], nullptr, 0), std::atoi(argv[4]));
    if (argc > 1 && std::string(argv[1]) == "one4") return one4(std::strtoull(argv[2], nullptr, 0), std::atoi(argv[3]));
    std::string mode = argc > 1 ? argv[1] : "fuzz";
    if (std::getenv("GEN3")) g_gen3 = 1;
    if (mode == "reg") return regMode(argc, argv);
    if (mode == "scan") return scanMode(std::atol(argv[2]), std::strtoull(argv[3], nullptr, 0));
    if (mode == "shrink") return shrinkMode(std::strtoull(argv[2], nullptr, 0));
    if (mode == "dv") return dvMode(std::strtoull(argv[2], nullptr, 0), argc > 3);
    if (mode == "fuzz2") return fuzz2(std::atol(argv[2]), std::strtoull(argv[3], nullptr, 0), argc > 4 && std::string(argv[4]) == "main", argc > 5 ? std::atoi(argv[5]) : -1);
    if (mode == "nc") return ncMode(std::atoi(argv[2]), std::strtoull(argv[3], nullptr, 0), std::atof(argv[4]));
    if (mode == "ncone") return ncOne(std::strtoull(argv[2], nullptr, 0), std::atoi(argv[3]), std::atof(argv[4]));
    if (mode == "subtol") return subtolMode(std::atol(argv[2]), std::strtoull(argv[3], nullptr, 0));
    if (mode == "one2") return one2(std::strtoull(argv[2], nullptr, 0));
    if (mode == "fuzz") {
        long N = argc > 2 ? std::atol(argv[2]) : 1000;
        std::uint64_t seed0 = argc > 3 ? std::strtoull(argv[3], nullptr, 0) : 0x7006A000;
        bool cmpMain = argc > 4 && std::string(argv[4]) == "main";
        bool cov = !(argc > 5 && std::string(argv[5]) == "nocov");
        long fails = 0, regress = 0, improved = 0, same = 0, differ = 0, issuesMain = 0;
        Metrics tot;
        double cpuB = 0, cpuM = 0, maxArcRel = 0, worstUnder = 0, worstOver = 0;
        long totalChains = 0, totalSpans = 0, covS = 0, covM = 0, selfXNJ = 0;
        for (long k = 0; k < N; ++k) {
            std::uint64_t seed = seed0 + static_cast<std::uint64_t>(k);
            Case c = makeCase(seed);
            double t0 = cpuNow();
            auto out = offsetOpenContour(c.in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
            double tb = cpuNow() - t0;
            cpuB += tb;
            Metrics m;
            check(c.in, out, c.d, c.side, c.join, c.tol, c.mitre, m, cov);
            maxArcRel = std::max(maxArcRel, m.maxArcGapRel);
            worstUnder = std::max(worstUnder, m.worstUnder);
            worstOver = std::max(worstOver, m.worstOver);
            totalChains += m.chains; totalSpans += m.spans; covS += m.covSamples; covM += m.covMiss; selfXNJ += m.selfXNearJ;
            int bad = m.lineJunctionNotBitwise + m.arcGapBeyond + m.reversals + m.zeroLen + m.subTolAtEnd + m.badSpan +
                      m.distViol + m.roundViol + m.selfX + m.covMiss + m.overlapIn + m.overlapX + m.shortBack;
            if (bad) {
                fails++;
                if (fails <= 30)
                    std::printf("BRANCH ISSUE subtolIn=%d seed=0x%llx gen=%d n=%zu closed=%d d=%.6g %s side=%d mitre=%.3g : %s [lj%d ag%d rev%d z%d se%d dv%d rv%d sx%d cm>tol %d (subtol %d)]\n",
                                subTolIn(c), static_cast<unsigned long long>(seed), c.gen, c.in.spans.size(), c.in.closed, c.d, jn(c.join),
                                static_cast<int>(c.side), c.mitre, m.firstIssue.c_str(), m.lineJunctionNotBitwise, m.arcGapBeyond,
                                m.reversals, m.zeroLen, m.subTolAtEnd, m.distViol, m.roundViol, m.selfX, m.covMiss, m.covMissTol);
            }
            if (cmpMain) {
                double t1 = cpuNow();
                auto mp = mainOffset(toP(c.in), c.d, c.side == OffsetSide::Left ? 0 : 1, static_cast<int>(c.join), c.tol, c.mitre);
                cpuM += cpuNow() - t1;
                std::vector<Contour> mo;
                for (auto& p : mp) mo.push_back(fromP(p));
                bool identical = mo == out;
                if (identical) { same++; continue; }
                differ++;
                Metrics mm;
                check(c.in, mo, c.d, c.side, c.join, c.tol, c.mitre, mm, cov);
                int mbad = mm.lineJunctionNotBitwise + mm.arcGapBeyond + mm.reversals + mm.zeroLen + mm.distViol + mm.roundViol + mm.selfX + mm.covMiss;
                if (mbad) issuesMain++;
                bool reg = (m.distViol > 0 && m.worstUnder > mm.worstUnder + 1e-12) || m.covMiss > mm.covMiss || m.selfX > mm.selfX ||
                           m.roundViol > mm.roundViol || m.reversals > mm.reversals;
                if (reg) {
                    regress++;
                    if (regress <= 30)
                        std::printf("REGRESSION subtolIn=%d seed=0x%llx gen=%d d=%.6g %s side=%d mitre=%.3g: branch[dv%d under%.3g cm%d sx%d rv%d rev%d] main[dv%d under%.3g cm%d sx%d rv%d rev%d] br:%s | main:%s\n",
                                    subTolIn(c), static_cast<unsigned long long>(seed), c.gen, c.d, jn(c.join), static_cast<int>(c.side), c.mitre,
                                    m.distViol, m.worstUnder, m.covMiss, m.selfX, m.roundViol, m.reversals, mm.distViol, mm.worstUnder,
                                    mm.covMiss, mm.selfX, mm.roundViol, mm.reversals, m.firstIssue.c_str(), mm.firstIssue.c_str());
                } else if (mbad > bad) improved++;
            }
        }
        std::printf("fuzz N=%ld seed0=0x%llx: branch cases with issues %ld; chains %ld spans %ld; maxArcGapRel %.3g; worstUnder %.3g; worstOver(Round) %.3g; coverage samples %ld misses %ld; selfX-near-junction %ld; cpu branch %.2fs\n",
                    N, static_cast<unsigned long long>(seed0), fails, totalChains, totalSpans, maxArcRel, worstUnder, worstOver, covS, covM, selfXNJ, cpuB);
        if (cmpMain)
            std::printf("vs main: identical %ld, differ %ld (main had issues in %ld of them; branch strictly better in %ld), regressions %ld; cpu main %.2fs\n",
                        same, differ, issuesMain, improved, regress, cpuM);
        return 0;
    }
    if (mode == "one") {  // repro a fuzz seed with dump
        std::uint64_t seed = std::strtoull(argv[2], nullptr, 0);
        Case c = makeCase(seed);
        auto dump = [](const Contour& k) {
            for (auto& s : k.spans) {
                if (auto* l = std::get_if<LineSpan>(&s)) std::printf("  L (%.17g,%.17g)->(%.17g,%.17g)\n", l->start.x, l->start.y, l->end.x, l->end.y);
                else if (auto* a = std::get_if<ArcSpan>(&s)) std::printf("  A c(%.17g,%.17g) r %.17g sa %.17g ea %.17g %s  [%.12g,%.12g]->[%.12g,%.12g]\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, a->direction == ArcDirection::CCW ? "CCW" : "CW", a->startPoint().x, a->startPoint().y, a->endPoint().x, a->endPoint().y);
            }
        };
        std::printf("input closed=%d d=%.17g %s side=%d mitre=%.17g\n", c.in.closed, c.d, jn(c.join), static_cast<int>(c.side), c.mitre);
        dump(c.in);
        auto out = offsetOpenContour(c.in, c.d, c.side, c.join, Tolerance{c.tol, 1e-9, 0.01}, c.mitre);
        for (std::size_t i = 0; i < out.size(); ++i) { std::printf("branch chain %zu closed=%d\n", i, out[i].closed); dump(out[i]); }
        Metrics m;
        check(c.in, out, c.d, c.side, c.join, c.tol, c.mitre, m);
        std::printf("branch: %s [coderBack %d rev %d overlaps %d/%d]\n", m.firstIssue.c_str(), m.coderBack, m.reversals, m.overlapIn, m.overlapX);
        if (argc > 3) {
            auto mp = mainOffset(toP(c.in), c.d, c.side == OffsetSide::Left ? 0 : 1, static_cast<int>(c.join), c.tol, c.mitre);
            std::vector<Contour> mo;
            for (auto& p : mp) mo.push_back(fromP(p));
            for (std::size_t i = 0; i < mo.size(); ++i) { std::printf("main chain %zu closed=%d\n", i, mo[i].closed); dump(mo[i]); }
            Metrics mm;
            check(c.in, mo, c.d, c.side, c.join, c.tol, c.mitre, mm);
            std::printf("main: %s [coderBack %d rev %d overlaps %d/%d]\n", mm.firstIssue.c_str(), mm.coderBack, mm.reversals, mm.overlapIn, mm.overlapX);
        }
        return 0;
    }
    if (mode == "time") {
        auto run = [](const char* name, const Contour& in, double d, bool check_, bool alsoMain) {
            double worst = 0, worstMain = 0;
            for (JoinType j : {JoinType::Round, JoinType::Sharp, JoinType::Square})
                for (OffsetSide s : {OffsetSide::Left, OffsetSide::Right}) {
                    double t0 = cpuNow();
                    auto out = offsetOpenContour(in, d, s, j, Tolerance::defaults(), 2.0);
                    double t = cpuNow() - t0;
                    worst = std::max(worst, t);
                    if (alsoMain) {
                        double t1 = cpuNow();
                        auto mo = mainOffset(toP(in), d, s == OffsetSide::Left ? 0 : 1, static_cast<int>(j), 0.001, 2.0);
                        worstMain = std::max(worstMain, cpuNow() - t1);
                    }
                    if (check_) {
                        Metrics m;
                        check(in, out, d, s, j, 0.001, 2.0, m, false, true);
                        int bad = m.lineJunctionNotBitwise + m.arcGapBeyond + m.reversals + m.zeroLen + m.subTolAtEnd + m.badSpan + m.distViol + m.roundViol + m.selfX;
                        if (bad) std::printf("   ISSUE %s %s side %d: %s\n", name, jn(j), static_cast<int>(s), m.firstIssue.c_str());
                    }
                }
            if (alsoMain) std::printf("%-40s spans %6zu d=%.3g worst CPU %.4f s (main %.4f s)\n", name, in.spans.size(), d, worst, worstMain);
            else std::printf("%-40s spans %6zu d=%.3g worst CPU %.4f s\n", name, in.spans.size(), d, worst);
            return worst;
        };
        bool withMain = argc > 2 && std::string(argv[2]) == "main";
        for (std::uint64_t sd : {0xA11CEull, 0xB0B5ull, 0xC4A1ull, 0xD00Dull}) {
            for (double d : {1.0, 2.0}) {
                Rng g(sd);
                Contour w = walk(g, 2000, 0.5, kPi);
                char nm[80];
                snprintf(nm, sizeof nm, "walk2000 seed %llx", static_cast<unsigned long long>(sd));
                run(nm, w, d, true, withMain);
            }
        }
        run("hypotrochoid8000 (closed)", hypotrochoid(8000), 2.0, true, withMain);
        run("textLoops8000", textLoops(8000), 2.0, true, withMain);
        run("zigzag 4000 pitch .05 amp .2 d=5", zigzag(4000, 0.05, 0.2), 5.0, true, withMain);
        if (!std::getenv("VSKIPZZ")) run("zigzag 4000 pitch .02 amp 1 d=7", zigzag(4000, 0.02, 1.0), 7.0, true, withMain);
        else run("zigzag 600 pitch .02 amp 1 d=7", zigzag(600, 0.02, 1.0), 7.0, true, withMain);
        if (std::getenv("VSKIPZZ")) return 0;
        run("arcSpiral 300 pitch .3 d=4", arcSpiral(300, 0.3), 4.0, true, withMain);
        run("lineSpiral 6000 pitch .4 d=3", lineSpiral(6000, 0.4), 3.0, true, withMain);
        {
            Rng g(0xFEEDF00Dull);
            run("walk20000 step .5 d=2", walk(g, 20000, 0.5, kPi), 2.0, true, withMain);
        }
        {
            Rng g(0x5EED0006ull);
            run("mixed arcs 4000 d=2", mixed(g, 4000, 0.5), 2.0, true, withMain);
        }
        // scaling: time(2N)/time(N)
        std::printf("scaling (Round Left, best of 3):\n");
        auto best = [](const Contour& in, double d) {
            double b = 1e9;
            for (int r = 0; r < 3; ++r) {
                double t0 = cpuNow();
                for (JoinType j : {JoinType::Round, JoinType::Sharp, JoinType::Square})
                    for (OffsetSide s : {OffsetSide::Left, OffsetSide::Right}) (void)offsetOpenContour(in, d, s, j, Tolerance::defaults(), 2.0);
                b = std::min(b, cpuNow() - t0);
            }
            return b;
        };
        for (int N : {2500, 5000, 10000}) {
            Rng g1(0x51CA1Eull), g2(0x51CA1Eull);
            Contour a = walk(g1, N, 0.5, kPi), b = walk(g2, 2 * N, 0.5, kPi);
            double ta = best(a, 2.0), tb = best(b, 2.0);
            std::printf("  walk N=%d: %.4f s, 2N: %.4f s, ratio %.2f\n", N, ta, tb, tb / ta);
            Contour za = zigzag(N, 0.05, 0.2), zb = zigzag(2 * N, 0.05, 0.2);
            double tza = best(za, 5.0), tzb = best(zb, 5.0);
            std::printf("  zigzag N=%d d=5: %.4f s, 2N: %.4f s, ratio %.2f\n", N, tza, tzb, tzb / tza);
            Contour la = lineSpiral(N, 0.4), lb = lineSpiral(2 * N, 0.4);
            double tla = best(la, 3.0), tlb = best(lb, 3.0);
            std::printf("  lineSpiral N=%d d=3: %.4f s, 2N: %.4f s, ratio %.2f\n", N, tla, tlb, tlb / tla);
        }
        return 0;
    }
    if (mode == "mitre") return mitreMode(std::atol(argv[2]), std::strtoull(argv[3], nullptr, 0), argc > 4 && std::string(argv[4]) == "v", argc > 5 && std::string(argv[5]) == "tiny");
    if (mode == "keyhole") return keyholeMode();
    if (mode == "scale") {
        double d = std::atof(argv[2]);
        for (int k = 3; k < argc; ++k) {
            int N = std::atoi(argv[k]);
            Rng g(0x51CA1Eull);
            Contour w = walk(g, N, 0.5, kPi);
            for (JoinType j : {JoinType::Round, JoinType::Sharp, JoinType::Square}) {
                double best = 1e9;
                for (int r = 0; r < 3; ++r) { double t0 = cpuNow(); (void)offsetOpenContour(w, d, OffsetSide::Left, j, Tolerance::defaults(), 2.0); best = std::min(best, cpuNow() - t0); }
                std::printf("walk N=%d d=%g %s: %.4f s\n", N, d, jn(j), best);
                std::fflush(stdout);
            }
        }
        return 0;
    }
    std::printf("unknown mode\n");
    return 1;
}
