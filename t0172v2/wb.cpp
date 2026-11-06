// T-0172 verifier white-box: #includes the branch offset.cpp to reach selfCrossingCuts / BoxTree (round 2).
//  wb sx <trials> <seed>   selfCrossingCuts vs the pairwise test it replaced (carrierIntersections + inUnit, j >= i+2),
//                          both the yes/no (partCrossesItself) and the greedy cuts; adversarial strands.
//  wb reach <trials> <seed> pairs with spansMeet true: max of (raw box Chebyshev gap)/(meetReach a + meetReach b); > 1 = miss.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <tuple>
#include <utility>
#include <vector>
#include <variant>
#include <string>
#include <memory>
#include <array>
#include <numeric>
#include <unordered_map>
#include <deque>
#include <queue>
#include <cstring>
#include <random>
#include <chrono>
#define private public
#include "offset.cpp"
#undef private
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
using namespace carvetto::geometry;
namespace cg = carvetto::geometry;

static bool pairMeets(const Span& a, const Span& b) {
    for (const Vec2 p : cg::carrierIntersections(a, b))
        if (cg::inUnit(cg::carrierParam(a, p)) && cg::inUnit(cg::carrierParam(b, p))) return true;
    return false;
}
static bool oldCrosses(const std::vector<Span>& s) {  // origin/main partCrossesItself
    for (size_t i = 0; i < s.size(); ++i)
        for (size_t j = i + 2; j < s.size(); ++j)
            if (pairMeets(s[i], s[j])) return true;
    return false;
}
static std::vector<size_t> oldGreedy(const std::vector<Span>& s) {
    std::vector<size_t> cuts; size_t from = 0;
    if (s.size() < 3) return cuts;
    for (size_t i = 0; i < s.size(); ++i) {
        bool hit = false;
        for (size_t j = from; j + 2 <= i && !hit; ++j) hit = pairMeets(s[j], s[i]);
        if (hit) { cuts.push_back(i); from = i; }
    }
    return cuts;
}

struct G {
    std::mt19937_64 r;
    double u(double a, double b) { return std::uniform_real_distribution<double>(a, b)(r); }
    int i(int a, int b) { return std::uniform_int_distribution<int>(a, b)(r); }
    double logu(double a, double b) { return std::exp(u(std::log(a), std::log(b))); }
};

static ArcSpan arcFrom(Vec2 c, double r, double a0, double sw) {
    return ArcSpan{c, r, a0, a0 + sw, sw > 0 ? ArcDirection::CCW : ArcDirection::CW};
}

// One adversarial strand. Mode picks the probe shape.
static std::vector<Span> strand(G& g, int& modeOut) {
    std::vector<Span> s;
    const double big = std::vector<double>{0, 0, 1e3, 1e4, 1e5, 3.3e5}[g.i(0, 5)];
    const double sc = g.logu(1e-4, 10.0);  // span size scale
    Vec2 o{big * g.u(-1, 1), big * g.u(-1, 1)};
    Vec2 p = o;
    const int pre = g.i(1, 12);
    for (int k = 0; k < pre; ++k) {
        if (g.u(0, 1) < 0.5) {
            Vec2 q = p + Vec2::fromAngle(g.u(0, 2 * kPi), sc * g.u(0.2, 1.5));
            s.emplace_back(LineSpan{p, q}); p = q;
        } else {
            double r = sc * g.logu(0.1, 20), a = g.u(0, 2 * kPi), sw = g.u(-2.5, 2.5);
            if (std::fabs(sw) < 1e-3) sw = 0.1;
            Vec2 c = p - Vec2::fromAngle(a, r);
            ArcSpan A = arcFrom(c, r, a, sw); s.emplace_back(A); p = A.endPoint();
        }
    }
    // probe against an earlier span k
    const size_t k = static_cast<size_t>(g.i(0, static_cast<int>(s.size()) - 1));
    const Span tgt = s[k];
    const int mode = g.i(0, 7); modeOut = mode;
    const double gap = std::vector<double>{0.0, 1e-16, 1e-13, 1e-12, 1e-11, 5e-11, 1e-10, 1.2e-10, 3e-10, 9e-10, 1e-9, 1.5e-9}[g.i(0, 11)]
                       * (g.u(0, 1) < 0.5 ? 1.0 : std::max({1.0, std::fabs(o.x), std::fabs(o.y)}) * (g.u(0,1)<0.5?1.0:1e-6));
    const double plen = sc * g.logu(1e-4, 2.0);  // probe size
    std::vector<Span> probe;
    if (const auto* A = std::get_if<ArcSpan>(&tgt)) {
        const double t = g.u(-1e-9, 1 + 1e-9);
        const double ang = A->angleAt(t);
        const Vec2 n = Vec2::fromAngle(ang, 1.0);
        const Vec2 tg = n.perp();
        if (mode <= 2) {  // line tangent outside (mode0), inside chord-like (1), exactly through (2)
            const double off = mode == 0 ? A->radius + gap : (mode == 1 ? A->radius - gap : A->radius);
            const Vec2 foot = A->centre + n * off;
            double s0 = g.u(-1, 1) * plen, s1 = s0 + plen * (g.u(0,1)<0.5?1:-1);
            probe.emplace_back(LineSpan{foot + tg * s0, foot + tg * s1});
        } else if (mode <= 4) {  // arc externally / internally near-tangent
            const double r2 = plen * g.logu(0.5, 1e4);
            const bool ext = mode == 3 || r2 > A->radius;
            const double dist = mode == 3 ? A->radius + r2 + gap : std::fabs(A->radius - r2) + (r2 > A->radius ? gap : -gap);
            const Vec2 c2 = mode == 3 ? A->centre + n * dist : (r2 > A->radius ? A->centre - n * dist : A->centre + n * dist);
            (void)ext;
            const Vec2 contact = A->centre + n * A->radius;
            const double a2 = (contact - c2).angle();
            const double half = std::min(3.0, plen / r2);
            const double a0 = a2 - half * g.u(0, 1);
            probe.emplace_back(arcFrom(c2, r2, a0, (g.u(0,1)<0.5?1:-1) * half * g.u(1.0, 2.0)));
            if (std::get<ArcSpan>(probe.back()).direction == ArcDirection::CW) probe.back() = arcFrom(c2, r2, a0 + 2*half, -half*2);
        } else if (mode == 5) {  // line ending (T) on/near the arc
            const Vec2 q = A->centre + n * (A->radius + gap);
            const Vec2 d = Vec2::fromAngle(g.u(0, 2*kPi), plen);
            probe.emplace_back(LineSpan{q + d, q});
        } else {  // line crossing at grazing angle near arc endpoint
            const Vec2 q = A->centre + n * (A->radius + gap);
            const Vec2 d = (tg + n * g.logu(1e-9, 1e-2)).normalized() * plen;
            probe.emplace_back(LineSpan{q - d, q + d});
        }
    } else {
        const auto& L = std::get<LineSpan>(tgt);
        const Vec2 u = (L.end - L.start); const double len = u.length(); const Vec2 dir = u / len; const Vec2 n = dir.perp();
        const double t = g.u(-2e-9, 1 + 2e-9);
        const Vec2 q0 = L.start + u * t;
        if (mode <= 1) {  // collinear overlap / parallel at gap
            const Vec2 q = q0 + n * (mode == 0 ? 0.0 : gap);
            probe.emplace_back(LineSpan{q - dir * plen * g.u(0, 1), q + dir * plen * g.u(0, 1)});
        } else if (mode <= 3) {  // T at gap: endpoint just off the line
            const Vec2 q = q0 + n * gap;
            probe.emplace_back(LineSpan{q + Vec2::fromAngle(g.u(0, 2*kPi), plen), q});
        } else if (mode <= 5) {  // arc tangent to line at gap
            const double r2 = plen * g.logu(0.5, 1e4);
            const double side = g.u(0,1) < 0.5 ? 1 : -1;
            const Vec2 c2 = q0 + n * side * (r2 + gap);
            const double a2 = (q0 - c2).angle();
            const double half = std::min(3.0, plen / r2);
            probe.emplace_back(arcFrom(c2, r2, a2 - half * g.u(0, 1), half * g.u(1.0, 2.0)));
        } else {  // grazing crossing line
            const Vec2 q = q0 + n * gap;
            const Vec2 d = (dir + n * g.logu(1e-12, 1e-2)).normalized() * plen;
            probe.emplace_back(LineSpan{q - d, q + d});
        }
    }
    // Isolate the pair: tgt and probe separated by disjoint fillers far away (only the pair can meet),
    // unless mode is odd-trial "walk" where the original walk prefix stays (wb sx ... WALK=1).
    if (std::getenv("WALK")) {
        if (g.u(0, 1) < 0.85) s.emplace_back(LineSpan{p, startPoint(probe[0])});
        s.push_back(probe[0]);
        return s;
    }
    std::vector<Span> out;
    const Vec2 far = o + Vec2{0, 1e3 * sc + 50 * sc};
    int fid = 0;
    auto filler = [&](int m) { for (int q = 0; q < m; ++q, ++fid) { Vec2 a = far + Vec2{3.0 * sc * fid, 0}; out.emplace_back(LineSpan{a, a + Vec2{sc, 0}}); } };
    filler(g.i(0, 40));
    out.push_back(tgt);
    filler(g.i(1, 40));
    out.push_back(probe[0]);
    filler(g.i(0, 5));
    return out;
    return s;
}

static int runSx(long trials, uint64_t seed) {
    G g{std::mt19937_64(seed)};
    long bad1 = 0, badc = 0, pos = 0, poscuts = 0; long perMode[8] = {0}, badMode[8] = {0};
    for (long t = 0; t < trials; ++t) {
        int mode = 0;
        std::vector<Span> s = strand(g, mode);
        const bool o = oldCrosses(s);
        const bool n = !cg::selfCrossingCuts(s, true).empty();
        const auto oc = oldGreedy(s);
        const auto nc = cg::selfCrossingCuts(s);
        perMode[mode]++;
        pos += o; poscuts += !oc.empty();
        if (o != n || oc != nc) {
            if (o != n) ++bad1; if (oc != nc) ++badc; badMode[mode]++;
            if (bad1 + badc <= 6) {
                std::printf("DIFF trial %ld mode %d old %d new %d cutsOld %zu cutsNew %zu n=%zu\n", t, mode, o, n, oc.size(), nc.size(), s.size());
                for (size_t a = 0; a < s.size(); ++a) for (size_t b = a + 2; b < s.size(); ++b) if (pairMeets(s[a], s[b])) {
                    Box2 ba = bounds(s[a]), bb = bounds(s[b]);
                    double dx = std::max({0.0, ba.min.x - bb.max.x, bb.min.x - ba.max.x}), dy = std::max({0.0, ba.min.y - bb.max.y, bb.min.y - ba.max.y});
                    std::printf("   meets %zu(%s len %.3g) %zu(%s len %.3g) box gap %.3g %.3g\n", a, std::holds_alternative<LineSpan>(s[a]) ? "L" : "A", length(s[a]), b,
                                std::holds_alternative<LineSpan>(s[b]) ? "L" : "A", length(s[b]), dx, dy);
                }
            }
        }
    }
    std::printf("sx trials %ld seed %#llx: old-positive %ld (greedy %ld), yes/no differ %ld, cuts differ %ld | per mode", trials, (unsigned long long)seed, pos, poscuts, bad1, badc);
    for (int m = 0; m < 8; ++m) std::printf(" %d:%ld/%ld", m, badMode[m], perMode[m]);
    std::printf("\n");
    return 0;
}


// ---- meetReach soundness ----
static double boxGap(const Box2& a, const Box2& b) {
    return std::max({0.0, a.min.x - b.max.x, b.min.x - a.max.x, a.min.y - b.max.y, b.min.y - a.max.y});
}
static int runReach(long trials, uint64_t seed) {
    G g{std::mt19937_64(seed)};
    long meets = 0, miss = 0; double worst = 0; long perK[6] = {0}; double worstK[6] = {0};
    const double eps = std::numeric_limits<double>::epsilon();
    for (long t = 0; t < trials; ++t) {
        const int kind = g.i(0, 5);
        const double big = std::vector<double>{0, 1, 1e3, 1e5, 1e6, 3e7}[g.i(0, 5)];
        const Vec2 o{big * g.u(-1, 1), big * g.u(-1, 1)};
        Span a, b;
        if (kind <= 2) {  // line vs arc tangent-ish, long line, tiny/huge r, contact near a line end
            const double L = g.logu(1e-4, 1e5);
            const double r = kind == 0 ? g.logu(1e-12, 1e-2) : (kind == 1 ? g.logu(1e-3, 1e3) : g.logu(1e3, 1e8));
            const double th = g.u(0, 2 * kPi);
            const Vec2 dir = Vec2::fromAngle(th, 1.0), n = dir.perp();
            const double tc = std::vector<double>{0, 1, g.u(0, 1), -1e-9, 1 + 1e-9, 1e-12}[g.i(0, 5)];
            const Vec2 s0 = o, s1 = o + dir * L;
            const Vec2 foot = s0 + dir * (L * tc);
            const double gscale = std::vector<double>{1e-10 * r, std::sqrt(eps) * (L + r), 1e-9, 64 * eps * std::max({1.0, r, std::fabs(o.x), std::fabs(o.y)}), eps * L * L / std::max(r, 1e-300)}[g.i(0, 4)];
            const double gap = g.u(-1, 3) * gscale;
            const double side = g.u(0, 1) < 0.5 ? 1 : -1;
            const Vec2 c = foot + n * side * (r + gap);
            const double ac = (foot - c).angle();
            const double half = std::min(3.0, g.logu(1e-9, 3.0));
            const double off = g.u(-1, 1) * half;
            ArcSpan A = arcFrom(c, r, ac - half + off, 2 * half * (g.u(0, 1) < 0.5 ? 1 : -1));
            if (A.direction == ArcDirection::CW) A = arcFrom(c, r, ac + half + off, -2 * half);
            a = LineSpan{g.u(0,1)<0.5 ? s0 : s1, g.u(0,1)<0.5 ? s1 : s0};
            if (std::get<LineSpan>(a).start == std::get<LineSpan>(a).end) a = LineSpan{s0, s1};
            b = A;
        } else if (kind <= 4) {  // arc-arc near tangency, incl. near-concentric
            const double r1 = g.logu(1e-6, 1e6);
            const double r2 = kind == 3 ? g.logu(1e-6, 1e6) : r1 * (1 + g.u(-1, 1) * g.logu(1e-16, 1e-3));
            const double gap = g.u(-1, 1.5) * std::vector<double>{1e-9, 1e-12, 1e-10 * std::max(r1, r2), 1e-15}[g.i(0, 3)];
            const int topo = kind == 4 ? 1 : g.i(0, 1);
            double dist = topo == 0 ? r1 + r2 + gap : std::fabs(r1 - r2) - gap;
            if (dist <= 0) dist = g.logu(1e-12, 1e-6);
            const Vec2 u = Vec2::fromAngle(g.u(0, 2 * kPi), 1.0);
            const Vec2 c1 = o, c2 = o + u * dist;
            const double a1 = u.angle() + g.u(-1e-6, 1e-6);
            const double a2 = topo == 0 ? a1 + kPi : (r2 > r1 ? a1 : a1);
            const double h1 = std::min(3.0, g.logu(1e-9, 3.0)), h2 = std::min(3.0, g.logu(1e-9, 3.0));
            a = arcFrom(c1, r1, a1 - h1 * g.u(0, 1), h1 * g.u(1, 2));
            b = arcFrom(c2, r2, a2 - h2 * g.u(0, 1), h2 * g.u(1, 2));
        } else {  // line-line near parallel crossing at an end
            const double L1 = g.logu(1e-4, 1e5), L2 = g.logu(1e-4, 1e5);
            const double th = g.u(0, 2 * kPi), dth = g.logu(1e-12, 1e-3) * (g.u(0,1)<0.5?1:-1);
            const Vec2 d1 = Vec2::fromAngle(th, 1.0), d2 = Vec2::fromAngle(th + dth, 1.0);
            const Vec2 p = o + d1 * (L1 * std::vector<double>{0, 1, g.u(0, 1)}[g.i(0, 2)]);
            a = LineSpan{o, o + d1 * L1};
            const double t2 = std::vector<double>{0, 1, g.u(0, 1)}[g.i(0, 2)];
            const Vec2 q = p + d1.perp() * g.u(-1, 1) * 1e-9 * L1;
            b = LineSpan{q - d2 * (L2 * t2), q + d2 * (L2 * (1 - t2))};
        }
        for (int sw = 0; sw < 2; ++sw) {
            const Span& x = sw ? b : a; const Span& y = sw ? a : b;
            if (!cg::spansMeet(x, y)) continue;
            ++meets; ++perK[kind];
            const Box2 bx = bounds(x), by = bounds(y);
            const double need = boxGap(bx, by);
            const double have = cg::meetReach(x, bx) + cg::meetReach(y, by);
            const double ratio = need / have;
            // also the exact check selfCrossingCuts makes
            const bool touch = bx.inflated(cg::meetReach(x, bx)).intersects(by.inflated(cg::meetReach(y, by)));
            if (ratio > worstK[kind]) worstK[kind] = ratio;
            if (ratio > worst) { worst = ratio; }
            if (!touch) { if (++miss <= 5) std::printf("MISS kind %d trial %ld need %.3g have %.3g\n", kind, t, need, have); }
        }
    }
    std::printf("reach trials %ld seed %llu: meets %ld miss %ld worst need/have %.3g | per kind", trials, (unsigned long long)seed, meets, miss, worst);
    for (int k = 0; k < 6; ++k) std::printf(" %d:%ld/%.3g", k, perK[k], worstK[k]);
    std::printf("\n");
    return miss != 0;
}

int main(int argc, char** argv) {
    if (argc < 4) { std::printf("usage: wb sx|reach trials seed\n"); return 2; }
    const long n = atol(argv[2]); const uint64_t s = strtoull(argv[3], nullptr, 0);
    if (!std::strcmp(argv[1], "sx")) return runSx(n, s);
    return runReach(n, s);
}
