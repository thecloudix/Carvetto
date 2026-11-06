// T-0172 verifier white-box: #includes the branch offset.cpp to reach selfCrossingCuts / ClusterGrid.
//  wb sx <trials> <seed>   selfCrossingCuts vs the pairwise test it replaced (carrierIntersections + inUnit, j >= i+2),
//                          both the yes/no (partCrossesItself) and the greedy cuts; adversarial strands.
//  wb cg <trials> <seed>   ClusterGrid query sets vs brute force on adversarial box sets (+ build stats).
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

// ---- ClusterGrid ----
static std::vector<Box2> boxSet(G& g, int kind, size_t n) {
    std::vector<Box2> b;
    auto add = [&](Vec2 c, double w, double h) { b.push_back(Box2{c - Vec2{w / 2, h / 2}, c + Vec2{w / 2, h / 2}}); };
    switch (kind) {
    case 0: for (size_t i = 0; i < n; ++i) add({g.u(0, 100), g.u(0, 100)}, g.logu(1e-3, 5), g.logu(1e-3, 5)); break;  // uniform
    case 1: {  // dense cluster + one far item (the lhatch shape), several nested scales
        for (size_t i = 0; i < n; ++i) { double s = std::pow(10.0, -g.i(0, 6)); add({g.u(0, 1) * s, g.u(0, 1) * s}, s * 1e-3 * g.u(0, 1), s * 1e-3 * g.u(0, 1)); }
        add({1e5, 1e5}, 1, 1); break; }
    case 2: for (size_t i = 0; i < n; ++i) add({1.0, 1.0}, 0, 0); add({1e9, -1e9}, 0, 0); break;  // all identical points + far
    case 3: for (size_t i = 0; i < n; ++i) add({g.u(0, 1e-9), 3.0}, 0, g.logu(1e-12, 1)); break;  // degenerate widths
    case 4: for (size_t i = 0; i < n; ++i) add({g.u(-1, 1) * 1e8, g.u(-1, 1) * 1e-8}, g.logu(1e-6, 1e6), 0); break;  // huge spread, flat
    case 5: {  // hatch at pitch 0.004 (lines 10 long) + far U
        for (size_t i = 0; i < n; ++i) add({5, 0.004 * double(i)}, 10, 0); add({1e5, 1e5}, 2, 2); break; }
    case 6: {  // items each straddling cell boundaries (sizes ~ cell), clustered
        for (size_t i = 0; i < n; ++i) add({g.u(0, 1e-3), g.u(0, 1e-3)}, g.logu(1e-6, 1e-3), g.logu(1e-6, 1e-3)); add({1e4, 0}, 0, 0); add({-1e4, 7}, 0, 0); break; }
    case 7: {  // 3+ levels: clusters within clusters within clusters
        for (size_t i = 0; i < n; ++i) { double s = 1.0; Vec2 c{0, 0}; for (int l = 0; l < 5; ++l) { c = c + Vec2{g.u(0, 1) * s, g.u(0, 1) * s}; s *= 1e-3; } add(c, s * g.u(0, 2), s * g.u(0, 2)); }
        add({1e3, 1e3}, 0, 0); break; }
    default: for (size_t i = 0; i < n; ++i) add({g.u(0, 1), g.u(0, 1)}, 2, 2);  // all cover everything
    }
    return b;
}

static int runCg(long trials, uint64_t seed) {
    G g{std::mt19937_64(seed)};
    long bad = 0, queries = 0;
    double worstBuild = 0;
    for (long t = 0; t < trials; ++t) {
        const int kind = static_cast<int>(t % 9);
        size_t n = static_cast<size_t>(g.logu(1, 20000));
        if (kind == 3) n = std::min<size_t>(n, 40);  // thin overlapping boxes: build blows up beyond (see cgone)
        if (kind == 6) n = std::min<size_t>(n, 3000);
        if (kind == 8) n = std::min<size_t>(n, 400);  // heavily overlapping big boxes: entries explode beyond  // clustered log-uniform sizes: entries grow ~n^2 (see cgone)
        if (std::getenv("CGV")) std::fprintf(stderr, "gen kind %d n %zu\n", kind, n);
        std::vector<Box2> layout = boxSet(g, kind, n);
        if (std::getenv("CGV")) std::fprintf(stderr, "generated\n");
        const bool fill = g.u(0, 1) < 0.5;
        auto t0 = std::chrono::steady_clock::now();
        cg::ClusterGrid grid(layout, fill);
        std::vector<Box2> items;
        if (fill) items = layout;
        else {  // add the layout items, some extra outside the layout, in random order
            items = layout;
            std::vector<Box2> extra = boxSet(g, static_cast<int>(g.i(0, 8)), static_cast<size_t>(g.i(0, 50)));
            for (auto& e : extra) { e.min = e.min * 3.0 + Vec2{-5, 7}; e.max = e.max * 3.0 + Vec2{-5, 7}; }
            items.insert(items.end(), extra.begin(), extra.end());
            std::shuffle(items.begin(), items.end(), g.r);
            for (const Box2& b : items) grid.add(b);
        }
        double bt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (std::getenv("CGV")) std::printf("trial %ld kind %d n %zu fill %d build %.3g s levels %zu\n", t, kind, n, fill, bt, grid.levels_.size()), std::fflush(stdout);
        worstBuild = std::max(worstBuild, bt / std::max<size_t>(n, 1));
        for (int q = 0; q < 200; ++q) {
            const Box2& base = items[static_cast<size_t>(g.i(0, static_cast<int>(items.size()) - 1))];
            const double gr = g.logu(1e-15, 1e3) * (g.u(0, 1) < 0.2 ? 0 : 1);
            Box2 Q = base.inflated(gr);
            if (q % 7 == 0) Q = Box2{base.min, base.min};  // corner point
            if (q % 11 == 0) Q = Box2{Vec2{-1e12, -1e12}, Vec2{1e12, 1e12}};
            std::vector<size_t> got, want;
            grid.query(Q, [&](size_t i) { got.push_back(i); });
            for (size_t i = 0; i < items.size(); ++i) if (items[i].intersects(Q)) want.push_back(i);
            std::sort(got.begin(), got.end());
            bool dup = std::adjacent_find(got.begin(), got.end()) != got.end();
            ++queries;
            if (got != want || dup) {
                if (++bad <= 5) std::printf("CGDIFF trial %ld kind %d n %zu fill %d got %zu want %zu dup %d\n", t, kind, n, fill, got.size(), want.size(), dup);
            }
            // queryUntil stops at first and reports true iff any
            bool any = grid.queryUntil(Q, [&](size_t) { return true; });
            if (any != !want.empty()) { if (++bad <= 5) std::printf("CGUNTIL trial %ld\n", t); }
        }
    }
    std::printf("cg trials %ld seed %#llx: queries %ld, wrong %ld, worst build %.3g s/item\n", trials, (unsigned long long)seed, queries, bad, worstBuild);
    return 0;
}

int main(int argc, char** argv) {
    if (argc < 4) return 2;
    long trials = std::atol(argv[2]); uint64_t seed = std::strtoull(argv[3], nullptr, 0);
    if (!std::strcmp(argv[1], "sx")) return runSx(trials, seed);
    if (!std::strcmp(argv[1], "cg")) return runCg(trials, seed);
    return 2;
}
