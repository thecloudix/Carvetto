// Closed loops of arcs only (the T-0017 generator, more loops and places): junction gaps / back-steps of the
// open-offset output; the I lib prints EJNEW / EJGAP on stderr. Built against the branch (I) or e87f549 (cvmain).
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <optional>
#include <vector>
#include <variant>
#include "carvetto/geometry/offset.hpp"
#include "offset_chain_checks.hpp"
using namespace carvetto::geometry;
const Tolerance kTol{};
struct SplitMix { std::uint64_t s; explicit SplitMix(std::uint64_t v) : s(v) {} std::uint64_t next() { std::uint64_t z = (s += 0x9E3779B97F4A7C15ULL); z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL; z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL; return z ^ (z >> 31); }
  double uniform(double a, double b) { return a + (b - a) * ((next() >> 11) * (1.0 / 9007199254740992.0)); } double logUniform(double a, double b) { return std::exp(uniform(std::log(a), std::log(b))); } };
struct JunctionStats {
    std::size_t junctions{0};
    std::size_t gaps{0};          // end/start further apart than 1e-12
    std::size_t notIdentical{0};  // a junction with a line on either side that is not bit-identical
    std::size_t backward{0};      // next span starts behind the previous end, or retraces it
    double maxGap{0.0};
};

// One junction a -> b: gap and bit-identity. Backward steps are counted per chain by
// countBackwardSteps (offset_chain_checks.hpp): starting behind the previous end, retracing it
// (a near-reversal passes only as a needle tip whose sides diverge), and short spans that run
// against both neighbours.
void checkJunction(const Span& a, const Span& b, JunctionStats& st) {
    ++st.junctions;
    const Vec2 e = endPoint(a);
    const Vec2 s = startPoint(b);
    const double gap = distance(e, s);
    st.maxGap = std::max(st.maxGap, gap);
    if (gap > 1e-12) {
        ++st.gaps;
    }
    const bool lineSide = std::holds_alternative<LineSpan>(a) || std::holds_alternative<LineSpan>(b);
    if (lineSide && !(e == s)) {
        ++st.notIdentical;
    }
}

void checkJunctions(const std::vector<Contour>& chains, JunctionStats& st) {
    for (const Contour& c : chains) {
        const std::size_t n = c.spans.size();
        for (std::size_t k = 1; k < n; ++k) {
            checkJunction(c.spans[k - 1], c.spans[k], st);
        }
        if (c.closed && n > 1) {
            checkJunction(c.spans[n - 1], c.spans[0], st);
        }
        st.backward += carvetto::geometry::testing::countBackwardSteps(c, kTol);
    }
}

double ulpSteps(double v, int k) {
    const double inf = std::numeric_limits<double>::infinity();
    for (; k > 0; --k) {
        v = std::nextafter(v, inf);
    }
    for (; k < 0; ++k) {
        v = std::nextafter(v, -inf);
    }
    return v;
}

// A closed loop of arcs through `pts` (sweeps in `sweeps`, signed: > 0 CCW), made bit-continuous
// by nudging each arc's centre and radius a few ulps onto the previous end and the last arc's
// end angle onto the first start. Empty where that does not succeed.
std::optional<Contour> continuousArcLoop(const std::vector<Vec2>& pts, const std::vector<double>& sweeps) {
    Contour c;
    c.closed = true;
    const std::size_t n = pts.size();
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 p0 = pts[i];
        const Vec2 p1 = pts[(i + 1) % n];
        const double sw = sweeps[i];
        const Vec2 chord = p1 - p0;
        const double len = chord.length();
        const double r = len / (2.0 * std::sin(std::fabs(sw) / 2.0));
        const Vec2 normal = chord.perp() / len;
        const double h = std::sqrt(std::max(0.0, r * r - len * len / 4.0));
        const Vec2 centre = (p0 + p1) * 0.5 + normal * (sw > 0.0 ? h : -h);
        c.spans.emplace_back(ArcSpan{centre, r, (p0 - centre).angle(), (p1 - centre).angle(),
                                     sw > 0.0 ? ArcDirection::CCW : ArcDirection::CW});
    }
    for (std::size_t i = 1; i < n; ++i) {
        const Vec2 j = endPoint(c.spans[i - 1]);
        auto& arc = std::get<ArcSpan>(c.spans[i]);
        ArcSpan base = arc;
        base.centre = j - Vec2::fromAngle(arc.startAngle, arc.radius);
        bool hit = base.startPoint() == j;
        ArcSpan found = base;
        for (int dr = -4; dr <= 4 && !hit; ++dr) {
            for (int dx = -4; dx <= 4 && !hit; ++dx) {
                for (int dy = -4; dy <= 4 && !hit; ++dy) {
                    found = base;
                    found.radius = ulpSteps(base.radius, dr);
                    found.centre = Vec2{ulpSteps(base.centre.x, dx), ulpSteps(base.centre.y, dy)};
                    hit = found.startPoint() == j;
                }
            }
        }
        if (!hit) {
            return std::nullopt;
        }
        arc = found;
    }
    const Vec2 j0 = startPoint(c.spans[0]);
    auto& last = std::get<ArcSpan>(c.spans[n - 1]);
    const double e0 = last.endAngle + std::remainder((j0 - last.centre).angle() - last.endAngle, 2.0 * kPi);
    for (int k = -64; k <= 64; ++k) {
        ArcSpan t = last;
        t.endAngle = ulpSteps(e0, k);
        if (t.endPoint() == j0) {
            last = t;
            return c;
        }
    }
    return std::nullopt;
}

int main(int argc, char** argv) {
    const int N = argc > 1 ? std::atoi(argv[1]) : 1000; SplitMix rng(argc > 2 ? std::strtoull(argv[2], nullptr, 0) : 0x7017A0C5u);
    const Vec2 places[] = {{1e4, -8e3}, {-1e5, -4e4}, {3e4, 7e4}, {2e5, 1.3e5}, {0, 0}, {3.3e5, -2.1e5}};
    const JoinType joins[] = {JoinType::Round, JoinType::Sharp, JoinType::Square};
    long loops = 0, offsets = 0, arcLoops = 0, gaps = 0, notId = 0, back = 0, bigGap = 0; double maxRel = 0;
    while (loops < N) {
        const Vec2 at = places[loops % 6];
        const int n = 3 + static_cast<int>(rng.uniform(0.0, 4.0)); const double radius = rng.logUniform(0.3, 30.0); const double a0 = rng.uniform(0.0, 2.0 * kPi);
        std::vector<Vec2> pts; std::vector<double> sweeps;
        for (int i = 0; i < n; ++i) { const double a = a0 + 2.0 * kPi * (i + rng.uniform(-0.3, 0.3)) / n; pts.push_back(at + Vec2::fromAngle(a, radius * rng.uniform(0.6, 1.4))); const double sw = rng.uniform(0.2, 2.6); sweeps.push_back(rng.uniform(0.0, 1.0) < 0.3 ? -sw : sw); }
        const double d = rng.logUniform(0.01, 3.0); const OffsetSide side = rng.uniform(0.0, 1.0) < 0.5 ? OffsetSide::Left : OffsetSide::Right;
        const std::optional<Contour> in = continuousArcLoop(pts, sweeps); if (!in) continue; ++loops;
        for (const JoinType join : joins) for (int k = -4; k <= 4; ++k) {
            const auto chains = offsetOpenContour(*in, ulpSteps(d, k), side, join, kTol, 2.0); ++offsets;
            for (const Contour& c : chains) { bool oa = std::all_of(c.spans.begin(), c.spans.end(), [](const Span& sp) { return std::holds_alternative<ArcSpan>(sp); }); if (c.closed && c.spans.size() >= 2 && oa) ++arcLoops;
              const std::size_t m = c.spans.size(); for (std::size_t q = c.closed ? 0 : 1; q < m; ++q) { Vec2 e = endPoint(c.spans[(q + m - 1) % m]), s = startPoint(c.spans[q]); double sc = std::max({1.0, std::fabs(e.x), std::fabs(e.y)}); maxRel = std::max(maxRel, distance(e, s) / sc); if (distance(e, s) > 1e-12 * sc) { ++bigGap; if (bigGap <= 10) std::printf("GAP loop %ld k %d join %d gap %.3g rel %.3g closed %d m %zu q %zu\n", loops, k, (int)join, distance(e, s), distance(e, s) / sc, (int)c.closed, m, q); } } }
            JunctionStats js; checkJunctions(chains, js); notId += js.notIdentical; back += js.backward; if (js.backward && std::getenv("SHOWBACK")) std::printf("BACK loop %ld place (%g,%g) n %d d %.17g k %d join %d side %d backward %zu\n", loops, at.x, at.y, n, d, k, (int)join, (int)side, js.backward);
        }
    }
    std::printf("arcloops N=%d offsets %ld closedArcLoops %ld | gaps>1e-12*scale %ld (max rel %.3g) notIdentical %ld backward %ld\n", N, offsets, arcLoops, bigGap, maxRel, notId, back);
}
