// T-0009 round 4: regression seeds of the T-0009 verifier (rounds 2 and 3), judged against the input
// as given by an independent check that allows only what the owner decisions allow:
//  (a) a corner between two sub-tolerance spans is Round whatever the join (no mitre region there);
//  (c) a sub-tolerance near-reversal never flips the offset to the other side: around a cluster of
//      sub-tolerance spans holding a near-reversal (a corner turning by more than 135 degrees at such
//      a span, or a tiny arc sweeping more than that), or whose turns add up to more than half a
//      turn, the loci of its own spans and corners are not required; instead the corner it
//      collapses to (end of the span A before it, tangents of A and of the span B after it) is,
//      within a tolerance-sized slack (3 x tol), and a Sharp/Square mitre may fill that corner.
// Everything else is exact: every result point at least d from the input (Round: at most d),
// every valid point of the requested-side offset of a span or of a convex corner (at least d + tol/2
// from the rest of the input; nearer, the offset may run along a strip narrower than the tolerance,
// documented since T-0017) within the tolerance of the result, and for Sharp/Square every point
// beyond d inside the mitre or chamfer region of a corner that keeps the requested join.

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/offset.hpp"
#include "offset_chain_checks.hpp"
#include "offset_subtol_cases.hpp"

using namespace carvetto::geometry;
namespace sc = carvetto::geometry::testing::subtol_cases;

namespace {

const Tolerance kTol{};  // distance 0.001
constexpr double kCos135 = -0.70710678118654752;

double dSpan(Vec2 p, const Span& s) {
    return carvetto::geometry::testing::distanceToSpanExact(p, s);
}

struct Judge {
    std::vector<Span> in;  // spans of positive length, in order
    bool closed{false};
    double d{0.0};
    double sgn{1.0};
    JoinType join{JoinType::Round};
    double mitre{2.0};
    std::vector<int> clusterOf;                 // cluster index per span, -1 if none
    std::vector<std::pair<int, int>> clusters;  // [first, last] span indices

    bool tiny(int k) const { return length(in[static_cast<std::size_t>(k)]) <= kTol.distance; }
    int count() const { return static_cast<int>(in.size()); }
    int next(int k) const { return k + 1 < count() ? k + 1 : (closed ? 0 : -1); }
    int prev(int k) const { return k > 0 ? k - 1 : (closed ? count() - 1 : -1); }
    const Span& at(int k) const { return in[static_cast<std::size_t>(k)]; }

    double distIn(Vec2 p, int skipA = -1, int skipB = -1, bool skipCluster = false, int cl = -1) const {
        double best = 1e300;
        for (int i = 0; i < count(); ++i) {
            if (i == skipA || i == skipB || (skipCluster && clusterOf[static_cast<std::size_t>(i)] == cl)) {
                continue;
            }
            best = std::min(best, dSpan(p, at(i)));
        }
        return best;
    }

    // Clusters of decision (c): windows of sub-tolerance spans around each near-reversal, grown
    // alternately backwards and forwards while their bounding box keeps a diagonal of 2 tol, joined
    // where they share or touch spans; and runs of sub-tolerance spans between longer spans, in such
    // a box, whose turns add up to more than half a turn.
    void findClusters() {
        const int n = count();
        clusterOf.assign(in.size(), -1);
        std::vector<std::pair<int, int>> windows;
        const double lim = 2.0 * kTol.distance * (1.0 + 1e-9);
        auto boxOf = [&](int lo, int hi) {
            Box2 b;
            for (int k = lo; k <= hi; ++k) {
                b.expand(bounds(at(k)));
            }
            return distance(b.min, b.max);
        };
        for (int k = 0; k < n; ++k) {
            const int nx = next(k);
            const auto* arc = std::get_if<ArcSpan>(&at(k));
            const bool arcSeed = arc && tiny(k) && std::fabs(arc->sweepAngle()) > 0.75 * kPi;
            const bool cornerSeed = nx >= 0 && (tiny(k) || tiny(nx)) &&
                                    tangentAt(at(k), 1.0).dot(tangentAt(at(nx), 0.0)) < kCos135;
            if (!arcSeed && !cornerSeed) {
                continue;
            }
            int lo = -1;
            int hi = -1;
            if (tiny(k) && boxOf(k, k) <= lim) {
                lo = hi = k;
            }
            if (cornerSeed && tiny(nx) && nx == k + 1 && boxOf(lo < 0 ? nx : lo, nx) <= lim) {
                lo = lo < 0 ? nx : lo;
                hi = nx;
            } else if (cornerSeed && tiny(nx) && lo < 0 && boxOf(nx, nx) <= lim) {
                lo = hi = nx;  // the closing corner of a closed path
            }
            if (lo < 0) {
                continue;
            }
            for (bool grew = true; grew;) {
                grew = false;
                if (lo > 0 && tiny(lo - 1) && boxOf(lo - 1, hi) <= lim) {
                    --lo;
                    grew = true;
                }
                if (hi + 1 < n && tiny(hi + 1) && boxOf(lo, hi + 1) <= lim) {
                    ++hi;
                    grew = true;
                }
            }
            windows.emplace_back(lo, hi);
        }
        for (int i = 0; i < n; ++i) {
            if (!tiny(i) || (i > 0 && tiny(i - 1)) || prev(i) < 0) {
                continue;
            }
            int j = i;
            while (j < n && tiny(j)) {
                ++j;
            }
            if (next(j - 1) < 0 || tiny(next(j - 1)) || boxOf(i, j - 1) > lim) {
                continue;
            }
            double turn = 0.0;
            for (int k = i; k <= j; ++k) {
                const Span& a = at(prev(k) < 0 ? k : prev(k));
                const Span& b = at(k < n ? k : next(j - 1));
                turn += std::atan2(tangentAt(a, 1.0).cross(tangentAt(b, 0.0)), tangentAt(a, 1.0).dot(tangentAt(b, 0.0)));
                if (k < j) {
                    if (const auto* arc = std::get_if<ArcSpan>(&at(k))) {
                        turn += arc->sweepAngle();
                    }
                }
            }
            if (std::fabs(turn) > kPi + 1e-9) {
                windows.emplace_back(i, j - 1);
            }
        }
        std::sort(windows.begin(), windows.end());
        for (const auto& w : windows) {
            if (!clusters.empty() && w.first <= clusters.back().second + 1) {
                clusters.back().second = std::max(clusters.back().second, w.second);
            } else {
                clusters.push_back(w);
            }
        }
        for (std::size_t c = 0; c < clusters.size(); ++c) {
            for (int k = clusters[c].first; k <= clusters[c].second; ++k) {
                clusterOf[static_cast<std::size_t>(k)] = static_cast<int>(c);
            }
        }
    }

    // Corner between spans a and b convex on the requested side (or, with `reversal`, also an exact
    // reversal, capped ahead of the travel): start normal and sweep of its arc about the vertex.
    bool corner(int a, int b, Vec2& n1, double& sweep, bool reversal = false) const {
        const Vec2 t1 = tangentAt(at(a), 1.0);
        const Vec2 t2 = tangentAt(at(b), 0.0);
        const bool rev = t1.dot(t2) < -0.999999;
        if (sgn * t1.cross(t2) >= 0.0 && !(reversal && rev)) {
            return false;
        }
        n1 = t1.perp() * sgn;
        sweep = std::remainder((t2.perp() * sgn).angle() - n1.angle(), kTwoPi);
        if (rev) {
            const double ahead = std::remainder(t1.angle() - n1.angle(), kTwoPi);
            sweep = ahead >= 0.0 ? std::fabs(sweep) : -std::fabs(sweep);
        }
        return true;
    }

    // Sharp/Square: inside the mitre or chamfer region of a corner at `v` between normals n1 and n2
    // (rotated by `sweep`), its second edge through `edge2` (the vertex, or B's start for a collapsed
    // corner), with arcs given an angular slack.
    bool inKite(Vec2 p, Vec2 v, Vec2 n1, double sweep, Vec2 edge2, bool arcs) const {
        const Vec2 q = p - v;
        const double lim = std::max(mitre, 1.5) * d;
        if (q.length() > lim * 1.000001 + 4.0 * kTol.distance) {
            return false;
        }
        const double rel = std::remainder(q.angle() - n1.angle(), kTwoPi);
        const double slack = arcs ? 0.2 : 1e-9;
        if (sweep >= 0.0 ? (rel < -slack || rel > sweep + slack) : (rel > slack || rel < sweep - slack)) {
            return false;
        }
        if (arcs) {
            return true;
        }
        const Vec2 n2 = Vec2::fromAngle(n1.angle() + sweep);
        const double e = 1e-9 * std::max(1.0, d);
        if (q.dot(n1) > d + e || (p - edge2).dot(n2) > d + e) {
            return false;
        }
        const Vec2 bis = (n1 + n2).normalized();
        return q.dot(bis) <= (join == JoinType::Square ? 1.5 * d : mitre * d) + e;
    }

    // A cluster reverses as a whole when the turn from A to B exceeds 135 degrees; then it is one
    // corner and the corners touching it keep no region of their own.
    bool reverses(int cl) const {
        const int a = prev(clusters[static_cast<std::size_t>(cl)].first);
        const int b = next(clusters[static_cast<std::size_t>(cl)].second);
        return a >= 0 && b >= 0 && a != b && tangentAt(at(a), 1.0).dot(tangentAt(at(b), 0.0)) < kCos135;
    }

    bool excused(Vec2 p) const {
        for (int a = 0; a < count(); ++a) {
            const int b = next(a);
            const int ca = clusterOf[static_cast<std::size_t>(a)];
            const int cb = b < 0 ? -1 : clusterOf[static_cast<std::size_t>(b)];
            if (b < 0 || (tiny(a) && tiny(b)) || (ca >= 0 && reverses(ca)) || (cb >= 0 && reverses(cb))) {
                continue;
            }
            if ((tiny(a) || tiny(b)) && tangentAt(at(a), 1.0).dot(tangentAt(at(b), 0.0)) < kCos135) {
                continue;
            }
            Vec2 n1;
            double sweep = 0.0;
            const Vec2 v = endPoint(at(a));
            if (corner(a, b, n1, sweep, true) &&
                inKite(p, v, n1, sweep, v,
                       std::holds_alternative<ArcSpan>(at(a)) || std::holds_alternative<ArcSpan>(at(b)))) {
                return true;
            }
        }
        for (const auto& [first, last] : clusters) {
            const int a = prev(first);
            const int b = next(last);
            Vec2 n1;
            double sweep = 0.0;
            if (a < 0 || b < 0 || a == b) {
                continue;
            }
            const Vec2 v = endPoint(at(a));
            if (corner(a, b, n1, sweep, true) &&
                inKite(p, v, n1, sweep, startPoint(at(b)),
                       std::holds_alternative<ArcSpan>(at(a)) || std::holds_alternative<ArcSpan>(at(b)))) {
                return true;
            }
        }
        return false;
    }
};

struct Verdict {
    double under{0.0};
    double over{0.0};
    double excess{0.0};
    double miss{0.0};
    int misses{0};
    std::string first;
};

double distOut(Vec2 p, const std::vector<Contour>& out) {
    double best = 1e300;
    for (const Contour& c : out) {
        for (const Span& s : c.spans) {
            best = std::min(best, dSpan(p, s));
        }
    }
    return best;
}

Verdict judge(const sc::SeedCase& c, JoinType join, OffsetSide side, const std::vector<Contour>& out) {
    Judge j;
    for (const Span& s : c.in.spans) {
        if (length(s) > 0.0) {
            j.in.push_back(s);
        }
    }
    j.closed = c.in.closed;
    j.d = c.d;
    j.sgn = side == OffsetSide::Left ? 1.0 : -1.0;
    j.join = join;
    j.mitre = c.mitre;
    j.findClusters();
    const double t = kTol.distance;
    const double allow = std::max(t * t / (8.0 * c.d), std::min(5e-7 * std::max(1.0, c.d), 0.1 * t * t / c.d));
    Verdict v;
    for (const Contour& ch : out) {
        for (const Span& s : ch.spans) {
            for (int q = 0; q <= 400; ++q) {
                const Vec2 p = pointAt(s, q / 400.0);
                const double m = j.distIn(p);
                const double slack = allow + 1e-12 * std::max({1.0, c.d, std::fabs(p.x), std::fabs(p.y)});
                v.under = std::max(v.under, c.d - m - slack);
                if (join == JoinType::Round) {
                    v.over = std::max(v.over, m - c.d - slack);
                } else if (m > c.d + 1e-6 * std::max(1.0, c.d) && !j.excused(p)) {
                    if (m - c.d > v.excess && v.first.empty()) {
                        v.first = "excess at (" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")";
                    }
                    v.excess = std::max(v.excess, m - c.d);
                }
            }
        }
    }
    if (join != JoinType::Round) {
        return v;
    }
    const double tie = c.d + 0.5 * t;
    auto expect = [&](Vec2 q, double within) {
        const double o = distOut(q, out);
        if (o > within) {
            if (v.misses == 0) {
                v.first = "coverage miss at (" + std::to_string(q.x) + ", " + std::to_string(q.y) + ")";
            }
            ++v.misses;
            v.miss = std::max(v.miss, o);
        }
    };
    for (int i = 0; i < j.count(); ++i) {
        if (j.clusterOf[static_cast<std::size_t>(i)] >= 0) {
            continue;
        }
        const Span& s = j.at(i);
        for (int k = 1; k <= 9; ++k) {
            const Vec2 q = pointAt(s, k / 10.0) + tangentAt(s, k / 10.0).perp() * (j.sgn * c.d);
            if (std::fabs(dSpan(q, s) - c.d) <= 1e-9 * std::max(1.0, c.d) && j.distIn(q, i) >= tie) {
                expect(q, t);
            }
        }
    }
    for (int a = 0; a < j.count(); ++a) {
        const int b = j.next(a);
        if (b < 0 || j.clusterOf[static_cast<std::size_t>(a)] >= 0 || j.clusterOf[static_cast<std::size_t>(b)] >= 0) {
            continue;
        }
        Vec2 n1;
        double sweep = 0.0;
        const Vec2 vtx = endPoint(j.at(a));
        if (!j.corner(a, b, n1, sweep)) {
            continue;
        }
        for (int k = 1; k <= 4; ++k) {
            const Vec2 q = vtx + Vec2::fromAngle(n1.angle() + sweep * k / 5.0, c.d);
            if (j.distIn(q) >= c.d - 1e-9 && j.distIn(q, a, b) >= tie) {
                expect(q, t);
            }
        }
    }
    for (std::size_t ci = 0; ci < j.clusters.size(); ++ci) {
        const int a = j.prev(j.clusters[ci].first);
        const int b = j.next(j.clusters[ci].second);
        Vec2 n1;
        double sweep = 0.0;
        if (a < 0 || b < 0 || a == b) {
            continue;
        }
        const Vec2 vtx = endPoint(j.at(a));
        if (!j.corner(a, b, n1, sweep)) {
            continue;
        }
        for (int k = 0; k <= 8; ++k) {
            const Vec2 q = vtx + Vec2::fromAngle(n1.angle() + sweep * k / 8.0, c.d);
            if (j.distIn(q) >= c.d - 2.0 * t && j.distIn(q, a, b, true, static_cast<int>(ci)) >= tie) {
                expect(q, 3.0 * t);
            }
        }
    }
    return v;
}

void requireClean(const Verdict& v) {
    CAPTURE(v.first, v.under, v.over, v.excess, v.misses, v.miss);
    CHECK(v.under <= 0.0);
    CHECK(v.over <= 0.0);
    CHECK(v.excess == 0.0);
    CHECK(v.misses == 0);
}

const JoinType kJoins[] = {JoinType::Round, JoinType::Sharp, JoinType::Square};
const OffsetSide kSides[] = {OffsetSide::Left, OffsetSide::Right};

std::vector<Contour> offsetOf(const sc::SeedCase& c, JoinType join, OffsetSide side) {
    return offsetOpenContour(c.in, c.d, side, join, kTol, c.mitre);
}

void judgeAll(const sc::SeedCase& c) {
    for (const JoinType join : kJoins) {
        for (const OffsetSide side : kSides) {
            CAPTURE(static_cast<int>(join), static_cast<int>(side));
            std::vector<Contour> out;
            REQUIRE_NOTHROW(out = offsetOf(c, join, side));
            requireClean(judge(c, join, side, out));
        }
    }
}

int closedChains(const std::vector<Contour>& out) {
    return static_cast<int>(std::count_if(out.begin(), out.end(), [](const Contour& k) { return k.closed; }));
}

}  // namespace

TEST_CASE("T-0009 round 4: an arc's inner offset next to a sub-tolerance spike keeps its corner (0xf0003fad)",
          "[geometry][offset][open][t0009]") {
    // Round 2 lost 1.6 to 2.5 mm of the inner offset of a large arc, Sharp and Square, Left: a
    // 2e-15 mm span at the path's end turning by 143 degrees kept a chamfer, and the offset of a
    // 4.5e-15 mm span (its direction swamped by rounding at 24 mm from it) was mitred 3.8 mm off.
    const sc::SeedCase c = sc::v9_f0003fad();
    judgeAll(c);
    for (const JoinType join : {JoinType::Sharp, JoinType::Square}) {
        const auto out = offsetOf(c, join, OffsetSide::Left);
        CHECK(distOut(Vec2{-11.0357250261285, 1.6731309704291832}, out) <= kTol.distance);
    }
    // Round, Left: the arc about the tip of the spike of spans 20/21 (round 3 lost it, main is 3e-4 off).
    const auto out = offsetOf(c, JoinType::Round, OffsetSide::Left);
    const Vec2 tip{-11.464758455798535, 13.414282157728405};
    for (int deg = 85; deg <= 141; deg += 4) {
        CAPTURE(deg);
        CHECK(distOut(tip + Vec2::fromAngle(deg * kPi / 180.0, c.d), out) <= kTol.distance);
    }
}

TEST_CASE("T-0009 round 4: a closed loop stays closed past a sub-tolerance near-reversal (0xe0000119, 0x35000108)",
          "[geometry][offset][open][t0009]") {
    // Round 2 opened the loop of 0xe0000119 (Round, Right) with a 0.67 mm gap; round 3 opened that of
    // 0x35000108 (Round, Left, a cluster whose turns add up to +185 degrees, no corner beyond 135),
    // 10.6 mm. Main keeps both closed.
    const sc::SeedCase e = sc::v9_e0000119();
    const auto outE = offsetOf(e, JoinType::Round, OffsetSide::Right);
    CHECK(outE.size() == 1);
    CHECK(closedChains(outE) == 1);
    judgeAll(e);
    const sc::SeedCase f = sc::v9_35000108();
    const auto outF = offsetOf(f, JoinType::Round, OffsetSide::Left);
    CHECK(outF.size() == 1);
    CHECK(closedChains(outF) == 1);
    judgeAll(f);
}

TEST_CASE("T-0009 round 4: collapsed clusters keep the offset next to them (round-3 F1, F5 seeds)",
          "[geometry][offset][open][t0009]") {
    judgeAll(sc::v9_30000054());
    judgeAll(sc::v9_33000048());
    const sc::SeedCase a = sc::v9_312003dd();
    judgeAll(a);
    const auto outA = offsetOf(a, JoinType::Round, OffsetSide::Left);
    CHECK(distOut(Vec2{-10003.171654950755, -29.95412398179927}, outA) <= kTol.distance);
    // 1.4e-4 inside d (a cluster point sticks out): covered within the slack of decision (c).
    CHECK(distOut(Vec2{-9985.667869745781, -24.22555833465961}, outA) <= 3.0 * kTol.distance);
    const sc::SeedCase b = sc::v9_31600b4f();
    judgeAll(b);
    CHECK(distOut(Vec2{-10001.27500410687, -933.55003557966552}, offsetOf(b, JoinType::Round, OffsetSide::Left)) <=
          kTol.distance);
}

TEST_CASE("T-0009 round 4: no crash on a closed loop of 133 sub-tolerance spans (g2 0x32000123)",
          "[geometry][offset][open][t0009]") {
    // Round 3 threw std::length_error (a NaN length turned into a sample count) for every join.
    judgeAll(sc::g2_32000123());
}

TEST_CASE("T-0009 round 4: a Sharp/Square corner at a sub-tolerance span stays in its own corner region (F4)",
          "[geometry][offset][open][t0009]") {
    const sc::SeedCase a = sc::v9_313007af();
    judgeAll(a);
    CHECK(distOut(Vec2{-16.840201239003022, 19.60592157528097}, offsetOf(a, JoinType::Sharp, OffsetSide::Left)) <=
          kTol.distance);
    const sc::SeedCase b = sc::v9_b0300988();
    judgeAll(b);
    CHECK(distOut(Vec2{15.2473187657562, -3.9359180763716743}, offsetOf(b, JoinType::Sharp, OffsetSide::Left)) <=
          kTol.distance);
    const sc::SeedCase c = sc::v9_b0001625();
    judgeAll(c);
    CHECK(distOut(Vec2{-0.79198606434071439, 2.0805226387695623}, offsetOf(c, JoinType::Sharp, OffsetSide::Right)) <=
          kTol.distance);
}

TEST_CASE("T-0009 round 4: a cluster that would wrap round is joined as one corner on the requested side only",
          "[geometry][offset][open][t0009]") {
    // A line east, a hook of two sub-tolerance spans turning left by 95 and 90 degrees (no corner
    // beyond 135), then a line on at 185 degrees: in all the path turns left by 185 degrees, which is
    // a right turn of 175 degrees. Left (the outside of that turn, here the outside of the closed
    // shape) gets the cap round the tip and the loop stays closed; Right gets no cap wrapped round
    // the outside of the tip.
    Contour c;
    c.closed = true;
    const Vec2 p0{0.0, 0.0};
    const Vec2 p1{10.0, 0.0};
    Vec2 p = p1;
    c.spans.emplace_back(LineSpan{p0, p1});
    for (const double deg : {95.0, 185.0}) {
        const Vec2 q = p + Vec2::fromAngle(deg * kPi / 180.0, 3e-4);
        c.spans.emplace_back(LineSpan{p, q});
        p = q;
    }
    const Vec2 far = p + Vec2::fromAngle(185.0 * kPi / 180.0, 10.0);
    c.spans.emplace_back(LineSpan{p, far});
    c.spans.emplace_back(LineSpan{far, p0});
    for (const JoinType join : kJoins) {
        CAPTURE(static_cast<int>(join));
        const auto left = offsetOpenContour(c, 1.0, OffsetSide::Left, join, kTol);
        REQUIRE(left.size() == 1);
        CHECK(left.front().closed);
        // Nothing of the Right offset lies beyond the tip (x > 10): the hook does not flip it.
        const auto right = offsetOpenContour(c, 1.0, OffsetSide::Right, join, kTol);
        for (const Contour& k : right) {
            for (const Span& s : k.spans) {
                for (int q = 0; q <= 20; ++q) {
                    CHECK(pointAt(s, q / 20.0).x <= 10.0 + 1e-9);
                }
            }
        }
    }
}
