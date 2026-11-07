// T-0180: work bounds of intersect(Bezier, Bezier) and the contour-level calls (ADR-0005 A2):
// the explicit budget with its Truncated status, up-front refusal of non-finite spans, the hash
// grid behind the contour-level duplicate test, and the callers' handling of a truncated answer.
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/edit_ops.hpp"
#include "carvetto/geometry/intersect.hpp"
#include "carvetto/geometry/span.hpp"
#include "cpu_scaling.hpp"
#include "cpu_time.hpp"
#include "intersect_defaults.hpp"
#include "test_rng.hpp"
#include "../toolpath/v_carve_test_util.hpp"
#include "carvetto/toolpath/v_carve.hpp"

using namespace carvetto::geometry;
using carvetto::test::processCpuSeconds;

namespace {

#ifdef NDEBUG
constexpr bool kRelease = true;
#else
constexpr bool kRelease = false;
#endif

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// Two long loops with coincident ends and nearly the same shape: the pair that made the old
// code run for tens of seconds (T-0178 verifier). `extent` scales the control points.
BezierSpan loopOf(double extent) {
    return BezierSpan{{0, 0}, {extent, extent}, {-extent, extent}, {0, 0}};
}
BezierSpan shiftedLoopOf(double extent) {
    return BezierSpan{{1e-4, 0}, {extent, extent}, {-extent, extent}, {0, 1e-4}};
}
// The same loop run backwards: every point is a touch (512 hits at extent 1000).
BezierSpan reversedLoopOf(double extent) {
    return loopOf(extent).reversed();
}

IntersectLimits pairLimit(std::uint64_t pair) {
    IntersectLimits l;
    l.pairWork = pair;
    return l;
}

// Every reported hit lies on both spans (to the contact tolerance).
bool hitsAreGenuine(const IntersectionResult& r, const Span& a, const Span& b, const Tolerance& tol) {
    return std::all_of(r.points.begin(), r.points.end(), [&](const Intersection& h) {
        return distance(pointAt(a, h.tA), pointAt(b, h.tB)) <= 1.5 * tol.distance + 1e-9;
    });
}

bool samePoints(const IntersectionResult& l, const IntersectionResult& r) {
    if (l.points.size() != r.points.size() || l.overlaps.size() != r.overlaps.size() || l.status != r.status) {
        return false;
    }
    for (std::size_t i = 0; i < l.points.size(); ++i) {
        const Intersection& x = l.points[i];
        const Intersection& y = r.points[i];
        if (x.point.x != y.point.x || x.point.y != y.point.y || x.tA != y.tA || x.tB != y.tB ||
            x.tangent != y.tangent) {
            return false;
        }
    }
    return true;
}

}  // namespace

TEST_CASE("Work budget: a small budget truncates, a large one reproduces the default exactly", "[bezier][intersect][budget][geometry]") {
    const Tolerance tol = Tolerance::defaults();
    const Span a = loopOf(1000.0);
    const Span b = reversedLoopOf(1000.0);
    const IntersectionResult full = intersect(a, b, tol);
    REQUIRE(full.complete());
    REQUIRE(full.status == IntersectStatus::Complete);
    REQUIRE(full.points.size() > 100);  // 512 in fact
    REQUIRE(hitsAreGenuine(full, a, b, tol));

    // Far more than the pair needs: the same answer, bit for bit, in both argument orders.
    const IntersectionResult big = intersect(a, b, tol, pairLimit(100'000'000));
    REQUIRE(samePoints(full, big));
    const IntersectionResult bigSwapped = intersect(b, a, tol, pairLimit(100'000'000));
    REQUIRE(bigSwapped.complete());
    REQUIRE(bigSwapped.points.size() == full.points.size());

    // Budgets that are too small say so, keep only genuine hits, and are deterministic.
    for (const std::uint64_t limit : {std::uint64_t{1}, std::uint64_t{2'000}, std::uint64_t{100'000}, std::uint64_t{600'000}}) {
        INFO("pairWork = " << limit);
        const double t0 = processCpuSeconds();
        const IntersectionResult cut = intersect(a, b, tol, pairLimit(limit));
        REQUIRE(processCpuSeconds() - t0 < 3.0);
        REQUIRE(cut.status == IntersectStatus::Truncated);
        REQUIRE_FALSE(cut.complete());
        REQUIRE(cut.points.size() <= full.points.size());
        REQUIRE(hitsAreGenuine(cut, a, b, tol));
        const IntersectionResult again = intersect(a, b, tol, pairLimit(limit));
        REQUIRE(samePoints(cut, again));
        const IntersectionResult cutSwapped = intersect(b, a, tol, pairLimit(limit));
        REQUIRE(cutSwapped.status == IntersectStatus::Truncated);
    }
    // The best-effort part is not empty once the budget buys something.
    REQUIRE_FALSE(intersect(a, b, tol, pairLimit(600'000)).points.empty());
}

TEST_CASE("Work budget: lines and arcs are closed-form and never truncate", "[bezier][intersect][budget][geometry]") {
    const Tolerance tol = Tolerance::defaults();
    const IntersectLimits none = pairLimit(0);
    const IntersectionResult r = intersect(Span{LineSpan{{0, 0}, {10, 10}}}, Span{LineSpan{{0, 10}, {10, 0}}}, tol, none);
    REQUIRE(r.complete());
    REQUIRE(r.points.size() == 1);
    const IntersectionResult ra =
        intersect(Span{LineSpan{{-20, 0}, {20, 0}}}, Span{ArcSpan::fullCircle({0, 0}, 5.0)}, tol, none);
    REQUIRE(ra.complete());
    REQUIRE(ra.points.size() == 2);
    // A Bezier pair with no budget at all is truncated at once (and empty), not wrong.
    const IntersectionResult rb = intersect(Span{loopOf(10.0)}, Span{shiftedLoopOf(10.0)}, tol, none);
    REQUIRE(rb.status == IntersectStatus::Truncated);
}

TEST_CASE("Work budget: a huge arc against a Bezier is charged before it is sampled", "[bezier][intersect][budget][geometry]") {
    // R = 1e12 with the default chordal tolerance would be ~6e7 samples (a gigabyte and seconds).
    const Tolerance tol = Tolerance::defaults();
    const ArcSpan arc{{0, 0}, 1e12, 0.0, 6.0, ArcDirection::CCW};
    const BezierSpan bz{{1e12 - 5, -5}, {1e12 + 5, 1}, {1e12 - 3, 4}, {1e12 + 9, 9}};
    const double t0 = processCpuSeconds();
    const IntersectionResult r = intersect(Span{arc}, Span{bz}, tol, pairLimit(50'000'000));
    // Sampling the circle would take seconds (and 1.4 GB): the charge refuses it up front.
    REQUIRE(processCpuSeconds() - t0 < 1.0);
    REQUIRE(r.status == IntersectStatus::Truncated);
}

TEST_CASE("Non-finite spans are refused up front", "[bezier][intersect][budget][nonfinite][geometry]") {
    const Tolerance tol = Tolerance::defaults();
    const BezierSpan okBez{{0, 0}, {5, 5}, {10, -5}, {20, 0}};
    const LineSpan okLine{{-5, 1}, {25, 1}};
    const ArcSpan okArc{{10, 0}, 5.0, 0.0, 3.0, ArcDirection::CCW};
    const double t0 = processCpuSeconds();
    for (const double bad : {kNaN, kInf, -kInf}) {
        std::vector<Span> broken;
        for (int field = 0; field < 8; ++field) {
            BezierSpan z = okBez;
            double* f[8] = {&z.p0.x, &z.p0.y, &z.p1.x, &z.p1.y, &z.p2.x, &z.p2.y, &z.p3.x, &z.p3.y};
            *f[field] = bad;
            broken.emplace_back(z);
        }
        for (int field = 0; field < 4; ++field) {
            LineSpan l = okLine;
            double* f[4] = {&l.start.x, &l.start.y, &l.end.x, &l.end.y};
            *f[field] = bad;
            broken.emplace_back(l);
        }
        for (int field = 0; field < 5; ++field) {
            ArcSpan arc = okArc;
            double* f[5] = {&arc.centre.x, &arc.centre.y, &arc.radius, &arc.startAngle, &arc.endAngle};
            *f[field] = bad;
            broken.emplace_back(arc);
        }
        for (const Span& s : broken) {
            for (const Span& partner : {Span{okBez}, Span{okLine}, Span{okArc}, s}) {
                const IntersectionResult r1 = intersect(s, partner, tol);
                REQUIRE(r1.status == IntersectStatus::InvalidInput);
                REQUIRE(r1.isEmpty());
                const IntersectionResult r2 = intersect(partner, s, tol);
                REQUIRE(r2.status == IntersectStatus::InvalidInput);
                REQUIRE(r2.isEmpty());
            }
            Contour c;
            c.append(okLine);
            c.append(s);
            Contour good;
            good.append(okBez);
            REQUIRE(intersect(c, good, tol).status == IntersectStatus::InvalidInput);
            REQUIRE(intersect(good, c, tol).status == IntersectStatus::InvalidInput);
            const ContourIntersections self = selfIntersections(c, tol);
            REQUIRE(self.status == IntersectStatus::InvalidInput);
            REQUIRE(self.isEmpty());
            // "Cannot rule a self-intersection out": true, and at once.
            REQUIRE(hasSelfIntersections(c, tol));
        }
    }
    // Before T-0180 most of these ran for tens of seconds each.
    REQUIRE(processCpuSeconds() - t0 < 5.0);
    // Finite neighbours of the refused values are still computed.
    const IntersectionResult fine = intersect(Span{okBez}, Span{okLine}, tol);
    REQUIRE(fine.complete());
    REQUIRE_FALSE(fine.points.empty());
}

TEST_CASE("Contour-level calls report truncation and stay conservative", "[bezier][intersect][budget][contour][geometry]") {
    const Tolerance tol = Tolerance::defaults();
    // A smooth S-curve of extent 1e5 has no self-intersection and ~2000 chords.
    Contour s;
    s.append(BezierSpan{{0, 0}, {1e5, 1e5}, {-1e5, 2e5}, {1e5, 3e5}});
    {
        const ContourIntersections r = selfIntersections(s, tol);
        REQUIRE(r.complete());
        REQUIRE(r.isEmpty());
        REQUIRE_FALSE(hasSelfIntersections(s, tol));
    }
    IntersectLimits tiny;
    tiny.totalWork = 5'000;
    const ContourIntersections cut = selfIntersections(s, tol, tiny);
    REQUIRE(cut.status == IntersectStatus::Truncated);
    {
        // The default limits of the overloads without a limits argument can be replaced in tests.
        const detail::ScopedDefaultLimits scoped(tiny);
        REQUIRE(selfIntersections(s, tol).status == IntersectStatus::Truncated);
        // No hit found, but not ruled out either: "may self-intersect".
        REQUIRE(hasSelfIntersections(s, tol));
    }
    REQUIRE_FALSE(hasSelfIntersections(s, tol));

    // Pair budget and total budget: a contour pair of two near-coincident loops.
    Contour a;
    a.append(loopOf(1000.0));
    Contour b;
    b.append(reversedLoopOf(1000.0));
    const ContourIntersections full = intersect(a, b, tol);
    REQUIRE(full.complete());
    REQUIRE(full.points.size() > 100);  // 512 in fact
    IntersectLimits pairOnly;
    pairOnly.pairWork = 100'000;
    const ContourIntersections cutPair = intersect(a, b, tol, pairOnly);
    REQUIRE(cutPair.status == IntersectStatus::Truncated);
    IntersectLimits totalOnly;
    totalOnly.totalWork = 100'000;
    const ContourIntersections cutTotal = intersect(a, b, tol, totalOnly);
    REQUIRE(cutTotal.status == IntersectStatus::Truncated);
    // The pair is held to what is left of the total, not just to pairWork: it is cut short, not
    // run to completion and only then found to have overspent.
    REQUIRE(cutTotal.points.size() < full.points.size());
    // Plenty of both: identical to the default.
    IntersectLimits plenty;
    plenty.pairWork = 1'000'000'000;
    plenty.totalWork = 4'000'000'000;
    const ContourIntersections same = intersect(a, b, tol, plenty);
    REQUIRE(same.complete());
    REQUIRE(same.points.size() == full.points.size());
    for (std::size_t i = 0; i < full.points.size(); ++i) {
        REQUIRE(same.points[i].spanA == full.points[i].spanA);
        REQUIRE(same.points[i].hit.tA == full.points[i].hit.tA);
        REQUIRE(same.points[i].hit.tB == full.points[i].hit.tB);
    }
    // Self-intersection of a contour whose pair hits the budget.
    Contour two;
    two.append(loopOf(1000.0));
    two.append(shiftedLoopOf(1000.0));
    IntersectLimits pairTiny;
    pairTiny.pairWork = 50'000;
    REQUIRE(selfIntersections(two, tol, pairTiny).status == IntersectStatus::Truncated);
}

namespace {

// A random contour of lines, arcs and Beziers on a coarse grid, so that many pairs meet at the
// same points and the duplicate test matters.
Contour randomContour(carvetto::test::SplitMix& g, int spans, double extent) {
    Contour c;
    const auto pt = [&] {
        return Vec2{std::round(g.uniform(-extent, extent)) * 0.5, std::round(g.uniform(-extent, extent)) * 0.5};
    };
    Vec2 at = pt();
    for (int i = 0; i < spans; ++i) {
        const Vec2 next = pt();
        switch (g.next() % 3) {
            case 0:
                c.append(LineSpan{at, next});
                break;
            case 1:
                c.append(BezierSpan{at, pt(), pt(), next});
                break;
            default:
                c.append(LineSpan{at, next});
                break;
        }
        at = next;
    }
    return c;
}

}  // namespace

TEST_CASE("Contour intersect equals the per-pair hits with a first-wins duplicate scan", "[intersect][contour][hitindex][geometry]") {
    // Reference: every span pair through the public span intersect, hits within tolerance of an
    // earlier one dropped (the contour call's rule), in (spanA, spanB) order.
    carvetto::test::SplitMix g{20180};
    for (int round = 0; round < 40; ++round) {
        const Tolerance tol = round % 3 == 0 ? Tolerance{0.05, 1e-9, 0.01} : Tolerance::defaults();
        const Contour a = randomContour(g, 2 + static_cast<int>(g.next() % 6), 4.0);
        const Contour b = randomContour(g, 2 + static_cast<int>(g.next() % 6), 4.0);
        std::vector<ContourIntersection> want;
        for (std::size_t i = 0; i < a.spans.size(); ++i) {
            for (std::size_t j = 0; j < b.spans.size(); ++j) {
                for (const Intersection& h : intersect(a.spans[i], b.spans[j], tol).points) {
                    const bool dup = std::any_of(want.begin(), want.end(), [&](const ContourIntersection& e) {
                        return tol.equalPoints(e.hit.point, h.point);
                    });
                    if (!dup) {
                        want.push_back({i, j, h});
                    }
                }
            }
        }
        const ContourIntersections got = intersect(a, b, tol);
        INFO("round " << round);
        REQUIRE(got.complete());
        REQUIRE(got.points.size() == want.size());
        for (std::size_t k = 0; k < want.size(); ++k) {
            REQUIRE(got.points[k].spanA == want[k].spanA);
            REQUIRE(got.points[k].spanB == want[k].spanB);
            REQUIRE(got.points[k].hit.tA == want[k].hit.tA);
            REQUIRE(got.points[k].hit.tB == want[k].hit.tB);
        }
    }
}

TEST_CASE("Contour hits that straddle a hash cell border are still merged", "[intersect][contour][hitindex][geometry]") {
    // Cells are 2 * tol.distance wide. Many crossings of one comb of lines with a baseline cut
    // into two pieces: the crossing points sit at every offset within a cell.
    const Tolerance tol = Tolerance::defaults();
    const double d = tol.distance;
    for (const double base : {0.0, 2.0 * d, 7.0 * d, -3.0 * d}) {
        for (const double gap : {0.0, 0.2 * d, 0.9 * d, 1.0 * d, 1.1 * d, 1.9 * d}) {
            Contour a;
            a.append(LineSpan{{base - 1.0, 0.0}, {base + 1.0, 0.0}});
            Contour b;
            // Two crossings of the line `a` `gap` apart along x (a V whose tip is below a).
            b.append(LineSpan{{base + gap * 0.5 - 0.5, 1.0}, {base + gap * 0.5, -1.0}});
            b.append(LineSpan{{base + gap * 0.5, -1.0}, {base - gap * 0.5 + 0.5, 1.0}});
            const ContourIntersections r = intersect(a, b, tol);
            std::vector<Vec2> kept;
            for (const ContourIntersection& h : r.points) {
                for (const Vec2& k : kept) {
                    INFO("base " << base << " gap " << gap);
                    REQUIRE(distance(k, h.hit.point) > d);  // no two reported hits within tolerance
                }
                kept.push_back(h.hit.point);
            }
            INFO("base " << base << " gap " << gap);
            REQUIRE_FALSE(kept.empty());
        }
    }
}

namespace {

// n crossings: a zigzag of n segments against a baseline cut into n pieces.
Contour zigzag(int n) {
    Contour c;
    for (int k = 0; k < n; ++k) {
        const double y0 = k % 2 == 0 ? -1.0 : 1.0;
        c.append(LineSpan{{static_cast<double>(k), y0}, {static_cast<double>(k + 1), -y0}});
    }
    return c;
}
Contour baseline(int n) {
    Contour c;
    for (int k = 0; k < n; ++k) {
        c.append(LineSpan{{static_cast<double>(k), 0.0}, {static_cast<double>(k + 1), 0.0}});
    }
    return c;
}
// The zigzag, a connector and the baseline back: n crossings within one contour.
Contour looped(int n) {
    Contour c = zigzag(n);
    c.append(LineSpan{{static_cast<double>(n), (n % 2 == 0) ? -1.0 : 1.0}, {static_cast<double>(n), 0.0}});
    for (int k = n; k > 0; --k) {
        c.append(LineSpan{{static_cast<double>(k), 0.0}, {static_cast<double>(k - 1), 0.0}});
    }
    return c;
}

}  // namespace

TEST_CASE("Contour-level intersect is near-linear in spans and hits", "[intersect][contour][perf][geometry]") {
    const Tolerance tol = Tolerance::defaults();
    const int nSmall = kRelease ? 2500 : 1000;
    const int nLarge = kRelease ? 10000 : 4000;
    const Contour zs = zigzag(nSmall);
    const Contour bs = baseline(nSmall);
    const Contour zl = zigzag(nLarge);
    const Contour bl = baseline(nLarge);
    ContourIntersections rs;
    ContourIntersections rl;
    const carvetto::test::ScalingTimes t = carvetto::test::interleavedCpuSeconds(
        3, [&] { rs = intersect(zs, bs, tol); }, [&] { rl = intersect(zl, bl, tol); });
    INFO("CPU seconds: " << nSmall << " spans " << t.smallSeconds << ", " << nLarge << " spans " << t.largeSeconds
                         << ", ratio " << t.ratio());
    REQUIRE(rs.complete());
    REQUIRE(rl.complete());
    // Each zigzag segment crosses the baseline once (the junctions are hits of two pieces, kept once).
    REQUIRE(rs.points.size() == static_cast<std::size_t>(nSmall));
    REQUIRE(rl.points.size() == static_cast<std::size_t>(nLarge));
    // 4x the spans: linear is ~4, the quadratic duplicate scan or pair loop ~16.
    CHECK(t.ratio() <= 7.0);
    if constexpr (kRelease) {
        CHECK(t.largeSeconds < 1.0);
    }
}

TEST_CASE("Contour-level selfIntersections is near-linear in spans and hits", "[intersect][contour][perf][geometry]") {
    const Tolerance tol = Tolerance::defaults();
    const int nSmall = kRelease ? 1250 : 500;
    const int nLarge = kRelease ? 5000 : 2000;
    const Contour cs = looped(nSmall);
    const Contour cl = looped(nLarge);
    ContourIntersections rs;
    ContourIntersections rl;
    const carvetto::test::ScalingTimes t = carvetto::test::interleavedCpuSeconds(
        3, [&] { rs = selfIntersections(cs, tol); }, [&] { rl = selfIntersections(cl, tol); });
    INFO("CPU seconds: " << nSmall << " spans " << t.smallSeconds << ", " << nLarge << " spans " << t.largeSeconds
                         << ", ratio " << t.ratio());
    REQUIRE(rs.complete());
    REQUIRE(rl.complete());
    REQUIRE(rs.points.size() == static_cast<std::size_t>(nSmall));
    REQUIRE(rl.points.size() == static_cast<std::size_t>(nLarge));
    CHECK(t.ratio() <= 7.0);
    if constexpr (kRelease) {
        CHECK(t.largeSeconds < 1.0);
    }
}

TEST_CASE("Default budget: the old multi-second Bezier pairs finish quickly or report truncation", "[bezier][intersect][budget][perf][geometry]") {
    const Tolerance tol = Tolerance::defaults();
    struct Case {
        const char* name;
        Span a;
        Span b;
        bool mustTruncate;  // needs more than the Debug limit below
    };
    const double huge = 1e100;
    const std::vector<Case> cases{
        {"loops 1e6 shifted", loopOf(1e6), shiftedLoopOf(1e6), true},
        {"loops 1e7", loopOf(1e7), shiftedLoopOf(1e7), true},
        {"loops 1e8", loopOf(1e8), shiftedLoopOf(1e8), true},
        {"loops 1e8 reversed", loopOf(1e8), reversedLoopOf(1e8), true},
        {"huge 1e8", BezierSpan{{1e8, 0}, {1e8, 1e8}, {0, 1e8}, {-1e8, -1e8}},
         BezierSpan{{0, 0}, {1e8, 1e8}, {-1e8, 1e8}, {10, 10}}, false},
        {"huge 1e100", BezierSpan{{huge, 0}, {huge, huge}, {0, huge}, {-huge, -huge}},
         BezierSpan{{0, 0}, {huge, huge}, {-huge, huge}, {10, 10}}, false},
        {"huge 1e100 with itself", BezierSpan{{0, 0}, {huge, huge}, {-huge, huge}, {10, 10}},
         BezierSpan{{0, 0}, {huge, huge}, {-huge, huge}, {10, 10}}, true},
    };
    for (const Case& c : cases) {
        INFO(c.name);
        // Optimised builds use the default budget (about 2 s of CPU at most); a Debug build,
        // an order of magnitude slower, a smaller explicit one.
        const IntersectLimits limits = kRelease ? IntersectLimits{} : pairLimit(3'000'000);
        const double t0 = processCpuSeconds();
        const IntersectionResult r = intersect(c.a, c.b, tol, limits);
        const double cpu = processCpuSeconds() - t0;
        INFO("CPU " << cpu << " s, hits " << r.points.size());
        REQUIRE(cpu < (kRelease ? 6.0 : 30.0));  // ~2 s unloaded; the margin is for a busy machine
        if (!kRelease && c.mustTruncate) {
            REQUIRE(r.status == IntersectStatus::Truncated);
        }
    }
}

// ----------------------------------------------------------------------------- callers

namespace {

Contour curveContour() {
    Contour c;
    c.append(BezierSpan{{0, 0}, {10, 20}, {20, -20}, {30, 0}});
    return c;
}
Contour crossingLine() {
    Contour c;
    c.append(LineSpan{{5, -5}, {25, 5}});
    return c;
}

}  // namespace

TEST_CASE("edit ops refuse a truncated intersection instead of cutting at the hits found", "[edit][intersect][budget][geometry]") {
    const Tolerance tol = Tolerance::defaults();
    const Contour curve = curveContour();
    const Contour line = crossingLine();
    Contour sq = Contour::rectangle(Box2{{8, -30}, {22, 30}});

    // With the default budget the calls work.
    {
        const TrimAtResult ok = trimSpansAt(curve, {7.5, 5.625}, {line}, {}, tol);
        REQUIRE(ok.status == EditStatus::Ok);
        REQUIRE_FALSE(ok.contours.empty());
        const TrimResult okTrim = trimToBoundary({curve}, {sq}, TrimSide::ClearInside, false, tol);
        REQUIRE(okTrim.status == EditStatus::Ok);
        REQUIRE_FALSE(okTrim.contours.empty());
        Contour open;
        open.append(LineSpan{{-10, 0}, {-5, 0}});
        const ExtendResult okExt = extendSpan(open, ContourEnd::End, {curve}, tol);
        REQUIRE(okExt.status == EditStatus::Ok);
    }

    IntersectLimits none;
    none.pairWork = 0;
    none.totalWork = 0;
    const detail::ScopedDefaultLimits scoped(none);
    const TrimAtResult cutAt = trimSpansAt(curve, {7.5, 5.625}, {line}, {}, tol);
    REQUIRE(cutAt.status == EditStatus::TooComplex);
    REQUIRE(cutAt.contours.empty());
    // The contour's own self-intersection test fits the budget, only the pair with `line` does not.
    {
        IntersectLimits pairNone;
        pairNone.pairWork = 0;
        const detail::ScopedDefaultLimits pairOnly(pairNone);
        const TrimAtResult cutOthers = trimSpansAt(curve, {7.5, 5.625}, {line}, {}, tol);
        REQUIRE(cutOthers.status == EditStatus::TooComplex);
        REQUIRE(cutOthers.contours.empty());
        // Without others there is no pair: the same call works.
        REQUIRE(trimSpansAt(curve, {7.5, 5.625}, {}, {}, tol).status != EditStatus::TooComplex);
    }
    // Self-intersection part alone (no others): the contour's own Bezier chords spend the total.
    const TrimAtResult cutSelf = trimSpansAt(curve, {7.5, 5.625}, {}, {}, tol);
    REQUIRE(cutSelf.status == EditStatus::TooComplex);
    const TrimResult cutTrim = trimToBoundary({line, curve}, {sq}, TrimSide::ClearInside, false, tol);
    REQUIRE(cutTrim.status == EditStatus::TooComplex);
    REQUIRE(cutTrim.contours.empty());
    REQUIRE(cutTrim.sources.empty());
    Contour open;
    open.append(LineSpan{{-10, 0}, {-5, 0}});
    const ExtendResult cutExt = extendSpan(open, ContourEnd::End, {curve}, tol);
    REQUIRE(cutExt.status == EditStatus::TooComplex);
    REQUIRE(std::string(toString(EditStatus::TooComplex)).find("limit") != std::string::npos);
}

TEST_CASE("V-carve treats a truncated crossing test as a possible crossing", "[toolpath][v_carve][intersect][budget]") {
    // Two separate closed vectors with a Bezier edge each: nothing crosses.
    const auto lens = [](double x) {
        Contour c;
        c.append(BezierSpan{{x, 0}, {x + 3, 6}, {x + 7, 6}, {x + 10, 0}});
        c.append(LineSpan{{x + 10, 0}, {x, 0}});
        c.closed = true;
        return c;
    };
    const std::vector<Contour> vectors{lens(0.0), lens(30.0)};
    const carvetto::tooling::Tool tool = carvetto::test::v_carve::vBit(90.0, 2.0);
    carvetto::toolpath::VCarvingParams p;
    p.tolerance.chordal = 0.02;
    const auto plain = carvetto::toolpath::vCarvingToolpath(vectors, tool, std::nullopt, {}, p);
    REQUIRE(plain.ok());
    REQUIRE_FALSE(plain.has(carvetto::toolpath::ToolpathIssue::CrossingVectorsResolved));
    IntersectLimits none;
    none.pairWork = 0;
    none.totalWork = 0;
    const detail::ScopedDefaultLimits scoped(none);
    // The intersection tests cannot answer within the budget: the vectors are resolved together
    // (the union is exact for vectors that do not cross), and the user is told.
    const auto cut = carvetto::toolpath::vCarvingToolpath(vectors, tool, std::nullopt, {}, p);
    REQUIRE(cut.ok());
    REQUIRE(cut.has(carvetto::toolpath::ToolpathIssue::CrossingVectorsResolved));
}

TEST_CASE("V-carve resolves a nested pair whose crossing test is truncated", "[toolpath][v_carve][intersect][budget]") {
    // A small teardrop (one closed Bezier span) inside a large one: the boxes overlap, nothing
    // crosses. Only the pair test (not the self-intersection tests, which have no pair inside a
    // single span) runs out of budget.
    const auto teardrop = [](double y, double halfWidth, double height) {
        Contour c;
        c.append(BezierSpan{{0, y}, {halfWidth, y + height}, {-halfWidth, y + height}, {0, y}});
        c.closed = true;
        return c;
    };
    const std::vector<Contour> vectors{teardrop(0.0, 20.0, 15.0), teardrop(3.0, 3.0, 5.0)};
    const carvetto::tooling::Tool tool = carvetto::test::v_carve::vBit(90.0, 2.0);
    carvetto::toolpath::VCarvingParams p;
    p.tolerance.chordal = 0.02;
    const auto plain = carvetto::toolpath::vCarvingToolpath(vectors, tool, std::nullopt, {}, p);
    REQUIRE(plain.ok());
    REQUIRE_FALSE(plain.has(carvetto::toolpath::ToolpathIssue::CrossingVectorsResolved));
    IntersectLimits pairNone;
    pairNone.pairWork = 0;
    const detail::ScopedDefaultLimits scoped(pairNone);
    const auto cut = carvetto::toolpath::vCarvingToolpath(vectors, tool, std::nullopt, {}, p);
    REQUIRE(cut.ok());
    REQUIRE(cut.has(carvetto::toolpath::ToolpathIssue::CrossingVectorsResolved));
}
