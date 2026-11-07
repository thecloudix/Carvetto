// Hostile-input, determinism and cost tests for the geometry edit operations (T-0208).

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/edit_ops.hpp"
#include "cpu_scaling.hpp"
#include "fail_fast.hpp"
#include "test_rng.hpp"

using namespace carvetto::geometry;
using carvetto::test::failFast;
using carvetto::test::SplitMix;

namespace {

Vec2 randomPoint(SplitMix& rng, double scale) {
    return {rng.uniform(-scale, scale), rng.uniform(-scale, scale)};
}

double hostile(SplitMix& rng, double scale) {
    switch (rng.next() % 12) {
        case 0: return 0.0;
        case 1: return std::numeric_limits<double>::quiet_NaN();
        case 2: return std::numeric_limits<double>::infinity();
        case 3: return 1e300;
        case 4: return -1e-300;
        default: return rng.uniform(-scale, scale);
    }
}

Span randomSpan(SplitMix& rng, Vec2 from, double scale, bool hostileValues) {
    const Vec2 to = hostileValues && rng.next() % 6 == 0 ? Vec2{hostile(rng, scale), hostile(rng, scale)}
                                                          : (rng.next() % 5 == 0 ? from : from + randomPoint(rng, scale));
    switch (rng.next() % 3) {
        case 0: return LineSpan{from, to};
        case 1: {
            const Vec2 centre = from + randomPoint(rng, scale);
            return ArcSpan::fromCentre(centre, from, to, rng.next() % 2 ? ArcDirection::CCW : ArcDirection::CW);
        }
        default: return BezierSpan{from, from + randomPoint(rng, scale), to + randomPoint(rng, scale), to};
    }
}

Contour randomContour(SplitMix& rng, double scale, bool hostileValues) {
    Contour c;
    const std::size_t n = 1 + rng.next() % 6;
    Vec2 at = randomPoint(rng, scale);
    for (std::size_t i = 0; i < n; ++i) {
        Span s = randomSpan(rng, at, scale, hostileValues);
        at = endPoint(s);
        c.append(std::move(s));
    }
    c.closed = rng.next() % 2 == 0;
    return c;
}

bool finiteContour(const Contour& c) {
    for (const Span& s : c.spans) {
        const Box2 b = bounds(s);
        if (b.isEmpty() || !std::isfinite(b.min.x) || !std::isfinite(b.min.y) || !std::isfinite(b.max.x) ||
            !std::isfinite(b.max.y)) {
            return false;
        }
    }
    return true;
}

/// `nonFinite` counts outputs with a non-finite box (no assertions here: failFast runs this on a
/// worker thread; the caller asserts the count is zero).
void exercise(SplitMix& rng, double scale, bool hostileValues, std::vector<Contour>& outputs, int& nonFinite) {
    // Tolerances scale with the drawing: curve intersection cost grows with extent / chordal.
    const Tolerance tol{scale * 1e-5, 1e-9, scale * 1e-4};
    const Contour a = randomContour(rng, scale, hostileValues);
    const Contour b = randomContour(rng, scale, hostileValues);
    Contour boundary = randomContour(rng, scale, false);
    boundary.closed = true;
    const double radius = hostileValues ? hostile(rng, scale) : rng.uniform(0.01, scale);
    FilletOptions opt;
    opt.clampToFit = rng.next() % 2 == 0;
    opt.tBoneAlongIncoming = rng.next() % 2 == 0;
    for (FilletKind kind : {FilletKind::Normal, FilletKind::DogBone, FilletKind::TBone}) {
        const FilletAllResult f = filletAll(a, radius, kind, opt, tol);
        if (f.status == EditStatus::Ok) {
            nonFinite += finiteContour(f.contour) ? 0 : 1;
            outputs.push_back(f.contour);
        }
        const FilletResult one = filletCorner(a, rng.next() % 8, radius, kind, opt, tol);
        if (one.status == EditStatus::Ok) {
            nonFinite += finiteContour(one.contour) ? 0 : 1;
        }
    }
    for (TrimSide side : {TrimSide::ClearInside, TrimSide::ClearOutside}) {
        const TrimResult t = trimToBoundary({a, b}, {boundary}, side, rng.next() % 2 == 0, tol);
        for (const Contour& c : t.contours) {
            nonFinite += finiteContour(c) ? 0 : 1;
            outputs.push_back(c);
        }
    }
    const TrimAtResult ta = trimSpansAt(a, randomPoint(rng, scale), {b}, TrimAtOptions{rng.next() % 2 == 0}, tol);
    for (const Contour& c : ta.contours) {
        nonFinite += finiteContour(c) ? 0 : 1;
        outputs.push_back(c);
    }
    const ExtendResult e = extendSpan(a, rng.next() % 2 ? ContourEnd::Start : ContourEnd::End, {b, boundary}, tol);
    if (e.status == EditStatus::Ok) {
        nonFinite += finiteContour(e.contour) ? 0 : 1;
        outputs.push_back(e.contour);
    }
    const JoinResult j = joinContours({a, b, boundary}, tol);
    for (const Contour& c : j.contours) {
        outputs.push_back(c);
    }
    const CloseResult cl = closeContour(a, tol);
    if (cl.status == EditStatus::Ok) {
        outputs.push_back(cl.contour);
    }
}

}  // namespace

TEST_CASE("edit ops: random finite inputs never crash and are deterministic", "[edit][fuzz]") {
    for (double scale : {1e-4, 1.0, 100.0, 1e7}) {
        std::vector<Contour> first;
        std::vector<Contour> second;
        int nonFinite = 0;
        SplitMix r1{static_cast<std::uint64_t>(scale * 1000.0) + 1};
        SplitMix r2{static_cast<std::uint64_t>(scale * 1000.0) + 1};
        for (int i = 0; i < 50; ++i) {
            exercise(r1, scale, false, first, nonFinite);
            exercise(r2, scale, false, second, nonFinite);
        }
        CHECK(nonFinite == 0);
        CHECK(first == second);
    }
}

TEST_CASE("edit ops: hostile values (NaN, infinity, huge, zero) are rejected or handled", "[edit][fuzz]") {
    SplitMix rng{99};
    std::vector<Contour> sink;
    int nonFinite = 0;
    for (int i = 0; i < 300; ++i) {
        exercise(rng, 10.0, true, sink, nonFinite);
    }
    CHECK(nonFinite == 0);
}

TEST_CASE("edit ops: trimming scales near-linearly with the box index", "[edit][perf]") {
    // N radial segments crossing a ring of N edges: every segment is cut once.
    auto build = [](std::size_t n, std::vector<Contour>& segs, std::vector<Contour>& ring) {
        segs.clear();
        ring.assign(1, Contour{});
        const double radius = 100.0;
        // Segments about as long as a ring edge (the box prefilter's cost is the number of
        // boundary boxes a span's box meets, so very long diagonal spans cost more by design).
        const double reach = 1.5 * kTwoPi * radius / static_cast<double>(n);
        for (std::size_t i = 0; i < n; ++i) {
            const double a0 = kTwoPi * static_cast<double>(i) / static_cast<double>(n);
            const double a1 = kTwoPi * static_cast<double>(i + 1) / static_cast<double>(n);
            ring[0].append(LineSpan{Vec2::fromAngle(a0, radius), Vec2::fromAngle(a1, radius)});
            const double am = 0.5 * (a0 + a1);
            Contour s;
            s.append(LineSpan{Vec2::fromAngle(am, radius - reach), Vec2::fromAngle(am, radius + reach)});
            segs.push_back(std::move(s));
        }
        ring[0].closed = true;
    };
    std::vector<Contour> segsSmall;
    std::vector<Contour> ringSmall;
    std::vector<Contour> segsLarge;
    std::vector<Contour> ringLarge;
    build(2500, segsSmall, ringSmall);
    build(10000, segsLarge, ringLarge);
    TrimResult small_;
    TrimResult large_;
    const auto times = carvetto::test::interleavedCpuSeconds(
        3, [&] { small_ = trimToBoundary(segsSmall, ringSmall, TrimSide::ClearOutside); },
        [&] { large_ = trimToBoundary(segsLarge, ringLarge, TrimSide::ClearOutside); });
    REQUIRE(small_.contours.size() == 2500);
    REQUIRE(large_.contours.size() == 10000);
    const double reachLarge = 1.5 * kTwoPi * 100.0 / 10000.0;
    for (const Contour& c : large_.contours) {
        CHECK(std::fabs(c.length() - (100.0 * std::cos(kPi / 10000.0) - (100.0 - reachLarge))) < 1e-6);
    }
    // 4x the data: linear is ~4, n log n ~5; quadratic would be 16.
    CHECK(times.ratio() < 9.0);
#ifdef NDEBUG
    CHECK(times.largeSeconds < 2.0);
#endif
}

TEST_CASE("edit ops: filleting a 20000-corner contour is linear in practice", "[edit][perf]") {
    Contour c;
    c.closed = true;
    const std::size_t n = 20000;
    for (std::size_t i = 0; i < n; ++i) {
        const double a = kTwoPi * static_cast<double>(i) / static_cast<double>(n);
        const double b = kTwoPi * static_cast<double>(i + 1) / static_cast<double>(n);
        const double ra = i % 2 == 0 ? 100.0 : 101.0;
        const double rb = (i + 1) % 2 == 0 ? 100.0 : 101.0;
        c.append(LineSpan{Vec2::fromAngle(a, ra), Vec2::fromAngle(b, n == i + 1 ? 100.0 : rb)});
    }
    const double t0 = carvetto::test::processCpuSeconds();
    const FilletAllResult r = filletAll(c, 0.001, FilletKind::Normal, FilletOptions{true, false}, Tolerance{1e-6, 1e-9, 1e-4});
    const double elapsed = carvetto::test::processCpuSeconds() - t0;
    REQUIRE(r.status == EditStatus::Ok);
    CHECK(r.applied > n / 2);
    CHECK(elapsed < 30.0);  // loose Debug bound; quadratic insertion would be far above
}

namespace {

/// One open line (no Polyline: <windows.h>, pulled in by cpu_time.hpp, declares a GDI Polyline).
Contour openLine(Vec2 a, Vec2 b) {
    Contour c;
    c.append(LineSpan{a, b});
    return c;
}

Contour hugeBezier(double big) {
    Contour bz;
    bz.append(BezierSpan{{0, 0}, {big, big}, {-big, big}, {10, 10}});
    return bz;
}

}  // namespace

TEST_CASE("edit ops: Beziers with huge control points are refused at once", "[edit][fuzz][guard]") {
    const Contour ln = openLine({-5, 5}, {15, 5});
    const Contour sq = Contour::rectangle(Box2{{0, 0}, {10, 10}});
    for (double big : {1e100, 1e154, 1e155, 1e160, 1e200, 1e300, 1e13}) {
        INFO("big = " << big);
        const Contour bz = hugeBezier(big);
        Contour closedBz = bz;
        closedBz.append(LineSpan{{10, 10}, {0, 0}});
        closedBz.closed = true;
        CHECK(failFast(20.0, "trimSpansAt", [&] { return trimSpansAt(bz, {1, 1}, {ln}).status; }) ==
              EditStatus::InvalidArgument);
        CHECK(failFast(20.0, "trimToBoundary (curve)",
                       [&] { return trimToBoundary({bz}, {sq}, TrimSide::ClearInside).status; }) ==
              EditStatus::InvalidArgument);
        CHECK(failFast(20.0, "trimToBoundary (boundary)",
                       [&] { return trimToBoundary({ln}, {closedBz}, TrimSide::ClearInside).status; }) ==
              EditStatus::InvalidArgument);
        CHECK(failFast(20.0, "extendSpan", [&] {
                  return extendSpan(openLine({-30, 0}, {-25, 0}), ContourEnd::End, {bz}).status;
              }) == EditStatus::InvalidArgument);
        CHECK(failFast(20.0, "filletAll", [&] { return filletAll(closedBz, 1.0, FilletKind::Normal).status; }) ==
              EditStatus::InvalidArgument);
        CHECK(failFast(20.0, "closeContour", [&] { return closeContour(bz).status; }) == EditStatus::InvalidArgument);
        CHECK(failFast(20.0, "joinContours", [&] { return joinContours({bz, ln}).contours.size(); }) == 2);
    }
    // A control polygon far larger than its chordal tolerance (but within the magnitude cap) is
    // refused by the cost rule, again without running the intersection.
    const Contour wide = hugeBezier(1e9);
    CHECK(failFast(20.0, "wide trimSpansAt", [&] { return trimSpansAt(wide, {1, 1}, {ln}).status; }) ==
          EditStatus::InvalidArgument);
    // With a tolerance that scales with the drawing the same call is accepted and returns quickly.
    const Tolerance scaled{1e3, 1e-9, 1e4};
    const auto st = failFast(120.0, "scaled trimSpansAt", [&] { return trimSpansAt(wide, {1, 1}, {ln}, {}, scaled).status; });
    CHECK((st == EditStatus::Ok || st == EditStatus::NothingToTrim));
}

TEST_CASE("edit ops: fixed regression seeds of the hostile Bezier fuzz finish and stay finite", "[edit][fuzz][guard]") {
    // The verifier's seeds 981 and 6967 hit the unguarded Bezier bounds overflow; this generator
    // is ours (SplitMix), so the same numbers give different drawings, but they stay as fixed
    // hostile cases next to a run of neighbours.
    for (std::uint64_t seed : {std::uint64_t{981}, std::uint64_t{6967}, std::uint64_t{1}, std::uint64_t{2},
                               std::uint64_t{3}, std::uint64_t{4}, std::uint64_t{5}, std::uint64_t{6}}) {
        INFO("seed = " << seed);
        const int nonFinite = failFast(120.0, "hostile fuzz seed", [seed] {
            SplitMix rng{seed};
            std::vector<Contour> sink;
            int bad = 0;
            for (int i = 0; i < 12; ++i) {
                exercise(rng, 10.0, true, sink, bad);
                // Huge Beziers on top of the generic hostile spans.
                Contour bz = hugeBezier(std::pow(10.0, rng.uniform(100.0, 300.0)));
                const Tolerance tol{};
                (void)trimSpansAt(bz, {1, 1}, {sink.empty() ? bz : sink.back()}, {}, tol);
                (void)trimToBoundary({sink.empty() ? bz : sink.back()}, {bz}, TrimSide::ClearInside);
            }
            return bad;
        });
        CHECK(nonFinite == 0);
    }
}
