// Hostile-input, determinism and cost tests for the geometry edit operations (T-0208).

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <vector>

#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/edit_ops.hpp"
#include "cpu_scaling.hpp"
#include "test_rng.hpp"

using namespace carvetto::geometry;
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

void exercise(SplitMix& rng, double scale, bool hostileValues, std::vector<Contour>& outputs) {
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
            REQUIRE(finiteContour(f.contour));
            outputs.push_back(f.contour);
        }
        const FilletResult one = filletCorner(a, rng.next() % 8, radius, kind, opt, tol);
        if (one.status == EditStatus::Ok) {
            REQUIRE(finiteContour(one.contour));
        }
    }
    for (TrimSide side : {TrimSide::ClearInside, TrimSide::ClearOutside}) {
        const TrimResult t = trimToBoundary({a, b}, {boundary}, side, rng.next() % 2 == 0, tol);
        for (const Contour& c : t.contours) {
            REQUIRE(finiteContour(c));
            outputs.push_back(c);
        }
    }
    const TrimAtResult ta = trimSpansAt(a, randomPoint(rng, scale), {b}, TrimAtOptions{rng.next() % 2 == 0}, tol);
    for (const Contour& c : ta.contours) {
        REQUIRE(finiteContour(c));
        outputs.push_back(c);
    }
    const ExtendResult e = extendSpan(a, rng.next() % 2 ? ContourEnd::Start : ContourEnd::End, {b, boundary}, tol);
    if (e.status == EditStatus::Ok) {
        REQUIRE(finiteContour(e.contour));
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
        SplitMix r1{static_cast<std::uint64_t>(scale * 1000.0) + 1};
        SplitMix r2{static_cast<std::uint64_t>(scale * 1000.0) + 1};
        for (int i = 0; i < 50; ++i) {
            exercise(r1, scale, false, first);
            exercise(r2, scale, false, second);
        }
        CHECK(first == second);
    }
}

TEST_CASE("edit ops: hostile values (NaN, infinity, huge, zero) are rejected or handled", "[edit][fuzz]") {
    SplitMix rng{99};
    std::vector<Contour> sink;
    for (int i = 0; i < 300; ++i) {
        exercise(rng, 10.0, true, sink);
    }
    SUCCEED();
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
