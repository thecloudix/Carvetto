// T-0186: the Profile work budget (ADR-0016 section 19). Up-front refusals of hostile parameters
// and inputs, the deterministic work meter and its limit, the cancel hook, and "nothing partial".
// Every refusal is WorkLimitExceeded (Cancelled for a cancel), comes with exactly that one
// diagnostic and an empty toolpath. Calls that must not run long are wrapped in failFast (CPU time).

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include "../geometry/cpu_time.hpp"
#include "../geometry/fail_fast.hpp"
#include "../geometry/test_rng.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "carvetto/toolpath/tab_placement.hpp"
#include "profile_test_util.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using carvetto::geometry::BezierSpan;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::LineSpan;
using carvetto::geometry::Vec2;
using carvetto::test::failFast;
using Catch::Matchers::ContainsSubstring;

namespace {

ProfileParams baseParams() {
    ProfileParams p;
    p.passes.cutDepth = 6.0;
    p.passes.passDepth = 3.0;
    return p;
}

Contour rect(double left, double bottom, double right, double top) {
    return Contour::rectangle(Box2{{left, bottom}, {right, top}});
}

std::vector<Contour> plate() { return {rect(0, 0, 100, 50)}; }

/// A thin, self-intersecting closed shape: one cubic Bezier closed by a line, `scale` / 10 times as
/// large as the 100 mm-class shape that took 0.5 s (harness search, T-0186).
std::vector<Contour> thinLoop(double scale) {
    const double k = scale / 10.0;
    Contour c;
    c.closed = true;
    const Vec2 a{-97.1726 * k, 730.289 * k};
    const Vec2 b{-50.4386 * k, 349.652 * k};
    c.append(BezierSpan{a, {48.4072 * k, 204.133 * k}, {60.7096 * k, 726.054 * k}, b});
    c.append(LineSpan{b, a});
    return {c};
}

/// A closed shape of four Bezier quarter circles of radius `r` about `c`.
Contour bezierCircle(Vec2 c, double r) {
    const double k = 0.5523 * r;
    const Vec2 p0 = c + Vec2{r, 0}, p1 = c + Vec2{0, r}, p2 = c + Vec2{-r, 0}, p3 = c + Vec2{0, -r};
    Contour q;
    q.closed = true;
    q.append(BezierSpan{p0, p0 + Vec2{0, k}, p1 + Vec2{k, 0}, p1});
    q.append(BezierSpan{p1, p1 + Vec2{-k, 0}, p2 + Vec2{0, k}, p2});
    q.append(BezierSpan{p2, p2 + Vec2{0, -k}, p3 + Vec2{-k, 0}, p3});
    q.append(BezierSpan{p3, p3 + Vec2{k, 0}, p0 + Vec2{0, -k}, p0});
    return q;
}

/// A closed polygon of `n` line spans on a circle of radius `r` about the origin.
Contour polygonCircle(std::size_t n, double r) {
    Contour k;
    k.closed = true;
    for (std::size_t i = 0; i < n; ++i) {
        const double a0 = geometry::kTwoPi * static_cast<double>(i) / static_cast<double>(n);
        const double a1 = geometry::kTwoPi * static_cast<double>(i + 1) / static_cast<double>(n);
        k.append(LineSpan{{r * std::cos(a0), r * std::sin(a0)}, {r * std::cos(a1), r * std::sin(a1)}});
    }
    return k;
}

/// Runs the profile with a cancel flag set from another thread after `delayMs`; returns the result
/// and the CPU seconds from the flag being set to the call returning.
struct CancelRun {
    ProfileResult result;
    double latency{0.0};
};
CancelRun cancelAfter(int delayMs, const std::vector<Contour>& cs, const ProfileParams& p, double tool) {
    std::atomic<bool> cancel{false};
    std::atomic<double> setAtCpu{-1.0};
    ToolpathRunControl control;
    control.cancel = &cancel;
    std::thread canceller([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
        setAtCpu = test::processCpuSeconds();
        cancel = true;
    });
    CancelRun out;
    out.result = failFast(120.0, "cancel", [&] { return profileToolpath(cs, test::endMill(tool), p, control); });
    const double returnedAtCpu = test::processCpuSeconds();
    canceller.join();
    out.latency = returnedAtCpu - setAtCpu.load();
    return out;
}

/// Two long rectangles close enough for their offsets to merge: the cost grows with the length.
std::vector<Contour> longNeighbours(double length) {
    return {rect(0, 0, length, 10), rect(0, 12, length, 22)};
}

/// The result is a refusal: nothing cut, exactly one diagnostic, of the given issue.
void requireRefused(const ProfileResult& r, ToolpathIssue issue) {
    REQUIRE_FALSE(r.ok());
    REQUIRE(r.toolpath.moves.empty());
    REQUIRE(r.diagnostics.size() == 1);
    REQUIRE(r.diagnostics.front().issue == issue);
    REQUIRE_FALSE(r.diagnostics.front().message.empty());
}

ProfileResult run(const std::vector<Contour>& cs, const ProfileParams& p, double tool = 6.0) {
    return profileToolpath(cs, test::endMill(tool), p);
}

}  // namespace

// ------------------------------------------------------------------------------ parameter ranges

TEST_CASE("T-0186: distances beyond 1000 mm are refused up front", "[toolpath][profile][budget]") {
    static_assert(kProfileMaxOffsetMm == 1000.0);
    SECTION("allowance, both sides of the limit") {
        ProfileParams p = baseParams();
        p.allowance = 1e5;
        requireRefused(run(plate(), p), ToolpathIssue::WorkLimitExceeded);
        p.side = ProfileSide::Inside;
        requireRefused(run(plate(), p), ToolpathIssue::WorkLimitExceeded);
        p.allowance = 1000.0;  // exactly the limit: cut as before
        p.side = ProfileSide::Outside;
        CHECK(run(plate(), p).ok());
        p.allowance = std::nextafter(1000.0, 2000.0);
        requireRefused(run(plate(), p), ToolpathIssue::WorkLimitExceeded);
    }
    SECTION("last pass allowance, only with a separate last pass") {
        ProfileParams p = baseParams();
        p.lastPassAllowance = 1e5;
        CHECK(run(plate(), p).ok());  // not used
        p.separateLastPass = true;
        requireRefused(run(plate(), p), ToolpathIssue::WorkLimitExceeded);
    }
    SECTION("lead length, radius and overcut, only when leads are on") {
        ProfileParams p = baseParams();
        p.leads.length = 1e6;
        p.leads.radius = 1e6;
        p.leads.overcut = 1e6;
        CHECK(run(plate(), p).ok());  // LeadType::None: none of them is used
        p.leads.type = LeadType::Straight;
        requireRefused(run(plate(), p), ToolpathIssue::WorkLimitExceeded);  // overcut
        p.leads.overcut = 0.0;
        requireRefused(run(plate(), p), ToolpathIssue::WorkLimitExceeded);  // length
        p.leads.type = LeadType::Circular;
        requireRefused(run(plate(), p), ToolpathIssue::WorkLimitExceeded);  // radius
        p.leads.radius = 5.0;
        CHECK(run(plate(), p).ok());
    }
    SECTION("ramp distance and tab length") {
        ProfileParams p = baseParams();
        p.ramp.distance = 1e6;
        CHECK(run(plate(), p).ok());  // RampType::None
        p.ramp.type = RampType::Smooth;
        requireRefused(run(plate(), p), ToolpathIssue::WorkLimitExceeded);
        p = baseParams();
        p.tabs.length = 1e9;
        CHECK(run(plate(), p).ok());  // tabs off
        p.tabs.enabled = true;
        requireRefused(run(plate(), p), ToolpathIssue::WorkLimitExceeded);
    }
}

TEST_CASE("T-0186: the refusal names the parameter", "[toolpath][profile][budget]") {
    ProfileParams p = baseParams();
    p.allowance = 5000.0;
    const ProfileResult r = run(plate(), p);
    requireRefused(r, ToolpathIssue::WorkLimitExceeded);
    CHECK_THAT(r.diagnostics.front().message, ContainsSubstring("allowance"));
    CHECK_THAT(r.diagnostics.front().message, ContainsSubstring("1000"));
    CHECK_THAT(r.diagnostics.front().message, ContainsSubstring("nothing is cut"));
}

// ------------------------------------------------------------------------------ passes, tabs, moves

TEST_CASE("T-0186: more than 10000 depth passes are refused, exactly 10000 are cut", "[toolpath][profile][budget]") {
    static_assert(kProfileMaxLevels == 10000);
    ProfileParams p = baseParams();
    p.passes.passDepth = 0.0001;  // 60000 passes
    requireRefused(run(plate(), p), ToolpathIssue::WorkLimitExceeded);
    p.passes.cutDepth = 1e9;  // the allocation of the levels must not be tried
    requireRefused(failFast(5.0, "huge pass count", [&] { return run(plate(), p); }), ToolpathIssue::WorkLimitExceeded);
    p.passes.cutDepth = 10000.0;
    p.passes.passDepth = 1.0;
    const ProfileResult exact = run({rect(0, 0, 10, 10)}, p);
    REQUIRE(exact.ok());
    CHECK(p.passes.levels().size() == 10000);
    p.passes.cutDepth = 10001.0;
    requireRefused(run({rect(0, 0, 10, 10)}, p), ToolpathIssue::WorkLimitExceeded);
}

TEST_CASE("T-0186: a tab every nanometre or a billion tabs is refused, not allocated", "[toolpath][profile][budget]") {
    ProfileParams p = baseParams();
    p.tabs.enabled = true;
    p.tabs.spacing = TabSpacing::ConstantDistance;
    p.tabs.distance = 1e-9;
    const ProfileResult pitch = failFast(5.0, "nanometre tab pitch", [&] { return run(plate(), p); });
    requireRefused(pitch, ToolpathIssue::WorkLimitExceeded);
    CHECK_THAT(pitch.diagnostics.front().message, ContainsSubstring("tabs would be placed"));  // the tab limit, not the move estimate
    p.tabs.spacing = TabSpacing::ConstantNumber;
    p.tabs.count = 1000000000;
    const ProfileResult billion = failFast(5.0, "billion tabs", [&] { return run(plate(), p); });
    requireRefused(billion, ToolpathIssue::WorkLimitExceeded);
    CHECK_THAT(billion.diagnostics.front().message, ContainsSubstring("tabs would be placed"));
    p.tabs.count = 100000;  // asked for, mostly left out with a diagnostic, as before
    const ProfileResult many = run(plate(), p);
    CHECK(many.ok());
    CHECK(many.has(ToolpathIssue::TabsNotPlaced));
}

TEST_CASE("T-0186: a ramp with legs without end is refused by the move estimate", "[toolpath][profile][budget]") {
    ProfileParams p = baseParams();
    p.ramp.type = RampType::ZigZag;
    p.ramp.useAngle = true;
    p.ramp.angle = 1e-9;
    p.ramp.distance = 1e-3;
    requireRefused(run(plate(), p), ToolpathIssue::WorkLimitExceeded);
    p.ramp.angle = 0.0872664625997165;  // 5 degrees
    p.ramp.distance = 10.0;
    CHECK(run(plate(), p).ok());
}

TEST_CASE("T-0186: more moves than 20 million are refused", "[toolpath][profile][budget]") {
    // 10000 passes of 2100 spans: 42 million moves by the estimate.
    Contour c;
    c.closed = true;
    const int n = 2100;
    for (int i = 0; i < n; ++i) {
        c.append(LineSpan{{static_cast<double>(i), 0.0}, {static_cast<double>(i + 1), 0.0}});
    }
    c.append(LineSpan{{static_cast<double>(n), 0.0}, {static_cast<double>(n), 5.0}});
    c.append(LineSpan{{static_cast<double>(n), 5.0}, {0.0, 5.0}});
    c.append(LineSpan{{0.0, 5.0}, {0.0, 0.0}});
    ProfileParams p = baseParams();
    p.passes.cutDepth = 10000.0;
    p.passes.passDepth = 1.0;
    const ProfileResult r = failFast(10.0, "move estimate", [&] { return run({c}, p); });
    requireRefused(r, ToolpathIssue::WorkLimitExceeded);
    CHECK_THAT(r.diagnostics.front().message, ContainsSubstring("moves"));
}

// ------------------------------------------------------------------------------ the input

TEST_CASE("T-0186: coordinates, total length and span count are limited", "[toolpath][profile][budget]") {
    static_assert(kProfileMaxCoordinateMm == 1.0e5 && kProfileMaxLengthMm == 1.0e7 && kProfileMaxSpans == 2000000);
    ProfileParams p = baseParams();
    SECTION("coordinates") {
        requireRefused(run({rect(1e9, 1e9, 1e9 + 100, 1e9 + 50)}, p), ToolpathIssue::WorkLimitExceeded);
        requireRefused(run({rect(0, 0, 100, std::nextafter(1e5, 1e6))}, p), ToolpathIssue::WorkLimitExceeded);
        CHECK(run({rect(1e5 - 100, 1e5 - 50, 1e5, 1e5)}, p).ok());  // exactly at the limit
        requireRefused(run({Contour::circle({0, 0}, 1e12)}, p), ToolpathIssue::WorkLimitExceeded);
        // A nearly straight arc has a huge radius and a far-away centre; its end points decide.
        auto shallow = [](double radius) {
            const double half = 20.0;
            const double angle = std::asin(half / radius);
            const geometry::ArcSpan arc{{0.0, -std::sqrt(radius * radius - half * half)}, radius, geometry::kPi / 2 - angle,
                                        geometry::kPi / 2 + angle, geometry::ArcDirection::CCW};
            Contour c;
            c.closed = true;
            c.spans = {geometry::Span{LineSpan{arc.endPoint(), arc.startPoint()}}, geometry::Span{arc}};
            return c;
        };
        CHECK(run({shallow(4e6)}, p).ok());  // as before the budget
        requireRefused(run({shallow(1e13)}, p), ToolpathIssue::WorkLimitExceeded);
    }
    SECTION("total length") {
        // 30 rectangles of 4e5 mm perimeter each: 1.2e7 mm in all, every coordinate in range.
        std::vector<Contour> cs;
        for (int i = 0; i < 30; ++i) {
            cs.push_back(rect(0, 0, 1e5, 1e5 - 1000.0 * i));
        }
        requireRefused(failFast(10.0, "total length", [&] { return run(cs, p); }), ToolpathIssue::WorkLimitExceeded);
    }
    SECTION("spans") {
        Contour c;
        c.closed = true;
        const std::size_t n = kProfileMaxSpans + 1;
        for (std::size_t i = 0; i < n; ++i) {
            c.append(LineSpan{{0.0, 0.0}, {1e-9, 0.0}});
        }
        const ProfileResult r = failFast(20.0, "span count", [&] { return run({c}, p); });
        requireRefused(r, ToolpathIssue::WorkLimitExceeded);
        CHECK_THAT(r.diagnostics.front().message, ContainsSubstring("spans in the vectors"));
    }
}

TEST_CASE("T-0186: a vector with a NaN coordinate is still skipped by the old checks", "[toolpath][profile][budget]") {
    const double nan = std::nan("");
    Contour bad;
    bad.closed = true;
    bad.append(LineSpan{{0, 0}, {nan, 0}});
    bad.append(LineSpan{{nan, 0}, {10, 10}});
    bad.append(LineSpan{{10, 10}, {0, 0}});
    const ProfileResult r = run({bad, rect(0, 0, 40, 20)}, baseParams());
    CHECK_FALSE(r.has(ToolpathIssue::WorkLimitExceeded));
    CHECK(r.ok());  // the good vector is cut
}

// ------------------------------------------------------------------------------ the work meter

TEST_CASE("T-0186: the work count is deterministic and the same with or without a control", "[toolpath][profile][budget]") {
    ProfileParams p = baseParams();
    p.tabs.enabled = true;
    const std::vector<Contour> cs = longNeighbours(300.0);
    const ProfileResult a = run(cs, p);
    const ProfileResult b = run(cs, p);
    ToolpathRunControl control;
    control.workLimit = a.workUnits;  // exactly enough
    const ProfileResult c = profileToolpath(cs, test::endMill(6.0), p, control);
    REQUIRE(a.ok());
    CHECK(a.workUnits > 0);
    CHECK(a.workUnits == b.workUnits);
    CHECK(a.workUnits == c.workUnits);
    CHECK(a.toolpath.moves.size() == c.toolpath.moves.size());
    CHECK(test::formatMoves(a.toolpath) == test::formatMoves(c.toolpath));
    CHECK(a.diagnostics == c.diagnostics);
    CHECK(a.workUnits < kProfileDefaultWorkUnits / 100);
}

TEST_CASE("T-0186: one unit under the job's count the job is refused, with nothing partial", "[toolpath][profile][budget]") {
    ProfileParams p = baseParams();
    p.separateLastPass = true;
    p.tabs.enabled = true;
    p.leads.type = LeadType::Circular;
    const std::vector<Contour> cs = longNeighbours(300.0);
    const ProfileResult full = run(cs, p);
    REQUIRE(full.ok());
    REQUIRE(full.workUnits > 10);
    ToolpathRunControl control;
    control.workLimit = full.workUnits - 1;
    const ProfileResult refused = profileToolpath(cs, test::endMill(6.0), p, control);
    requireRefused(refused, ToolpathIssue::WorkLimitExceeded);
    CHECK(refused.workUnits <= control.workLimit);
    CHECK_THAT(refused.diagnostics.front().message, ContainsSubstring("work units"));
    // The diagnostics of the full run (merges, trims) do not leak into the refusal.
    CHECK(refused.diagnostics.size() == 1);
    CHECK(refused.toolpath.name == p.name);
    // Any smaller limit is refused too, down to the smallest.
    for (std::uint64_t limit : {std::uint64_t{1}, full.workUnits / 4, full.workUnits / 2}) {
        control.workLimit = limit;
        requireRefused(profileToolpath(cs, test::endMill(6.0), p, control), ToolpathIssue::WorkLimitExceeded);
    }
}

TEST_CASE("T-0186: a long edge with a neighbour is refused within a CPU bound", "[toolpath][profile][budget]") {
    // The edge costs one clearance sample per millimetre: 1e6 mm took 17 s (Release) before the
    // budget. Now 1e6 mm is outside the coordinate limit and refused at once...
    requireRefused(failFast(5.0, "1e6 mm edge", [&] { return run(longNeighbours(1e6), baseParams()); }),
                   ToolpathIssue::WorkLimitExceeded);
    // ...and the longest edge inside it (1e5 mm, about 200 million units) is stopped by the work
    // limit after about as much work as the limit says.
    ToolpathRunControl control;
    control.workLimit = 20000000;  // about 0.6 s of CPU in Release
    const ProfileResult r = failFast(60.0, "1e5 mm edge with a neighbour", [&] {
        return profileToolpath(longNeighbours(99999.0), test::endMill(6.0), baseParams(), control);
    });
    requireRefused(r, ToolpathIssue::WorkLimitExceeded);
    CHECK(r.workUnits <= control.workLimit);
}

TEST_CASE("T-0186: thin self-intersecting loops of 100 to 1000 mm are bounded by the limit", "[toolpath][profile][budget]") {
    for (double size : {100.0, 1000.0}) {
        ToolpathRunControl control;
        control.workLimit = 2000000;
        const ProfileResult r = failFast(60.0, "thin loop", [&] {
            return profileToolpath(thinLoop(size), test::endMill(3.0), baseParams(), control);
        });
        INFO("size " << size);
        if (r.ok()) {
            CHECK(r.workUnits <= control.workLimit);
        } else {
            requireRefused(r, ToolpathIssue::WorkLimitExceeded);
        }
    }
    // The 1000 mm loop needs far more than 2 million units: refused at that limit.
    ToolpathRunControl tight;
    tight.workLimit = 2000000;
    const ProfileResult big = failFast(60.0, "thin loop 1000", [&] {
        return profileToolpath(thinLoop(1000.0), test::endMill(3.0), baseParams(), tight);
    });
    requireRefused(big, ToolpathIssue::WorkLimitExceeded);
}

TEST_CASE("T-0186: the work of a Bezier-heavy job is counted where its time goes", "[toolpath][profile][budget]") {
    // A 73 mm thin loop takes about 0.3 s in Release, nearly all of it in exact closest-point tests
    // on Bezier spans (two to three microseconds each, 110 units); 19.6 million units were counted when the
    // meter was calibrated (30 ns a unit). Bounds are wide: they pin the order of magnitude.
    ProfileParams p = baseParams();
    const ProfileResult r = failFast(120.0, "thin loop 73 mm", [&] { return run(thinLoop(1.0), p, 3.0); });
    REQUIRE(r.ok());
    CHECK(r.workUnits > 10000000);
    CHECK(r.workUnits < 60000000);
}

TEST_CASE("T-0186: every Bezier span costs the exact offset's length computation", "[toolpath][profile][budget]") {
    // The exact length of a Bezier takes 8 to 15 ms whatever its size (a circle of four Beziers
    // offsets in 40 to 65 ms): about 2e5 units a quarter circle by the shape of its control polygon.
    const ProfileResult r = failFast(60.0, "bezier circle", [&] { return run({bezierCircle({0, 0}, 10.0)}, baseParams(), 3.0); });
    REQUIRE(r.ok());
    CHECK(r.workUnits > 500000);
    CHECK(r.workUnits < 4000000);
}

TEST_CASE("T-0186: the default limit leaves ordinary jobs alone", "[toolpath][profile][budget]") {
    // A sheet of 400 small parts with tabs, two passes: nowhere near the limit.
    std::vector<Contour> cs;
    for (int i = 0; i < 400; ++i) {
        cs.push_back(rect((i % 20) * 30.0, (i / 20) * 20.0, (i % 20) * 30.0 + 20.0, (i / 20) * 20.0 + 10.0));
    }
    ProfileParams p = baseParams();
    p.tabs.enabled = true;
    const ProfileResult r = run(cs, p, 3.0);
    REQUIRE(r.ok());
    CHECK(r.workUnits < kProfileDefaultWorkUnits / 20);
    CHECK_FALSE(r.has(ToolpathIssue::WorkLimitExceeded));
}

TEST_CASE("T-0186: random jobs give the same result with and without a control", "[toolpath][profile][budget]") {
    test::SplitMix rng{0x186ULL};
    for (int s = 0; s < 25; ++s) {
        std::vector<Contour> cs;
        const int n = 1 + static_cast<int>(rng.next() % 4);
        for (int i = 0; i < n; ++i) {
            const Vec2 c{rng.uniform(0, 80), rng.uniform(0, 60)};
            cs.push_back(rng.next() % 2 == 0 ? Contour::circle(c, rng.uniform(3, 15))
                                              : Contour::rectangle(Box2::fromCentre(c, rng.uniform(5, 40), rng.uniform(5, 30))));
        }
        ProfileParams p = baseParams();
        p.side = rng.next() % 2 == 0 ? ProfileSide::Outside : ProfileSide::Inside;
        p.tabs.enabled = rng.next() % 2 == 0;
        p.separateLastPass = rng.next() % 3 == 0;
        p.leads.type = rng.next() % 2 == 0 ? LeadType::Circular : LeadType::None;
        const ProfileResult plain = run(cs, p);
        ToolpathRunControl control;
        control.workLimit = plain.workUnits + 7;
        const ProfileResult limited = profileToolpath(cs, test::endMill(6.0), p, control);
        INFO("scene " << s);
        CHECK(test::formatMoves(plain.toolpath) == test::formatMoves(limited.toolpath));
        CHECK(plain.diagnostics == limited.diagnostics);
        CHECK(plain.workUnits == limited.workUnits);
    }
}

// ------------------------------------------------------------------------------ cancellation

TEST_CASE("T-0186: a flag that is already set cancels before any work", "[toolpath][profile][budget][cancel]") {
    std::atomic<bool> cancel{true};
    ToolpathRunControl control;
    control.cancel = &cancel;
    const ProfileResult r = profileToolpath(plate(), test::endMill(6.0), baseParams(), control);
    requireRefused(r, ToolpathIssue::Cancelled);
    CHECK(r.workUnits == 0);
    CHECK_THAT(r.diagnostics.front().message, ContainsSubstring("cancelled"));
}

TEST_CASE("T-0186: a flag that stays clear changes nothing", "[toolpath][profile][budget][cancel]") {
    std::atomic<bool> cancel{false};
    ToolpathRunControl control;
    control.cancel = &cancel;
    ProfileParams p = baseParams();
    p.tabs.enabled = true;
    const ProfileResult with = profileToolpath(longNeighbours(200.0), test::endMill(6.0), p, control);
    const ProfileResult without = run(longNeighbours(200.0), p);
    REQUIRE(with.ok());
    CHECK(test::formatMoves(with.toolpath) == test::formatMoves(without.toolpath));
    CHECK(with.diagnostics == without.diagnostics);
    CHECK(with.workUnits == without.workUnits);
}

TEST_CASE("T-0186: a cancel during a long run stops it promptly with nothing partial", "[toolpath][profile][budget][cancel]") {
    // Each job needs seconds of CPU (Release); the flag is set from another thread after 30 ms.
    // The latency bound (1 s of CPU, Debug included) is far above what is measured (under 0.1 s in
    // Release): a poll interval is 1024 work units, and the only uncounted stretch is one
    // geometry call (an offset or a union of one vector or merged group) of a few milliseconds here.
    const ProfileParams base = baseParams();
    SECTION("a thin self-intersecting loop") {
        const CancelRun r = cancelAfter(30, thinLoop(3.0), base, 3.0);
        requireRefused(r.result, ToolpathIssue::Cancelled);
        CHECK(r.latency < 2.0);
    }
    SECTION("a grid of merging circles") {
        std::vector<Contour> cs;
        for (int i = 0; i < 2500; ++i) {
            cs.push_back(Contour::circle({(i % 50) * 4.0, (i / 50) * 4.0}, 3.0));
        }
        const CancelRun r = cancelAfter(30, cs, base, 3.0);
        requireRefused(r.result, ToolpathIssue::Cancelled);
        CHECK(r.latency < 2.0);
    }
    SECTION("a sheet of many parts in fine passes") {
        std::vector<Contour> cs;
        for (int i = 0; i < 8000; ++i) {
            cs.push_back(rect((i % 100) * 30.0, (i / 100) * 15.0, (i % 100) * 30.0 + 20.0, (i / 100) * 15.0 + 10.0));
        }
        ProfileParams p = base;
        p.passes.cutDepth = 2.0;
        p.passes.passDepth = 0.1;
        const CancelRun r = cancelAfter(30, cs, p, 3.0);
        requireRefused(r.result, ToolpathIssue::Cancelled);
        CHECK(r.latency < 2.0);
    }
}

TEST_CASE("T-0186: the default limit is 9e8 units; one huge vector is refused before its offset runs",
          "[toolpath][profile][budget]") {
    static_assert(kProfileDefaultWorkUnits == 900000000ULL);
    // A polygon of 350,000 spans: its exact offset alone is charged about 7e8 units, more than the
    // 6e8 one uncancellable geometry call may be charged, so it is refused in the first stage with
    // the vector named, not after minutes of work (a 1.5 million span polygon: the same, cheaply).
    for (std::size_t n : {std::size_t{350000}, std::size_t{1500000}}) {
        const std::vector<Contour> cs{polygonCircle(n, 1000.0)};
        const ProfileResult r = failFast(120.0, "huge polygon", [&] { return run(cs, baseParams()); });
        requireRefused(r, ToolpathIssue::WorkLimitExceeded);
        CHECK(r.workUnits < 100000000);
        CHECK_THAT(r.diagnostics.front().message, ContainsSubstring("vector #1"));
        CHECK_THAT(r.diagnostics.front().message, ContainsSubstring("too complex to offset in one step"));
    }
    // The same polygon at 100,000 spans (2 s in Release) is within the step limit and runs.
    CHECK(failFast(300.0, "100k polygon", [&] { return run({polygonCircle(100000, 1000.0)}, baseParams()); }).ok());
}

namespace {

/// A comb of `teeth` thin teeth (0.2 mm pitch, 20 mm high) closed by a bar below.
Contour comb(int teeth) {
    Contour k;
    k.closed = true;
    const double pitch = 0.2;
    for (int i = 0; i < teeth; ++i) {
        const double x = i * pitch;
        k.append(LineSpan{{x, 0}, {x + pitch / 2, 20}});
        k.append(LineSpan{{x + pitch / 2, 20}, {x + pitch, 0}});
    }
    const double end = teeth * pitch;
    k.append(LineSpan{{end, 0}, {end, -3}});
    k.append(LineSpan{{end, -3}, {0, -3}});
    k.append(LineSpan{{0, -3}, {0, 0}});
    return k;
}

/// A closed Catmull-Rom spline through `n` points of a wobbly circle: n smooth Bezier spans.
Contour smoothSpline(int n, double radius) {
    std::vector<Vec2> pts;
    for (int i = 0; i < n; ++i) {
        const double t = geometry::kTwoPi * i / n;
        const double rr = radius * (1 + 0.05 * std::sin(9 * t) + 0.02 * std::sin(31 * t));
        pts.push_back({rr * std::cos(t), rr * std::sin(t)});
    }
    Contour k;
    k.closed = true;
    for (int i = 0; i < n; ++i) {
        const Vec2 p0 = pts[static_cast<std::size_t>((i + n - 1) % n)];
        const Vec2 p1 = pts[static_cast<std::size_t>(i)];
        const Vec2 p2 = pts[static_cast<std::size_t>((i + 1) % n)];
        const Vec2 p3 = pts[static_cast<std::size_t>((i + 2) % n)];
        k.append(BezierSpan{p1, p1 + (p2 - p0) * (1.0 / 6), p2 - (p3 - p1) * (1.0 / 6), p2});
    }
    return k;
}

}  // namespace

TEST_CASE("T-0186: smooth Bezier outlines are charged by their shape, curvy ones more", "[toolpath][profile][budget]") {
    // 400 smooth spans (traced outlines, DXF splines): the exact length of each takes tens of
    // microseconds, a flat 400,000 units per span would have refused 4,000 of them in main's 0.4 s.
    const ProfileResult smooth = failFast(120.0, "smooth spline", [&] { return run({smoothSpline(400, 100.0)}, baseParams(), 3.0); });
    REQUIRE(smooth.ok());
    CHECK(smooth.workUnits < 30000000);
    const ProfileResult circle = failFast(60.0, "bezier circle", [&] { return run({bezierCircle({0, 0}, 10.0)}, baseParams(), 3.0); });
    REQUIRE(circle.ok());
    // Four quarter circles (a control polygon 19% longer than its chord) cost more each than 100
    // smooth spans.
    CHECK(circle.workUnits * 100 > smooth.workUnits);
}

TEST_CASE("T-0186: a comb of thousands of thin teeth is charged for its sharp junctions", "[toolpath][profile][budget]") {
    ProfileParams p = baseParams();
    // Offset inwards, 40,000 teeth took 41 s and 60,000 teeth 122 s (quadratic and worse): refused
    // up front, naming the vector, in microseconds of CPU.
    p.side = ProfileSide::Inside;
    const ProfileResult in = failFast(60.0, "inside comb", [&] { return run({comb(40000)}, p, 3.0); });
    requireRefused(in, ToolpathIssue::WorkLimitExceeded);
    CHECK_THAT(in.diagnostics.front().message, ContainsSubstring("too complex to offset in one step"));
    // A comb a tenth of that size is cut (outside the comb: 2.5 s at 20,000 teeth).
    p.side = ProfileSide::Outside;
    const ProfileResult out = failFast(120.0, "outside comb", [&] { return run({comb(2000)}, p, 3.0); });
    CHECK(out.ok());
}

TEST_CASE("T-0186: tabs are charged and cost no more than linear work", "[toolpath][profile][budget]") {
    // A square with a tab every 3 mm: 2.2 / 8.7 / 33 / 128 s of CPU at 3,000 / 6,000 / 12,000 / 24,000
    // mm before the sample covering and the tab ceiling were made linear (at 3e5 units counted).
    auto job = [](double side) {
        ProfileParams p = baseParams();
        p.tabs.enabled = true;
        p.tabs.spacing = TabSpacing::ConstantDistance;
        p.tabs.distance = 3.0;
        p.tabs.length = 1.0;
        p.tabs.thickness = 2.0;
        return run({rect(0, 0, side, side)}, p, 3.0);
    };
    const ProfileResult small = failFast(60.0, "tabs 600", [&] { return job(600.0); });
    REQUIRE(small.ok());
    // 800 tabs: about 500 units each for the stage, over a run without tabs.
    ProfileParams none = baseParams();
    const ProfileResult plain = run({rect(0, 0, 600, 600)}, none, 3.0);
    REQUIRE(plain.ok());
    CHECK(small.workUnits > plain.workUnits + 600000);
    const ProfileResult big = failFast(60.0, "tabs 12000", [&] { return job(12000.0); });
    REQUIRE(big.ok());
    CHECK(big.workUnits > 15 * small.workUnits / 2);  // roughly 20x the tabs, charged 20x
    // A cancel in the tab stage is prompt.
    ProfileParams p = baseParams();
    p.tabs.enabled = true;
    p.tabs.spacing = TabSpacing::ConstantDistance;
    p.tabs.distance = 3.0;
    p.tabs.length = 1.0;
    p.tabs.thickness = 2.0;
    const CancelRun c = cancelAfter(20, {rect(0, 0, 24000, 24000)}, p, 3.0);
    REQUIRE(c.result.has(ToolpathIssue::Cancelled));  // the job needs far longer than 20 ms
    CHECK(c.latency < 2.0);
}

TEST_CASE("T-0186: the tab ceiling of sorted stretches equals the repeated maximum", "[toolpath][profile][budget]") {
    // tabCeiling appends a stretch after the ceiling's last knot instead of rebuilding with max;
    // the knots must be the ones the rebuilding gives, flat and ramped, overlapping or not.
    test::SplitMix rng{0x4ABULL};
    Contour path = rect(0, 0, 400, 300);
    const ArcLengthPath walk(path, geometry::Tolerance::defaults());
    for (int scene = 0; scene < 60; ++scene) {
        std::vector<TabSpan> spans;
        double at = rng.uniform(0, 20);
        const int n = 1 + static_cast<int>(rng.next() % 40);
        for (int i = 0; i < n; ++i) {
            const double len = rng.uniform(1, 12);
            spans.push_back({at, at + len});
            at += len + rng.uniform(scene % 3 == 0 ? -2.0 : 0.5, 30);  // some overlap in a third
            if (at > walk.length()) break;
        }
        const bool ramp = scene % 2 == 0;
        const double u0 = rng.uniform(0, 100);
        const double u1 = u0 + rng.uniform(100, 900) * (scene % 5 == 0 ? -1.0 : 1.0);
        const ZProfile got = tabCeiling(walk, u0, u1, spans, -1.0, -3.0, ramp);
        // Reference: the same stretches folded with max, one at a time.
        const double dir = u1 >= u0 ? 1.0 : -1.0;
        const double len = walk.length();
        const double travel = std::fabs(u1 - u0);
        const double rampLen = ramp ? 2.0 : 0.0;
        std::vector<std::pair<double, double>> st;
        for (const TabSpan& t : spans) {
            const long kmin = static_cast<long>(std::floor((std::min(u0, u1) - t.b - rampLen) / len));
            const long kmax = static_cast<long>(std::ceil((std::max(u0, u1) - t.a + rampLen) / len));
            for (long k = kmin; k <= kmax; ++k) {
                const double a = (t.a + static_cast<double>(k) * len - u0) * dir;
                const double b = (t.b + static_cast<double>(k) * len - u0) * dir;
                const double s0 = std::min(a, b);
                const double s1 = std::max(a, b);
                if (s1 + rampLen < 0.0 || s0 - rampLen > travel) continue;
                st.push_back({s0, s1});
            }
        }
        std::sort(st.begin(), st.end());
        ZProfile want = ZProfile::flat(kNoCeiling);
        for (const auto& [s0, s1] : st) {
            ZProfile tab;
            if (ramp) {
                tab.knots = {ZKnot{s0 - rampLen, kNoCeiling, -3.0}, ZKnot{s0, -1.0, -1.0}, ZKnot{s1, -1.0, -1.0},
                             ZKnot{s1 + rampLen, -3.0, kNoCeiling}};
            } else {
                tab.knots = {ZKnot{s0, kNoCeiling, -1.0}, ZKnot{s1, -1.0, kNoCeiling}};
            }
            want = ZProfile::max(want, tab);
        }
        INFO("scene " << scene);
        REQUIRE(got.knots.size() == want.knots.size());
        for (std::size_t i = 0; i < got.knots.size(); ++i) {
            CHECK(got.knots[i] == want.knots[i]);
        }
    }
}

TEST_CASE("T-0186: crowded chords and merged star groups are refused as one step", "[toolpath][profile][budget]") {
    // 100,000 chords of 1.3 micrometres offset by 1.5 mm (27 s in Release): the box-pair term.
    const ProfileResult dense =
        failFast(60.0, "dense chords", [&] { return run({polygonCircle(100000, 20.0)}, baseParams(), 3.0); });
    requireRefused(dense, ToolpathIssue::WorkLimitExceeded);
    CHECK_THAT(dense.diagnostics.front().message, ContainsSubstring("too complex to offset in one step"));
    // The pair term below the cap: 8,000 chords of 16 micrometres offset by 1.5 mm (about 190 box
    // pairs a chord) are charged about 3 times a polygon of the same span count without crowding.
    const ProfileResult crowded = failFast(120.0, "crowded chords", [&] { return run({polygonCircle(8000, 20.0)}, baseParams(), 3.0); });
    REQUIRE(crowded.ok());
    CHECK(crowded.workUnits > 6000000);
    // 50 concentric 500-spike stars merge into one region whose union and offset (many crossings)
    // is one step of billions of units.
    std::vector<Contour> stars;
    for (int j = 0; j < 50; ++j) {
        Contour k;
        k.closed = true;
        const int spikes = 500;
        for (int i = 0; i < spikes; ++i) {
            const double t0 = geometry::kTwoPi * i / spikes;
            const double t1 = geometry::kTwoPi * (i + 1) / spikes;
            const double r0 = (i % 2 ? 20 : 30) + j * 0.3;
            const double r1 = ((i + 1) % 2 ? 20 : 30) + j * 0.3;
            k.append(LineSpan{{r0 * std::cos(t0), r0 * std::sin(t0)}, {r1 * std::cos(t1), r1 * std::sin(t1)}});
        }
        stars.push_back(k);
    }
    const ProfileResult merged = failFast(120.0, "merged stars", [&] { return run(stars, baseParams(), 3.0); });
    requireRefused(merged, ToolpathIssue::WorkLimitExceeded);
    CHECK_THAT(merged.diagnostics.front().message, ContainsSubstring("too complex to offset in one step"));
}

TEST_CASE("T-0186: the move limit is checked up front and as moves are emitted", "[toolpath][profile][budget]") {
    ProfileParams p = baseParams();
    p.passes.cutDepth = 300.0;  // 100 passes of a 4-span rectangle: about 800 moves by the estimate
    ToolpathRunControl control;
    control.moveLimit = 500;
    const ProfileResult est = profileToolpath(plate(), test::endMill(6.0), p, control);
    requireRefused(est, ToolpathIssue::WorkLimitExceeded);
    CHECK_THAT(est.diagnostics.front().message, ContainsSubstring("500"));
    control.moveLimit = 100000;
    CHECK(profileToolpath(plate(), test::endMill(6.0), p, control).ok());

    // A Bezier circle is 4 spans for the estimate and about a thousand moves a pass once cut.
    ProfileParams q = baseParams();
    control.moveLimit = 1500;
    const ProfileResult emitted = profileToolpath({bezierCircle({0, 0}, 10.0)}, test::endMill(3.0), q, control);
    requireRefused(emitted, ToolpathIssue::WorkLimitExceeded);
    CHECK_THAT(emitted.diagnostics.front().message, ContainsSubstring("more than 1500 moves"));
    control.moveLimit = 0;  // the default
    CHECK(profileToolpath({bezierCircle({0, 0}, 10.0)}, test::endMill(3.0), q, control).ok());
}

TEST_CASE("T-0186: runs on different threads do not share a meter", "[toolpath][profile][budget]") {
    // One thread keeps running jobs that the work limit stops while another runs a normal job
    // again and again: the normal job's count and result must not change.
    ProfileParams p = baseParams();
    p.tabs.enabled = true;
    const std::vector<Contour> cs = longNeighbours(200.0);
    const ProfileResult expected = run(cs, p);
    REQUIRE(expected.ok());
    std::atomic<bool> stop{false};
    std::atomic<int> refused{0};
    std::thread other([&] {
        ToolpathRunControl tight;
        tight.workLimit = 50;
        while (!stop.load()) {
            const ProfileResult r = profileToolpath(cs, test::endMill(6.0), p, tight);
            if (r.has(ToolpathIssue::WorkLimitExceeded)) {
                ++refused;
            }
        }
    });
    int differing = 0;
    for (int i = 0; i < 30; ++i) {
        const ProfileResult r = run(cs, p);
        if (r.workUnits != expected.workUnits || r.toolpath.moves.size() != expected.toolpath.moves.size()) {
            ++differing;
        }
    }
    stop = true;
    other.join();
    CHECK(differing == 0);
    CHECK(refused.load() > 0);
}

TEST_CASE("T-0186: every coordinate of every span kind is checked", "[toolpath][profile][budget]") {
    ProfileParams p = baseParams();
    const double big = 2e5;
    for (int field = 0; field < 4; ++field) {  // a line's start x, start y, end x, end y
        Contour c;
        c.closed = true;
        LineSpan mid{{40, 0}, {40, 30}};
        double* v[4] = {&mid.start.x, &mid.start.y, &mid.end.x, &mid.end.y};
        *v[field] = big;
        c.spans = {geometry::Span{LineSpan{{0, 0}, {40, 0}}}, geometry::Span{mid}, geometry::Span{LineSpan{{40, 30}, {0, 0}}}};
        INFO("line field " << field);
        requireRefused(run({c}, p), ToolpathIssue::WorkLimitExceeded);
    }
    for (int field = 0; field < 9; ++field) {  // a Bezier's eight coordinates; the ninth is NaN
        Contour c = bezierCircle({0, 0}, 10.0);
        BezierSpan b = std::get<BezierSpan>(c.spans[1]);
        double* v[8] = {&b.p0.x, &b.p0.y, &b.p1.x, &b.p1.y, &b.p2.x, &b.p2.y, &b.p3.x, &b.p3.y};
        *v[field % 8] = field == 8 ? std::nan("") : big;
        c.spans[1] = b;
        INFO("bezier field " << field);
        const ProfileResult r = run({c}, p);
        requireRefused(r, ToolpathIssue::WorkLimitExceeded);
    }
}

