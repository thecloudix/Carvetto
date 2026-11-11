// T-0239: Fluting toolpath (fluting.hpp, ADR-0041): the depth profile at sample points along a
// straight and a curved vector for every flute type and ramp type, ramp length and percentage,
// clamping, passes, tools, ordering and hostile input.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/fluting.hpp"
#include "carvetto/toolpath/move.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using Catch::Approx;
using geometry::Contour;
using geometry::LineSpan;
using geometry::Vec2;

namespace {

tooling::Tool endMill(double passDepth = 20.0, tooling::ToolType type = tooling::ToolType::EndMill) {
    tooling::Tool t;
    t.id = "em";
    t.name = "End mill";
    t.type = type;
    t.diameter = 3.0;
    t.defaultCuttingData.toolNumber = 1;
    t.defaultCuttingData.feedRate = 1000.0;
    t.defaultCuttingData.plungeRate = 300.0;
    t.defaultCuttingData.spindleSpeed = 16000.0;
    t.defaultCuttingData.passDepth = passDepth;
    t.defaultCuttingData.stepover = 1.0;
    return t;
}

Contour line(Vec2 a, Vec2 b) {
    Contour c;
    c.append(LineSpan{a, b});
    return c;
}

/// Straight vector along +X from the origin, `length` long.
Contour straight(double length = 100.0) {
    return line({0, 0}, {length, 0});
}

/// Half circle of radius 20 about the origin from (20, 0) counter-clockwise to (-20, 0).
Contour halfCircle() {
    Contour c;
    c.append(geometry::ArcSpan::fromCentre({0, 0}, {20, 0}, {-20, 0}, geometry::ArcDirection::CCW));
    return c;
}

struct Pt {
    Vec2 p;
    double z;
};

/// The feed moves' end points (the depth profile samples).
std::vector<Pt> feeds(const Toolpath& tp) {
    std::vector<Pt> out;
    for (const Move& m : tp.moves) {
        if (m.kind == MoveKind::Feed) {
            out.push_back({{m.to.x, m.to.y}, m.to.z});
        }
    }
    return out;
}

FlutingParams params(FluteType type = FluteType::RampAtStart) {
    FlutingParams p;
    p.type = type;
    p.fluteDepth = 3.0;
    p.rampLength = 20.0;
    p.rampPercent = 20.0;
    return p;
}

FlutingResult run(const std::vector<Contour>& c, const FlutingParams& p, const tooling::Tool& t = endMill()) {
    return flutingToolpath(c, t, std::nullopt, p);
}

/// Linear interpolation of the cut along a straight vector at x.
double zAt(const std::vector<Pt>& cut, double x) {
    for (std::size_t i = 1; i < cut.size(); ++i) {
        if (x >= cut[i - 1].p.x - 1e-12 && x <= cut[i].p.x + 1e-12 && cut[i].p.x > cut[i - 1].p.x) {
            const double t = (x - cut[i - 1].p.x) / (cut[i].p.x - cut[i - 1].p.x);
            return cut[i - 1].z + (cut[i].z - cut[i - 1].z) * t;
        }
    }
    return std::nan("");
}

}  // namespace

TEST_CASE("fluting: ramp length and depth profile functions", "[fluting]") {
    FlutingParams p = params(FluteType::RampAtStart);
    CHECK(flutingRampLength(p, 100.0) == Approx(20.0));
    CHECK(flutingRampLength(p, 15.0) == Approx(15.0));  // clamped to the vector
    p.type = FluteType::RampAtStartAndEnd;
    CHECK(flutingRampLength(p, 100.0) == Approx(20.0));
    CHECK(flutingRampLength(p, 30.0) == Approx(15.0));  // two ramps never overlap
    p.type = FluteType::RampOverLength;
    CHECK(flutingRampLength(p, 100.0) == Approx(100.0));
    p.type = FluteType::RampAtStart;
    p.rampBasis = FluteRampBasis::Percent;
    p.rampPercent = 25.0;
    CHECK(flutingRampLength(p, 80.0) == Approx(20.0));
    p.rampPercent = 100.0;
    p.type = FluteType::RampAtStartAndEnd;
    CHECK(flutingRampLength(p, 80.0) == Approx(40.0));
    CHECK(flutingRampLength(p, 0.0) == 0.0);
    CHECK(flutingRampLength(p, -1.0) == 0.0);
    CHECK(flutingRampLength(p, std::nan("")) == 0.0);

    FlutingParams a = params(FluteType::RampAtStart);
    CHECK(flutingDepthAt(a, 100.0, 0.0) == 0.0);
    CHECK(flutingDepthAt(a, 100.0, 5.0) == Approx(0.75));
    CHECK(flutingDepthAt(a, 100.0, 10.0) == Approx(1.5));
    CHECK(flutingDepthAt(a, 100.0, 20.0) == Approx(3.0));
    CHECK(flutingDepthAt(a, 100.0, 70.0) == Approx(3.0));
    CHECK(flutingDepthAt(a, 100.0, 100.0) == Approx(3.0));
    CHECK(flutingDepthAt(a, 100.0, -5.0) == 0.0);
    CHECK(flutingDepthAt(a, 100.0, 500.0) == Approx(3.0));
    a.rampType = FluteRampType::Smooth;
    CHECK(flutingDepthAt(a, 100.0, 5.0) == Approx(3.0 * 0.15625));  // 3t^2 - 2t^3 at t = 0.25
    CHECK(flutingDepthAt(a, 100.0, 10.0) == Approx(1.5));
    CHECK(flutingDepthAt(a, 100.0, 15.0) == Approx(3.0 * 0.84375));
    FlutingParams b = params(FluteType::RampAtStartAndEnd);
    CHECK(flutingDepthAt(b, 100.0, 10.0) == Approx(1.5));
    CHECK(flutingDepthAt(b, 100.0, 50.0) == Approx(3.0));
    CHECK(flutingDepthAt(b, 100.0, 90.0) == Approx(1.5));
    CHECK(flutingDepthAt(b, 100.0, 100.0) == 0.0);
    FlutingParams c = params(FluteType::RampOverLength);
    CHECK(flutingDepthAt(c, 100.0, 25.0) == Approx(0.75));
    CHECK(flutingDepthAt(c, 100.0, 100.0) == Approx(3.0));
    CHECK(flutingDepthAt(c, 0.0, 0.0) == 0.0);
}

TEST_CASE("fluting: straight vector, ramp at start (linear)", "[fluting]") {
    const FlutingResult r = run({straight()}, params(FluteType::RampAtStart));
    REQUIRE(r.ok());
    REQUIRE(r.flutes.size() == 1);
    CHECK(r.flutes[0].length == Approx(100.0));
    CHECK(r.flutes[0].rampLength == Approx(20.0));
    const std::vector<Pt> cut = feeds(r.toolpath);
    REQUIRE(cut.size() == 2);  // vertices, plus the ramp's end
    CHECK(cut.front().p.x == Approx(20.0));  // the first sample (x = 0) is the plunge
    CHECK(zAt(cut, 20.0) == Approx(-3.0));
    CHECK(cut.back().p.x == Approx(100.0));
    CHECK(cut.back().z == Approx(-3.0));
    // The plunge is at the start, at the surface; the ramp reaches full depth at 20.
    const Move& plunge = r.toolpath.moves[1];
    CHECK(plunge.kind == MoveKind::Plunge);
    CHECK(plunge.to.x == Approx(0.0));
    CHECK(plunge.to.z == Approx(0.0));
    CHECK(r.toolpath.moves.front().kind == MoveKind::Rapid);
    CHECK(r.toolpath.moves.front().to.z == 5.0);
    CHECK(r.toolpath.moves.back().kind == MoveKind::Retract);
    CHECK(r.toolpath.moves.back().to.z == 5.0);
}

TEST_CASE("fluting: depth at sample points, all flute types", "[fluting]") {
    for (const FluteRampType rt : {FluteRampType::Linear, FluteRampType::Smooth}) {
        for (const FluteType ft : {FluteType::RampOverLength, FluteType::RampAtStart, FluteType::RampAtStartAndEnd}) {
            FlutingParams p = params(ft);
            p.rampType = rt;
            const FlutingResult r = run({straight()}, p);
            REQUIRE(r.ok());
            std::vector<Pt> cut = feeds(r.toolpath);
            cut.insert(cut.begin(), {{0.0, 0.0}, r.toolpath.moves[1].to.z});
            // Smooth ramps are sampled, so compare at the sample points themselves.
            for (const Pt& s : cut) {
                INFO("type " << static_cast<int>(ft) << " ramp " << static_cast<int>(rt) << " x " << s.p.x);
                CHECK(-s.z == Approx(flutingDepthAt(p, 100.0, s.p.x)).margin(1e-9));
                CHECK(s.p.y == Approx(0.0).margin(1e-12));
            }
            // And between samples within a small tolerance (the chord of a smoothstep step).
            for (double x = 0.5; x < 100.0; x += 3.7) {
                const double z = zAt(cut, x);
                REQUIRE(std::isfinite(z));
                CHECK(-z == Approx(flutingDepthAt(p, 100.0, x)).margin(0.01));
            }
            CHECK(cut.front().z == Approx(0.0).margin(1e-12));
            CHECK(cut.back().z == Approx(ft == FluteType::RampAtStartAndEnd ? 0.0 : -3.0).margin(1e-12));
        }
    }
}

TEST_CASE("fluting: ramp at start and end, percentage ramp", "[fluting]") {
    FlutingParams p = params(FluteType::RampAtStartAndEnd);
    p.rampBasis = FluteRampBasis::Percent;
    p.rampPercent = 10.0;
    const FlutingResult r = run({straight(200.0)}, p);
    REQUIRE(r.ok());
    CHECK(r.flutes[0].rampLength == Approx(20.0));
    std::vector<Pt> cut = feeds(r.toolpath);
    cut.insert(cut.begin(), {{0.0, 0.0}, r.toolpath.moves[1].to.z});
    CHECK(zAt(cut, 10.0) == Approx(-1.5));
    CHECK(zAt(cut, 20.0) == Approx(-3.0));
    CHECK(zAt(cut, 180.0) == Approx(-3.0));
    CHECK(zAt(cut, 190.0) == Approx(-1.5));
    CHECK(cut.back().z == Approx(0.0).margin(1e-12));
}

TEST_CASE("fluting: ramp longer than the vector is shortened and reported", "[fluting]") {
    FlutingParams p = params(FluteType::RampAtStartAndEnd);
    p.rampLength = 80.0;
    const FlutingResult r = run({straight(100.0)}, p);
    REQUIRE(r.ok());
    CHECK(r.has(ToolpathIssue::LeadShortened));
    CHECK(r.flutes[0].rampLength == Approx(50.0));
    const std::vector<Pt> cut = feeds(r.toolpath);
    CHECK(zAt(cut, 50.0) == Approx(-3.0));
    CHECK(cut.back().z == Approx(0.0).margin(1e-12));
    // A ramp that fits is not reported.
    CHECK_FALSE(run({straight(100.0)}, params(FluteType::RampAtStart)).has(ToolpathIssue::LeadShortened));
    // The complete-length ramp ignores the ramp fields.
    FlutingParams q = params(FluteType::RampOverLength);
    q.rampLength = 1e9;
    CHECK(run({straight(100.0)}, q).ok());
}

TEST_CASE("fluting: curved vector, depth follows arc length", "[fluting]") {
    FlutingParams p = params(FluteType::RampAtStartAndEnd);
    p.rampLength = 10.0;
    p.rampType = FluteRampType::Smooth;
    const double total = geometry::kPi * 20.0;
    const FlutingResult r = run({halfCircle()}, p);
    REQUIRE(r.ok());
    CHECK(r.flutes[0].length == Approx(total).epsilon(1e-3));
    std::vector<Pt> cut = feeds(r.toolpath);
    cut.insert(cut.begin(), {{20.0, 0.0}, r.toolpath.moves[1].to.z});
    REQUIRE(cut.size() > 40);
    // Every sample lies on the circle; its depth is the profile at its arc length (angle * R).
    double maxDepth = 0.0;
    for (const Pt& s : cut) {
        CHECK(std::hypot(s.p.x, s.p.y) == Approx(20.0).margin(0.011));
        const double arc = std::atan2(s.p.y, s.p.x) * 20.0;
        CHECK(-s.z == Approx(flutingDepthAt(p, r.flutes[0].length, arc)).margin(0.02));
        maxDepth = std::max(maxDepth, -s.z);
    }
    CHECK(maxDepth == Approx(3.0).margin(1e-9));
    CHECK(cut.front().z == Approx(0.0).margin(1e-12));
    CHECK(cut.back().z == Approx(0.0).margin(1e-9));
    // The middle of the arc is at full depth; a quarter of the ramp in (2.5 mm of 10) is not.
    for (const Pt& s : cut) {
        const double arc = std::atan2(s.p.y, s.p.x) * 20.0;
        if (arc > 12.0 && arc < total - 12.0) {
            CHECK(-s.z == Approx(3.0).margin(1e-9));
        }
    }
}

TEST_CASE("fluting: a closed vector starts and ends at its start point", "[fluting]") {
    FlutingParams p = params(FluteType::RampAtStartAndEnd);
    p.rampLength = 10.0;
    const Contour circle = Contour::circle({0, 0}, 20.0);
    const FlutingResult r = run({circle}, p);
    REQUIRE(r.ok());
    CHECK(r.flutes[0].length == Approx(2.0 * geometry::kPi * 20.0).epsilon(1e-3));
    const std::vector<Pt> cut = feeds(r.toolpath);
    CHECK(cut.back().p.x == Approx(20.0).margin(1e-9));
    CHECK(cut.back().p.y == Approx(0.0).margin(1e-9));
    CHECK(cut.back().z == Approx(0.0).margin(1e-9));
}

TEST_CASE("fluting: ball nose and flat end mill are both accepted, other tools refused", "[fluting]") {
    CHECK(flutingToolAllowed(tooling::ToolType::EndMill));
    CHECK(flutingToolAllowed(tooling::ToolType::BallNose));
    CHECK_FALSE(flutingToolAllowed(tooling::ToolType::VBit));
    CHECK_FALSE(flutingToolAllowed(tooling::ToolType::Drill));
    const FlutingResult ball = run({straight()}, params(), endMill(20.0, tooling::ToolType::BallNose));
    REQUIRE(ball.ok());
    CHECK(ball.toolpath.tool.diameter == 3.0);
    tooling::Tool v = endMill();
    v.type = tooling::ToolType::VBit;
    v.includedAngle = 1.5;
    const FlutingResult bad = run({straight()}, params(), v);
    CHECK_FALSE(bad.ok());
    CHECK(bad.has(ToolpathIssue::InvalidTool));
    tooling::Tool zero = endMill();
    zero.diameter = 0.0;
    CHECK(run({straight()}, params(), zero).has(ToolpathIssue::InvalidTool));
    tooling::Tool noFeed = endMill();
    noFeed.defaultCuttingData.feedRate = 0.0;
    CHECK(run({straight()}, params(), noFeed).has(ToolpathIssue::InvalidTool));
    tooling::Tool noPass = endMill();
    noPass.defaultCuttingData.passDepth = 0.0;
    CHECK(run({straight()}, params(), noPass).has(ToolpathIssue::InvalidTool));
}

TEST_CASE("fluting: a deep flute is cut in passes, each only where it is deeper than the last", "[fluting]") {
    FlutingParams p = params(FluteType::RampAtStartAndEnd);
    p.fluteDepth = 3.0;
    p.rampLength = 20.0;
    const FlutingResult r = run({straight()}, p, endMill(1.0));
    REQUIRE(r.ok());
    // Walk the moves: the deepest Z of each run (plunge .. retract) and its extent.
    std::vector<double> runMin;
    std::vector<double> runMinX, runMaxX;
    double mn = 0.0;
    double lo = 1e9, hi = -1e9;
    bool in = false;
    for (const Move& m : r.toolpath.moves) {
        if (m.kind == MoveKind::Plunge) {
            in = true;
            mn = m.to.z;
            lo = hi = m.to.x;
        } else if (m.kind == MoveKind::Feed && in) {
            mn = std::min(mn, m.to.z);
            lo = std::min(lo, m.to.x);
            hi = std::max(hi, m.to.x);
        } else if (m.kind == MoveKind::Retract && in) {
            runMin.push_back(mn);
            runMinX.push_back(lo);
            runMaxX.push_back(hi);
            in = false;
        }
    }
    REQUIRE(runMin.size() == 3);
    CHECK(runMin[0] == Approx(-1.0));
    CHECK(runMin[1] == Approx(-2.0));
    CHECK(runMin[2] == Approx(-3.0));
    // The first pass starts at the beginning of the ramp; later passes start where the profile reaches
    // the previous level (x = 20/3 and 40/3) and end where it falls back (mirror).
    CHECK(runMinX[0] == Approx(0.0).margin(1e-9));
    CHECK(runMinX[1] == Approx(20.0 / 3.0).margin(1e-9));
    CHECK(runMinX[2] == Approx(40.0 / 3.0).margin(1e-9));
    CHECK(runMaxX[0] == Approx(100.0).margin(1e-9));
    CHECK(runMaxX[1] == Approx(100.0 - 20.0 / 3.0).margin(1e-9));
    CHECK(runMaxX[2] == Approx(100.0 - 40.0 / 3.0).margin(1e-9));
    // No cut is deeper than the profile.
    for (const Move& m : r.toolpath.moves) {
        if (m.kind == MoveKind::Feed || m.kind == MoveKind::Plunge) {
            CHECK(-m.to.z <= flutingDepthAt(p, 100.0, m.to.x) + 1e-9);
        }
    }
}

TEST_CASE("fluting: a flute shallower than one pass is cut once; a depth that divides evenly makes no empty pass",
          "[fluting]") {
    const FlutingResult once = run({straight()}, params(), endMill(5.0));
    REQUIRE(once.ok());
    int plunges = 0;
    for (const Move& m : once.toolpath.moves) {
        plunges += m.kind == MoveKind::Plunge ? 1 : 0;
    }
    CHECK(plunges == 1);
    FlutingParams p = params(FluteType::RampOverLength);
    p.fluteDepth = 2.0;
    const FlutingResult two = run({straight()}, p, endMill(1.0));
    plunges = 0;
    for (const Move& m : two.toolpath.moves) {
        plunges += m.kind == MoveKind::Plunge ? 1 : 0;
    }
    CHECK(plunges == 2);
    p.fluteDepth = 1e5;
    CHECK(run({straight()}, p, endMill(0.01)).has(ToolpathIssue::WorkLimitExceeded));
}

TEST_CASE("fluting: several vectors are cut in the nearest-first order from the start position", "[fluting]") {
    FlutingParams p = params();
    p.startPosition = {0, 50};
    const FlutingResult r = run({line({0, 0}, {50, 0}), line({0, 40}, {50, 40}), line({0, 100}, {50, 100})}, p);
    REQUIRE(r.ok());
    REQUIRE(r.flutes.size() == 3);
    CHECK(r.flutes[0].contour == 1);
}

TEST_CASE("fluting: degenerate vectors are skipped and named", "[fluting]") {
    Contour gap;
    gap.append(LineSpan{{0, 0}, {10, 0}});
    gap.append(LineSpan{{20, 0}, {30, 0}});
    const FlutingResult r = run({Contour{}, gap, line({5, 5}, {5, 5}), straight()}, params());
    REQUIRE(r.ok());
    REQUIRE(r.flutes.size() == 1);
    CHECK(r.flutes[0].contour == 3);
    CHECK(r.has(ToolpathIssue::EmptyContour));
    CHECK(r.has(ToolpathIssue::DiscontinuousContour));
    CHECK(r.has(ToolpathIssue::NoRegion));
    const FlutingResult none = run({Contour{}}, params());
    CHECK_FALSE(none.ok());
    CHECK(none.has(ToolpathIssue::EmptyContour));
    const FlutingResult nothing = run({}, params());
    CHECK_FALSE(nothing.ok());
    CHECK(nothing.has(ToolpathIssue::NoRegion));
}

TEST_CASE("fluting: hostile parameters give an empty toolpath and a diagnostic", "[fluting]") {
    const double nan = std::nan("");
    const double inf = std::numeric_limits<double>::infinity();
    const auto bad = [](auto edit) {
        FlutingParams p = params();
        edit(p);
        const FlutingResult r = run({straight()}, p);
        CHECK_FALSE(r.ok());
        CHECK(r.has(ToolpathIssue::InvalidParameters));
        CHECK(r.toolpath.moves.empty());
    };
    bad([&](FlutingParams& p) { p.fluteDepth = nan; });
    bad([&](FlutingParams& p) { p.fluteDepth = inf; });
    bad([](FlutingParams& p) { p.fluteDepth = 0.0; });
    bad([](FlutingParams& p) { p.fluteDepth = -1.0; });
    bad([](FlutingParams& p) { p.fluteDepth = 1e9; });
    bad([&](FlutingParams& p) { p.rampLength = nan; });
    bad([](FlutingParams& p) { p.rampLength = 0.0; });
    bad([](FlutingParams& p) { p.rampLength = -5.0; });
    bad([](FlutingParams& p) {
        p.rampBasis = FluteRampBasis::Percent;
        p.rampPercent = 0.0;
    });
    bad([](FlutingParams& p) {
        p.rampBasis = FluteRampBasis::Percent;
        p.rampPercent = 100.5;
    });
    bad([&](FlutingParams& p) { p.safeZ = inf; });
    bad([](FlutingParams& p) { p.safeZ = 0.0; });
    bad([&](FlutingParams& p) { p.startPosition.x = nan; });
    bad([&](FlutingParams& p) { p.tolerance.chordal = nan; });
    bad([](FlutingParams& p) { p.tolerance.chordal = 0.0; });
    bad([](FlutingParams& p) { p.tolerance.distance = 0.5; });
    // Non-finite geometry.
    const double big = std::numeric_limits<double>::max();
    const FlutingResult h = run({line({0, 0}, {nan, 1}), line({-big, 0}, {big, 0}), straight()}, params());
    for (const Move& m : h.toolpath.moves) {
        CHECK(std::isfinite(m.to.x));
        CHECK(std::isfinite(m.to.y));
        CHECK(std::isfinite(m.to.z));
    }
    // A ramp field that is unused for the complete-length ramp is not validated.
    FlutingParams q = params(FluteType::RampOverLength);
    q.rampLength = -1.0;
    CHECK(run({straight()}, q).ok());
}
