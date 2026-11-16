// T-0322: Rounding toolpath (rounding.hpp, ADR-0115): pass radii and Z, stroke counts and spacing for each
// method on a round and a square blank, rest machining, allowance, joins between strokes, limits and hostile
// input.

#include <algorithm>
#include <atomic>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <set>
#include <vector>

#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/move.hpp"
#include "carvetto/toolpath/rounding.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using Catch::Approx;
using geometry::Vec3;

namespace {

constexpr double kPi = std::numbers::pi_v<double>;

tooling::Tool mill(double diameter = 6.0, double pass = 2.0, double step = 3.0) {
    tooling::Tool t;
    t.id = "mill";
    t.name = "Mill";
    t.type = tooling::ToolType::EndMill;
    t.diameter = diameter;
    t.defaultCuttingData.toolNumber = 1;
    t.defaultCuttingData.feedRate = 1000.0;
    t.defaultCuttingData.plungeRate = 300.0;
    t.defaultCuttingData.spindleSpeed = 12000.0;
    t.defaultCuttingData.passDepth = pass;
    t.defaultCuttingData.stepover = step;
    return t;
}

/// Cylinder of radius 10 (circumference 2 pi 10), 30 mm long, turned from 10 to 6 in passes of 2.
RoundingParams base() {
    RoundingParams p;
    p.blankRadius = 10.0;
    p.targetRadius = 6.0;
    p.alongStart = 0.0;
    p.alongEnd = 30.0;
    p.surfaceZ = 0.0;
    p.safeZ = 5.0;
    return p;
}

RoundingResult run(const RoundingParams& p, const tooling::Tool& t = mill()) {
    return roundingToolpath(t, std::nullopt, p);
}

std::vector<Move> feeds(const RoundingResult& r) {
    std::vector<Move> out;
    for (const Move& m : r.toolpath.moves) {
        if (m.kind == MoveKind::Feed) {
            out.push_back(m);
        }
    }
    return out;
}

/// Index of every feed move.
std::vector<std::size_t> feedIndices(const RoundingResult& r) {
    std::vector<std::size_t> out;
    for (std::size_t i = 0; i < r.toolpath.moves.size(); ++i) {
        if (r.toolpath.moves[i].kind == MoveKind::Feed) {
            out.push_back(i);
        }
    }
    return out;
}

std::size_t count(const RoundingResult& r, MoveKind k) { return r.toolpath.count(k); }

}  // namespace

TEST_CASE("rounding: pass radii are equal steps ending at the bottom", "[rounding]") {
    auto r = roundingPassRadii(10.0, 6.0, 2.0);
    REQUIRE(r.size() == 2);
    CHECK(r[0] == Approx(8.0));
    CHECK(r[1] == Approx(6.0));
    r = roundingPassRadii(10.0, 6.0, 1.5);  // 4 / 1.5 -> 3 passes of 4/3
    REQUIRE(r.size() == 3);
    CHECK(r[0] == Approx(10.0 - 4.0 / 3.0));
    CHECK(r[1] == Approx(10.0 - 8.0 / 3.0));
    CHECK(r[2] == 6.0);
    r = roundingPassRadii(10.0, 6.0, 100.0);
    REQUIRE(r.size() == 1);
    CHECK(r[0] == 6.0);
    CHECK(roundingPassRadii(6.0, 6.0, 1.0).empty());
    CHECK(roundingPassRadii(5.0, 6.0, 1.0).empty());
    CHECK(roundingPassRadii(10.0, 6.0, 0.0).empty());
    CHECK(roundingPassRadii(10.0, 6.0, 1e-9).empty());  // more than the pass limit
    CHECK(roundingPassRadii(10.0, 6.0, std::nan("")).empty());
    // A depth that is an exact multiple does not make an extra pass.
    CHECK(roundingPassRadii(10.0, 6.0, 1.0).size() == 4);
    CHECK(roundingPassRadii(10.0, 6.0, 1.0000001).size() == 4);
    // Rounding error in the quotient (2.1 / 0.3 is 7.000000000000001) does not add a pass.
    CHECK(roundingPassRadii(2.1, 0.0, 0.3).size() == 7);
    CHECK(roundingPassRadii(2.1, 0.0, 0.3000001).size() == 7);
    CHECK(roundingPassRadii(2.1, 0.0, 0.29).size() == 8);
}

TEST_CASE("rounding: square blank material stretches", "[rounding]") {
    const double r = 10.0;
    const double h = r / std::numbers::sqrt2;
    const double turn = 2.0 * kPi * r;
    auto full = roundingMaterial(RoundingBlank::Square, r, h - 0.1, 0.0);
    REQUIRE(full.size() == 1);
    CHECK(full[0].first == 0.0);
    CHECK(full[0].second == Approx(turn));
    CHECK(roundingMaterial(RoundingBlank::Square, r, h, 0.0).size() == 1);  // at the inscribed circle
    CHECK(roundingMaterial(RoundingBlank::Square, r, r, 0.0).empty());       // at the corners: nothing
    CHECK(roundingMaterial(RoundingBlank::Square, r, r + 1.0, 0.0).empty());
    CHECK(roundingMaterial(RoundingBlank::Round, r, r, 0.0).empty());
    // Radius 9: corners at angles 45, 135, 225, 315 degrees with half-width 45 deg - acos(h / 9).
    const auto four = roundingMaterial(RoundingBlank::Square, r, 9.0, 0.0);
    REQUIRE(four.size() == 4);
    const double half = (0.25 * kPi - std::acos(h / 9.0)) * r;
    for (int q = 0; q < 4; ++q) {
        const double centre = (0.25 * kPi + 0.5 * kPi * q) * r;
        CHECK(four[static_cast<std::size_t>(q)].first == Approx(centre - half));
        CHECK(four[static_cast<std::size_t>(q)].second == Approx(centre + half));
        // The edges are exactly where the blank's radius equals 9.
        const double theta = (four[static_cast<std::size_t>(q)].first) / r;
        const double phi = theta - 0.5 * kPi * std::round(theta / (0.5 * kPi));
        CHECK(h / std::cos(phi) == Approx(9.0));
    }
    // A start inside the turn clips and splits the stretch that spans the start.
    const double c0 = 0.25 * kPi * r;  // the middle of the first corner
    const auto cut = roundingMaterial(RoundingBlank::Square, r, 9.0, c0);
    REQUIRE(cut.size() == 5);
    CHECK(cut.front().first == Approx(c0));
    CHECK(cut.front().second == Approx(c0 + half));
    CHECK(cut.back().first == Approx(c0 + turn - half));
    CHECK(cut.back().second == Approx(c0 + turn));
    // A start in an earlier turn gives the same stretches shifted.
    const auto negative = roundingMaterial(RoundingBlank::Square, r, 9.0, -turn);
    REQUIRE(negative.size() == 4);
    CHECK(negative[0].first == Approx(four[0].first - turn));
}

TEST_CASE("rounding: raster strokes are full turns spaced along the axis", "[rounding]") {
    RoundingParams p = base();
    p.method = RoundingMethod::Raster;
    const RoundingResult r = run(p);
    REQUIRE(r.ok());
    REQUIRE(r.radii.size() == 2);
    CHECK(r.radii[0] == Approx(8.0));
    CHECK(r.radii[1] == Approx(6.0));
    // stepover 3 over 30 mm: 10 steps, 11 turns per pass, two passes.
    CHECK(r.strokes == 22);
    CHECK(count(r, MoveKind::Plunge) == 22);  // lifted between every stroke
    CHECK(count(r, MoveKind::Feed) == 22);
    CHECK(count(r, MoveKind::Rapid) == 22);
    CHECK(count(r, MoveKind::Retract) == 22);
    const double turn = 2.0 * kPi * 10.0;
    std::size_t i = 0;
    for (const std::size_t at : feedIndices(r)) {
        const Move& m = r.toolpath.moves[at];
        const std::size_t ring = i % 11;
        const Vec3 from = r.toolpath.startOf(at);
        CHECK(from.x == Approx(3.0 * static_cast<double>(ring)));  // alongX: X is along the axis
        CHECK(m.to.x == Approx(3.0 * static_cast<double>(ring)));
        CHECK(from.y == Approx(0.0));
        CHECK(m.to.y == Approx(turn));  // one full turn, always the same way
        CHECK(m.to.z == Approx(i < 11 ? 8.0 - 10.0 : 6.0 - 10.0));  // surface 0, axis -10
        ++i;
    }
    CHECK(r.toolpath.maxZ().value() == Approx(5.0));
    CHECK(r.toolpath.minZ().value() == Approx(-4.0));
    CHECK(r.toolpath.moves.back().kind == MoveKind::Retract);
    CHECK(r.toolpath.moves.back().to.z == 5.0);
    CHECK(r.toolpath.moves.back().to.x == Approx(30.0));  // lifts where the last stroke ended
    CHECK(r.toolpath.moves.back().to.y == Approx(turn));
}

TEST_CASE("rounding: optimised raster alternates and stays down between turns", "[rounding]") {
    RoundingParams p = base();
    p.method = RoundingMethod::OptimizedRaster;
    const RoundingResult r = run(p);
    REQUIRE(r.ok());
    CHECK(r.strokes == 22);
    // One plunge per pass; every other turn joins its neighbour with a step along the axis.
    CHECK(count(r, MoveKind::Plunge) == 2);
    CHECK(count(r, MoveKind::Rapid) == 2);
    CHECK(count(r, MoveKind::Retract) == 2);
    CHECK(count(r, MoveKind::Feed) == 22 + 20);  // strokes plus the joins
    const double turn = 2.0 * kPi * 10.0;
    const auto& mv = r.toolpath.moves;
    // First pass: moves are rapid, plunge, then (feed turn, feed join) x ...
    REQUIRE(mv.size() > 6);
    CHECK(mv[0].kind == MoveKind::Rapid);
    CHECK(mv[0].to == Vec3{0.0, 0.0, 5.0});
    CHECK(mv[1].kind == MoveKind::Plunge);
    CHECK(mv[1].to == Vec3{0.0, 0.0, -2.0});
    CHECK(mv[2].to.y == Approx(turn));  // out
    CHECK(mv[2].to.x == 0.0);
    CHECK(mv[3].kind == MoveKind::Feed);  // join: the step along the axis at the end of the turn
    CHECK(mv[3].to.x == Approx(3.0));
    CHECK(mv[3].to.y == Approx(turn));
    CHECK(mv[4].to.x == Approx(3.0));
    CHECK(mv[4].to.y == Approx(0.0));  // and back
    CHECK(mv[5].to.x == Approx(6.0));  // join at the start
    CHECK(mv[5].to.y == Approx(0.0));
    // The last turn of an odd count (11 turns, indices 0..10) goes out, so the pass ends at the far side.
    const std::size_t retract = static_cast<std::size_t>(
        std::find_if(mv.begin(), mv.end(), [](const Move& m) { return m.kind == MoveKind::Retract; }) - mv.begin());
    CHECK(mv[retract - 1].to.x == Approx(30.0));
    CHECK(mv[retract - 1].to.y == Approx(turn));
}

TEST_CASE("rounding: radial strokes run along the axis at even angles", "[rounding]") {
    RoundingParams p = base();
    p.method = RoundingMethod::Radial;
    p.stepover = 5.0;
    const RoundingResult r = run(p);
    REQUIRE(r.ok());
    const double turn = 2.0 * kPi * 10.0;
    const std::size_t per = static_cast<std::size_t>(std::ceil(turn / 5.0));  // 13 strokes round the circumference
    CHECK(per == 13);
    CHECK(r.strokes == 2 * per);
    // The tool stays down between neighbouring strokes: one plunge per pass.
    CHECK(count(r, MoveKind::Plunge) == 2);
    const auto& mv = r.toolpath.moves;
    CHECK(mv[1].kind == MoveKind::Plunge);
    CHECK(mv[2].to.x == Approx(30.0));  // out along the whole length
    CHECK(mv[2].to.y == Approx(0.0));
    CHECK(mv[3].kind == MoveKind::Feed);  // join: step round the circumference at the end
    CHECK(mv[3].to.x == Approx(30.0));
    CHECK(mv[3].to.y == Approx(turn / 13.0));
    CHECK(mv[4].to.x == Approx(0.0));  // and back
    CHECK(mv[4].to.y == Approx(turn / 13.0));
    CHECK(mv[5].to.y == Approx(2.0 * turn / 13.0));
}

TEST_CASE("rounding: a cylinder along Y swaps the axes", "[rounding]") {
    RoundingParams p = base();
    p.method = RoundingMethod::Raster;
    p.alongX = false;
    p.alongStart = 5.0;
    p.alongEnd = 35.0;
    p.circStart = 2.0;
    const RoundingResult r = run(p);
    REQUIRE(r.ok());
    const auto f = feedIndices(r);
    REQUIRE(!f.empty());
    CHECK(r.toolpath.startOf(f[0]).y == Approx(5.0));
    CHECK(r.toolpath.startOf(f[0]).x == Approx(2.0));
    CHECK(r.toolpath.moves[f[0]].to.y == Approx(5.0));
    CHECK(r.toolpath.moves[f[0]].to.x == Approx(2.0 + 2.0 * kPi * 10.0));
    CHECK(r.toolpath.bounds().min.y == Approx(5.0));
    CHECK(r.toolpath.bounds().max.y == Approx(35.0));
}

TEST_CASE("rounding: Z follows the zero position, the allowance and the rest radius", "[rounding]") {
    RoundingParams p = base();
    p.method = RoundingMethod::Raster;
    p.surfaceZ = 10.0;  // axis zero: the surface is at +radius
    p.safeZ = 15.0;
    p.allowance = 0.5;
    RoundingResult r = run(p);
    REQUIRE(r.ok());
    CHECK(r.radii.back() == Approx(6.5));
    CHECK(r.toolpath.minZ().value() == Approx(6.5));  // axis at Z 0
    CHECK(r.toolpath.maxZ().value() == Approx(15.0));
    // Rest machining starts from the radius already turned.
    p = base();
    p.method = RoundingMethod::Raster;
    p.startRadius = 8.0;
    r = run(p);
    REQUIRE(r.ok());
    REQUIRE(r.radii.size() == 1);
    CHECK(r.radii[0] == 6.0);
    CHECK(r.strokes == 11);
    // A start radius above the blank changes nothing; one at the finish leaves nothing.
    p.startRadius = 12.0;
    CHECK(run(p).radii.size() == 2);
    p.startRadius = 6.0;
    r = run(p);
    CHECK(!r.ok());
    CHECK(r.has(ToolpathIssue::NoRestMaterial));
    p = base();
    p.targetRadius = 10.0;
    CHECK(run(p).has(ToolpathIssue::NoRestMaterial));
    p.targetRadius = 9.5;
    p.allowance = 0.5;
    CHECK(run(p).has(ToolpathIssue::NoRestMaterial));
}

TEST_CASE("rounding: pass depth and stepover override the tool's", "[rounding]") {
    RoundingParams p = base();
    p.method = RoundingMethod::Raster;
    p.passDepth = 1.0;
    p.stepover = 5.0;
    const RoundingResult r = run(p);
    REQUIRE(r.ok());
    CHECK(r.radii.size() == 4);
    CHECK(r.strokes == 4 * 7);  // 30 / 5 = 6 steps, 7 turns
    // A stepover wider than the tool is limited to its diameter (6 mm: 5 steps of 6).
    p.stepover = 20.0;
    const RoundingResult wide = run(p);
    REQUIRE(wide.ok());
    CHECK(wide.has(ToolpathIssue::StepoverLimited));
    CHECK(wide.strokes == 4 * 6);
    // A stepover that is not a divisor spreads the strokes evenly (never wider than asked).
    p.stepover = 4.0;  // 30 / 4 = 7.5 -> 8 steps of 3.75
    const RoundingResult even = run(p);
    CHECK(even.strokes == 4 * 9);
    std::set<long long> xs;
    for (const Move& m : feeds(even)) {
        xs.insert(std::llround(m.to.x * 1000.0));
    }
    CHECK(xs.size() == 9);
    CHECK(*xs.begin() == 0);
    CHECK(*xs.rbegin() == 30000);
    CHECK(*std::next(xs.begin()) == 3750);
}

TEST_CASE("rounding: strokes are counted without rounding error; a stepover as wide as the tool is not limited",
          "[rounding]") {
    RoundingParams p = base();
    p.method = RoundingMethod::Raster;
    p.alongEnd = 2.1;
    p.stepover = 0.3;  // 2.1 / 0.3 is 7.000000000000001: seven steps, eight turns, not nine
    p.targetRadius = 9.0;
    p.passDepth = 1.0;
    const RoundingResult r = run(p);
    REQUIRE(r.ok());
    CHECK(r.strokes == 8);
    p = base();
    p.method = RoundingMethod::Raster;
    p.stepover = 6.0;  // exactly the tool's diameter: allowed as asked
    CHECK(!run(p).has(ToolpathIssue::StepoverLimited));
    p.stepover = 6.0001;
    CHECK(run(p).has(ToolpathIssue::StepoverLimited));
}

TEST_CASE("rounding: a square blank is cut only where material is left", "[rounding]") {
    RoundingParams p = base();
    p.blank = RoundingBlank::Square;
    p.method = RoundingMethod::Raster;
    p.passDepth = 1.0;  // radii 9, 8, 7, 6; the inscribed circle is at 7.07
    const RoundingResult r = run(p);
    REQUIRE(r.ok());
    REQUIRE(r.radii.size() == 4);
    // Passes at 9 and 8 only touch the four corners, 7.07 < 7 is false: the pass at 7 is full, so is 6.
    CHECK(r.strokes == 11 * 4 + 11 * 4 + 11 + 11);
    // All strokes of the first pass lie on corner stretches (half-width 45 deg - acos(7.071 / 9)).
    const double half = (0.25 * kPi - std::acos(10.0 / std::numbers::sqrt2 / 9.0)) * 10.0;
    for (const Move& m : feeds(r)) {
        if (m.to.z < -1.5) {
            break;  // only the first pass is checked (z = 9 - 10 = -1)
        }
        const double quarter = 0.5 * kPi * 10.0;
        // distance of the stroke from the nearest corner centre (corners sit mid-way in each quarter)
        const double d = std::fabs(std::fmod(m.to.y, quarter) - 0.5 * quarter);
        if (m.to.y > 1e-9 && m.to.y < 2.0 * kPi * 10.0 - 1e-9) {
            CHECK(d <= half + 1e-6);
        }
    }
    // The optimised raster joins the stretches of neighbouring turns, never different stretches.
    p.method = RoundingMethod::OptimizedRaster;
    const RoundingResult o = run(p);
    REQUIRE(o.ok());
    CHECK(o.strokes == r.strokes);
    // First pass: 4 stretches per turn, 11 turns; a join per turn after the first: 10 joins; so plunges are
    // one per stretch group: 4 stretch starts (the first of each turn joins, the other 3 re-plunge each turn).
    // plunges = 1 + 3 per turn x 11 for pass 1.
    // (pass 1: 1 + 33, pass 2: 1 + 33, pass 3: 1, pass 4: 1)
    CHECK(count(o, MoveKind::Plunge) == 34 + 34 + 1 + 1);
    // Radial: a stroke exists only where its angle has material above the pass.
    p.method = RoundingMethod::Radial;
    p.stepover = 5.0;
    const RoundingResult radial = run(p);
    REQUIRE(radial.ok());
    std::size_t expected = 0;
    for (const double rho : radial.radii) {
        for (int j = 0; j < 13; ++j) {
            const double theta = 2.0 * kPi * j / 13.0;
            const double phi = theta - 0.5 * kPi * std::round(theta / (0.5 * kPi));
            if (10.0 / std::numbers::sqrt2 / std::cos(phi) > rho + 1e-9) {
                ++expected;
            }
        }
    }
    CHECK(radial.strokes == expected);
    CHECK(radial.strokes < 4 * 13);
    CHECK(radial.strokes > 2 * 13);
    // The tool stays down along runs of neighbouring strokes only: one plunge per run, and every stroke in a
    // run after the first is joined to the one before it. The angles are those of the circumference coordinates,
    // so a start of the turn other than 0 shifts which strokes have material.
    for (const double start : {0.0, 3.7}) {
        p.circStart = start;
        const RoundingResult shifted = run(p);
        REQUIRE(shifted.ok());
        std::size_t strokes = 0;
        std::size_t runs = 0;
        for (const double rho : shifted.radii) {
            bool before = false;
            for (int j = 0; j < 13; ++j) {
                const double c = start + 2.0 * kPi * 10.0 * j / 13.0;
                const double theta = c / 10.0;
                const double phi = theta - 0.5 * kPi * std::round(theta / (0.5 * kPi));
                const bool present = 10.0 / std::numbers::sqrt2 / std::cos(phi) > rho + 1e-9;
                strokes += present ? 1 : 0;
                runs += present && !before ? 1 : 0;
                before = present;
            }
        }
        INFO("circStart " << start);
        CHECK(shifted.strokes == strokes);
        CHECK(count(shifted, MoveKind::Plunge) == runs);
    }
}

TEST_CASE("rounding: square blank rested inside the inscribed circle is cut all round", "[rounding]") {
    RoundingParams p = base();
    p.blank = RoundingBlank::Square;
    p.method = RoundingMethod::Raster;
    p.startRadius = 7.0;  // below the inscribed circle: round already
    p.targetRadius = 6.0;
    const RoundingResult r = run(p);
    REQUIRE(r.ok());
    CHECK(r.radii.size() == 1);
    CHECK(r.strokes == 11);
}

TEST_CASE("rounding: refusals", "[rounding]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const auto bad = [&](auto edit, ToolpathIssue issue = ToolpathIssue::InvalidParameters) {
        RoundingParams p = base();
        edit(p);
        const RoundingResult r = run(p);
        CHECK(!r.ok());
        CHECK(r.has(issue));
    };
    bad([&](RoundingParams& p) { p.blankRadius = nan; });
    bad([&](RoundingParams& p) { p.targetRadius = inf; });
    bad([](RoundingParams& p) { p.blankRadius = 0.0; });
    bad([](RoundingParams& p) { p.blankRadius = 1e6; });
    bad([](RoundingParams& p) { p.targetRadius = 0.0; });
    bad([](RoundingParams& p) { p.targetRadius = -1.0; });
    bad([](RoundingParams& p) { p.startRadius = -1.0; });
    bad([](RoundingParams& p) { p.allowance = -0.1; });
    bad([](RoundingParams& p) { p.alongEnd = p.alongStart; });
    bad([](RoundingParams& p) { p.stepover = -1.0; });
    bad([](RoundingParams& p) { p.passDepth = -1.0; });
    bad([](RoundingParams& p) { p.safeZ = -5.0; });
    bad([](RoundingParams& p) { p.safeZ = 0.0; });  // the surface itself is not clear of the blank
    bad([](RoundingParams& p) { p.surfaceZ = 1e6; });
    bad([&](RoundingParams& p) { p.circStart = nan; });
    bad([](RoundingParams& p) { p.passDepth = 1e-5; }, ToolpathIssue::WorkLimitExceeded);
    bad([](RoundingParams& p) {
        p.stepover = 1e-5;
        p.alongEnd = 1e5;
    }, ToolpathIssue::WorkLimitExceeded);
    bad([](RoundingParams& p) { p.stepover = 1e-4; p.alongEnd = 10.0; }, ToolpathIssue::WorkLimitExceeded);
    // Tools.
    tooling::Tool vbit = mill();
    vbit.type = tooling::ToolType::VBit;
    CHECK(roundingToolpath(vbit, std::nullopt, base()).has(ToolpathIssue::InvalidTool));
    tooling::Tool noFeed = mill();
    noFeed.defaultCuttingData.feedRate = 0.0;
    CHECK(roundingToolpath(noFeed, std::nullopt, base()).has(ToolpathIssue::InvalidTool));
    tooling::Tool noStep = mill(6.0, 2.0, 0.0);
    CHECK(roundingToolpath(noStep, std::nullopt, base()).has(ToolpathIssue::InvalidTool));
    RoundingParams own = base();
    own.stepover = 2.0;
    CHECK(roundingToolpath(noStep, std::nullopt, own).ok());  // a stepover of its own is enough
    tooling::Tool ball = mill();
    ball.type = tooling::ToolType::BallNose;
    CHECK(roundingToolpath(ball, std::nullopt, base()).ok());
    CHECK(roundingToolAllowed(tooling::ToolType::EndMill));
    CHECK(roundingToolAllowed(tooling::ToolType::BallNose));
    CHECK(!roundingToolAllowed(tooling::ToolType::VBit));
}

TEST_CASE("rounding: cancel and cutting data choice", "[rounding]") {
    std::atomic<bool> cancel{true};
    ToolpathRunControl control;
    control.cancel = &cancel;
    const RoundingResult r = roundingToolpath(mill(), std::nullopt, base(), control);
    CHECK(!r.ok());
    CHECK(r.has(ToolpathIssue::Cancelled));
    // Chosen cutting data wins over the tool's default.
    tooling::CuttingData data = mill().defaultCuttingData;
    data.passDepth = 4.0;
    data.stepover = 6.0;
    data.feedRate = 777.0;
    const RoundingResult chosen = roundingToolpath(mill(), data, base());
    REQUIRE(chosen.ok());
    CHECK(chosen.radii.size() == 1);
    CHECK(chosen.toolpath.tool.feedRate == 777.0);
    CHECK(chosen.strokes == 6);  // 30 / 6 = 5 steps
    CHECK(chosen.toolpath.tool.diameter == 6.0);
}
