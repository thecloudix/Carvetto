// T-0324: Laser Sketch Engraving (laser_sketch.hpp, ADR-0105): tracks and tone, hatch lines and cross hatch, fill
// orders, vector boundary, machine limits, overscan, the tone image of a relief model, refusals.

#include <algorithm>
#include <atomic>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/relief/relief.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/laser_sketch.hpp"
#include "carvetto/toolpath/move.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using Catch::Approx;
using geometry::Box2;
using geometry::Contour;
using geometry::LineSpan;
using geometry::Vec2;

namespace {

const double kNan = std::numeric_limits<double>::quiet_NaN();

tooling::Tool beam(double diameter = 0.2, tooling::ToolType type = tooling::ToolType::EndMill) {
    tooling::Tool t;
    t.id = "laser";
    t.name = "Laser";
    t.type = type;
    t.diameter = diameter;
    t.defaultCuttingData.toolNumber = 1;
    t.defaultCuttingData.feedRate = 900.0;
    t.defaultCuttingData.plungeRate = 900.0;
    t.defaultCuttingData.spindleSpeed = 0.0;
    t.defaultCuttingData.passDepth = 1.0;
    t.defaultCuttingData.maxBurnRate = 2000.0;
    return t;
}

/// A uniform gray image of `n` x `n` pixels over (0,0)-(10,10).
GrayImage uniform(int gray, int n = 10) {
    GrayImage g;
    g.width = n;
    g.height = n;
    g.pixels.assign(static_cast<std::size_t>(n) * static_cast<std::size_t>(n), static_cast<std::uint8_t>(gray));
    return g;
}

const Box2 kPlacement({0.0, 0.0}, {10.0, 10.0});

LaserSketchParams params() {
    LaserSketchParams p;
    p.lineThickness = 0.2;  // one track for the 0.2 mm beam
    p.stepover = 2.0;
    p.power = 80.0;
    return p;
}

LaserSketchResult run(const GrayImage& g, const LaserSketchParams& p, const std::vector<Contour>& cs = {},
                      const std::optional<Box2>& limits = std::nullopt, double dia = 0.2) {
    return laserSketchToolpath(g, kPlacement, beam(dia), std::nullopt, p, cs, limits);
}

/// A stroke as seen in the moves: the rapid that starts it and the feed moves that follow.
struct Seen {
    Vec2 start;
    Vec2 end;
    double power{0.0};
};

std::vector<Seen> strokesOf(const LaserSketchResult& r) {
    std::vector<Seen> out;
    for (const Move& m : r.toolpath.moves) {
        if (m.kind == MoveKind::Rapid && m.to.z == 0.0) {
            out.push_back({{m.to.x, m.to.y}, {m.to.x, m.to.y}, 0.0});
        } else if (m.kind == MoveKind::Feed && !out.empty()) {
            out.back().end = {m.to.x, m.to.y};
            if (m.jetPower > 0.0) {
                out.back().power = m.jetPower;
            }
        }
    }
    return out;
}

Contour square(double x0, double y0, double x1, double y1) {
    return Contour::rectangle(Box2({x0, y0}, {x1, y1}));
}

Contour line(Vec2 a, Vec2 b) {
    Contour c;
    c.append(LineSpan{a, b});
    return c;
}

}  // namespace

TEST_CASE("T-0324: the track count follows thickness and kerf", "[toolpath][laser][sketch]") {
    CHECK(laserSketchTracks(0.2, 0.2) == 1);
    CHECK(laserSketchTracks(0.1, 0.2) == 1);
    CHECK(laserSketchTracks(0.0, 0.2) == 1);
    CHECK(laserSketchTracks(0.4, 0.2) == 2);
    CHECK(laserSketchTracks(0.6, 0.2) == 3);
    CHECK(laserSketchTracks(0.5, 0.2) == 3);   // 2.5 rounds up
    CHECK(laserSketchTracks(0.44, 0.2) == 2);  // 2.2
    CHECK(laserSketchTracks(100.0, 0.1) == 64);
    CHECK(laserSketchTracks(1.0, 0.0) == 1);
    CHECK(laserSketchTracks(std::numeric_limits<double>::quiet_NaN(), 0.2) == 1);
}

TEST_CASE("T-0324: the lit tracks are the tone rounded to whole tracks", "[toolpath][laser][sketch]") {
    CHECK(laserSketchLit(0, 3) == 0);
    CHECK(laserSketchLit(42, 3) == 0);
    CHECK(laserSketchLit(43, 3) == 1);
    CHECK(laserSketchLit(127, 3) == 1);
    CHECK(laserSketchLit(128, 3) == 2);
    CHECK(laserSketchLit(212, 3) == 2);
    CHECK(laserSketchLit(213, 3) == 3);
    CHECK(laserSketchLit(255, 3) == 3);
    CHECK(laserSketchLit(127, 1) == 0);
    CHECK(laserSketchLit(128, 1) == 1);
    CHECK(laserSketchLit(1000, 3) == 3);
    CHECK(laserSketchLit(-4, 3) == 0);
}

TEST_CASE("T-0324: hatch lines on the multiples of the stepover, serpentine, full power", "[toolpath][laser][sketch]") {
    const LaserSketchResult r = run(uniform(0), params());
    REQUIRE(r.ok());
    CHECK(r.tracks == 1);
    CHECK(r.toolpath.tool.laser);
    CHECK(r.toolpath.tool.spindleSpeed == 0.0);
    CHECK(r.toolpath.tool.feedRate == 900.0);
    const std::vector<Seen> s = strokesOf(r);
    // y = 0, 2, 4, 6, 8 (the lines at k x stepover below the top edge).
    REQUIRE(s.size() == 5);
    CHECK(r.strokes == 5);
    for (std::size_t i = 0; i < s.size(); ++i) {
        CHECK(s[i].start.y == Approx(2.0 * static_cast<double>(i)).margin(1e-9));
        CHECK(s[i].start.x == Approx(i % 2 == 0 ? 0.0 : 10.0).margin(1e-9));
        CHECK(s[i].end.x == Approx(i % 2 == 0 ? 10.0 : 0.0).margin(1e-9));
        CHECK(s[i].power == 80.0);
    }
    CHECK(r.toolpath.moves.back().kind == MoveKind::Rapid);
    CHECK(r.toolpath.moves.back().to.z == 5.0);
}

TEST_CASE("T-0324: a white source has nothing to sketch", "[toolpath][laser][sketch]") {
    const LaserSketchResult r = run(uniform(255), params());
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::NoRegion));
}

TEST_CASE("T-0324: inverting sketches the light areas", "[toolpath][laser][sketch]") {
    LaserSketchParams p = params();
    p.invert = true;
    CHECK(run(uniform(255), p).ok());
    CHECK_FALSE(run(uniform(0), p).ok());
}

TEST_CASE("T-0324: a lighter tone lights fewer tracks, the centre first", "[toolpath][laser][sketch]") {
    LaserSketchParams p = params();
    p.lineThickness = 0.6;  // three tracks 0.2 apart
    p.stepover = 3.0;
    // Black: three tracks around each hatch line (y = 3 and 6 and 9; the line at y = 0 loses the track below the edge).
    const std::vector<Seen> black = strokesOf(run(uniform(0), p));
    std::vector<double> ys;
    for (const Seen& s : black) {
        ys.push_back(s.start.y);
    }
    std::sort(ys.begin(), ys.end());
    REQUIRE(ys.size() == 3 + 3 + 3 + 2);  // y = 0 keeps 0 and 0.2
    CHECK(ys[0] == Approx(0.0).margin(1e-9));
    CHECK(ys[1] == Approx(0.2).margin(1e-9));
    CHECK(ys[2] == Approx(2.8).margin(1e-9));
    CHECK(ys[3] == Approx(3.0).margin(1e-9));
    CHECK(ys[4] == Approx(3.2).margin(1e-9));
    // Darkness 127 (gray 128): one lit track, the centre one.
    const std::vector<Seen> mid = strokesOf(run(uniform(128), p));
    std::vector<double> mys;
    for (const Seen& s : mid) {
        mys.push_back(s.start.y);
    }
    std::sort(mys.begin(), mys.end());
    REQUIRE(mys.size() == 4);
    CHECK(mys[0] == Approx(0.0).margin(1e-9));
    CHECK(mys[1] == Approx(3.0).margin(1e-9));
    CHECK(mys[2] == Approx(6.0).margin(1e-9));
    CHECK(mys[3] == Approx(9.0).margin(1e-9));
    // Darkness 170 (gray 85): two lit tracks, the centre and the lower one.
    const std::vector<Seen> two = strokesOf(run(uniform(85), p));
    CHECK(two.size() == 2 + 2 + 2 + 1);  // y = 0: centre (0) lit, lower (-0.2) outside
}

TEST_CASE("T-0324: with two tracks the lower one lights first", "[toolpath][laser][sketch]") {
    LaserSketchParams p = params();
    p.lineThickness = 0.4;
    p.stepover = 5.0;
    // Darkness 128 (gray 127): one of two tracks lit.
    const std::vector<Seen> s = strokesOf(run(uniform(127), p));
    // Track offsets are -0.1 and +0.1: the lower one burns; for the hatch line y = 0 it lies at -0.1, outside the source.
    REQUIRE(s.size() == 1);
    CHECK(s[0].start.y == Approx(4.9).margin(1e-9));
}

TEST_CASE("T-0324: the tracks of one hatch line run as a serpentine too", "[toolpath][laser][sketch]") {
    LaserSketchParams p = params();
    p.lineThickness = 0.4;
    p.stepover = 5.0;
    const std::vector<Seen> s = strokesOf(run(uniform(0), p));
    // y = 0 keeps only its upper track (0.1); y = 5 has 4.9 (forward) then 5.1 (back).
    REQUIRE(s.size() == 3);
    CHECK(s[0].start.y == Approx(0.1).margin(1e-9));
    CHECK(s[0].start.x == Approx(0.0).margin(1e-9));
    CHECK(s[1].start.y == Approx(4.9).margin(1e-9));
    CHECK(s[1].start.x == Approx(10.0).margin(1e-9));
    CHECK(s[2].start.y == Approx(5.1).margin(1e-9));
    CHECK(s[2].start.x == Approx(0.0).margin(1e-9));
}

TEST_CASE("T-0324: the tone along a line decides where it burns", "[toolpath][laser][sketch]") {
    // Left half black, right half white (rows all the same).
    GrayImage g = uniform(255);
    for (int y = 0; y < 10; ++y) {
        for (int x = 0; x < 4; ++x) {
            g.pixels[static_cast<std::size_t>(y * 10 + x)] = 0;
        }
    }
    const std::vector<Seen> s = strokesOf(run(g, params()));
    REQUIRE(s.size() == 5);
    for (std::size_t i = 0; i < s.size(); ++i) {
        const double lo = std::min(s[i].start.x, s[i].end.x);
        const double hi = std::max(s[i].start.x, s[i].end.x);
        CHECK(lo == Approx(0.0).margin(1e-9));
        CHECK(hi == Approx(4.0).margin(1e-9));
    }
}

TEST_CASE("T-0324: cross hatch adds lines at right angles on the same grid", "[toolpath][laser][sketch]") {
    LaserSketchParams p = params();
    p.crossHatch = true;
    const std::vector<Seen> s = strokesOf(run(uniform(0), p));
    REQUIRE(s.size() == 10);
    // The second set runs along y; its lines are at x = 0, 2, ... 8 (angle +90: the first line has the lowest v
    // = -x, so it sits at the largest x).
    // The first cross line is the one at the largest x, run from the bottom up.
    CHECK(s[5].start.x == Approx(10.0).margin(1e-9));
    CHECK(s[5].start.y == Approx(0.0).margin(1e-9));
    CHECK(s[5].end.y == Approx(10.0).margin(1e-9));
    CHECK(s[6].start.y == Approx(10.0).margin(1e-9));
    std::vector<double> xs;
    for (std::size_t i = 5; i < 10; ++i) {
        CHECK(s[i].start.x == Approx(s[i].end.x).margin(1e-9));
        xs.push_back(s[i].start.x);
    }
    std::sort(xs.begin(), xs.end());
    for (std::size_t i = 0; i < xs.size(); ++i) {
        // v = -x on multiples of 2 below... region x in [0, 10]: v in [-10, 0]: lines x = 10 - ... even values.
        CHECK(std::fmod(xs[i], 2.0) == Approx(0.0).margin(1e-9));
    }
}

TEST_CASE("T-0324: the hatch angle turns the lines", "[toolpath][laser][sketch]") {
    LaserSketchParams p = params();
    p.hatchAngle = 0.7853981633974483;
    const std::vector<Seen> s = strokesOf(run(uniform(0), p));
    REQUIRE(s.size() >= 5);
    for (const Seen& line : s) {
        const double dx = line.end.x - line.start.x;
        const double dy = line.end.y - line.start.y;
        CHECK(std::fabs(dx) == Approx(std::fabs(dy)).margin(1e-6));
    }
}

TEST_CASE("T-0324: overscan runs past each stroke with the beam off", "[toolpath][laser][sketch]") {
    LaserSketchParams p = params();
    p.stepover = 5.0;
    p.overscan = 1.5;
    const LaserSketchResult r = run(uniform(0), p);
    REQUIRE(r.ok());
    const auto& m = r.toolpath.moves;
    // y = 0: rapid to -1.5, feed to 0 (off), feed to 10 (on), feed to 11.5 (off); y = 5 comes back.
    REQUIRE(m.size() == 4 + 4 + 1);
    CHECK(m[0].to.x == Approx(-1.5));
    CHECK(m[1].to.x == Approx(0.0));
    CHECK(m[1].jetPower == 0.0);
    CHECK(m[2].to.x == Approx(10.0));
    CHECK(m[2].jetPower == 80.0);
    CHECK(m[3].to.x == Approx(11.5));
    CHECK(m[3].jetPower == 0.0);
    CHECK(m[4].to.x == Approx(11.5));
    CHECK(m[5].to.x == Approx(10.0));
    CHECK(m[7].to.x == Approx(-1.5));
}

TEST_CASE("T-0324: the vectors bound the sketch, inset by the beam radius", "[toolpath][laser][sketch]") {
    LaserSketchParams p = params();
    p.boundary = SketchBoundary::Vectors;
    p.stepover = 3.0;
    const LaserSketchResult r = run(uniform(0), p, {square(1, 1, 9, 9)});
    REQUIRE(r.ok());
    const std::vector<Seen> s = strokesOf(r);
    REQUIRE(s.size() == 2);  // y = 3, 6 (9 is outside the inset region)
    CHECK(s[0].start.y == Approx(3.0).margin(1e-9));
    CHECK(s[0].start.x == Approx(1.1).margin(1e-6));
    CHECK(s[0].end.x == Approx(8.9).margin(1e-6));
}

TEST_CASE("T-0324: a narrow vector still gets its hatch segments", "[toolpath][laser][sketch]") {
    LaserSketchParams p = params();
    p.boundary = SketchBoundary::Vectors;
    p.stepover = 2.0;
    // 0.6 mm wide, 0.4 mm after the inset: the segments of the horizontal lines are 0.4 mm long.
    const std::vector<Seen> t = strokesOf(run(uniform(0), p, {Contour::rectangle(Box2({3.0, 1.0}, {3.6, 9.0}))}));
    REQUIRE(t.size() == 4);  // y = 2, 4, 6, 8
    CHECK(std::fabs(t[0].end.x - t[0].start.x) == Approx(0.4).margin(1e-6));
}

TEST_CASE("T-0324: open vectors are ignored with a diagnostic, an empty boundary is refused", "[toolpath][laser][sketch]") {
    LaserSketchParams p = params();
    p.boundary = SketchBoundary::Vectors;
    LaserSketchResult r = run(uniform(0), p, {line({0, 0}, {5, 5})});
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::OpenContourIgnored));
    r = run(uniform(0), p, {});
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::NoRegion));
    r = run(uniform(0), p, {square(1, 1, 9, 9), line({0, 0}, {5, 5})});
    CHECK(r.ok());
    CHECK(r.has(ToolpathIssue::OpenContourIgnored));
    // A region too small for the beam.
    r = run(uniform(0), p, {square(1, 1, 1.1, 1.1)});
    CHECK_FALSE(r.ok());
}

TEST_CASE("T-0324: fill together crosses all regions line by line, regionally one region at a time",
          "[toolpath][laser][sketch]") {
    LaserSketchParams p = params();
    p.boundary = SketchBoundary::Vectors;
    const std::vector<Contour> two{square(0.5, 1, 4.5, 5), square(5.5, 1, 9.5, 5)};
    p.fill = SketchFill::Together;
    std::vector<Seen> s = strokesOf(run(uniform(0), p, two));
    REQUIRE(s.size() == 4);  // two lines (y = 2, 4) across two squares
    CHECK(s[0].start.y == Approx(2.0).margin(1e-9));
    CHECK(s[1].start.y == Approx(2.0).margin(1e-9));
    CHECK(s[2].start.y == Approx(4.0).margin(1e-9));
    CHECK(s[3].start.y == Approx(4.0).margin(1e-9));
    CHECK(std::min(s[0].start.x, s[0].end.x) < 5.0);
    CHECK(std::min(s[1].start.x, s[1].end.x) > 5.0);
    p.fill = SketchFill::Regionally;
    p.order.leftToRight = true;
    s = strokesOf(run(uniform(0), p, two));
    REQUIRE(s.size() == 4);
    CHECK(std::max(s[0].start.x, s[0].end.x) < 5.0);
    CHECK(std::max(s[1].start.x, s[1].end.x) < 5.0);
    CHECK(std::min(s[2].start.x, s[2].end.x) > 5.0);
    CHECK(std::min(s[3].start.x, s[3].end.x) > 5.0);
    CHECK(s[0].start.y == Approx(2.0).margin(1e-9));
    CHECK(s[1].start.y == Approx(4.0).margin(1e-9));
    CHECK(s[2].start.y == Approx(2.0).margin(1e-9));
}

TEST_CASE("T-0324: a vector inside another is a hole only between vectors", "[toolpath][laser][sketch]") {
    LaserSketchParams p = params();
    p.boundary = SketchBoundary::Vectors;
    p.stepover = 4.0;
    const std::vector<Contour> nested{square(0.5, 0.5, 9.5, 9.5), square(3, 3, 7, 7)};
    p.fill = SketchFill::Regionally;
    std::vector<Seen> s = strokesOf(run(uniform(0), p, nested));
    // Lines at y = 4 and 8 across the whole outer vector.
    REQUIRE(s.size() == 2);
    CHECK(std::fabs(s[0].end.x - s[0].start.x) == Approx(8.8).margin(1e-6));
    p.fill = SketchFill::BetweenVectors;
    s = strokesOf(run(uniform(0), p, nested));
    // The line at y = 4 is split by the hole (3.. 7, inset by the beam radius).
    REQUIRE(s.size() == 3);
    CHECK(std::fabs(s[0].end.x - s[0].start.x) < 3.0);
    CHECK(std::fabs(s[1].end.x - s[1].start.x) < 3.0);
}

TEST_CASE("T-0324: the machine limit boundary clips the sketch", "[toolpath][laser][sketch]") {
    LaserSketchParams p = params();
    p.machineLimitBoundary = true;
    const Box2 limits({1.0, 3.0}, {6.0, 7.0});
    const LaserSketchResult r = run(uniform(0), p, {}, limits);
    REQUIRE(r.ok());
    const std::vector<Seen> s = strokesOf(r);
    REQUIRE(s.size() == 2);  // y = 4, 6
    CHECK(s[0].start.y == Approx(4.0).margin(1e-9));
    CHECK(s[0].start.x == Approx(1.0).margin(1e-9));
    CHECK(s[0].end.x == Approx(6.0).margin(1e-9));
    // Without the flag, or without limits, nothing is clipped.
    p.machineLimitBoundary = false;
    CHECK(strokesOf(run(uniform(0), p, {}, limits)).size() == 5);
    p.machineLimitBoundary = true;
    CHECK(strokesOf(run(uniform(0), p, {}, std::nullopt)).size() == 5);
    // Limits that miss the source leave nothing.
    const LaserSketchResult off = run(uniform(0), p, {}, Box2({20, 20}, {30, 30}));
    CHECK_FALSE(off.ok());
    CHECK(off.has(ToolpathIssue::NoRegion));
    // The limits also clip a vector boundary.
    p.boundary = SketchBoundary::Vectors;
    p.stepover = 3.0;
    const std::vector<Seen> v = strokesOf(run(uniform(0), p, {square(0.5, 0.5, 9.5, 9.5)}, Box2({0, 0}, {5, 10})));
    REQUIRE(!v.empty());
    for (const Seen& line : v) {
        CHECK(std::max(line.start.x, line.end.x) <= 5.0 + 1e-9);
    }
}

TEST_CASE("T-0326: with the machine limit boundary the beam-off overscan stays inside the limits",
          "[toolpath][laser][sketch]") {
    LaserSketchParams p = params();
    p.machineLimitBoundary = true;
    p.overscan = 2.0;
    const auto inside = [](const LaserSketchResult& r, const Box2& lim) {
        for (const Move& m : r.toolpath.moves) {
            if (m.to.x < lim.min.x - 1e-9 || m.to.x > lim.max.x + 1e-9 || m.to.y < lim.min.y - 1e-9 ||
                m.to.y > lim.max.y + 1e-9) {
                return false;
            }
        }
        return true;
    };
    // Limits that cut the lines: the burn ends at the limit, and so does the overscan (nothing to run out into).
    const Box2 tight({1.0, 3.0}, {6.0, 7.0});
    LaserSketchResult r = run(uniform(0), p, {}, tight);
    REQUIRE(r.ok());
    CHECK(inside(r, tight));
    const std::vector<Seen> s = strokesOf(r);
    REQUIRE(s.size() == 2);
    CHECK(s[0].start.x == Approx(1.0).margin(1e-9));
    CHECK(s[0].end.x == Approx(6.0).margin(1e-9));
    // The first stroke has no approach feed: the rapid goes to the burn start, and no run-out follows the burn.
    REQUIRE(r.toolpath.moves.size() >= 3);
    CHECK(r.toolpath.moves[0].kind == MoveKind::Rapid);
    CHECK(r.toolpath.moves[0].to.x == Approx(1.0).margin(1e-9));
    CHECK(r.toolpath.moves[1].kind == MoveKind::Feed);
    CHECK(r.toolpath.moves[1].jetPower > 0.0);
    // Limits a little wider than the source: the overscan is shortened to the limit, not dropped.
    const Box2 wide({-0.5, -1.0}, {10.5, 11.0});
    r = run(uniform(0), p, {}, wide);
    REQUIRE(r.ok());
    CHECK(inside(r, wide));
    CHECK(r.toolpath.moves[0].to.x == Approx(-0.5).margin(1e-9));
    CHECK(r.toolpath.moves[1].to.x == Approx(0.0).margin(1e-9));
    CHECK(r.toolpath.moves[1].jetPower == Approx(0.0));  // the approach is beam off
    const auto extent = [](const LaserSketchResult& res, bool useX, bool wantMax) {
        double v = wantMax ? -1e9 : 1e9;
        for (const Move& m : res.toolpath.moves) {
            if (m.kind == MoveKind::Feed) {  // the beam-off run-outs are feeds too
                const double c = useX ? m.to.x : m.to.y;
                v = wantMax ? std::max(v, c) : std::min(v, c);
            }
        }
        return v;
    };
    CHECK(extent(r, true, true) == Approx(10.5).margin(1e-9));   // the run-out ends at the limit, 0.5 past the burn
    CHECK(extent(r, true, false) == Approx(-0.5).margin(1e-9));
    // Vertical lines: the same along Y.
    LaserSketchParams vertical = p;
    vertical.hatchAngle = 1.5707963267948966;
    r = run(uniform(0), vertical, {}, wide);
    REQUIRE(r.ok());
    CHECK(inside(r, wide));
    CHECK(extent(r, false, false) == Approx(-1.0).margin(1e-6));
    CHECK(extent(r, false, true) == Approx(11.0).margin(1e-6));
    // Limits far wider than the overscan: the full overscan is kept.
    r = run(uniform(0), p, {}, Box2({-20.0, -20.0}, {30.0, 30.0}));
    CHECK(r.toolpath.moves[0].to.x == Approx(-2.0).margin(1e-9));
    // Without the boundary the overscan is as asked, limits or no limits.
    p.machineLimitBoundary = false;
    r = run(uniform(0), p, {}, tight);
    CHECK(r.toolpath.moves[0].to.x == Approx(-2.0).margin(1e-9));
    // Rotated lines (cross hatch at 45 degrees): the same rule along the slanted direction.
    p.machineLimitBoundary = true;
    p.hatchAngle = 0.7853981633974483;
    r = run(uniform(0), p, {}, wide);
    REQUIRE(r.ok());
    CHECK(inside(r, wide));
}

TEST_CASE("T-0324: bad parameters, tools and sources are refused", "[toolpath][laser][sketch]") {
    const auto refused = [&](LaserSketchParams p) {
        const LaserSketchResult r = run(uniform(0), p);
        return !r.ok() && r.has(ToolpathIssue::InvalidParameters);
    };
    CHECK(refused([] { auto p = params(); p.power = 0.0; return p; }()));
    CHECK(refused([] { auto p = params(); p.power = 101.0; return p; }()));
    CHECK(refused([] { auto p = params(); p.lineThickness = 0.0; return p; }()));
    CHECK(refused([] { auto p = params(); p.stepover = 0.0; return p; }()));
    CHECK(refused([] { auto p = params(); p.stepover = kNan; return p; }()));
    CHECK(refused([] { auto p = params(); p.overscan = -1.0; return p; }()));
    CHECK(refused([] { auto p = params(); p.hatchAngle = kNan; return p; }()));
    CHECK(refused([] { auto p = params(); p.moveSpeed = -2.0; return p; }()));
    CHECK(refused([] { auto p = params(); p.safeZ = -1.0; return p; }()));
    CHECK(refused([] { auto p = params(); p.tolerance.chordal = 5.0; return p; }()));
    LaserSketchResult r = laserSketchToolpath(uniform(0), kPlacement, beam(0.2, tooling::ToolType::VBit), std::nullopt,
                                              params());
    CHECK(r.has(ToolpathIssue::InvalidTool));
    r = laserSketchToolpath(GrayImage{}, kPlacement, beam(), std::nullopt, params());
    CHECK(r.has(ToolpathIssue::InvalidParameters));
    r = laserSketchToolpath(uniform(0), Box2({0, 0}, {10, 0}), beam(), std::nullopt, params());
    CHECK(r.has(ToolpathIssue::InvalidParameters));
    LaserSketchParams lim = params();
    lim.machineLimitBoundary = true;
    Box2 bad;
    bad.min = {0.0, 0.0};
    bad.max = {kNan, 5.0};
    r = laserSketchToolpath(uniform(0), kPlacement, beam(), std::nullopt, lim, {}, bad);
    CHECK(r.has(ToolpathIssue::InvalidParameters));
}

TEST_CASE("T-0324: work and move limits are counted, a cancel flag stops the run", "[toolpath][laser][sketch]") {
    ToolpathRunControl c;
    c.moveLimit = 6;
    LaserSketchResult r = laserSketchToolpath(uniform(0), kPlacement, beam(), std::nullopt, params(), {}, std::nullopt, c);
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::WorkLimitExceeded));
    CHECK(r.toolpath.moves.empty());
    c = {};
    c.workLimit = 50;  // five lines of 10 mm over 1 mm pixels cross 5 x 10 x 2 edges
    r = laserSketchToolpath(uniform(0), kPlacement, beam(), std::nullopt, params(), {}, std::nullopt, c);
    CHECK(r.has(ToolpathIssue::WorkLimitExceeded));
    // Every track counts as work: three tracks need three times the crossings of one.
    c = {};
    c.workLimit = 150;
    CHECK(laserSketchToolpath(uniform(0), kPlacement, beam(), std::nullopt, params(), {}, std::nullopt, c).ok());
    LaserSketchParams thick = params();
    thick.lineThickness = 0.6;
    r = laserSketchToolpath(uniform(0), kPlacement, beam(), std::nullopt, thick, {}, std::nullopt, c);
    CHECK(r.has(ToolpathIssue::WorkLimitExceeded));
    LaserSketchParams dense = params();
    dense.stepover = 0.001;
    dense.lineThickness = 100.0;
    r = laserSketchToolpath(uniform(0), Box2({0, 0}, {100000, 100000}), beam(), std::nullopt, dense);
    CHECK(r.has(ToolpathIssue::WorkLimitExceeded));
    std::atomic<bool> stop{true};
    c = {};
    c.cancel = &stop;
    r = laserSketchToolpath(uniform(0), kPlacement, beam(), std::nullopt, params(), {}, std::nullopt, c);
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::Cancelled));
}

TEST_CASE("T-0324: the same input gives the same toolpath", "[toolpath][laser][sketch]") {
    GrayImage g = uniform(0);
    for (std::size_t i = 0; i < g.pixels.size(); ++i) {
        g.pixels[i] = static_cast<std::uint8_t>((i * 37) % 256);
    }
    LaserSketchParams p = params();
    p.lineThickness = 0.6;
    p.crossHatch = true;
    p.hatchAngle = 0.4;
    p.overscan = 0.3;
    const LaserSketchResult a = run(g, p);
    const LaserSketchResult b = run(g, p);
    REQUIRE(a.ok());
    CHECK(a.toolpath.moves == b.toolpath.moves);
}

// ---- 3D model source -------------------------------------------------------------------------------------------

namespace {

relief::HeightField field(std::size_t w, std::size_t h, double cell, relief::Point origin,
                          const std::vector<float>& data) {
    relief::HeightField f;
    REQUIRE(relief::HeightField::fromData({w, h, cell, origin}, data, f) == relief::Status::Ok);
    return f;
}

std::vector<float> ramp(std::size_t w, std::size_t h, float slope) {
    std::vector<float> d(w * h);
    for (std::size_t j = 0; j < h; ++j) {
        for (std::size_t i = 0; i < w; ++i) {
            d[j * w + i] = slope * static_cast<float>(i);
        }
    }
    return d;
}

}  // namespace

TEST_CASE("T-0324: the depth tone of a model is black at the lowest point and white at the highest",
          "[toolpath][laser][sketch][model]") {
    const relief::HeightField f = field(4, 4, 1.0, {0.0, 0.0}, ramp(4, 4, 1.0F));
    Box2 placement;
    const GrayImage g = toneFromHeightField(f, SketchModelTone::Depth, placement);
    REQUIRE(g.valid());
    CHECK(g.width == 4);
    CHECK(g.height == 4);
    CHECK(placement.min.x == Approx(-0.5));
    CHECK(placement.min.y == Approx(-0.5));
    CHECK(placement.max.x == Approx(3.5));
    CHECK(placement.max.y == Approx(3.5));
    for (int row = 0; row < 4; ++row) {
        CHECK(g.pixels[static_cast<std::size_t>(row * 4 + 0)] == 0);
        CHECK(g.pixels[static_cast<std::size_t>(row * 4 + 1)] == 85);
        CHECK(g.pixels[static_cast<std::size_t>(row * 4 + 2)] == 170);
        CHECK(g.pixels[static_cast<std::size_t>(row * 4 + 3)] == 255);
    }
}

TEST_CASE("T-0324: image rows run from the top of the model", "[toolpath][laser][sketch][model]") {
    // Row j = 0 (lowest Y) is high, the others low: it must be the last image row.
    std::vector<float> d(9, 0.0F);
    d[0] = d[1] = d[2] = 5.0F;
    const relief::HeightField f = field(3, 3, 2.0, {10.0, 20.0}, d);
    Box2 placement;
    const GrayImage g = toneFromHeightField(f, SketchModelTone::Depth, placement);
    REQUIRE(g.valid());
    CHECK(placement.min.x == Approx(9.0));
    CHECK(placement.max.x == Approx(15.0));
    CHECK(placement.min.y == Approx(19.0));
    CHECK(placement.max.y == Approx(25.0));
    for (std::size_t i = 0; i < 6; ++i) {
        CHECK(g.pixels[i] == 0);
    }
    for (std::size_t i = 6; i < 9; ++i) {
        CHECK(g.pixels[i] == 255);
    }
}

TEST_CASE("T-0324: transparent cells are white and an empty model gives no image", "[toolpath][laser][sketch][model]") {
    std::vector<float> d{0.0F, relief::kTransparent, 1.0F, 1.0F};
    const relief::HeightField f = field(2, 2, 1.0, {0.0, 0.0}, d);
    Box2 placement;
    const GrayImage g = toneFromHeightField(f, SketchModelTone::Depth, placement);
    REQUIRE(g.valid());
    // Row 0 is j = 1: {1, 1} are the top; row 1 is j = 0: {0, transparent}.
    CHECK(g.pixels[0] == 255);
    CHECK(g.pixels[1] == 255);
    CHECK(g.pixels[2] == 0);
    CHECK(g.pixels[3] == 255);
    const relief::HeightField none = field(2, 2, 1.0, {0.0, 0.0},
                                           {relief::kTransparent, relief::kTransparent, relief::kTransparent,
                                            relief::kTransparent});
    CHECK_FALSE(toneFromHeightField(none, SketchModelTone::Depth, placement).valid());
    CHECK_FALSE(toneFromHeightField(relief::HeightField{}, SketchModelTone::Shaded, placement).valid());
    // A flat model is white.
    const relief::HeightField flat = field(2, 2, 1.0, {0.0, 0.0}, {3.0F, 3.0F, 3.0F, 3.0F});
    const GrayImage fg = toneFromHeightField(flat, SketchModelTone::Depth, placement);
    CHECK(fg.pixels == std::vector<std::uint8_t>{255, 255, 255, 255});
}

TEST_CASE("T-0324: shaded tone: flat and light-facing surfaces are white, surfaces facing away are dark",
          "[toolpath][laser][sketch][model]") {
    Box2 placement;
    // The light comes from the upper left (-x, +y), 45 degrees up.
    const relief::HeightField flat = field(5, 5, 1.0, {0.0, 0.0}, std::vector<float>(25, 2.0F));
    for (const std::uint8_t v : toneFromHeightField(flat, SketchModelTone::Shaded, placement).pixels) {
        CHECK(v == 255);
    }
    const relief::HeightField up = field(5, 5, 1.0, {0.0, 0.0}, ramp(5, 5, 1.0F));  // rises to +x: faces the light
    for (const std::uint8_t v : toneFromHeightField(up, SketchModelTone::Shaded, placement).pixels) {
        CHECK(v == 255);
    }
    const relief::HeightField down = field(5, 5, 1.0, {0.0, 0.0}, ramp(5, 5, -1.0F));  // falls to +x: faces away
    const GrayImage dg = toneFromHeightField(down, SketchModelTone::Shaded, placement);
    for (const std::uint8_t v : dg.pixels) {
        CHECK(v == 53);  // n = (1, 0, 1) / sqrt 2: (-0.5 + 0.7071) / 1.4142 / 0.7071 x 255
    }
    // A slope that rises toward -y faces the light (which has a +y part): white. One that rises toward +y faces away.
    std::vector<float> falls(25);
    for (std::size_t j = 0; j < 5; ++j) {
        for (std::size_t i = 0; i < 5; ++i) {
            falls[j * 5 + i] = static_cast<float>(4 - j);
        }
    }
    for (const std::uint8_t v : toneFromHeightField(field(5, 5, 1.0, {0.0, 0.0}, falls), SketchModelTone::Shaded, placement).pixels) {
        CHECK(v == 255);
    }
    std::vector<float> rises(25);
    for (std::size_t j = 0; j < 5; ++j) {
        for (std::size_t i = 0; i < 5; ++i) {
            rises[j * 5 + i] = static_cast<float>(j);
        }
    }
    const GrayImage ng = toneFromHeightField(field(5, 5, 1.0, {0.0, 0.0}, rises), SketchModelTone::Shaded, placement);
    for (const std::uint8_t v : ng.pixels) {
        CHECK(v == 53);  // rises to +y: normal (0, -1, 1) / sqrt 2 faces away from the light
    }
}

TEST_CASE("T-0324: a model wider than the image limit is shrunk by an integer factor", "[toolpath][laser][sketch][model]") {
    std::vector<float> d(20000U * 2U, 0.0F);
    for (std::size_t j = 0; j < 2; ++j) {
        for (std::size_t i = 10000; i < 20000; ++i) {
            d[j * 20000U + i] = 1.0F;
        }
    }
    const relief::HeightField f = field(20000, 2, 0.5, {0.0, 0.0}, d);
    Box2 placement;
    const GrayImage g = toneFromHeightField(f, SketchModelTone::Depth, placement);
    REQUIRE(g.valid());
    CHECK(g.width == 10000);
    CHECK(g.height == 1);
    CHECK(g.pixels[0] == 0);
    CHECK(g.pixels[9999] == 255);
    CHECK(placement.max.x == Approx(0.5 * 19999.5 + 0.0).margin(1e-6));
    CHECK(placement.max.y == Approx(0.75));
}

TEST_CASE("T-0324: a model sketch burns where the model is low", "[toolpath][laser][sketch][model]") {
    // Ten cells wide, rising to +x: darkness >= half for the first five columns.
    const relief::HeightField f = field(10, 10, 1.0, {0.0, 0.0}, ramp(10, 10, 1.0F));
    Box2 placement;
    const GrayImage g = toneFromHeightField(f, SketchModelTone::Depth, placement);
    LaserSketchParams p = params();
    p.source = SketchSource::Model;
    const LaserSketchResult r = laserSketchToolpath(g, placement, beam(), std::nullopt, p);
    REQUIRE(r.ok());
    const std::vector<Seen> s = strokesOf(r);
    REQUIRE(!s.empty());
    for (const Seen& line : s) {
        const double lo = std::min(line.start.x, line.end.x);
        const double hi = std::max(line.start.x, line.end.x);
        CHECK(lo == Approx(-0.5).margin(1e-9));
        CHECK(hi == Approx(4.5).margin(1e-9));
    }
}
