// T-0238: straight inlay (inlay.hpp, ADR-0040): male and female pieces of the same vectors mate
// within the allowance, corner radiusing, toolpath geometry of every piece and style, hostile input.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/cancel.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/inlay.hpp"
#include "carvetto/toolpath/move.hpp"
#include "../geometry/cpu_time.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using Catch::Approx;
using geometry::Box2;
using geometry::Contour;
using geometry::LineSpan;
using geometry::Vec2;

namespace {

tooling::Tool endMill(double diameter = 6.0) {
    tooling::Tool t;
    t.id = "em";
    t.name = "End mill";
    t.type = tooling::ToolType::EndMill;
    t.diameter = diameter;
    t.defaultCuttingData.toolNumber = 1;
    t.defaultCuttingData.feedRate = 1000.0;
    t.defaultCuttingData.plungeRate = 300.0;
    t.defaultCuttingData.spindleSpeed = 16000.0;
    t.defaultCuttingData.passDepth = 1.0;
    t.defaultCuttingData.stepover = diameter * 0.4;
    return t;
}

Contour square(double size = 20.0) {
    return Contour::rectangle(Box2{Vec2{0, 0}, Vec2{size, size}});
}

Contour polygon(const std::vector<Vec2>& pts) {
    Contour c;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        c.append(LineSpan{pts[i], pts[(i + 1) % pts.size()]});
    }
    c.closed = true;
    return c;
}

Contour lShape() {
    return polygon({{0, 0}, {30, 0}, {30, 12}, {12, 12}, {12, 30}, {0, 30}});
}

double areaOf(const std::vector<Contour>& cs) {
    double a = 0.0;
    for (const Contour& c : cs) {
        a += geometry::signedArea(c);
    }
    return std::fabs(a);
}

double distanceToAll(const std::vector<Contour>& cs, Vec2 p) {
    double d = std::numeric_limits<double>::infinity();
    for (const Contour& c : cs) {
        d = std::min(d, geometry::distanceTo(c, p));
    }
    return d;
}

bool inside(const std::vector<Contour>& cs, Vec2 p) {
    return geometry::locatePoint(cs, p, geometry::FillRule::EvenOdd, geometry::Tolerance::defaults()) ==
           geometry::PointLocation::Inside;
}

/// The feed moves (cutting) of a toolpath.
std::vector<const Move*> cuts(const Toolpath& tp) {
    std::vector<const Move*> out;
    for (const Move& m : tp.moves) {
        if (m.kind == MoveKind::Feed || m.kind == MoveKind::ArcCW || m.kind == MoveKind::ArcCCW) {
            out.push_back(&m);
        }
    }
    return out;
}

double minZ(const Toolpath& tp) {
    double z = 1e300;
    for (const Move& m : tp.moves) {
        z = std::min(z, m.to.z);
    }
    return z;
}

InlayParams params(InlayPiece piece) {
    InlayParams p;
    p.piece = piece;
    p.cutDepth = 3.0;
    p.allowance = 0.2;
    return p;
}

}  // namespace

TEST_CASE("inlay shapes: corners are rounded to the tool radius, convex and concave", "[inlay]") {
    const InlayShapes s = inlayShapes({lShape()}, 6.0, 0.0, geometry::Tolerance::defaults());
    REQUIRE_FALSE(s.female.empty());
    // Convex corner (0,0): rounded by r = 3.
    CHECK_FALSE(inside(s.female, Vec2{0.5, 0.5}));
    CHECK(inside(s.female, Vec2{1.0, 1.0}));
    // Reflex corner (12,12): filled to radius 3 (the outside profile cannot reach into it).
    CHECK(inside(s.female, Vec2{12.5, 12.5}));
    CHECK_FALSE(inside(s.female, Vec2{15.5, 15.5}));
    // Far from corners the shape is the vector.
    CHECK(inside(s.female, Vec2{15, 6}));
    CHECK_FALSE(inside(s.female, Vec2{-0.2, 6}));
}

TEST_CASE("inlay shapes: male fits female with the allowance all round", "[inlay]") {
    const geometry::Tolerance tol = geometry::Tolerance::defaults();
    for (const double allowance : {0.0, 0.1, 0.25, 1.0}) {
        const InlayShapes s = inlayShapes({lShape()}, 6.0, allowance, tol);
        REQUIRE_FALSE(s.female.empty());
        REQUIRE_FALSE(s.male.empty());
        // Male inside female (area difference nothing).
        const geometry::BooleanResult out = geometry::booleanOp(geometry::BooleanOp::Difference, s.male, s.female, tol);
        CHECK(areaOf(out.closed) < 1e-3);
        // Every male vertex is the allowance from the female outline (within the polygon tolerance).
        for (const Contour& c : s.male) {
            const geometry::Polyline pl = c.polygonise(tol);
            for (const Vec2& p : pl.points) {
                CHECK(distanceToAll(s.female, p) == Approx(allowance).margin(3.0 * tol.chordal));
            }
        }
        // Area: female less male is the gap ring (perimeter x allowance, less corner effects).
        if (allowance > 0.0) {
            CHECK(areaOf(s.female) > areaOf(s.male));
        } else {
            CHECK(areaOf(s.female) == Approx(areaOf(s.male)));
        }
    }
}

TEST_CASE("inlay shapes: an island vector keeps its hole in both pieces", "[inlay]") {
    const Contour hole = Contour::rectangle(Box2{Vec2{8, 8}, Vec2{22, 22}});
    const InlayShapes s = inlayShapes({square(30), hole}, 3.0, 0.3, geometry::Tolerance::defaults());
    REQUIRE_FALSE(s.male.empty());
    CHECK_FALSE(inside(s.female, Vec2{15, 15}));
    CHECK(inside(s.female, Vec2{4, 15}));
    CHECK_FALSE(inside(s.male, Vec2{15, 15}));
    // The male's hole is bigger than the vector's hole by the allowance.
    CHECK_FALSE(inside(s.male, Vec2{7.9, 15}));
    CHECK(distanceToAll(s.male, Vec2{8.0, 15}) == Approx(0.3).margin(1e-2));
}

TEST_CASE("inlay male: tool runs outside the plug, to the depth, with the allowance", "[inlay]") {
    const tooling::Tool tool = endMill(6.0);
    const InlayParams p = params(InlayPiece::Male);
    const InlayResult r = inlayToolpath({lShape()}, tool, std::nullopt, p);
    REQUIRE(r.ok());
    CHECK(minZ(r.toolpath) == Approx(-3.0));
    const InlayShapes s = inlayShapes({lShape()}, 6.0, p.allowance, p.tolerance);
    std::set<long> levels;
    for (const Move* m : cuts(r.toolpath)) {
        const Vec2 xy{m->to.x, m->to.y};
        CHECK(distanceToAll(s.male, xy) == Approx(3.0).margin(2e-3));
        CHECK_FALSE(inside(s.male, xy));
        levels.insert(std::lround(m->to.z * 1000.0));
    }
    CHECK(levels.count(-1000) == 1);
    CHECK(levels.count(-3000) == 1);
    CHECK(r.toolpath.tool.diameter == Approx(6.0));
    // Deterministic.
    CHECK(inlayToolpath({lShape()}, tool, std::nullopt, p).toolpath == r.toolpath);
}

TEST_CASE("inlay male: start depth shifts the cut", "[inlay]") {
    InlayParams p = params(InlayPiece::Male);
    p.startDepth = 2.0;
    p.cutDepth = 2.0;
    const InlayResult r = inlayToolpath({square()}, endMill(), std::nullopt, p);
    REQUIRE(r.ok());
    CHECK(minZ(r.toolpath) == Approx(-4.0));
    for (const Move* m : cuts(r.toolpath)) {
        CHECK(m->to.z <= -3.0 + 1e-9);
    }
}

TEST_CASE("inlay female pocket: floor at depth, the cut stays within the cavity", "[inlay]") {
    const tooling::Tool tool = endMill(6.0);
    InlayParams p = params(InlayPiece::Female);
    const InlayResult r = inlayToolpath({lShape()}, tool, std::nullopt, p);
    REQUIRE(r.ok());
    CHECK(minZ(r.toolpath) == Approx(-3.0));
    const InlayShapes s = inlayShapes({lShape()}, 6.0, p.allowance, p.tolerance);
    double nearest = 1e300;
    for (const Move* m : cuts(r.toolpath)) {
        const Vec2 xy{m->to.x, m->to.y};
        CHECK(inside(s.female, xy));
        // Tool edge never past the cavity wall.
        CHECK(distanceToAll(s.female, xy) >= 3.0 - 2e-3);
        nearest = std::min(nearest, distanceToAll(s.female, xy));
    }
    // The boundary pass runs the tool against the wall (within the pocket's safety margin).
    CHECK(nearest == Approx(3.0).margin(2.0 * p.tolerance.chordal + 1e-3));
}

TEST_CASE("inlay female hole: inside profile on the cavity outline", "[inlay]") {
    const tooling::Tool tool = endMill(6.0);
    InlayParams p = params(InlayPiece::Female);
    p.femaleStyle = InlayFemaleStyle::Hole;
    p.cutDepth = 4.0;
    const InlayResult r = inlayToolpath({lShape()}, tool, std::nullopt, p);
    REQUIRE(r.ok());
    CHECK(minZ(r.toolpath) == Approx(-4.0));
    const InlayShapes s = inlayShapes({lShape()}, 6.0, p.allowance, p.tolerance);
    for (const Move* m : cuts(r.toolpath)) {
        const Vec2 xy{m->to.x, m->to.y};
        CHECK(inside(s.female, xy));
        CHECK(distanceToAll(s.female, xy) == Approx(3.0).margin(2e-3));
    }
}

TEST_CASE("inlay: the allowance changes the male only", "[inlay]") {
    InlayParams a = params(InlayPiece::Female);
    InlayParams b = a;
    a.allowance = 0.0;
    b.allowance = 0.5;
    CHECK(inlayToolpath({square()}, endMill(), std::nullopt, a).toolpath ==
          inlayToolpath({square()}, endMill(), std::nullopt, b).toolpath);
    InlayParams ma = params(InlayPiece::Male);
    InlayParams mb = ma;
    ma.allowance = 0.0;
    mb.allowance = 0.5;
    CHECK_FALSE(inlayToolpath({square()}, endMill(), std::nullopt, ma).toolpath ==
                inlayToolpath({square()}, endMill(), std::nullopt, mb).toolpath);
}

TEST_CASE("inlay male stepped: shoulder level further out, band below clears to the plug", "[inlay]") {
    const tooling::Tool tool = endMill(3.0);
    InlayParams p = params(InlayPiece::Male);
    p.maleStyle = InlayMaleStyle::Stepped;
    p.stepDepth = 1.0;
    p.stepWidth = 4.0;  // wider than the tool: a band of several passes
    p.cutDepth = 3.0;
    const InlayResult r = inlayToolpath({square(30)}, tool, std::nullopt, p);
    REQUIRE(r.ok());
    CHECK(minZ(r.toolpath) == Approx(-3.0));
    const InlayShapes s = inlayShapes({square(30)}, 3.0, p.allowance, p.tolerance);
    std::set<long> bandOffsets;  // distances of the tool centre from the plug, in um, at the lower levels
    for (const Move* m : cuts(r.toolpath)) {
        const double d = distanceToAll(s.male, Vec2{m->to.x, m->to.y});
        if (m->to.z >= -1.0 - 1e-9) {
            // Shoulder level: tool edge at the shoulder outline.
            CHECK(d == Approx(1.5 + 4.0).margin(2e-3));
        } else {
            CHECK(d >= 1.5 - 2e-3);
            CHECK(d <= 1.5 + 4.0 + 2e-3);
            bandOffsets.insert(std::lround((d - 1.5) * 100.0));
        }
    }
    CHECK(bandOffsets.count(0) == 1);  // a pass along the plug itself
    // Neighbouring band passes are at most a tool diameter apart: the band is cleared.
    long prev = -1;
    for (const long o : bandOffsets) {
        if (prev >= 0 && o > 0) {
            CHECK(static_cast<double>(o - prev) / 100.0 <= 3.0 + 1e-9);
        }
        prev = o;
    }
    CHECK(static_cast<double>(*bandOffsets.rbegin()) / 100.0 + 3.0 >= 4.0 - 0.02);
}

TEST_CASE("inlay male stepped: a narrow step is one pass below the shoulder", "[inlay]") {
    InlayParams p = params(InlayPiece::Male);
    p.maleStyle = InlayMaleStyle::Stepped;
    p.stepDepth = 2.0;
    p.stepWidth = 1.0;
    const InlayResult r = inlayToolpath({square(30)}, endMill(6.0), std::nullopt, p);
    REQUIRE(r.ok());
    const InlayShapes s = inlayShapes({square(30)}, 6.0, p.allowance, p.tolerance);
    for (const Move* m : cuts(r.toolpath)) {
        const double d = distanceToAll(s.male, Vec2{m->to.x, m->to.y});
        CHECK(d == Approx(m->to.z >= -2.0 - 1e-9 ? 4.0 : 3.0).margin(2e-3));
    }
}

TEST_CASE("inlay: hostile parameters and vectors are refused with a diagnostic", "[inlay][robust]") {
    const tooling::Tool tool = endMill(6.0);
    const std::vector<Contour> sq{square()};
    const auto refused = [&](const std::vector<Contour>& cs, const tooling::Tool& t, const InlayParams& p,
                             ToolpathIssue issue) {
        const InlayResult r = inlayToolpath(cs, t, std::nullopt, p);
        INFO((r.diagnostics.empty() ? std::string("no diagnostic") : r.diagnostics[0].message));
        CHECK_FALSE(r.ok());
        CHECK(r.has(issue));
    };
    const double nan = std::nan("");
    const double inf = std::numeric_limits<double>::infinity();
    for (const InlayPiece piece : {InlayPiece::Male, InlayPiece::Female}) {
        InlayParams p = params(piece);
        p.maleStyle = InlayMaleStyle::Stepped;
        refused(sq, tool, [&] { auto q = p; q.cutDepth = nan; return q; }(), ToolpathIssue::InvalidParameters);
        refused(sq, tool, [&] { auto q = p; q.cutDepth = 0.0; return q; }(), ToolpathIssue::InvalidParameters);
        refused(sq, tool, [&] { auto q = p; q.cutDepth = inf; return q; }(), ToolpathIssue::InvalidParameters);
        refused(sq, tool, [&] { auto q = p; q.allowance = -0.1; return q; }(), ToolpathIssue::InvalidParameters);
        refused(sq, tool, [&] { auto q = p; q.allowance = 1e9; return q; }(), ToolpathIssue::InvalidParameters);
        refused(sq, tool, [&] { auto q = p; q.startDepth = -1.0; return q; }(), ToolpathIssue::InvalidParameters);
        refused(sq, tool, [&] { auto q = p; q.safeZ = 0.0; return q; }(), ToolpathIssue::InvalidParameters);
        refused(sq, tool, [&] { auto q = p; q.startPosition.x = nan; return q; }(), ToolpathIssue::InvalidParameters);
        refused(sq, tool, [&] { auto q = p; q.tolerance.chordal = 0.0; return q; }(), ToolpathIssue::InvalidParameters);
    }
    InlayParams st = params(InlayPiece::Male);
    st.maleStyle = InlayMaleStyle::Stepped;
    refused(sq, tool, [&] { auto q = st; q.stepDepth = 3.0; return q; }(), ToolpathIssue::InvalidParameters);
    refused(sq, tool, [&] { auto q = st; q.stepDepth = 0.0; return q; }(), ToolpathIssue::InvalidParameters);
    refused(sq, tool, [&] { auto q = st; q.stepWidth = nan; return q; }(), ToolpathIssue::InvalidParameters);
    refused(sq, tool, [&] { auto q = st; q.stepWidth = 0.0; return q; }(), ToolpathIssue::InvalidParameters);
    // A step so wide it needs thousands of band passes.
    refused(sq, endMill(0.01), [&] { auto q = st; q.stepWidth = 900.0; return q; }(), ToolpathIssue::WorkLimitExceeded);
    // Not a flat end mill.
    tooling::Tool ball = tool;
    ball.type = tooling::ToolType::BallNose;
    refused(sq, ball, params(InlayPiece::Male), ToolpathIssue::InvalidTool);
    tooling::Tool bad = tool;
    bad.diameter = nan;
    refused(sq, bad, params(InlayPiece::Male), ToolpathIssue::InvalidParameters);
    // Vectors.
    refused({}, tool, params(InlayPiece::Male), ToolpathIssue::NoRegion);
    Contour open = square();
    open.closed = false;
    refused({open}, tool, params(InlayPiece::Female), ToolpathIssue::NoRegion);
    refused({Contour{}}, tool, params(InlayPiece::Male), ToolpathIssue::NoRegion);
    refused({square(4.0)}, tool, params(InlayPiece::Male), ToolpathIssue::ToolTooLarge);  // 6 mm tool, 4 mm square
    refused({square(20.0)}, tool, [&] { auto q = params(InlayPiece::Male); q.allowance = 12.0; return q; }(),
            ToolpathIssue::NoRegion);  // the allowance consumes the plug
    refused({polygon({{0, 0}, {1e9, 0}, {1e9, 5}})}, tool, params(InlayPiece::Male), ToolpathIssue::WorkLimitExceeded);
    refused({polygon({{0, 0}, {nan, 0}, {5, 5}})}, tool, params(InlayPiece::Female), ToolpathIssue::InvalidParameters);
    // A needle and a zero-area vector give nothing, never a crash.
    const InlayResult needle = inlayToolpath({polygon({{0, 0}, {50, 0}, {0, 0.001}})}, tool, std::nullopt, params(InlayPiece::Male));
    CHECK_FALSE(needle.ok());
    CHECK_FALSE(needle.diagnostics.empty());
    const InlayResult flat = inlayToolpath({polygon({{0, 0}, {10, 0}, {20, 0}})}, tool, std::nullopt, params(InlayPiece::Female));
    CHECK_FALSE(flat.ok());
    CHECK_FALSE(flat.diagnostics.empty());
    // An open vector among closed ones is ignored with a diagnostic; the rest is cut.
    const InlayResult mixed = inlayToolpath({open, square()}, tool, std::nullopt, params(InlayPiece::Male));
    CHECK(mixed.ok());
    CHECK(mixed.has(ToolpathIssue::OpenContourIgnored));
}

namespace {

Contour zigzagCircle(int n) {
    std::vector<Vec2> pts;
    for (int i = 0; i < n; ++i) {
        const double a = 2.0 * 3.14159265358979323846 * i / n;
        const double r = 500.0 + ((i % 2) != 0 ? 0.3 : 0.0);
        pts.push_back({r * std::cos(a), r * std::sin(a)});
    }
    return polygon(pts);
}

}  // namespace

TEST_CASE("inlay: a huge detailed vector is refused or finished in bounded CPU time", "[inlay][robust]") {
    const std::vector<Contour> big{zigzagCircle(20000)};
    const tooling::Tool tool = endMill(2.0);
    for (const InlayPiece piece : {InlayPiece::Male, InlayPiece::Female}) {
        InlayParams p = params(piece);
        p.allowance = 0.1;
        const double t0 = carvetto::test::processCpuSeconds();
        const InlayResult r = inlayToolpath(big, tool, std::nullopt, p);
        CHECK(carvetto::test::processCpuSeconds() - t0 < 5.0);
        CHECK_FALSE(r.ok());
        CHECK(r.has(ToolpathIssue::WorkLimitExceeded));
    }
}

TEST_CASE("inlay: a cancel request stops the run with a Cancelled diagnostic", "[inlay][robust]") {
    std::atomic<bool> stop{true};
    carvetto::geometry::CancelScope scope(&stop);
    const tooling::Tool tool = endMill(6.0);
    for (const InlayPiece piece : {InlayPiece::Male, InlayPiece::Female}) {
        const InlayResult r = inlayToolpath({square()}, tool, std::nullopt, params(piece));
        CHECK_FALSE(r.ok());
        CHECK(r.has(ToolpathIssue::Cancelled));
        CHECK(r.toolpath.isEmpty());
    }
    const InlayShapes sh = inlayShapes({square()}, 6.0, 0.2, carvetto::geometry::Tolerance::defaults());
    CHECK(sh.female.empty());
    CHECK(sh.male.empty());
}

TEST_CASE("inlay female: an allowance that consumes the shape does not warn the female", "[inlay]") {
    InlayParams p = params(InlayPiece::Female);
    p.allowance = 12.0;
    const InlayResult f = inlayToolpath({square()}, endMill(6.0), std::nullopt, p);
    CHECK(f.ok());
    CHECK_FALSE(f.has(ToolpathIssue::NoRegion));
    p.piece = InlayPiece::Male;
    const InlayResult m = inlayToolpath({square()}, endMill(6.0), std::nullopt, p);
    CHECK_FALSE(m.ok());
    CHECK(m.has(ToolpathIssue::NoRegion));
}
