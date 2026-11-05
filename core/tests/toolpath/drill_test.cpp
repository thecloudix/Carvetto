// T-0150: drilling toolpath (drill.hpp, ADR-0022). AC1: one hole per closed vector at its centre
// (area centroid), diagnostics for what is skipped, peck and dwell options, ordering through the
// shared cut-order helper. AC2 (move-level part): every rapid at safe Z, plunges at the plunge
// rate, peck retracts never below the cutting start + retract gap (AboveStart) or the depth just
// reached + retract gap (AbovePreviousPass), dwells > 0 only.

#include <algorithm>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/transform2.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/cut_order.hpp"
#include "carvetto/toolpath/drill.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using geometry::ArcDirection;
using geometry::ArcSpan;
using geometry::Box2;
using geometry::Contour;
using geometry::LineSpan;
using geometry::Vec2;
using geometry::Vec3;

namespace {

tooling::Tool drillTool(double diameter = 3.0, double passDepth = 2.0) {
    tooling::Tool t;
    t.id = "drill";
    t.name = "Drill 3 mm";
    t.type = tooling::ToolType::Drill;
    t.diameter = diameter;
    t.includedAngle = 118.0 * geometry::kPi / 180.0;
    t.defaultCuttingData.toolNumber = 7;
    t.defaultCuttingData.feedRate = 500.0;
    t.defaultCuttingData.plungeRate = 300.0;
    t.defaultCuttingData.spindleSpeed = 10000.0;
    t.defaultCuttingData.passDepth = passDepth;
    return t;
}

tooling::Tool endMill(double diameter = 6.0) {
    tooling::Tool t = drillTool(diameter);
    t.id = "em";
    t.name = "End Mill";
    t.type = tooling::ToolType::EndMill;
    t.includedAngle = 0.0;
    return t;
}

Contour polygon(const std::vector<Vec2>& pts) {
    Contour c;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        c.append(LineSpan{pts[i], pts[(i + 1) % pts.size()]});
    }
    c.closed = true;
    return c;
}

/// Polygon centroid (shoelace) of a fine polygonisation: an oracle independent of drillCentre's
/// closed-form arc and quadrature Bezier terms.
Vec2 polygonCentroid(const Contour& c, double chordal) {
    geometry::Tolerance tol;
    tol.chordal = chordal;
    const geometry::Polyline pl = c.polygonise(tol);
    const std::vector<Vec2>& v = pl.points;
    double a = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    const Vec2 o = v.front();
    for (std::size_t i = 0; i < v.size(); ++i) {
        const Vec2 p = v[i] - o;
        const Vec2 q = v[(i + 1) % v.size()] - o;
        const double k = p.cross(q);
        a += k;
        cx += (p.x + q.x) * k;
        cy += (p.y + q.y) * k;
    }
    return {o.x + cx / (3.0 * a), o.y + cy / (3.0 * a)};
}

DrillParams params(double cutDepth = 6.0) {
    DrillParams p;
    p.name = "Holes";
    p.cutDepth = cutDepth;
    p.safeZ = 5.0;
    return p;
}

/// Checks the AC2 move rules on a drilling toolpath; returns the first problem or an empty string.
std::string moveProblem(const DrillResult& r, const DrillParams& p) {
    const std::vector<Move>& m = r.toolpath.moves;
    const double startZ = -p.startDepth;
    const double bottomZ = -(p.startDepth + p.cutDepth);
    std::optional<Vec3> at;
    double deepest = 0.0;  // the depth reached in the current hole
    for (std::size_t i = 0; i < m.size(); ++i) {
        const Move& mv = m[i];
        const std::string where = "move " + std::to_string(i) + ": ";
        if (mv.feedOverride) return where + "feed override";
        if (mv.to.z < bottomZ - 1e-12) return where + "below the bottom of the holes";
        switch (mv.kind) {
            case MoveKind::Rapid:
                if (mv.to.z != p.safeZ) return where + "rapid not at safe Z";
                if (at && at->z != p.safeZ) return where + "rapid from below safe Z";
                deepest = startZ;
                break;
            case MoveKind::Plunge:
                if (!at || mv.to.x != at->x || mv.to.y != at->y) return where + "plunge moves in XY";
                if (!(mv.to.z < at->z)) return where + "plunge does not go down";
                deepest = std::min(deepest, mv.to.z);
                break;
            case MoveKind::Retract: {
                if (!at || mv.to.x != at->x || mv.to.y != at->y) return where + "retract moves in XY";
                if (!(mv.to.z > at->z)) return where + "retract does not go up";
                if (mv.to.z > p.safeZ) return where + "retract above safe Z";
                const bool last = i + 1 == m.size() || m[i + 1].kind == MoveKind::Rapid;
                if (last) {
                    if (mv.to.z != p.safeZ) return where + "the last retract of a hole is not to safe Z";
                } else {
                    const double retractFloor = p.peckRetract == PeckRetract::AboveStart ? startZ : deepest;
                    if (mv.to.z < std::min(retractFloor + p.retractGap, p.safeZ) - 1e-12) {
                        return where + "peck retract below the retract height";
                    }
                }
                break;
            }
            case MoveKind::Dwell:
                if (!(mv.dwellSeconds > 0.0)) return where + "dwell of zero seconds";
                if (!at || !(mv.to == *at)) return where + "dwell moves";
                if (m[i - 1].kind != MoveKind::Plunge) return where + "dwell not at the bottom of a pass";
                break;
            default:
                return where + "unexpected move kind";
        }
        at = mv.to;
    }
    if (at && at->z != p.safeZ) return "the toolpath does not end at safe Z";
    return {};
}

}  // namespace

// ------------------------------------------------------------------------------------- centre

TEST_CASE("drillCentre: circles of any construction give their centre", "[toolpath][drill][centre]") {
    const geometry::Tolerance tol;
    const Vec2 c{12.5, -7.25};
    CHECK(geometry::distance(*drillCentre(Contour::circle(c, 4.0), tol), c) < 1e-12);
    CHECK(geometry::distance(*drillCentre(Contour::circle(c, 4.0, ArcDirection::CW), tol), c) < 1e-12);

    // Two half arcs, starting at an arbitrary angle.
    Contour halves;
    halves.append(ArcSpan{c, 4.0, 0.3, 0.3 + geometry::kPi, ArcDirection::CCW});
    halves.append(ArcSpan{c, 4.0, 0.3 + geometry::kPi, 0.3, ArcDirection::CCW});
    halves.closed = true;
    CHECK(geometry::distance(*drillCentre(halves, tol), c) < 1e-12);

    // Four Bezier quarter circles (e.g. from an SVG or DXF spline).
    Contour bez;
    for (const geometry::BezierSpan& b : ArcSpan::fullCircle(c, 4.0).toBeziers()) {
        bez.append(b);
    }
    bez.closed = true;
    CHECK(geometry::distance(*drillCentre(bez, tol), c) < 1e-12);

    // A 36-gon from a DXF polyline.
    std::vector<Vec2> pts;
    for (int i = 0; i < 36; ++i) {
        pts.push_back(c + Vec2::fromAngle(i * geometry::kPi / 18.0 + 0.1, 4.0));
    }
    CHECK(geometry::distance(*drillCentre(polygon(pts), tol), c) < 1e-12);
}

TEST_CASE("drillCentre: the area centroid of polygons, slots and arcs", "[toolpath][drill][centre]") {
    const geometry::Tolerance tol;
    // Rectangle and triangle (centroid = mean of the vertices).
    CHECK(geometry::distance(*drillCentre(Contour::rectangle(Box2{Vec2{1, 2}, Vec2{11, 6}}), tol), Vec2{6, 4}) < 1e-12);
    CHECK(geometry::distance(*drillCentre(polygon({{0, 0}, {9, 0}, {0, 6}}), tol), Vec2{3, 2}) < 1e-12);
    // L shape: 4x1 bar (centroid (2, 0.5), area 4) plus 1x3 bar above its left end ((0.5, 2.5), 3).
    const Vec2 l = *drillCentre(polygon({{0, 0}, {4, 0}, {4, 1}, {1, 1}, {1, 4}, {0, 4}}), tol);
    CHECK(l.x == Catch::Approx((2.0 * 4 + 0.5 * 3) / 7.0).epsilon(1e-14));
    CHECK(l.y == Catch::Approx((0.5 * 4 + 2.5 * 3) / 7.0).epsilon(1e-14));
    // Obround slot (two half circles and two lines): the middle.
    Contour slot;
    slot.append(LineSpan{{0, 0}, {20, 0}});
    slot.append(ArcSpan{{20, 5}, 5.0, -geometry::kPi / 2, geometry::kPi / 2, ArcDirection::CCW});
    slot.append(LineSpan{{20, 10}, {0, 10}});
    slot.append(ArcSpan{{0, 5}, 5.0, geometry::kPi / 2, 3 * geometry::kPi / 2, ArcDirection::CCW});
    slot.closed = true;
    CHECK(geometry::distance(*drillCentre(slot, tol), Vec2{10, 5}) < 1e-12);
    // A D shape (half disc): centroid 4r/(3 pi) from the flat side.
    Contour d;
    d.append(LineSpan{{-3, 0}, {3, 0}});
    d.append(ArcSpan{{0, 0}, 3.0, 0.0, geometry::kPi, ArcDirection::CCW});
    d.closed = true;
    const Vec2 dc = *drillCentre(d, tol);
    CHECK(std::fabs(dc.x) < 1e-12);
    CHECK(dc.y == Catch::Approx(4.0 * 3.0 / (3.0 * geometry::kPi)).epsilon(1e-13));
}

TEST_CASE("drillCentre: open, empty and zero-area vectors have no centre", "[toolpath][drill][centre]") {
    const geometry::Tolerance tol;
    Contour open = polygon({{0, 0}, {5, 0}, {5, 5}});
    open.closed = false;
    CHECK_FALSE(drillCentre(open, tol));
    CHECK_FALSE(drillCentre(Contour{}, tol));
    CHECK_FALSE(drillCentre(polygon({{0, 0}, {5, 0}, {10, 0}}), tol));  // collinear: no area
    CHECK_FALSE(drillCentre(Contour::circle({1, 1}, 1e-4), tol));        // area below tol.distance^2
}

TEST_CASE("drillCentre: matches a fine polygon oracle and turns with the shape (generative)",
          "[toolpath][drill][centre]") {
    const geometry::Tolerance tol;
    std::mt19937_64 rng(150);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    for (int n = 0; n < 200; ++n) {
        // A star-shaped loop of lines, arcs (bulges) and Beziers round a random point.
        const Vec2 o{u(rng) * 500.0, u(rng) * 500.0};
        const int k = 3 + n % 6;
        std::vector<Vec2> v;
        for (int i = 0; i < k; ++i) {
            const double a = 2.0 * geometry::kPi * (i + 0.3 * u(rng)) / k;
            v.push_back(o + Vec2::fromAngle(a, 10.0 + 5.0 * u(rng)));
        }
        Contour c;
        for (int i = 0; i < k; ++i) {
            const Vec2 a = v[static_cast<std::size_t>(i)];
            const Vec2 b = v[static_cast<std::size_t>((i + 1) % k)];
            switch ((n + i) % 3) {
                case 0:
                    c.append(LineSpan{a, b});
                    break;
                case 1: {
                    // Outward bulge: arc through a and b about a centre inside the chord's side.
                    const Vec2 mid = (a + b) * 0.5;
                    const Vec2 inward = Vec2{a.y - b.y, b.x - a.x}.normalized();  // left of a -> b
                    const Vec2 centre = mid + inward * (0.3 * geometry::distance(a, b));
                    c.append(ArcSpan::fromCentre(centre, a, b, ArcDirection::CCW));
                    break;
                }
                default: {
                    const Vec2 d = b - a;
                    const Vec2 nrm{d.y, -d.x};
                    c.append(geometry::BezierSpan{a, a + d * 0.3 + nrm * 0.2, a + d * 0.7 - nrm * 0.1, b});
                }
            }
        }
        c.closed = true;
        INFO("case " << n);
        const std::optional<Vec2> got = drillCentre(c, tol);
        REQUIRE(got);
        CHECK(geometry::distance(*got, polygonCentroid(c, 1e-5)) < 1e-4);
        // Rotating and moving the vector moves its centre the same way (the bounding-box centre
        // would not).
        const geometry::Transform2 m =
            geometry::Transform2::translation({3.5, -8.0}) * geometry::Transform2::rotation(0.7 + u(rng));
        const std::optional<Vec2> turned = drillCentre(c.transformed(m), tol);
        REQUIRE(turned);
        CHECK(geometry::distance(*turned, m.apply(*got)) < 1e-9);
        // Reversing the direction of travel does not move it.
        CHECK(geometry::distance(*drillCentre(c.reversed(), tol), *got) < 1e-9);
    }
}

TEST_CASE("drillCentre: far from the origin the centre stays exact", "[toolpath][drill][centre]") {
    const geometry::Tolerance tol;
    const Vec2 c{1.0e6 + 0.125, -2.0e6 + 0.375};
    CHECK(geometry::distance(*drillCentre(Contour::circle(c, 2.0), tol), c) < 1e-9);
    CHECK(geometry::distance(*drillCentre(Contour::rectangle(Box2{c - Vec2{1, 3}, c + Vec2{1, 3}}), tol), c) < 1e-9);
}

// -------------------------------------------------------------------------------------- moves

TEST_CASE("AC1: one hole per closed vector, straight plunge to the bottom", "[toolpath][drill]") {
    const DrillResult r = drillToolpath({Contour::circle({10, 20}, 2.0)}, drillTool(), std::nullopt, params());
    REQUIRE(r.ok());
    CHECK(r.diagnostics.empty());
    CHECK(r.toolpath.name == "Holes");
    CHECK(r.toolpath.tool.name == "Drill 3 mm");
    CHECK(r.toolpath.tool.toolNumber == 7);
    CHECK(r.toolpath.tool.plungeRate == 300.0);
    REQUIRE(r.holes.size() == 1);
    CHECK(r.holes[0] == DrillHole{{10, 20}, 0});
    const std::vector<Move> want{Move::rapid({10, 20, 5}), Move::plunge({10, 20, -6}), Move::retract({10, 20, 5})};
    CHECK(r.toolpath.moves == want);
}

TEST_CASE("AC1: peck drilling retracting above the cutting start", "[toolpath][drill][peck]") {
    DrillParams p = params(6.0);
    p.startDepth = 1.0;
    p.peck = true;
    p.retractGap = 0.5;
    p.dwellTime = 0.25;
    // Pass depth 2.5 (exact steps, the last takes the remainder): -3.5, -6, -7.
    const DrillResult r = drillToolpath({Contour::circle({0, 0}, 3.0)}, drillTool(3.0, 2.5), std::nullopt, p);
    REQUIRE(r.ok());
    CHECK(r.diagnostics.empty());
    const std::vector<Move> want{
        Move::rapid({0, 0, 5}),        Move::plunge({0, 0, -3.5}),        Move::dwell({0, 0, -3.5}, 0.25),
        Move::retract({0, 0, -0.5}),   Move::plunge({0, 0, -6}),          Move::dwell({0, 0, -6}, 0.25),
        Move::retract({0, 0, -0.5}),   Move::plunge({0, 0, -7}),          Move::dwell({0, 0, -7}, 0.25),
        Move::retract({0, 0, 5}),
    };
    CHECK(r.toolpath.moves == want);
    CHECK(moveProblem(r, p).empty());
}

TEST_CASE("AC1: peck drilling retracting above the previous pass", "[toolpath][drill][peck]") {
    DrillParams p = params(5.0);
    p.peck = true;
    p.peckRetract = PeckRetract::AbovePreviousPass;
    p.retractGap = 0.5;
    const DrillResult r = drillToolpath({Contour::circle({4, 4}, 3.0)}, drillTool(3.0, 2.0), std::nullopt, p);
    REQUIRE(r.ok());
    const std::vector<Move> want{
        Move::rapid({4, 4, 5}),     Move::plunge({4, 4, -2}),   Move::retract({4, 4, -1.5}),
        Move::plunge({4, 4, -4}),   Move::retract({4, 4, -3.5}), Move::plunge({4, 4, -5}),
        Move::retract({4, 4, 5}),
    };
    CHECK(r.toolpath.moves == want);
    CHECK(moveProblem(r, p).empty());
}

TEST_CASE("AC1: peck retracts never go above safe Z; without peck the pass depth is ignored",
          "[toolpath][drill][peck]") {
    DrillParams p = params(4.0);
    p.peck = true;
    p.retractGap = 20.0;  // above safe Z: clamped to it
    const DrillResult r = drillToolpath({Contour::circle({0, 0}, 3.0)}, drillTool(3.0, 2.0), std::nullopt, p);
    REQUIRE(r.ok());
    CHECK(r.toolpath.moves == std::vector<Move>{Move::rapid({0, 0, 5}), Move::plunge({0, 0, -2}),
                                                Move::retract({0, 0, 5}), Move::plunge({0, 0, -4}),
                                                Move::retract({0, 0, 5})});
    // A pass depth of zero only matters when pecking.
    DrillParams single = params(4.0);
    const DrillResult s = drillToolpath({Contour::circle({0, 0}, 3.0)}, drillTool(3.0, 0.0), std::nullopt, single);
    REQUIRE(s.ok());
    CHECK(s.toolpath.count(MoveKind::Plunge) == 1);
    // Pass depth >= cut depth: one pass even when pecking.
    single.peck = true;
    const DrillResult one = drillToolpath({Contour::circle({0, 0}, 3.0)}, drillTool(3.0, 9.0), std::nullopt, single);
    REQUIRE(one.ok());
    CHECK(one.toolpath.count(MoveKind::Plunge) == 1);
    CHECK(one.toolpath.count(MoveKind::Retract) == 1);
}

TEST_CASE("AC1: dwell at the bottom of every pass only when the time is above zero", "[toolpath][drill][dwell]") {
    DrillParams p = params(6.0);
    p.peck = true;
    const std::vector<Contour> holes{Contour::circle({0, 0}, 2.0), Contour::circle({10, 0}, 2.0)};
    CHECK(drillToolpath(holes, drillTool(3.0, 2.0), std::nullopt, p).toolpath.count(MoveKind::Dwell) == 0);
    p.dwellTime = 1.5;
    const DrillResult r = drillToolpath(holes, drillTool(3.0, 2.0), std::nullopt, p);
    CHECK(r.toolpath.count(MoveKind::Dwell) == 6);  // 2 holes x 3 passes
    for (const Move& m : r.toolpath.moves) {
        if (m.kind == MoveKind::Dwell) {
            CHECK(m.dwellSeconds == 1.5);
        }
    }
    CHECK(moveProblem(r, p).empty());
}

TEST_CASE("AC1: the chosen cutting data overrides the tool's default data", "[toolpath][drill]") {
    tooling::CuttingData cd = drillTool().defaultCuttingData;
    cd.plungeRate = 120.0;
    cd.passDepth = 1.0;
    cd.toolNumber = 3;
    DrillParams p = params(3.0);
    p.peck = true;
    const DrillResult r = drillToolpath({Contour::circle({0, 0}, 2.0)}, drillTool(3.0, 2.0), cd, p);
    REQUIRE(r.ok());
    CHECK(r.toolpath.tool.plungeRate == 120.0);
    CHECK(r.toolpath.tool.toolNumber == 3);
    CHECK(r.toolpath.count(MoveKind::Plunge) == 3);
}

// -------------------------------------------------------------------------------- diagnostics

TEST_CASE("AC1: open, empty, gapped and zero-area vectors are reported", "[toolpath][drill][diagnostics]") {
    Contour open = polygon({{0, 0}, {5, 0}, {5, 5}});
    open.closed = false;
    Contour gapped;
    gapped.append(LineSpan{{20, 0}, {25, 0}});
    gapped.append(LineSpan{{25, 1}, {20, 5}});
    gapped.closed = true;
    const std::vector<Contour> in{open, Contour{}, Contour::circle({40, 0}, 2.0), gapped, open,
                                  polygon({{50, 0}, {55, 0}, {60, 0}})};
    const DrillResult r = drillToolpath(in, drillTool(), std::nullopt, params());
    REQUIRE(r.ok());
    REQUIRE(r.holes.size() == 1);
    CHECK(r.holes[0].contour == 2);
    REQUIRE(r.diagnostics.size() == 4);
    CHECK(r.diagnostics[0] == ToolpathDiagnostic{ToolpathIssue::EmptyContour, 1, "vector 1 has no spans; skipped", {}});
    CHECK(r.diagnostics[1] ==
          ToolpathDiagnostic{ToolpathIssue::DiscontinuousContour, 3, "vector 3 has gaps between spans; skipped", {}});
    CHECK(r.diagnostics[2] == ToolpathDiagnostic{ToolpathIssue::NoRegion, 5,
                                                 "vector 5 encloses no area, so it has no centre; skipped", {}});
    CHECK(r.diagnostics[3] == ToolpathDiagnostic{ToolpathIssue::OpenContourIgnored, 0,
                                                 "2 open vector(s) ignored: drilling needs closed vectors", {}});
}

TEST_CASE("AC1: a vector whose centre lies outside it is not drilled", "[toolpath][drill][diagnostics]") {
    // A C shape: its centroid is in the gap.
    const Contour c = polygon({{0, 0}, {10, 0}, {10, 2}, {2, 2}, {2, 8}, {10, 8}, {10, 10}, {0, 10}});
    const DrillResult r = drillToolpath({c, Contour::circle({30, 5}, 2.0)}, drillTool(), std::nullopt, params());
    REQUIRE(r.ok());
    REQUIRE(r.holes.size() == 1);
    CHECK(r.holes[0].contour == 1);
    REQUIRE(r.diagnostics.size() == 1);
    CHECK(r.diagnostics[0].issue == ToolpathIssue::CentreOutsideVector);
    CHECK(r.diagnostics[0].contour == std::optional<std::size_t>(0));
    CHECK(r.diagnostics[0].message.find("not inside it; not drilled") != std::string::npos);
    // Only such vectors: no toolpath, and the reason is given.
    const DrillResult none = drillToolpath({c}, drillTool(), std::nullopt, params());
    CHECK_FALSE(none.ok());
    CHECK(none.has(ToolpathIssue::CentreOutsideVector));
}

TEST_CASE("T-0143: a ring drawn as one loop with its bridges on the ray is not drilled",
          "[toolpath][drill][diagnostics]") {
    // Verifier repro (harness t0150v1/ring.cpp): outer arc, bridge in, inner arc reversed, bridge
    // back, with the bridges and the arc ends on the horizontal through the centre.
    for (const double deg : {0.0, 90.0, 180.0, 270.0, 30.0}) {
        INFO("bridge at " << deg << " deg");
        const double a = deg * geometry::kPi / 180.0;
        const Vec2 c{3.0, 4.0};
        const Vec2 po = c + Vec2::fromAngle(a, 10.0);
        const Vec2 pi = c + Vec2::fromAngle(a, 5.0);
        Contour ring;
        ring.closed = true;
        ring.append(ArcSpan{c, 10.0, a, a + 2.0 * geometry::kPi, ArcDirection::CCW});
        ring.append(LineSpan{po, pi});
        ring.append(ArcSpan{c, 5.0, a, a + 2.0 * geometry::kPi, ArcDirection::CW});
        ring.append(LineSpan{pi, po});
        const DrillResult r = drillToolpath({ring}, drillTool(), std::nullopt, params());
        CHECK_FALSE(r.ok());
        CHECK(r.holes.empty());
        CHECK(r.has(ToolpathIssue::CentreOutsideVector));
    }
}

TEST_CASE("AC1: vectors smaller than the tool are drilled and reported", "[toolpath][drill][diagnostics]") {
    // A 1 mm marker circle with a 3 mm drill; a 3 mm circle exactly fits (no report); a 2 mm wide
    // slot is narrower than the drill round its centre.
    const std::vector<Contour> in{Contour::circle({0, 0}, 0.5), Contour::circle({10, 0}, 1.5),
                                  Contour::rectangle(Box2{Vec2{20, -1}, Vec2{40, 1}})};
    const DrillResult r = drillToolpath(in, drillTool(3.0), std::nullopt, params());
    REQUIRE(r.holes.size() == 3);
    REQUIRE(r.diagnostics.size() == 2);
    CHECK(r.diagnostics[0].issue == ToolpathIssue::HoleWiderThanVector);
    CHECK(r.diagnostics[0].contour == std::optional<std::size_t>(0));
    CHECK(r.diagnostics[0].message ==
          "vector 0: the 3 mm hole is wider than the vector round its centre (0.5 mm from the centre to the "
          "vector); drilled anyway");
    CHECK(r.diagnostics[1].issue == ToolpathIssue::HoleWiderThanVector);
    CHECK(r.diagnostics[1].contour == std::optional<std::size_t>(2));
}

TEST_CASE("AC1: coincident holes are drilled once", "[toolpath][drill][diagnostics]") {
    // A counterbore drawing (two concentric circles) and a duplicated vector.
    const std::vector<Contour> in{Contour::circle({0, 0}, 5.0), Contour::circle({0, 0}, 2.0),
                                  Contour::rectangle(Box2{Vec2{10, 10}, Vec2{14, 14}}),
                                  Contour::rectangle(Box2{Vec2{10, 10}, Vec2{14, 14}})};
    const DrillResult r = drillToolpath(in, drillTool(), std::nullopt, params());
    REQUIRE(r.holes.size() == 2);
    REQUIRE(r.diagnostics.size() == 2);
    CHECK(r.diagnostics[0] == ToolpathDiagnostic{ToolpathIssue::DuplicateHole, 1,
                                                 "vector 1 has the same centre as vector 0; the hole is drilled once",
                                                 {}});
    CHECK(r.diagnostics[1].contour == std::optional<std::size_t>(3));
    CHECK(r.toolpath.count(MoveKind::Rapid) == 2);
}

TEST_CASE("AC1: drills and end mills are accepted, other tools refused", "[toolpath][drill][tool]") {
    const std::vector<Contour> in{Contour::circle({0, 0}, 5.0)};
    CHECK(drillToolpath(in, drillTool(), std::nullopt, params()).ok());
    CHECK(drillToolpath(in, endMill(), std::nullopt, params()).ok());
    CHECK(drillToolAllowed(tooling::ToolType::Drill));
    CHECK(drillToolAllowed(tooling::ToolType::EndMill));
    for (const tooling::ToolType type : tooling::allToolTypes()) {
        if (drillToolAllowed(type)) {
            continue;
        }
        INFO(tooling::toString(type));
        CHECK_FALSE(type == tooling::ToolType::Drill);
        tooling::Tool t = endMill();
        t.type = type;
        t.includedAngle = 1.0;
        t.tipDiameter = 1.0;
        t.cornerRadius = 1.0;
        t.profile = {{0, 0}, {3, 5}};
        const DrillResult r = drillToolpath(in, t, std::nullopt, params());
        CHECK_FALSE(r.ok());
        REQUIRE(r.diagnostics.size() == 1);
        CHECK(r.diagnostics[0].issue == ToolpathIssue::InvalidTool);
        CHECK(r.diagnostics[0].message.find("drilling needs a drill or an end mill") != std::string::npos);
    }
    tooling::Tool vbit = endMill();
    vbit.type = tooling::ToolType::VBit;
    vbit.includedAngle = 1.0;
    CHECK(drillToolpath(in, vbit, std::nullopt, params()).diagnostics[0].message ==
          "tool 'End Mill' is a V-bit; drilling needs a drill or an end mill");
}

TEST_CASE("AC1: unusable tools and cutting data are refused", "[toolpath][drill][tool]") {
    const std::vector<Contour> in{Contour::circle({0, 0}, 5.0)};
    auto refusal = [&](const tooling::Tool& t, const DrillParams& p) {
        const DrillResult r = drillToolpath(in, t, std::nullopt, p);
        CHECK_FALSE(r.ok());
        REQUIRE(r.diagnostics.size() == 1);
        CHECK(r.diagnostics[0].issue == ToolpathIssue::InvalidTool);
        return r.diagnostics[0].message;
    };
    tooling::Tool t = drillTool();
    t.includedAngle = 0.0;
    CHECK(refusal(t, params()) == "tool 'Drill 3 mm': included angle must be in (0, pi) radians");
    t = drillTool();
    t.diameter = 0.0;
    CHECK(refusal(t, params()) == "tool 'Drill 3 mm': diameter must be positive");
    t = drillTool();
    t.defaultCuttingData.plungeRate = 0.0;
    CHECK(refusal(t, params()) == "tool 'Drill 3 mm': the plunge rate must be greater than zero");
    t = drillTool();
    t.defaultCuttingData.feedRate = std::nan("");
    CHECK(refusal(t, params()) == "tool 'Drill 3 mm': the feed rate must be greater than zero");
    DrillParams peck = params();
    peck.peck = true;
    CHECK(refusal(drillTool(3.0, 0.0), peck) ==
          "tool 'Drill 3 mm': peck drilling needs a pass depth greater than zero in the cutting data");
}

TEST_CASE("AC1: invalid parameters are refused", "[toolpath][drill][params]") {
    const std::vector<Contour> in{Contour::circle({0, 0}, 5.0)};
    auto refusal = [&](const DrillParams& p) {
        const DrillResult r = drillToolpath(in, drillTool(), std::nullopt, p);
        CHECK_FALSE(r.ok());
        REQUIRE(r.diagnostics.size() == 1);
        CHECK(r.diagnostics[0].issue == ToolpathIssue::InvalidParameters);
        return r.diagnostics[0].message;
    };
    DrillParams p = params();
    p.cutDepth = 0.0;
    CHECK(refusal(p) == "cut depth must be greater than zero");
    p = params();
    p.startDepth = -1.0;
    CHECK(refusal(p) == "start depth must not be negative");
    p = params();
    p.startDepth = 2.0;
    p.safeZ = -2.0;  // at the cutting start
    CHECK(refusal(p) == "safe Z must be above the start of the cut");
    p = params();
    p.dwellTime = -0.5;
    CHECK(refusal(p) == "dwell time must not be negative");
    p = params();
    p.peck = true;
    p.retractGap = 0.0;
    CHECK(refusal(p) == "peck drilling needs a retract gap greater than zero");
    p.peck = false;  // the gap is not used without pecking
    CHECK(drillToolpath(in, drillTool(), std::nullopt, p).ok());
    for (double DrillParams::*field : {&DrillParams::startDepth, &DrillParams::cutDepth, &DrillParams::safeZ,
                                       &DrillParams::retractGap, &DrillParams::dwellTime}) {
        for (const double bad : {std::nan(""), HUGE_VAL, -HUGE_VAL}) {
            p = params();
            p.*field = bad;
            CHECK(refusal(p) == "a parameter is not a finite number");
        }
    }
    p = params();
    p.startPosition.y = std::nan("");
    CHECK(refusal(p) == "a parameter is not a finite number");
    p = params();
    p.tolerance.chordal = 1e-6;
    CHECK(refusal(p).find("invalid tolerance") == 0);
}

TEST_CASE("AC1: a pass depth too small for the cut is refused before any work", "[toolpath][drill][limits]") {
    DrillParams p = params(100.0);
    p.peck = true;
    const DrillResult r = drillToolpath({Contour::circle({0, 0}, 5.0)}, drillTool(3.0, 0.001), std::nullopt, p);
    CHECK_FALSE(r.ok());
    REQUIRE(r.diagnostics.size() == 1);
    CHECK(r.diagnostics[0].issue == ToolpathIssue::WorkLimitExceeded);
}

TEST_CASE("T-0143: exactly 10000 passes per hole are drilled, 10001 are refused", "[toolpath][drill][limits]") {
    // Pass depth 0.5 mm is exact in binary, so cut / pass is exact: 5000 mm -> 10000 passes.
    DrillParams p = params(5000.0);
    p.peck = true;
    p.safeZ = 5.0;
    const DrillResult ok = drillToolpath({Contour::circle({0, 0}, 5.0)}, drillTool(3.0, 0.5), std::nullopt, p);
    REQUIRE(ok.ok());
    CHECK(ok.toolpath.count(MoveKind::Plunge) == 10000);

    p.cutDepth = 5000.25;  // 10000 full passes and a 0.25 mm last one
    const DrillResult over = drillToolpath({Contour::circle({0, 0}, 5.0)}, drillTool(3.0, 0.5), std::nullopt, p);
    CHECK_FALSE(over.ok());
    REQUIRE(over.diagnostics.size() == 1);
    CHECK(over.diagnostics[0].issue == ToolpathIssue::WorkLimitExceeded);
    CHECK(over.diagnostics[0].message.find("more than 10000 passes per hole") != std::string::npos);
    CHECK(over.toolpath.moves.empty());

    // Within PassPlan's round-off guard of exactly 10000 passes: still 10000, still drilled.
    p.cutDepth = 5000.0 * (1.0 + 1e-15);
    const DrillResult guard = drillToolpath({Contour::circle({0, 0}, 5.0)}, drillTool(3.0, 0.5), std::nullopt, p);
    REQUIRE(guard.ok());
    CHECK(guard.toolpath.count(MoveKind::Plunge) == 10000);
}

// ----------------------------------------------------------------------------------- ordering

TEST_CASE("AC1: holes are ordered by the shared cut-order helper", "[toolpath][drill][order]") {
    // A scattered grid of different-size holes, in a shuffled selection order.
    std::vector<Contour> in;
    std::mt19937_64 rng(7);
    std::uniform_real_distribution<double> jitter(-0.2, 0.2);
    for (int n = 0; n < 25; ++n) {
        const int i = n * 7 % 25;  // selection order differs from the grid order
        const Vec2 c{10.0 * (i % 5) + jitter(rng), 10.0 * (i / 5) + jitter(rng)};
        in.push_back(i % 2 ? Contour::circle(c, 1.0 + 0.1 * i) : Contour::rectangle(Box2{c - Vec2{2, 1}, c + Vec2{2, 1}}));
    }
    for (int strategy = 0; strategy < 5; ++strategy) {
        INFO("strategy " << strategy);
        DrillParams p = params();
        p.startPosition = {100, -50};
        p.order.selectionOrder = strategy == 1;
        p.order.leftToRight = strategy == 2;
        p.order.bottomToTop = strategy == 3;
        p.order.grid = strategy == 4;
        const DrillResult r = drillToolpath(in, drillTool(), std::nullopt, p);
        REQUIRE(r.holes.size() == in.size());
        // The same order as orderCuts over the vectors (as open paths) starting and ending at the centres.
        std::vector<OrderItem> items;
        for (std::size_t i = 0; i < in.size(); ++i) {
            OrderItem it;
            it.path = in[i];
            it.path.closed = false;
            it.selectionIndex = i;
            it.start = it.end = *drillCentre(in[i], p.tolerance);
            items.push_back(it);
        }
        const std::vector<std::size_t> want = orderCuts(items, p.order, p.startPosition, p.tolerance);
        for (std::size_t k = 0; k < want.size(); ++k) {
            CHECK(r.holes[k].contour == want[k]);
        }
        // The moves visit the holes in that order.
        std::size_t hole = 0;
        for (const Move& m : r.toolpath.moves) {
            if (m.kind == MoveKind::Rapid) {
                REQUIRE(hole < r.holes.size());
                CHECK(Vec2{m.to.x, m.to.y} == r.holes[hole].centre);
                ++hole;
            }
        }
        CHECK(hole == r.holes.size());
        if (strategy == 1) {
            for (std::size_t k = 0; k < r.holes.size(); ++k) {
                CHECK(r.holes[k].contour == k);
            }
        }
        if (strategy == 0) {
            // Nearest first from the start position.
            CHECK(r.holes[0].centre.x > 35.0);
            CHECK(r.holes[0].centre.y < 5.0);
        }
    }
}

// ------------------------------------------------------------------------------- AC2 property

TEST_CASE("AC2: generated jobs obey the drilling move rules", "[toolpath][drill][safety]") {
    std::mt19937_64 rng(2026);
    std::uniform_real_distribution<double> u(0.0, 1.0);
    for (int n = 0; n < 300; ++n) {
        std::vector<Contour> in;
        const int count = 1 + static_cast<int>(u(rng) * 12);
        for (int i = 0; i < count; ++i) {
            const Vec2 c{u(rng) * 200 - 100, u(rng) * 200 - 100};
            const double s = 0.2 + u(rng) * 10;
            switch (static_cast<int>(u(rng) * 4)) {
                case 0: in.push_back(Contour::circle(c, s)); break;
                case 1: in.push_back(Contour::rectangle(Box2{c - Vec2{s, s * 0.5}, c + Vec2{s, s * 0.5}})); break;
                case 2: in.push_back(polygon({c, c + Vec2{s, 0}, c + Vec2{0, s}})); break;
                default: {
                    Contour open = polygon({c, c + Vec2{s, 0}, c + Vec2{0, s}});
                    open.closed = false;
                    in.push_back(open);
                }
            }
        }
        DrillParams p;
        p.startDepth = std::floor(u(rng) * 4) * 0.5;
        p.cutDepth = 0.1 + u(rng) * 15;
        p.safeZ = -p.startDepth + 0.5 + u(rng) * 10;
        p.peck = u(rng) < 0.7;
        p.peckRetract = u(rng) < 0.5 ? PeckRetract::AboveStart : PeckRetract::AbovePreviousPass;
        p.retractGap = 0.05 + u(rng) * 4;
        p.dwellTime = u(rng) < 0.5 ? 0.0 : 0.05 + u(rng) * 2;
        p.order.leftToRight = u(rng) < 0.3;
        const tooling::Tool tool = u(rng) < 0.5 ? drillTool(1 + u(rng) * 8, 0.3 + u(rng) * 5) : endMill(1 + u(rng) * 8);
        INFO("case " << n);
        const DrillResult r = drillToolpath(in, tool, std::nullopt, p);
        CHECK(moveProblem(r, p).empty());
        CHECK(r.ok() == !r.holes.empty());
        if (!r.ok()) {
            CHECK_FALSE(r.diagnostics.empty());
        }
        // Determinism.
        const DrillResult again = drillToolpath(in, tool, std::nullopt, p);
        CHECK(again.toolpath == r.toolpath);
        CHECK(again.diagnostics == r.diagnostics);
    }
}

TEST_CASE("Drilling many holes scales linearly", "[toolpath][drill][scale]") {
    // 20,000 holes on a grid (a page of hole markers), nearest-neighbour order.
    std::vector<Contour> in;
    for (int i = 0; i < 20000; ++i) {
        in.push_back(Contour::circle({5.0 * (i % 200), 5.0 * (i / 200)}, 1.0));
    }
    DrillParams p = params(3.0);
    p.peck = true;
    const DrillResult r = drillToolpath(in, drillTool(2.0, 1.0), std::nullopt, p);
    CHECK(r.holes.size() == in.size());
    CHECK(r.diagnostics.empty());
    CHECK(moveProblem(r, p).empty());
}
