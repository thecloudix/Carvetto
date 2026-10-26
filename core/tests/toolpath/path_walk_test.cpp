// Shared toolpath machinery added by T-0120: arc-length paths, height profiles, legs, clearance,
// tab placement, leads, ramps and cut ordering.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <variant>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/toolpath/clearance.hpp"
#include "carvetto/toolpath/cut_order.hpp"
#include "carvetto/toolpath/lead_moves.hpp"
#include "carvetto/toolpath/path_walk.hpp"
#include "carvetto/toolpath/ramp_entry.hpp"
#include "carvetto/toolpath/tab_placement.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using carvetto::geometry::ArcSpan;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::LineSpan;
using carvetto::geometry::Span;
using carvetto::geometry::Vec2;
using Catch::Approx;

TEST_CASE("ArcLengthPath: points, wrapping and exact extraction", "[toolpath][path_walk]") {
    const ArcLengthPath sq(Contour::rectangle(Box2{{0, 0}, {10, 10}}), geometry::Tolerance::defaults());
    CHECK(sq.length() == 40.0);
    CHECK(sq.pointAt(15.0) == Vec2{10, 5});
    CHECK(sq.pointAt(55.0) == Vec2{10, 5});
    CHECK(sq.pointAt(-5.0) == Vec2{0, 5});
    // Across the closing junction, longer than one revolution.
    const std::vector<Span> s = sq.extract(35.0, 85.0);
    double len = 0.0;
    for (const Span& x : s) len += geometry::length(x);
    CHECK(len == Approx(50.0));
    CHECK(geometry::approxEqual(geometry::startPoint(s.front()), Vec2{0, 5}));
    CHECK(geometry::approxEqual(geometry::endPoint(s.back()), Vec2{5, 0}));
    // Backwards.
    const std::vector<Span> b = sq.extract(15.0, 5.0);
    REQUIRE(b.size() == 2);
    CHECK(geometry::startPoint(b.front()) == Vec2{10, 5});
    CHECK(geometry::endPoint(b.back()) == Vec2{5, 0});
    CHECK(sq.project({12, 7}) == Approx(17.0));
    CHECK(sq.kinks(0.1).size() == 4);

    const ArcLengthPath circ(Contour::circle({0, 0}, 2.0), geometry::Tolerance::defaults());
    const std::vector<Span> half = circ.extract(0.0, geometry::kPi * 2.0);
    REQUIRE(half.size() == 1);
    const auto& arc = std::get<ArcSpan>(half.front());
    CHECK(arc.sweepAngle() == Approx(geometry::kPi));
    const std::vector<Span> whole = circ.extract(1.0, 1.0 + circ.length());
    REQUIRE(whole.size() == 1);
    CHECK(std::get<ArcSpan>(whole.front()).isFullCircle());
    CHECK(circ.kinks(1e-3).empty());

    Contour open;
    open.append(LineSpan{{0, 0}, {3, 0}});
    const ArcLengthPath op(open, geometry::Tolerance::defaults());
    CHECK(op.pointAt(10.0) == Vec2{3, 0});
    CHECK(op.extract(-1.0, 2.0).size() == 1);
}

TEST_CASE("ZProfile: evaluation and point-wise maximum", "[toolpath][path_walk]") {
    const ZProfile ramp = ZProfile::linear(0.0, -4.0, 8.0);
    CHECK(ramp.after(4.0) == -2.0);
    CHECK(ramp.before(20.0) == -4.0);
    const ZProfile tab{{ZKnot{5.0, kNoCeiling, -3.0}, ZKnot{7.0, -3.0, kNoCeiling}}};
    const ZProfile m = ZProfile::max(ramp, tab);
    CHECK(m.after(2.0) == -1.0);
    CHECK(m.before(5.0) == -2.5);  // ramp is above the tab top before 6 mm
    CHECK(m.after(5.0) == -2.5);
    CHECK(m.after(6.5) == -3.0);  // crossing at s = 6: the tab holds the tool at -3
    CHECK(m.after(7.0) == -3.5);  // jump back down to the ramp
    CHECK(m.before(7.0) == -3.0);
    // Redundant knots are removed.
    CHECK(ZProfile::max(ZProfile::flat(-1.0), ZProfile::flat(-2.0)).knots.size() == 1);
}

TEST_CASE("appendLeg: helical arcs, vertical jumps and inserts", "[toolpath][path_walk]") {
    const ArcLengthPath circ(Contour::circle({0, 0}, 5.0), geometry::Tolerance::defaults());
    Toolpath tp;
    tp.append(Move::rapid({5, 0, 0}));
    const double len = circ.length();
    appendLeg(circ, 0.0, len, ZProfile::linear(0.0, -2.0, len), tp);
    REQUIRE(tp.moves.size() == 2);
    CHECK(tp.moves[1].kind == MoveKind::ArcCCW);
    CHECK(tp.moves[1].to == Vec3{5, 0, -2});

    // Jump up at s = 2, insert at s = 4, on a line.
    Contour line;
    line.append(LineSpan{{0, 0}, {10, 0}});
    const ArcLengthPath lp(line, geometry::Tolerance::defaults());
    Toolpath t2;
    t2.append(Move::rapid({0, 0, -1}));
    const ZProfile z{{ZKnot{2.0, -1.0, 0.0}}};
    appendLeg(lp, 0.0, 10.0, z, t2, {LegInsert{4.0, {Move::feed({4, 1, 0}), Move::feed({4, 0, 0})}}});
    REQUIRE(t2.moves.size() == 7);
    CHECK(t2.moves[1].to == Vec3{2, 0, -1});
    CHECK(t2.moves[2].to == Vec3{2, 0, 0});  // vertical up
    CHECK(t2.moves[3].to == Vec3{4, 0, 0});
    CHECK(t2.moves[4].to == Vec3{4, 1, 0});
    CHECK(t2.moves[5].to == Vec3{4, 0, 0});
    CHECK(t2.moves[6].to == Vec3{10, 0, 0});
    // Backwards with a tiny arc written as a line.
    Contour tiny;
    tiny.append(ArcSpan{{0, 0}, 1.0, 0.0, 0.004, geometry::ArcDirection::CCW});
    const ArcLengthPath tp3(tiny, geometry::Tolerance::defaults());
    Toolpath t3;
    t3.append(Move::rapid({std::cos(0.004), std::sin(0.004), 0}));
    appendLeg(tp3, tp3.length(), 0.0, ZProfile::flat(0.0), t3);
    REQUIRE(t3.moves.size() == 2);
    CHECK(t3.moves[1].kind == MoveKind::Feed);
}

TEST_CASE("Clearance: violations along a path are found and bracketed", "[toolpath][clearance]") {
    Contour line;
    line.append(LineSpan{{0, 0}, {20, 0}});
    const ArcLengthPath lp(line, geometry::Tolerance::defaults());
    const ClearanceField field({Contour::circle({10, 2}, 1.0)});
    CHECK(field.distance({10, 0}) == Approx(1.0));
    // Within 1.5 of the circle: |x - 10| < sqrt(2.5^2 - 2^2) = 1.5.
    const auto v = findViolations(lp, 0.0, 20.0, field, 1.5, 1e-4);
    REQUIRE(v.size() == 1);
    CHECK(v[0].first == Approx(8.5).margin(2e-4));
    CHECK(v[0].second == Approx(11.5).margin(2e-4));
    CHECK(v[0].first <= 8.5);
    CHECK(v[0].second >= 11.5);
    CHECK(isClear(lp, field, 0.9, 1e-4));
}

TEST_CASE("Tabs: equal spacing, constant distance and corner avoidance", "[toolpath][tabs]") {
    const ArcLengthPath sq(Contour::rectangle(Box2{{0, 0}, {40, 10}}), geometry::Tolerance::defaults());
    TabOptions o;
    o.enabled = true;
    o.count = 4;
    CHECK(placeTabs(sq, 2.0, o).centres == std::vector<double>{12.5, 37.5, 62.5, 87.5});
    o.spacing = TabSpacing::ConstantDistance;
    o.distance = 30.0;  // 100 / 30 -> 3 tabs
    CHECK(placeTabs(sq, 2.0, o).centres.size() == 3);
    o.spacing = TabSpacing::ConstantNumber;
    o.count = 1;  // centre at 50: the far corner (40,10)
    o.avoidCorners = true;
    const TabPlacement tp = placeTabs(sq, 4.0, o);
    REQUIRE(tp.centres.size() == 1);
    CHECK(tp.centres[0] == Approx(48.0));  // 2 mm back along the 10 mm edge...
    CHECK(tp.notAvoided == 0);
    // Ceiling over a leg that wraps past the start.
    const ZProfile c = tabCeiling(sq, 90.0, 120.0, {5.0}, 2.0, -1.0, -3.0, false);
    CHECK(c.after(14.5) == -1.0);  // u = 104.5 -> 4.5 on the next revolution
    CHECK(c.after(17.0) == kNoCeiling);
}

TEST_CASE("Leads come from the waste side and join the path", "[toolpath][leads]") {
    LeadOptions o;
    o.type = LeadType::Straight;
    o.length = 4.0;
    o.angle = geometry::kPi / 2;
    const Contour in = leadIn({0, 0}, {1, 0}, /*wasteOnLeft*/ true, o);
    CHECK(in.startPoint().x == Approx(0.0).margin(1e-12));
    CHECK(in.startPoint().y == Approx(4.0));
    CHECK(in.endPoint() == Vec2{0, 0});
    const Contour out = leadOut({0, 0}, {1, 0}, true, o);
    CHECK(out.startPoint() == Vec2{0, 0});
    CHECK(out.endPoint().y == Approx(4.0));
    o.type = LeadType::Circular;
    o.radius = 2.0;
    o.sweep = geometry::kPi / 2;
    const Contour arc = leadIn({0, 0}, {1, 0}, false, o);
    const auto& a = std::get<ArcSpan>(arc.spans.front());
    CHECK(a.centre.y == Approx(-2.0));
    CHECK(a.direction == geometry::ArcDirection::CW);
    CHECK(geometry::distance(a.endPoint(), {0, 0}) < 1e-12);
    CHECK(a.tangentAt(1.0).x == Approx(1.0));
    CHECK(a.startPoint().y < 0.0);
}

TEST_CASE("planPass: ramps on closed and open paths", "[toolpath][ramps]") {
    const ArcLengthPath sq(Contour::rectangle(Box2{{0, 0}, {10, 10}}), geometry::Tolerance::defaults());
    RampOptions r;
    r.type = RampType::Smooth;
    r.distance = 5.0;
    PassLegs p = planPass(sq, 0.0, 0.0, -2.0, r);
    REQUIRE(p.legs.size() == 1);
    CHECK(p.legs[0].u1 == 45.0);
    CHECK(p.endU == 5.0);
    r.type = RampType::ZigZag;
    r.useAngle = true;
    r.angle = std::atan(0.1);  // 0.5 per 5 mm leg -> 4 legs for 2 mm
    p = planPass(sq, 0.0, 0.0, -2.0, r);
    CHECK(p.legs.size() == 5);
    CHECK(p.endU == 0.0);
    Contour open;
    open.append(LineSpan{{0, 0}, {20, 0}});
    const ArcLengthPath op(open, geometry::Tolerance::defaults());
    r.type = RampType::Smooth;
    r.useAngle = false;
    p = planPass(op, 20.0, 0.0, -2.0, r);
    REQUIRE(p.legs.size() == 3);
    CHECK(p.legs[0].u0 == 20.0);
    CHECK(p.legs[0].u1 == 15.0);
    CHECK(p.endU == 0.0);
}

TEST_CASE("orderCuts: containment first, then strategy", "[toolpath][order]") {
    std::vector<OrderItem> items(3);
    items[0].path = Contour::rectangle(Box2{{0, 0}, {50, 50}});
    items[1].path = Contour::rectangle(Box2{{10, 10}, {20, 20}});
    items[2].path = Contour::rectangle(Box2{{60, 0}, {70, 10}});
    for (std::size_t i = 0; i < 3; ++i) {
        items[i].selectionIndex = i;
        items[i].start = items[i].end = items[i].path.startPoint();
    }
    OrderOptions sel;
    sel.selectionOrder = true;
    CHECK(orderCuts(items, sel, {0, 0}, geometry::Tolerance::defaults()) == std::vector<std::size_t>{1, 0, 2});
    OrderOptions ltr;
    ltr.leftToRight = true;
    CHECK(orderCuts(items, ltr, {0, 0}, geometry::Tolerance::defaults()) == std::vector<std::size_t>{1, 0, 2});
    CHECK(travelLength(items, {1, 0, 2}, {0, 0}) == Approx(std::hypot(10, 10) * 2 + 60));
}
