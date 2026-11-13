// Shared toolpath machinery added by T-0120: arc-length paths, height profiles, legs, clearance,
// tab placement, leads, ramps and cut ordering.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <variant>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/fill_rule.hpp"
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

TEST_CASE("appendLeg (T-0156): a raised stretch ending at an insert stays flat to it", "[toolpath][path_walk]") {
    // A tab top at 0 from s = 3 to s = 6 over a floor at -1, and a corner insert exactly at the
    // end of the stretch (or a hair past it, as the corner's position is computed): the stretch
    // is cut flat and the tool goes down at its end, not along all of it.
    Contour line;
    line.append(LineSpan{{0, 0}, {10, 0}});
    const ArcLengthPath lp(line, geometry::Tolerance::defaults());
    const ZProfile z{{ZKnot{3.0, -1.0, 0.0}, ZKnot{6.0, 0.0, -1.0}}};
    for (const double at : {6.0, 6.0 + 4e-10, 6.0 - 4e-10}) {
        INFO("insert at " << at);
        Toolpath tp;
        tp.append(Move::rapid({0, 0, -1}));
        appendLeg(lp, 0.0, 10.0, z, tp, {LegInsert{at, {Move::feed({6, 1, -1}), Move::feed({6, 0, -1})}}});
        // Every move that travels along the line between x = 3 and 6 is at the tab top.
        double lowest = 1.0;
        Vec3 prev{0, 0, -1};
        for (const Move& m : tp.moves) {
            if (m.kind == MoveKind::Feed && prev.x >= 3.0 - 1e-6 && m.to.x <= 6.0 + 1e-6 && m.to.x > prev.x + 1e-6) {
                lowest = std::min({lowest, prev.z, m.to.z});
            }
            prev = m.to;
        }
        CHECK(lowest == Approx(0.0));
        CHECK(tp.moves.back().to == Vec3{10, 0, -1});
    }
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

TEST_CASE("Tabs moved off a keep-clear point are kept, never dropped by rounding", "[toolpath][tabs]") {
    // T-0135 (g): fitTabs moves a centre within `reach` of a keep-clear point to exactly `reach`
    // from it and then re-checks |c - f| < reach, which (f + reach) - f decided by rounding: about
    // half of these tabs were dropped. Now the moved tab is always kept, on either side.
    Contour line;
    line.append(LineSpan{{0, 0}, {100, 0}});
    const ArcLengthPath lp(line, geometry::Tolerance::defaults());
    const double footprint = 2.0;
    const double reach = 0.5 * footprint + kMinTabGap;
    std::size_t kept = 0;
    std::size_t exact = 0;
    const int n = 2000;
    for (int k = 0; k < n; ++k) {
        const double f = 10.0 + k * 0.0371234;
        for (const double offset : {0.3, -0.3}) {
            const TabPlacement tp = fitTabs(lp, {f + offset}, footprint, {f});
            REQUIRE(tp.centres.size() + tp.dropped == 1);
            if (tp.centres.size() == 1) {
                ++kept;
                CHECK(std::fabs(std::fabs(tp.centres.front() - f) - reach) < 1e-9);
                if (std::fabs(tp.centres.front() - f) < reach) ++exact;
            }
        }
    }
    CHECK(kept == 2 * static_cast<std::size_t>(n));
    CHECK(exact > 0);  // some moved centres really do come out a few ulps short
    // With a move margin (room for another pass's cover) the moved tab keeps that much more.
    for (const double offset : {0.3, -0.3}) {
        const TabPlacement tp = fitTabs(lp, {40.0 + offset}, footprint, {40.0}, kMinTabGap, 0.012);
        REQUIRE(tp.centres.size() == 1);
        CHECK(std::fabs(std::fabs(tp.centres.front() - 40.0) - (reach + 0.012)) < 1e-12);
    }
    // T-0135 round 2 (verifier gfit2): tabs are fitted in order along the path, moved ones too, as
    // T-0127 did. Footprint 4, gap 3, keep-clear 30, asked {29.5, 33, 36}: 29.5 moves to 27.999,
    // 33 overlaps it and is dropped, 36 is kept (a rule fitting moved tabs last kept only 33).
    {
        const TabPlacement tp = fitTabs(lp, {29.5, 33.0, 36.0}, 4.0, {30.0}, 3.0);
        REQUIRE(tp.centres.size() == 2);
        CHECK(tp.centres[0] == Approx(27.999).margin(1e-9));
        CHECK(tp.centres[1] == 36.0);
        CHECK(tp.dropped == 1);
        REQUIRE(tp.origin.size() == 2);
        CHECK(tp.origin[0] == 0);
        CHECK(tp.origin[1] == 2);
        Contour sq = Contour::rectangle(Box2{{0, 0}, {25, 25}});
        const ArcLengthPath sp(sq, geometry::Tolerance::defaults());
        const TabPlacement ts = fitTabs(sp, {29.5, 33.0, 36.0}, 4.0, {30.0}, 3.0);
        CHECK(ts.centres.size() == 2);
    }
    // A centre clearly inside the gap with no room to move (both sides blocked) is still dropped.
    const TabPlacement blocked = fitTabs(lp, {50.0}, footprint, {49.5, 51.0});
    CHECK(blocked.centres.empty());
    CHECK(blocked.dropped == 1);
}

TEST_CASE("T-0159: a negative clear slack refuses every tab moved to the gap, deterministically",
          "[toolpath][tabs]") {
    // Profile's T-0127 candidate is placed both ways (ADR-0016): a tab moved to exactly the gap
    // accepted (kKeepClearSlack) or refused (-kKeepClearSlack), instead of rounding deciding.
    Contour line;
    line.append(LineSpan{{0, 0}, {100, 0}});
    const ArcLengthPath lp(line, geometry::Tolerance::defaults());
    const double footprint = 2.0;
    for (int k = 0; k < 2000; ++k) {
        const double f = 10.0 + k * 0.0371234;
        for (const double offset : {0.3, -0.3}) {
            const TabPlacement refused = fitTabs(lp, {f + offset}, footprint, {f}, kMinTabGap, 0.0, -kKeepClearSlack);
            CHECK(refused.centres.empty());
            CHECK(refused.dropped == 1);
            const TabPlacement accepted = fitTabs(lp, {f + offset}, footprint, {f}, kMinTabGap, 0.0, kKeepClearSlack);
            CHECK(accepted.centres.size() == 1);
        }
    }
    // A tab clear of the point by more than the slack is kept either way.
    const TabPlacement clear = fitTabs(lp, {40.0}, footprint, {38.0}, kMinTabGap, 0.0, -kKeepClearSlack);
    CHECK(clear.centres.size() == 1);
}

namespace {

// Reference for T-0267: the quadratic scans orderCuts used before (containment over every closed
// item, nearest neighbour over every sibling), with selection order or nearest-first strategy.
void refWalk(std::size_t node, const std::vector<std::vector<std::size_t>>& tree, const std::vector<OrderItem>& items,
             bool nearest, Vec2& pos, std::vector<std::size_t>& out) {
    std::vector<std::size_t> kids = tree[node];
    std::sort(kids.begin(), kids.end(), [&](std::size_t a, std::size_t b) {
        return items[a].selectionIndex != items[b].selectionIndex ? items[a].selectionIndex < items[b].selectionIndex
                                                                  : a < b;
    });
    if (nearest) {
        std::vector<bool> used(kids.size(), false);
        for (std::size_t n = 0; n < kids.size(); ++n) {
            std::size_t best = kids.size();
            double bestD = 1e300;
            for (std::size_t k = 0; k < kids.size(); ++k) {
                if (used[k]) continue;
                const double d = geometry::distance(pos, items[kids[k]].start);
                if (d < bestD) {
                    bestD = d;
                    best = k;
                }
            }
            used[best] = true;
            refWalk(kids[best] + 1, tree, items, nearest, pos, out);
        }
    } else {
        for (std::size_t k : kids) refWalk(k + 1, tree, items, nearest, pos, out);
    }
    if (node > 0) {
        out.push_back(node - 1);
        pos = items[node - 1].end;
    }
}

std::vector<std::size_t> refOrder(const std::vector<OrderItem>& items, bool nearest, Vec2 from) {
    const std::size_t n = items.size();
    std::vector<std::vector<std::size_t>> tree(n + 1);
    const auto tol = geometry::Tolerance::defaults();
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 probe = geometry::pointAt(items[i].path.spans.front(), 0.5);
        std::size_t parent = n;
        double parentArea = 1e300;
        for (std::size_t j = 0; j < n; ++j) {
            if (j == i || !items[j].path.closed) continue;
            const double aj = geometry::area(items[j].path);
            const double ai = items[i].path.closed ? geometry::area(items[i].path) : 0.0;
            if (aj <= ai || aj >= parentArea) continue;
            if (geometry::locatePoint(items[j].path, probe, geometry::FillRule::EvenOdd, tol) ==
                geometry::PointLocation::Inside) {
                parent = j;
                parentArea = aj;
            }
        }
        tree[parent == n ? 0 : parent + 1].push_back(i);
    }
    std::vector<std::size_t> out;
    Vec2 pos = from;
    refWalk(0, tree, items, nearest, pos, out);
    return out;
}

OrderItem lineItem(Vec2 a, Vec2 b, std::size_t sel) {
    OrderItem it;
    it.path.append(LineSpan{a, b});
    it.selectionIndex = sel;
    it.start = a;
    it.end = b;
    return it;
}

OrderItem boxItem(double x0, double y0, double x1, double y1, std::size_t sel) {
    OrderItem it;
    it.path = Contour::rectangle(Box2{{x0, y0}, {x1, y1}});
    it.selectionIndex = sel;
    it.start = it.end = it.path.startPoint();
    return it;
}

}  // namespace

TEST_CASE("orderCuts: same order as the exhaustive scans on scenes full of coincident vectors",
          "[toolpath][order][T-0267]") {
    std::uint32_t seed = 12345;
    auto rnd = [&](std::uint32_t m) {
        seed = seed * 1664525u + 1013904223u;
        return (seed >> 8) % m;
    };
    for (int scene = 0; scene < 60; ++scene) {
        std::vector<OrderItem> items;
        const std::size_t count = 5 + rnd(40);
        for (std::size_t i = 0; i < count; ++i) {
            const double x = static_cast<double>(rnd(6)) * 10.0;
            const double y = static_cast<double>(rnd(6)) * 10.0;
            const std::size_t sel = rnd(4) == 0 ? rnd(8) : i;  // some equal selection indices
            switch (rnd(3)) {
                case 0:
                    items.push_back(lineItem({x, y}, {x + 5.0 * static_cast<double>(rnd(3)), y + 3.0}, sel));
                    break;
                case 1: {
                    const double s = 4.0 + 8.0 * static_cast<double>(rnd(4));  // sizes repeat: equal areas
                    items.push_back(boxItem(x, y, x + s, y + s, sel));
                    break;
                }
                default:
                    items.push_back(boxItem(x, y, x + 20.0, y + 20.0, sel));
                    break;
            }
        }
        for (const bool nearest : {true, false}) {
            OrderOptions o;
            o.selectionOrder = !nearest;
            const Vec2 from{static_cast<double>(rnd(60)), static_cast<double>(rnd(60))};
            CHECK(orderCuts(items, o, from, geometry::Tolerance::defaults()) == refOrder(items, nearest, from));
        }
    }
}

TEST_CASE("orderCuts: many coincident vectors cost near-linear work", "[toolpath][order][T-0267]") {
    // The work is counted, not timed: before T-0267 n identical vectors took n^2/2 distance
    // evaluations (nearest) and n^2 candidate checks (containment).
    const std::size_t n = 20000;
    const auto tol = geometry::Tolerance::defaults();
    {
        std::vector<OrderItem> items;
        for (std::size_t i = 0; i < n; ++i) items.push_back(lineItem({5, 5}, {5, 5}, i));
        OrderStats st;
        const std::vector<std::size_t> order = orderCuts(items, OrderOptions{}, {0, 0}, tol, &st);
        REQUIRE(order.size() == n);
        for (std::size_t i = 0; i < n; ++i) REQUIRE(order[i] == i);  // ties keep selection order
        CHECK(st.distanceEvals <= 4 * n);
    }
    {
        std::vector<OrderItem> items;
        for (std::size_t i = 0; i < n; ++i) items.push_back(boxItem(0, 0, 10, 10, i));
        OrderStats st;
        const std::vector<std::size_t> order = orderCuts(items, OrderOptions{}, {0, 0}, tol, &st);
        REQUIRE(order.size() == n);
        CHECK(st.distanceEvals <= 4 * n);
        CHECK(st.containmentTests <= 4 * n);
    }
    {
        // Identical big loops around n identical small ones: one containment test per group.
        std::vector<OrderItem> items;
        for (std::size_t i = 0; i < n / 2; ++i) items.push_back(boxItem(0, 0, 100, 100, i));
        for (std::size_t i = 0; i < n / 2; ++i) items.push_back(boxItem(10, 10, 20, 20, n / 2 + i));
        OrderStats st;
        const std::vector<std::size_t> order = orderCuts(items, OrderOptions{}, {0, 0}, tol, &st);
        REQUIRE(order.size() == n);
        CHECK(order.front() == n / 2);  // inner loops first, in selection order
        CHECK(order[n / 2] == 0);
        CHECK(st.containmentTests <= 4 * n);
        CHECK(st.distanceEvals <= 4 * n);
    }
}
