// 2D Profile toolpath (T-0120): acceptance criteria 1-5 and the Goal features. The golden files
// (AC1, AC6 G-code) are in profile_golden_test.cpp; the randomized gouge test in
// profile_property_test.cpp.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <functional>
#include <iterator>
#include <sstream>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "../geometry/test_rng.hpp"
#include "profile_test_util.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using carvetto::geometry::ArcSpan;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::LineSpan;
using carvetto::geometry::Vec2;
using carvetto::geometry::Vec3;
using Catch::Approx;

namespace {

ProfileParams baseParams(double cutDepth = 12.0, double passDepth = 4.0) {
    ProfileParams p;
    p.passes.cutDepth = cutDepth;
    p.passes.passDepth = passDepth;
    p.safeZ = 5.0;
    return p;
}

/// Splits a toolpath at its rapids into one toolpath per cut (each starting with its rapid).
std::vector<Toolpath> splitCuts(const Toolpath& tp) {
    std::vector<Toolpath> cuts;
    for (const Move& m : tp.moves) {
        if (m.kind == MoveKind::Rapid) {
            cuts.emplace_back();
        }
        if (!cuts.empty()) {
            cuts.back().moves.push_back(m);
        }
    }
    return cuts;
}

/// Signed area (shoelace) of the XY trajectory of a cut's feed/arc samples at depth z.
double trajectoryArea(const Toolpath& cut) {
    std::vector<Vec2> pts;
    test::sampleCuttingMoves(cut, 0.5, [&](Vec2 p) { pts.push_back(p); });
    double a = 0.0;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        const Vec2 p = pts[i];
        const Vec2 q = pts[(i + 1) % pts.size()];
        a += p.x * q.y - q.x * p.y;
    }
    return 0.5 * a;
}

/// Min and max distance of the samples of one cut's moves at z == level to the contours.
std::pair<double, double> distanceRange(const Toolpath& cut, const std::vector<Contour>& cs) {
    double lo = 1e300;
    double hi = 0.0;
    test::sampleCuttingMoves(cut, 0.1, [&](Vec2 p) {
        const double d = test::bruteDistance(p, cs);
        lo = std::min(lo, d);
        hi = std::max(hi, d);
    });
    return {lo, hi};
}

/// XY distance travelled (arcs by arc length) from the first move that leaves `zFrom` downwards
/// until Z first equals `level`; -1 when never reached.
double xyTravelToLevel(const Toolpath& tp, double zFrom, double level) {
    bool started = false;
    double travelled = 0.0;
    for (std::size_t i = 1; i < tp.moves.size(); ++i) {
        const Move& m = tp.moves[i];
        const Vec3 from = tp.moves[i - 1].to;
        if (!started && from.z == zFrom && m.to.z < from.z && m.kind != MoveKind::Plunge) {
            started = true;
        }
        if (started) {
            const double dz = m.to.z - from.z;
            const double l = moveLength(m, from);
            travelled += std::sqrt(std::max(0.0, l * l - dz * dz));
            if (m.to.z == level) {
                return travelled;
            }
        }
    }
    return -1.0;
}

Contour square(Vec2 c, double side) {
    return Contour::rectangle(Box2{{c.x - side / 2, c.y - side / 2}, {c.x + side / 2, c.y + side / 2}});
}

}  // namespace

// ----------------------------------------------------------------------------------- AC1

TEST_CASE("AC1: outside profile of a 100x50 rectangle, 3 passes, 2 tabs", "[toolpath][profile]") {
    ProfileParams p = baseParams();
    p.tabs.enabled = true;
    p.tabs.count = 2;
    p.tabs.length = 10.0;
    p.tabs.thickness = 2.0;
    const Contour rect = Contour::rectangle(Box2{{0, 0}, {100, 50}});
    const ProfileResult r = profileToolpath({rect}, test::endMill(6.0), p);
    REQUIRE(r.ok());
    CHECK(r.diagnostics.empty());
    const Toolpath& tp = r.toolpath;
    CHECK(tp.count(MoveKind::Rapid) == 1);
    CHECK(tp.count(MoveKind::Retract) == 1);
    CHECK(tp.minZ() == Approx(-12.0));
    CHECK(test::feedLevels(tp) == std::vector<double>{-4.0, -8.0, -10.0, -12.0});

    // Tab stretches: maximal runs of feed moves at Z = -(12 - 2) = -10, only in the last pass.
    std::vector<double> tabLengths;
    for (std::size_t i = 1; i < tp.moves.size(); ++i) {
        const Move& m = tp.moves[i];
        const Vec3 from = tp.moves[i - 1].to;
        if (m.kind == MoveKind::Feed && from.z == -10.0 && m.to.z == -10.0) {
            tabLengths.push_back(geometry::distance(from.xy(), m.to.xy()));
            // The tool rose from -12 straight up and goes back down to -12 after the tab.
            CHECK(tp.moves[i - 1].kind == MoveKind::Feed);
            CHECK(tp.moves[i - 2].to.z == -12.0);
            CHECK(tp.moves[i + 1].kind == MoveKind::Plunge);
            CHECK(tp.moves[i + 1].to.z == -12.0);
        }
    }
    // Raised stretch = tab length + tool diameter, so 10 mm of material remains.
    REQUIRE(tabLengths.size() == 2);
    CHECK(tabLengths[0] == Approx(16.0).margin(1e-9));
    CHECK(tabLengths[1] == Approx(16.0).margin(1e-9));
    // Every cutting move stays exactly 3 mm from the rectangle (outside, round corners).
    const auto [lo, hi] = distanceRange(tp, {rect});
    CHECK(lo == Approx(3.0).margin(1e-9));
    CHECK(hi == Approx(3.0).margin(1e-9));
}

// ----------------------------------------------------------------------------------- AC2

TEST_CASE("AC2: inside profile of a circle r=10 with a 6 mm tool is an r=7 arc", "[toolpath][profile]") {
    ProfileParams p = baseParams(3.0, 3.0);
    p.side = ProfileSide::Inside;
    const ProfileResult r = profileToolpath({Contour::circle({20, 30}, 10.0)}, test::endMill(6.0), p);
    REQUIRE(r.ok());
    const Toolpath& tp = r.toolpath;
    std::size_t arcs = 0;
    for (std::size_t i = 0; i < tp.moves.size(); ++i) {
        const Move& m = tp.moves[i];
        CHECK(m.kind != MoveKind::Feed);  // no line moves at all
        if (isArc(m.kind)) {
            ++arcs;
            CHECK(m.centre == Vec2{20, 30});
            CHECK(arcRadius(m, tp.moves[i - 1].to.xy()) == Approx(7.0).margin(1e-12));
            CHECK(std::fabs(arcSweep(m, tp.moves[i - 1].to.xy())) == Approx(geometry::kTwoPi));
        }
    }
    CHECK(arcs == 1);
    CHECK(tp.moves.size() == 4);  // rapid, plunge, full circle, retract
}

TEST_CASE("AC2: inside profile of a circle r=2.9 with a 6 mm tool yields no toolpath and a reason",
          "[toolpath][profile]") {
    ProfileParams p = baseParams(3.0, 3.0);
    p.side = ProfileSide::Inside;
    const ProfileResult r = profileToolpath({Contour::circle({0, 0}, 2.9)}, test::endMill(6.0), p);
    CHECK_FALSE(r.ok());
    CHECK(r.toolpath.isEmpty());
    REQUIRE(r.has(ProfileIssue::ToolTooLarge));
    CHECK(r.diagnostics.front().contour == std::size_t{0});
    CHECK_FALSE(r.diagnostics.front().message.empty());
}

// ----------------------------------------------------------------------------------- AC3

TEST_CASE("AC3: climb and conventional run in opposite directions", "[toolpath][profile]") {
    const Contour sq = Contour::rectangle(Box2{{0, 0}, {40, 40}});
    for (ProfileSide side : {ProfileSide::Outside, ProfileSide::Inside}) {
        ProfileParams p = baseParams(3.0, 3.0);
        p.side = side;
        p.direction = CutDirection::Climb;
        const double climb = trajectoryArea(profileToolpath({sq}, test::endMill(), p).toolpath);
        p.direction = CutDirection::Conventional;
        const double conv = trajectoryArea(profileToolpath({sq}, test::endMill(), p).toolpath);
        INFO("side " << static_cast<int>(side));
        CHECK(climb == Approx(-conv).epsilon(1e-6));
        // Clockwise spindle: climb keeps the vector on the right -> CW outside, CCW inside.
        if (side == ProfileSide::Outside) {
            CHECK(climb < 0.0);
        } else {
            CHECK(climb > 0.0);
        }
    }
    // A clockwise input vector gives the same result as a counter-clockwise one.
    ProfileParams p = baseParams(3.0, 3.0);
    const Toolpath a = profileToolpath({sq}, test::endMill(), p).toolpath;
    const Toolpath b = profileToolpath({sq.reversed()}, test::endMill(), p).toolpath;
    CHECK(trajectoryArea(a) == Approx(trajectoryArea(b)));
}

TEST_CASE("AC3: separate last pass with 0.2 mm allowance gives two loops at r+0.2 and r",
          "[toolpath][profile]") {
    const Contour sq = Contour::rectangle(Box2{{0, 0}, {40, 40}});
    for (bool reverse : {false, true}) {
        ProfileParams p = baseParams(6.0, 3.0);
        p.separateLastPass = true;
        p.lastPassAllowance = 0.2;
        p.lastPassReverse = reverse;
        const ProfileResult r = profileToolpath({sq}, test::endMill(6.0), p);
        REQUIRE(r.ok());
        const std::vector<Toolpath> cuts = splitCuts(r.toolpath);
        REQUIRE(cuts.size() == 2);
        const auto [rlo, rhi] = distanceRange(cuts[0], {sq});
        const auto [flo, fhi] = distanceRange(cuts[1], {sq});
        CHECK(rlo == Approx(3.2).margin(1e-9));
        CHECK(rhi == Approx(3.2).margin(1e-9));
        CHECK(flo == Approx(3.0).margin(1e-9));
        CHECK(fhi == Approx(3.0).margin(1e-9));
        // Roughing at both levels, finishing once at full depth.
        CHECK(test::feedLevels(cuts[0]) == std::vector<double>{-3.0, -6.0});
        CHECK(test::feedLevels(cuts[1]) == std::vector<double>{-6.0});
        const double ra = trajectoryArea(cuts[0]);
        const double fa = trajectoryArea(cuts[1]);
        const bool same = (ra < 0.0) == (fa < 0.0);
        CHECK(same == !reverse);
    }
}

TEST_CASE("Allowance offset moves the cut away from (or into) the vector", "[toolpath][profile]") {
    const Contour sq = Contour::rectangle(Box2{{0, 0}, {40, 40}});
    for (double a : {0.5, -0.5}) {
        ProfileParams p = baseParams(3.0, 3.0);
        p.allowance = a;
        const auto [lo, hi] = distanceRange(profileToolpath({sq}, test::endMill(6.0), p).toolpath, {sq});
        CHECK(lo == Approx(3.0 + a).margin(1e-9));
        CHECK(hi == Approx(3.0 + a).margin(1e-9));
    }
}

// ----------------------------------------------------------------------------------- AC4

TEST_CASE("AC4: smooth ramp over 20 mm reaches the pass depth exactly 20 mm along the path",
          "[toolpath][profile]") {
    const Contour rect = Contour::rectangle(Box2{{0, 0}, {100, 50}});
    ProfileParams p = baseParams(8.0, 4.0);
    p.ramp.type = RampType::Smooth;
    p.ramp.distance = 20.0;
    const Toolpath tp = profileToolpath({rect}, test::endMill(6.0), p).toolpath;
    REQUIRE(tp.moves.size() > 3);
    CHECK(tp.moves[1].kind == MoveKind::Plunge);
    CHECK(tp.moves[1].to.z == 0.0);  // plunge only to the material surface
    // Travel along the path from where each ramp leaves the previous level.
    CHECK(xyTravelToLevel(tp, 0.0, -4.0) == Approx(20.0).margin(1e-9));
    CHECK(xyTravelToLevel(tp, -4.0, -8.0) == Approx(20.0).margin(1e-9));
    // Never plunges below the surface vertically: the only plunge is the first one.
    CHECK(tp.count(MoveKind::Plunge) == 1);
}

TEST_CASE("AC4: smooth ramp by angle uses drop / tan(angle)", "[toolpath][profile]") {
    ProfileParams p = baseParams(4.0, 4.0);
    p.ramp.type = RampType::Smooth;
    p.ramp.useAngle = true;
    p.ramp.angle = std::atan(0.1);  // 4 mm drop over 40 mm
    const Toolpath tp = profileToolpath({Contour::rectangle(Box2{{0, 0}, {100, 50}})}, test::endMill(), p).toolpath;
    CHECK(xyTravelToLevel(tp, 0.0, -4.0) == Approx(40.0).margin(1e-9));
}

TEST_CASE("AC4: spiral ramp on a closed contour descends one pass depth per revolution", "[toolpath][profile]") {
    ProfileParams p = baseParams(12.0, 4.0);
    p.ramp.type = RampType::Spiral;
    const Toolpath tp = profileToolpath({Contour::circle({0, 0}, 20.0)}, test::endMill(6.0), p).toolpath;
    // rapid, plunge to surface, 3 helical revolutions, 1 flat revolution, retract.
    REQUIRE(tp.moves.size() == 7);
    CHECK(tp.moves[1].kind == MoveKind::Plunge);
    CHECK(tp.moves[1].to.z == 0.0);
    const double expected[] = {-4.0, -8.0, -12.0, -12.0};
    for (int k = 0; k < 4; ++k) {
        const Move& m = tp.moves[static_cast<std::size_t>(2 + k)];
        const Vec3 from = tp.moves[static_cast<std::size_t>(1 + k)].to;
        REQUIRE(isArc(m.kind));
        CHECK(m.to.xy() == from.xy());  // full revolution
        CHECK(arcRadius(m, from.xy()) == Approx(23.0));
        CHECK(m.to.z == expected[k]);
    }
}

TEST_CASE("Zig-zag ramp walks back and forth and returns to the start before the pass", "[toolpath][profile]") {
    ProfileParams p = baseParams(4.0, 4.0);
    p.ramp.type = RampType::ZigZag;
    p.ramp.distance = 10.0;
    const Contour rect = Contour::rectangle(Box2{{0, 0}, {100, 50}});
    const Toolpath tp = profileToolpath({rect}, test::endMill(6.0), p).toolpath;
    const Vec2 start = tp.moves[0].to.xy();
    // After the zig-zag the tool is back at the start at the pass level, then cuts the loop.
    bool found = false;
    for (std::size_t i = 2; i < tp.moves.size(); ++i) {
        if (tp.moves[i].to.z == -4.0) {
            CHECK(geometry::distance(tp.moves[i].to.xy(), start) < 1e-9);
            found = true;
            // Halfway down at the far end of the first leg (-2 after 10 mm).
            break;
        }
        CHECK(tp.moves[i].to.z >= -4.0);
    }
    CHECK(found);
    CHECK(tp.count(MoveKind::Plunge) == 1);
    CHECK(test::minCutDistance(tp, {rect}) == Approx(3.0).margin(1e-9));
}

TEST_CASE("AC4: leads start off the material and keep the tool radius from every vector", "[toolpath][profile]") {
    const Contour sq = Contour::rectangle(Box2{{0, 0}, {40, 40}});
    for (LeadType type : {LeadType::Straight, LeadType::Circular}) {
        ProfileParams p = baseParams(3.0, 3.0);
        p.leads.type = type;
        p.leads.length = 8.0;
        p.leads.radius = 6.0;
        p.leads.leadOut = true;
        p.leads.overcut = 3.0;
        const ProfileResult r = profileToolpath({sq}, test::endMill(6.0), p);
        REQUIRE(r.ok());
        const Toolpath& tp = r.toolpath;
        const Vec2 entry = tp.moves[0].to.xy();
        const Vec2 exit = tp.moves.back().to.xy();
        INFO("lead type " << static_cast<int>(type));
        // Entry and exit lie in the waste, away from the 3 mm cut line.
        CHECK(test::bruteDistance(entry, {sq}) > 3.5);
        CHECK(test::bruteDistance(exit, {sq}) > 3.5);
        CHECK_FALSE(sq.bounds().contains(entry));
        CHECK(test::minCutDistance(tp, {sq}) >= 3.0 - 1e-9);
    }
    // A neighbouring vector in the way: the lead is shortened (or dropped), never gouges it.
    const Contour near = Contour::rectangle(Box2{{-14, 15}, {-9, 25}});  // across the lead's line
    ProfileParams p = baseParams(3.0, 3.0);
    p.leads.type = LeadType::Straight;
    p.leads.length = 12.0;
    p.leads.angle = geometry::kPi / 2.0;
    p.startAt = StartAt::ClosestOnBoundingBox;
    p.startAnchor = BoxAnchor::CentreLeft;
    const ProfileResult r = profileToolpath({sq, near}, test::endMill(6.0), p);
    REQUIRE(r.ok());
    CHECK((r.has(ProfileIssue::LeadShortened) || r.has(ProfileIssue::LeadDropped)));
    CHECK(test::minCutDistance(r.toolpath, {sq, near}) >= 3.0 - 1e-3);
}

// ----------------------------------------------------------------------------------- AC5

namespace {

// 3x3 grid of 10 mm squares at a 30 mm pitch; input (selection) order is deliberately shuffled.
const int kShuffle[9] = {4, 8, 0, 6, 2, 7, 1, 5, 3};

std::vector<Contour> grid3x3() {
    std::vector<Contour> cs;
    for (int k : kShuffle) {
        cs.push_back(square({15.0 + 30.0 * (k % 3), 15.0 + 30.0 * (k / 3)}, 10.0));
    }
    return cs;
}

/// Grid cell (col + 3 * row) of each cut, in machining order.
std::vector<int> cutCells(const Toolpath& tp) {
    std::vector<int> cells;
    for (const Toolpath& c : splitCuts(tp)) {
        const Vec2 p = c.moves.front().to.xy();
        const int col = static_cast<int>(std::floor(p.x / 30.0));
        const int row = static_cast<int>(std::floor(p.y / 30.0));
        cells.push_back(col + 3 * row);
    }
    return cells;
}

std::vector<int> orderFor(OrderOptions o) {
    ProfileParams p = baseParams(3.0, 3.0);
    p.order = o;
    return cutCells(profileToolpath(grid3x3(), test::endMill(6.0), p).toolpath);
}

}  // namespace

TEST_CASE("AC5: ordering modes on a 3x3 grid of squares", "[toolpath][profile]") {
    OrderOptions sel;
    sel.selectionOrder = true;
    CHECK(orderFor(sel) == std::vector<int>(std::begin(kShuffle), std::end(kShuffle)));

    OrderOptions ltr;
    ltr.leftToRight = true;  // column by column, bottom to top
    CHECK(orderFor(ltr) == std::vector<int>{0, 3, 6, 1, 4, 7, 2, 5, 8});

    OrderOptions btt;
    btt.bottomToTop = true;  // row by row, left to right
    CHECK(orderFor(btt) == std::vector<int>{0, 1, 2, 3, 4, 5, 6, 7, 8});

    OrderOptions grid;
    grid.grid = true;  // serpentine rows
    CHECK(orderFor(grid) == std::vector<int>{0, 1, 2, 5, 4, 3, 6, 7, 8});

    // Nothing ticked: greedy nearest neighbour from the start position (0, 0).
    const std::vector<int> nearest = orderFor(OrderOptions{});
    CHECK(nearest.front() == 0);
    CHECK(nearest.size() == 9);

    // Several ticked: the one with the least travel wins (grid beats selection order).
    OrderOptions both;
    both.selectionOrder = true;
    both.grid = true;
    CHECK(orderFor(both) == orderFor(grid));
}

TEST_CASE("AC5: inner vectors are always cut before the vectors containing them", "[toolpath][profile]") {
    // Outer 100x100, a 30x30 hole, a 10x10 island inside the hole, and a separate hole.
    const std::vector<Contour> cs{square({50, 50}, 100.0), square({30, 30}, 30.0), square({30, 30}, 10.0),
                                  square({75, 75}, 20.0)};
    for (int mode = 0; mode < 4; ++mode) {
        ProfileParams p = baseParams(3.0, 3.0);
        p.order.selectionOrder = mode == 0;
        p.order.leftToRight = mode == 1;
        p.order.grid = mode == 2;
        const ProfileResult r = profileToolpath(cs, test::endMill(6.0), p);
        REQUIRE(r.ok());
        const std::vector<Toolpath> cuts = splitCuts(r.toolpath);
        REQUIRE(cuts.size() == 4);
        // Identify each cut by its distance range to each input vector.
        auto which = [&](const Toolpath& c) {
            for (std::size_t i = 0; i < cs.size(); ++i) {
                const auto [lo, hi] = distanceRange(c, {cs[i]});
                if (std::fabs(lo - 3.0) < 1e-6 && std::fabs(hi - 3.0) < 1e-6) {
                    return static_cast<int>(i);
                }
            }
            return -1;
        };
        std::vector<int> ids;
        for (const Toolpath& c : cuts) {
            ids.push_back(which(c));
        }
        INFO("mode " << mode);
        auto pos = [&](int id) { return std::find(ids.begin(), ids.end(), id) - ids.begin(); };
        CHECK(pos(2) < pos(1));  // island before its hole
        CHECK(pos(1) < pos(0));  // hole before the outer
        CHECK(pos(3) < pos(0));
        CHECK(ids.back() == 0);
        // Region semantics: outside profile keeps the part — the tool runs inside the holes and
        // outside the island and the outer boundary.
        const Toolpath& outerCut = cuts[static_cast<std::size_t>(pos(0))];
        CHECK_FALSE(Box2(Vec2{0, 0}, Vec2{100, 100}).contains(outerCut.moves.front().to.xy()));
    }
}

// --------------------------------------------------------------------------- Goal extras

TEST_CASE("Tabs: constant distance, avoid corners and 3D tabs", "[toolpath][profile][tabs]") {
    const Contour rect = Contour::rectangle(Box2{{0, 0}, {100, 50}});
    ProfileParams p = baseParams(6.0, 6.0);
    p.tabs.enabled = true;
    p.tabs.spacing = TabSpacing::ConstantDistance;
    p.tabs.distance = 80.0;  // perimeter 318.85 -> 4 tabs
    p.tabs.length = 6.0;
    p.tabs.thickness = 3.0;
    const Toolpath tp = profileToolpath({rect}, test::endMill(6.0), p).toolpath;
    std::size_t ups = 0;
    for (const Move& m : tp.moves) {
        if (m.kind == MoveKind::Feed && m.to.z == -3.0) {
            ++ups;
        }
    }
    CHECK(ups == 8);  // up + across, per tab

    // A tab that would land on a corner arc is moved onto a straight edge.
    p.tabs.spacing = TabSpacing::ConstantNumber;
    p.tabs.count = 1;
    p.startAt = StartAt::ClosestOnBoundingBox;
    p.startAnchor = BoxAnchor::TopRight;
    const Contour rounded = [] {
        // 20x20 square with large corner arcs (r 8) and 4 mm straight edges.
        Contour c;
        c.closed = true;
        c.append(LineSpan{{8, 0}, {12, 0}});
        c.append(ArcSpan{{12, 8}, 8, -geometry::kPi / 2, 0, geometry::ArcDirection::CCW});
        c.append(LineSpan{{20, 8}, {20, 12}});
        c.append(ArcSpan{{12, 12}, 8, 0, geometry::kPi / 2, geometry::ArcDirection::CCW});
        c.append(LineSpan{{12, 20}, {8, 20}});
        c.append(ArcSpan{{8, 12}, 8, geometry::kPi / 2, geometry::kPi, geometry::ArcDirection::CCW});
        c.append(LineSpan{{0, 12}, {0, 8}});
        c.append(ArcSpan{{8, 8}, 8, geometry::kPi, 1.5 * geometry::kPi, geometry::ArcDirection::CCW});
        return c;
    }();
    p.tabs.length = 1.0;  // raised stretch 1 + 6 = 7 mm does not fit any 4 mm edge: reported
    p.tabs.avoidCorners = true;
    const ProfileResult r1 = profileToolpath({rounded}, test::endMill(6.0), p);
    CHECK(r1.has(ProfileIssue::TabsNotPlaced));
    p.tabs.length = 6.0;  // raised 12 mm: moved onto a straight edge of the rectangle
    const ProfileResult r2 = profileToolpath({rect}, test::endMill(6.0), p);
    CHECK_FALSE(r2.has(ProfileIssue::TabsNotPlaced));
    for (std::size_t i = 1; i < r2.toolpath.moves.size(); ++i) {
        const Move& m = r2.toolpath.moves[i];
        if (m.to.z == -3.0) {
            // On a straight edge: never next to an arc move.
            CHECK_FALSE(isArc(m.kind));
        }
    }

    // 3D tabs ramp at 45 degrees: no vertical up-move into the tab.
    p.tabs.threeD = true;
    p.tabs.avoidCorners = false;
    const Toolpath t3 = profileToolpath({rect}, test::endMill(6.0), p).toolpath;
    bool sloped = false;
    for (std::size_t i = 1; i < t3.moves.size(); ++i) {
        const Vec3 a = t3.moves[i - 1].to;
        const Vec3 b = t3.moves[i].to;
        if (b.z > a.z && b.z <= -3.0) {
            CHECK(geometry::distance(a.xy(), b.xy()) == Approx(b.z - a.z).margin(1e-9));
            sloped = true;
        }
    }
    CHECK(sloped);
    CHECK(t3.minZ() == Approx(-6.0));
}

TEST_CASE("Open vectors: Right/Left offsets, alternating passes, On keeps the vector", "[toolpath][profile]") {
    Contour open;
    open.append(LineSpan{{0, 0}, {50, 0}});
    open.append(LineSpan{{50, 0}, {50, 30}});
    for (ProfileSide side : {ProfileSide::Right, ProfileSide::Left}) {
        ProfileParams p = baseParams(6.0, 3.0);
        p.side = side;
        const Toolpath tp = profileToolpath({open}, test::endMill(6.0), p).toolpath;
        REQUIRE_FALSE(tp.isEmpty());
        CHECK(test::minCutDistance(tp, {open}) >= 3.0 - 1e-9);
        // Right of travel along +x is -y.
        double ySum = 0.0;
        test::sampleCuttingMoves(tp, 1.0, [&](Vec2 q) {
            if (q.x < 45.0) ySum += q.y;
        });
        CHECK((side == ProfileSide::Right ? ySum < 0.0 : ySum > 0.0));
        CHECK(tp.count(MoveKind::Rapid) == 1);  // second level runs back without a retract
        CHECK(test::feedLevels(tp) == std::vector<double>{-3.0, -6.0});
    }
    ProfileParams p = baseParams(3.0, 3.0);
    p.side = ProfileSide::On;
    const Toolpath on = profileToolpath({open}, test::endMill(6.0), p).toolpath;
    REQUIRE(on.moves.size() == 5);
    CHECK(on.moves[0].to.xy() == Vec2{0, 0});
    CHECK(on.moves[3].to.xy() == Vec2{50, 30});
    p.direction = CutDirection::Conventional;
    const Toolpath onRev = profileToolpath({open}, test::endMill(6.0), p).toolpath;
    CHECK(onRev.moves[0].to.xy() == Vec2{50, 30});
}

TEST_CASE("Nearby closed vectors are offset together instead of gouging each other", "[toolpath][profile]") {
    // Two squares 4 mm apart: with a 6 mm tool their outside cuts would overlap.
    const std::vector<Contour> cs{square({10, 10}, 20.0), square({34, 10}, 20.0)};
    const ProfileResult r = profileToolpath(cs, test::endMill(6.0), baseParams(3.0, 3.0));
    REQUIRE(r.ok());
    CHECK(r.has(ProfileIssue::MergedWithNeighbour));
    CHECK(splitCuts(r.toolpath).size() == 1);
    CHECK(test::minCutDistance(r.toolpath, cs) >= 3.0 - 1e-3);
}

TEST_CASE("An open vector's cut is trimmed where it would gouge a neighbour", "[toolpath][profile]") {
    Contour line;
    line.append(LineSpan{{0, 0}, {60, 0}});
    const Contour block = square({30, -6}, 6.0);  // just below the line's right-hand offset
    ProfileParams p = baseParams(3.0, 3.0);
    p.side = ProfileSide::Right;
    const ProfileResult r = profileToolpath({line, block}, test::endMill(6.0), p);
    REQUIRE(r.ok());
    CHECK(r.has(ProfileIssue::TrimmedNearVector));
    CHECK(test::minCutDistance(r.toolpath, {line, block}) >= 3.0 - 1e-3);
}

TEST_CASE("Start points: vector start, optimised, closest on bounding box", "[toolpath][profile]") {
    Contour sq = square({50, 50}, 40.0);  // starts at (30, 30)
    ProfileParams p = baseParams(3.0, 3.0);
    p.side = ProfileSide::Inside;
    p.startAt = StartAt::VectorStart;
    CHECK(profileToolpath({sq}, test::endMill(6.0), p).toolpath.moves[0].to.xy() == Vec2{33, 33});
    p.startAt = StartAt::ClosestOnBoundingBox;
    p.startAnchor = BoxAnchor::TopCentre;
    const Vec2 tc = profileToolpath({sq}, test::endMill(6.0), p).toolpath.moves[0].to.xy();
    CHECK(tc.x == Approx(50.0));
    CHECK(tc.y == Approx(67.0));
    p.startAt = StartAt::Optimise;
    p.startPosition = {100, 50};
    const Vec2 op = profileToolpath({sq}, test::endMill(6.0), p).toolpath.moves[0].to.xy();
    CHECK(op.x == Approx(67.0));
    CHECK(op.y == Approx(50.0));
}

TEST_CASE("V-bit corners: sharp external mitres, sharp internal corner climbs", "[toolpath][profile][vbit]") {
    tooling::Tool v = test::endMill(12.0);
    v.type = tooling::ToolType::VBit;
    v.includedAngle = geometry::kPi / 2.0;  // 90 deg: radius at depth 3 is 3
    // L-shaped region: one inside corner at (20, 20) for an outside profile.
    Contour l;
    l.closed = true;
    for (auto [a, b] : std::vector<std::pair<Vec2, Vec2>>{{{0, 0}, {40, 0}},
                                                            {{40, 0}, {40, 20}},
                                                            {{40, 20}, {20, 20}},
                                                            {{20, 20}, {20, 40}},
                                                            {{20, 40}, {0, 40}},
                                                            {{0, 40}, {0, 0}}}) {
        l.append(LineSpan{a, b});
    }
    ProfileParams p = baseParams(3.0, 3.0);
    p.corners.sharpExternal = true;
    p.corners.sharpInternal = true;
    const ProfileResult r = profileToolpath({l}, v, p);
    REQUIRE(r.ok());
    CHECK(r.diagnostics.empty());
    std::size_t arcs = 0;
    bool climbed = false;
    for (const Move& m : r.toolpath.moves) {
        arcs += isArc(m.kind) ? 1u : 0u;
        if (m.to.z == 0.0 && m.kind == MoveKind::Feed) {
            CHECK(m.to.xy() == Vec2{20, 20});  // the tip rises to the vector's inside corner
            climbed = true;
        }
    }
    CHECK(arcs == 0);  // mitred corners, no round joins
    CHECK(climbed);
    // An end mill ignores the options and says so.
    const ProfileResult e = profileToolpath({l}, test::endMill(6.0), p);
    CHECK(e.has(ProfileIssue::CornerNeedsVBit));
}

TEST_CASE("Profile output is deterministic and invalid input is reported", "[toolpath][profile]") {
    ProfileParams p = baseParams();
    p.tabs.enabled = true;
    p.ramp.type = RampType::Smooth;
    p.leads.type = LeadType::Circular;
    p.leads.leadOut = true;
    const std::vector<Contour> cs = grid3x3();
    CHECK(profileToolpath(cs, test::endMill(), p).toolpath == profileToolpath(cs, test::endMill(), p).toolpath);

    ProfileParams bad = baseParams();
    bad.allowance = std::nan("");
    CHECK(profileToolpath(cs, test::endMill(), bad).has(ProfileIssue::InvalidParameters));
    ProfileParams none = baseParams(0.0, 4.0);
    CHECK(profileToolpath(cs, test::endMill(), none).has(ProfileIssue::NoPasses));
    const ProfileResult empty = profileToolpath({Contour{}}, test::endMill(), baseParams());
    CHECK(empty.has(ProfileIssue::EmptyContour));
    CHECK_FALSE(empty.ok());
    // Pass depth falls back to the tool's cutting data (4 mm) when not set.
    ProfileParams fromTool = baseParams(8.0, 0.0);
    CHECK(test::feedLevels(profileToolpath({square({0, 0}, 10)}, test::endMill(), fromTool).toolpath) ==
          std::vector<double>{-4.0, -8.0});
    // Tool snapshot comes from the tool and its cutting data.
    const Toolpath tp = profileToolpath({square({0, 0}, 10)}, test::endMill(), fromTool).toolpath;
    CHECK(tp.tool.diameter == 6.0);
    CHECK(tp.tool.feedRate == 1000.0);
    CHECK(tp.tool.toolNumber == 1);
    CHECK(tp.name == "Profile");
}

// ------------------------------------------------------------------ round-2 regressions

TEST_CASE("Tabs on a loop shorter than one tab are left out, so the hole is still cut through",
          "[toolpath][profile][tabs]") {
    for (int count : {1, 2}) {
        ProfileParams p = baseParams(6.0, 3.0);
        p.side = ProfileSide::Inside;
        p.tabs.enabled = true;
        p.tabs.count = count;
        p.tabs.length = 10.0;
        p.tabs.thickness = 2.0;
        // Loop of the r = 5 hole with a 6 mm tool: 2·pi·2 = 12.6 mm < raised stretch 16 mm.
        const ProfileResult r = profileToolpath({Contour::circle({0, 0}, 5.0)}, test::endMill(6.0), p);
        REQUIRE(r.ok());
        INFO("count " << count);
        CHECK(r.toolpath.minZ() == Approx(-6.0));
        CHECK(test::feedLevels(r.toolpath) == std::vector<double>{-3.0, -6.0});
        CHECK(r.has(ProfileIssue::TabsNotPlaced));
    }
}

TEST_CASE("3D tab ramps count towards overlap: neighbouring 3D tabs never merge above the bottom",
          "[toolpath][profile][tabs]") {
    for (bool threeD : {false, true}) {
        ProfileParams p = baseParams(12.0, 6.0);
        p.tabs.enabled = true;
        p.tabs.count = 16;
        p.tabs.length = 10.0;
        p.tabs.thickness = 4.0;
        p.tabs.threeD = threeD;
        const ProfileResult r =
            profileToolpath({Contour::rectangle(Box2{{0, 0}, {100, 50}})}, test::endMill(6.0), p);
        REQUIRE(r.ok());
        INFO("threeD " << threeD);
        CHECK(r.toolpath.minZ() == Approx(-12.0));
        // 16 footprints: 16 mm raised + 2 x 4 mm ramps = 24 mm each > 318.85 / 16 for 3D tabs.
        CHECK(r.has(ProfileIssue::TabsNotPlaced) == threeD);
        // Between consecutive tabs the last pass returns to the bottom.
        const std::vector<Toolpath> cuts = splitCuts(r.toolpath);
        REQUIRE(cuts.size() == 1);
        std::size_t rises = 0;
        bool atBottom = false;
        for (const Move& m : cuts[0].moves) {
            if (m.to.z == -12.0) {
                atBottom = true;
            } else if (atBottom && m.to.z > -12.0 && m.to.z < 0.0 && m.kind != MoveKind::Retract) {
                ++rises;
                atBottom = false;
            }
        }
        CHECK(rises >= 8);  // every placed tab is its own rise from the bottom
    }
    // Open path: a 3D tab footprint must not run past the ends.
    Contour line;
    line.append(LineSpan{{0, 0}, {30, 0}});
    ProfileParams p = baseParams(6.0, 6.0);
    p.side = ProfileSide::Right;
    p.tabs.enabled = true;
    p.tabs.count = 1;
    p.tabs.length = 14.0;  // raised 20 mm + 2 x 3 mm ramps = 26 mm on a 30 mm path: fits
    p.tabs.thickness = 3.0;
    p.tabs.threeD = true;
    const ProfileResult fits = profileToolpath({line}, test::endMill(6.0), p);
    CHECK_FALSE(fits.has(ProfileIssue::TabsNotPlaced));
    CHECK(fits.toolpath.minZ() == Approx(-6.0));
    p.tabs.thickness = 6.0;  // 20 + 12 = 32 mm > 30 mm: left out
    const ProfileResult nofit = profileToolpath({line}, test::endMill(6.0), p);
    CHECK(nofit.has(ProfileIssue::TabsNotPlaced));
    CHECK(nofit.toolpath.minZ() == Approx(-6.0));
}

TEST_CASE("Generative: tabbed and untabbed outlines are cut to full depth everywhere but the tabs",
          "[toolpath][profile][tabs][property]") {
    test::SplitMix rng{0x7AB5ULL};
    constexpr double kStep = 0.05;
    int checked = 0;
    for (int s = 0; s < 150; ++s) {
        const int kind = static_cast<int>(rng.next() % 3);
        const double cr = rng.uniform(3.5, 20.0);
        const Box2 box = Box2::fromCentre({0, 0}, rng.uniform(4, 60), rng.uniform(4, 40));
        Contour c;
        if (kind == 0) {
            c = Contour::circle({0, 0}, cr);
        } else if (kind == 1) {
            c = Contour::rectangle(box);
        } else {
            c.append(LineSpan{{0, 0}, {rng.uniform(2, 50), 0}});
            c.append(LineSpan{c.endPoint(), c.endPoint() + Vec2{0, rng.uniform(2, 30)}});
        }
        ProfileParams p = baseParams(rng.uniform(2, 12), rng.uniform(1, 6));
        p.side = rng.uniform(0, 1) < 0.5 ? ProfileSide::Outside : ProfileSide::Inside;
        p.tabs.enabled = rng.uniform(0, 1) < 0.75;
        p.tabs.spacing = rng.uniform(0, 1) < 0.5 ? TabSpacing::ConstantNumber : TabSpacing::ConstantDistance;
        p.tabs.count = 1 + static_cast<int>(rng.next() % 20);
        p.tabs.distance = rng.uniform(5, 60);
        p.tabs.length = rng.uniform(1, 15);
        p.tabs.thickness = rng.uniform(0.5, 5);
        p.tabs.threeD = rng.uniform(0, 1) < 0.5;
        p.tabs.avoidCorners = rng.uniform(0, 1) < 0.3;
        const RampType ramps[] = {RampType::None, RampType::Smooth, RampType::ZigZag, RampType::Spiral};
        p.ramp.type = ramps[rng.next() % 4];
        p.ramp.distance = rng.uniform(2, 20);
        p.separateLastPass = rng.uniform(0, 1) < 0.3;
        p.startAt = rng.uniform(0, 1) < 0.5 ? StartAt::VectorStart : StartAt::ClosestOnBoundingBox;
        p.startAnchor = static_cast<BoxAnchor>(rng.next() % 9);
        if (rng.uniform(0, 1) < 0.3) {
            p.leads.type = LeadType::Circular;
            p.leads.radius = rng.uniform(1, 5);
            p.leads.leadOut = true;
            p.leads.overcut = rng.uniform(0, 10);
        }
        const double d = rng.uniform(3, 8);
        const ProfileResult r = profileToolpath({c}, test::endMill(d), p);
        if (!r.ok()) {
            CHECK(r.has(ProfileIssue::ToolTooLarge));
            continue;
        }
        INFO("scene " << s << " kind " << kind);
        const double bottom = p.passes.bottomZ();
        const std::vector<double> got = test::feedLevels(r.toolpath);
        for (double level : p.passes.levels()) {
            CHECK(std::find_if(got.begin(), got.end(), [&](double z) { return std::fabs(z - level) < 1e-9; }) !=
                  got.end());
        }
        CHECK(r.toolpath.minZ() == Approx(bottom));
        if (kind == 2) {
            continue;
        }
        // Coverage along the analytic outline: at full depth except inside at most `count` tab
        // footprints, and nowhere left above the tab top.
        const double off = p.side == ProfileSide::Outside ? d / 2 : -d / 2;
        const std::vector<Vec2> outline =
            kind == 0 ? test::circleOutline({0, 0}, cr, off, kStep) : test::rectOutline(box, off, kStep);
        const test::CutCoverage atBottom(r.toolpath, bottom, kStep);
        const std::vector<double> runs = test::uncutRuns(outline, atBottom, 0.75 * kStep, kStep);
        const double footprint = p.tabs.length + d + (p.tabs.threeD ? 2.0 * p.tabs.thickness : 0.0);
        if (!p.tabs.enabled) {
            CHECK(runs.empty());
        } else {
            const long maxTabs = p.tabs.spacing == TabSpacing::ConstantNumber ? p.tabs.count : 1000;
            CHECK(static_cast<long>(runs.size()) <= maxTabs);
            for (double len : runs) {
                INFO("footprint " << footprint << " count " << p.tabs.count << " spacing " << static_cast<int>(p.tabs.spacing)
                                  << " 3d " << p.tabs.threeD << " avoid " << p.tabs.avoidCorners << " sep "
                                  << p.separateLastPass << " side " << static_cast<int>(p.side) << " d " << d
                                  << " box " << box.width() << "x" << box.height() << " runs " << runs.size() << " ramp "
                                  << static_cast<int>(p.ramp.type) << " rampd " << p.ramp.distance << " cut "
                                  << p.passes.cutDepth << " pass " << p.passes.passDepth << " thick " << p.tabs.thickness
                                  << " len " << p.tabs.length << " start " << static_cast<int>(p.startAt) << " anchor "
                                  << static_cast<int>(p.startAnchor) << " lead " << static_cast<int>(p.leads.type)
                                  << " lr " << p.leads.radius << " oc " << p.leads.overcut << " dist " << p.tabs.distance);
                CHECK(len <= footprint + 4 * kStep);
            }
            const test::CutCoverage atTab(r.toolpath, bottom + p.tabs.thickness, kStep);
            CHECK(test::uncutRuns(outline, atTab, 0.75 * kStep, kStep).empty());
        }
        ++checked;
    }
    CHECK(checked > 50);
}

TEST_CASE("Tabs never cover the cut start: plunge, lead-in and overcut lead-out stay out of tabs",
          "[toolpath][profile][tabs]") {
    // Verifier repro notch2: Inside circle r = 10, one 6 mm pass, 4 tabs 6 x 3 (footprint 12 mm on
    // a 44 mm loop): the first tab used to straddle the start and the plunge cut into it.
    {
        ProfileParams p = baseParams(6.0, 6.0);
        p.side = ProfileSide::Inside;
        p.tabs.enabled = true;
        p.tabs.count = 4;
        p.tabs.length = 6.0;
        p.tabs.thickness = 3.0;
        const ProfileResult r = profileToolpath({Contour::circle({0, 0}, 10.0)}, test::endMill(6.0), p);
        REQUIRE(r.ok());
        const Toolpath& tp = r.toolpath;
        CHECK(tp.moves[1].kind == MoveKind::Plunge);
        CHECK(tp.moves[1].to.z == -6.0);
        CHECK(tp.moves[2].to.z == -6.0);  // cuts at depth before the first tab
        const std::vector<Vec2> outline = test::circleOutline({0, 0}, 10.0, -3.0, 0.02);
        for (double len : test::uncutRuns(outline, test::CutCoverage(tp, -6.0), 0.015, 0.02)) {
            CHECK(len <= 12.0 + 0.1);  // each uncut stretch is one whole tab
        }
    }
    // Verifier repro notch mode 4: Outside 100 x 50, start at the bottom centre, one 6 mm pass,
    // 3 tabs 105 mm long (footprint 111 mm, loop 318.85 mm).
    {
        ProfileParams p = baseParams(6.0, 6.0);
        p.startAt = StartAt::ClosestOnBoundingBox;
        p.startAnchor = BoxAnchor::BottomCentre;
        p.tabs.enabled = true;
        p.tabs.count = 3;
        p.tabs.length = 105.0;
        p.tabs.thickness = 3.0;
        const ProfileResult r = profileToolpath({Contour::rectangle(Box2{{0, 0}, {100, 50}})}, test::endMill(6.0), p);
        REQUIRE(r.ok());
        CHECK(r.toolpath.moves[1].to.z == -6.0);
        CHECK(r.toolpath.moves[2].to.z == -6.0);
        const std::vector<Vec2> outline = test::rectOutline(Box2{{0, 0}, {100, 50}}, 3.0, 0.02);
        const std::vector<double> runs = test::uncutRuns(outline, test::CutCoverage(r.toolpath, -6.0), 0.015, 0.02);
        // Greedy fit: the first tab moves off the start, the next would overlap it and the last
        // would leave less than a tool radius at full depth across the start: one tab, two reported.
        CHECK(runs.size() == 1);
        CHECK(r.has(ProfileIssue::TabsNotPlaced));
        for (double len : runs) {
            CHECK(len == Approx(111.0).margin(0.1));
        }
    }
    // Verifier repro notch3: an overcut that ends inside a tab; the lead-out used to plunge there.
    {
        ProfileParams p = baseParams(6.0, 3.0);
        p.side = ProfileSide::Inside;
        p.tabs.enabled = true;
        p.tabs.count = 3;
        p.tabs.length = 5.0;
        p.tabs.thickness = 2.0;
        p.leads.type = LeadType::Circular;
        p.leads.radius = 3.0;
        p.leads.leadOut = true;
        p.leads.overcut = 10.0;
        const ProfileResult r = profileToolpath({Contour::circle({0, 0}, 15.0)}, test::endMill(6.0), p);
        REQUIRE(r.ok());
        const auto& mv = r.toolpath.moves;
        const std::size_t n = mv.size();
        REQUIRE(mv[n - 1].kind == MoveKind::Retract);
        REQUIRE(isArc(mv[n - 2].kind));  // the lead-out
        CHECK(mv[n - 3].kind != MoveKind::Plunge);
        CHECK(mv[n - 2].to.z == mv[n - 3].to.z);  // leaves at the height the overcut ended at
        const std::vector<Vec2> outline = test::circleOutline({0, 0}, 15.0, -3.0, 0.02);
        for (double len : test::uncutRuns(outline, test::CutCoverage(r.toolpath, -6.0), 0.015, 0.02)) {
            CHECK(len <= 11.0 + 0.1);
        }
    }
}

TEST_CASE("Without a ramp, closed cuts step down vertically and every level is a full flat loop",
          "[toolpath][profile]") {
    // Verifier repro ramp0: rounded rectangle 40 x 20 (corner r 0.7), 3 mm tool, allowance 0.31,
    // 9 deep in 3 mm passes: the first span used to be cut on a slope between levels.
    Contour c;
    c.closed = true;
    const double rr = 0.7;
    const double hp = geometry::kPi / 2;
    c.append(LineSpan{{rr, 0}, {40 - rr, 0}});
    c.append(ArcSpan{{40 - rr, rr}, rr, -hp, 0, geometry::ArcDirection::CCW});
    c.append(LineSpan{{40, rr}, {40, 20 - rr}});
    c.append(ArcSpan{{40 - rr, 20 - rr}, rr, 0, hp, geometry::ArcDirection::CCW});
    c.append(LineSpan{{40 - rr, 20}, {rr, 20}});
    c.append(ArcSpan{{rr, 20 - rr}, rr, hp, 2 * hp, geometry::ArcDirection::CCW});
    c.append(LineSpan{{0, 20 - rr}, {0, rr}});
    c.append(ArcSpan{{rr, rr}, rr, 2 * hp, 3 * hp, geometry::ArcDirection::CCW});
    ProfileParams p = baseParams(9.0, 3.0);
    p.allowance = 0.31;
    const ProfileResult r = profileToolpath({c}, test::endMill(3.0), p);
    REQUIRE(r.ok());
    const auto& mv = r.toolpath.moves;
    for (std::size_t i = 1; i < mv.size(); ++i) {
        const bool descends = mv[i].to.z < mv[i - 1].to.z;
        if (descends) {
            CHECK(mv[i].kind == MoveKind::Plunge);
            CHECK(mv[i].to.xy() == mv[i - 1].to.xy());  // vertical
        }
    }
    CHECK(r.toolpath.count(MoveKind::Plunge) == 3);
    // Every point of the loop is cut at -9: sample the cut at the bottom against the path the
    // tool takes at -3 (the first level is a full loop too).
    std::vector<Vec2> loop;
    for (std::size_t i = 2; i < mv.size(); ++i) {
        if (mv[i].to.z != -3.0 || mv[i].kind == MoveKind::Plunge) continue;
        Toolpath one;
        one.moves = {mv[i - 1], mv[i]};
        test::sampleCuttingMoves(one, 0.02, [&](Vec2 q) { loop.push_back(q); });
    }
    const test::CutCoverage bottom(r.toolpath, -9.0);
    for (const Vec2& q : loop) {
        INFO("at " << q.x << ", " << q.y);
        REQUIRE(bottom.covers(q, 0.015));
    }
}

TEST_CASE("V-bit sharp internal corners only at genuine corners, safe next to curved walls",
          "[toolpath][profile][vbit]") {
    tooling::Tool v = test::endMill(12.0);
    v.type = tooling::ToolType::VBit;
    v.includedAngle = geometry::kPi / 2.0;
    // Verifier repro vrep: 40 x 40 square whose bottom edge is an arc bulging `sag` mm into it;
    // inside, 3 deep. The two arc/line corners used to over-cut the curved wall.
    for (double sag : {6.0, 10.0}) {
        Contour c;
        c.closed = true;
        const double rad = (40.0 * 40.0 / 4 + sag * sag) / (2 * sag);
        c.append(ArcSpan::fromCentre({20, sag - rad}, {0, 0}, {40, 0}, geometry::ArcDirection::CW));
        c.append(LineSpan{{40, 0}, {40, 40}});
        c.append(LineSpan{{40, 40}, {0, 40}});
        c.append(LineSpan{{0, 40}, {0, 0}});
        for (double allowance : {0.0, 0.3}) {
            ProfileParams p = baseParams(3.0, 3.0);
            p.side = ProfileSide::Inside;
            p.allowance = allowance;
            p.corners.sharpInternal = true;
            const ProfileResult r = profileToolpath({c}, v, p);
            REQUIRE(r.ok());
            INFO("sag " << sag << " allowance " << allowance);
            CHECK(test::minConeClearance(r.toolpath, {c}, 1.0, 6.0, allowance, 0.01) >= -1e-4);
            // Climbs reach the surface at all four corners (two straight, two next to the arc).
            std::size_t climbs = 0;
            for (std::size_t i = 1; i < r.toolpath.moves.size(); ++i) {
                const Move& m = r.toolpath.moves[i];
                if (m.kind == MoveKind::Feed && m.to.z == 0.0 && r.toolpath.moves[i - 1].to.z < 0.0) {
                    ++climbs;
                    // With an allowance the climb stops that far from the walls (next to the curved
                    // wall the safe climb reaches the surface up to one 0.05 mm sample earlier).
                    const double dist = test::bruteDistance(m.to.xy(), {c});
                    CHECK(dist >= allowance - 1e-6);
                    CHECK(dist <= allowance + 0.051);
                }
            }
            CHECK(climbs == 4);
            // Vertical descents are plunges, sloped ones feeds; plunges are always vertical; no
            // zero-length moves.
            for (std::size_t i = 1; i < r.toolpath.moves.size(); ++i) {
                const Move& m = r.toolpath.moves[i];
                const Vec3 from = r.toolpath.moves[i - 1].to;
                CHECK((m.to != from || (isArc(m.kind) && m.to.xy() == from.xy())));
                if (m.to.z < from.z) {
                    CHECK((m.kind == MoveKind::Plunge) == (m.to.xy() == from.xy()));
                }
            }
        }
    }
    // Merged (polygonal) regions and polygonised curves have no genuine corners at their polyline
    // vertices: no climbs from across the part.
    {
        const std::vector<Contour> cs{square({10, 10}, 20.0), square({34, 10}, 20.0)};  // merge
        ProfileParams p = baseParams(3.0, 3.0);
        p.corners.sharpInternal = true;
        const ProfileResult r = profileToolpath(cs, v, p);
        REQUIRE(r.ok());
        CHECK(r.has(ProfileIssue::MergedWithNeighbour));
        for (const Move& m : r.toolpath.moves) {
            CHECK(m.to.z != 0.0);
        }
        CHECK(test::minConeClearance(r.toolpath, cs, 1.0, 6.0, 0.0) >= -1e-4);
    }
    {
        Contour blob;
        blob.closed = true;
        blob.append(geometry::BezierSpan{{0, 0}, {20, -10}, {40, 10}, {30, 30}});
        blob.append(geometry::BezierSpan{{30, 30}, {20, 45}, {-10, 30}, {0, 0}});
        for (ProfileSide side : {ProfileSide::Outside, ProfileSide::Inside}) {
            ProfileParams p = baseParams(3.0, 3.0);
            p.side = side;
            p.corners.sharpInternal = true;
            const ProfileResult r = profileToolpath({blob}, v, p);
            REQUIRE(r.ok());
            std::size_t climbs = 0;  // excursions that reach the surface
            for (std::size_t i = 1; i < r.toolpath.moves.size(); ++i) {
                climbs += (r.toolpath.moves[i].to.z == 0.0 && r.toolpath.moves[i - 1].to.z < 0.0) ? 1u : 0u;
            }
            INFO("side " << static_cast<int>(side) << " climbs " << climbs);
            CHECK(climbs <= 2);  // only the two real corners where the Beziers meet
            // Oracle on a dense polyline of the Beziers (chords of ~0.1 mm, sag < 2e-4 mm).
            Contour dense;
            dense.closed = true;
            for (const geometry::Span& sp : blob.spans) {
                const auto& b = std::get<geometry::BezierSpan>(sp);
                Vec2 prev = b.pointAt(0.0);
                for (int i = 1; i <= 600; ++i) {
                    const Vec2 q = b.pointAt(i / 600.0);
                    dense.append(LineSpan{prev, q});
                    prev = q;
                }
            }
            CHECK(test::minConeClearance(r.toolpath, {dense}, 1.0, 6.0, 0.0, 0.05) >= -4e-4);
        }
    }
    // On profiles get no corner moves.
    {
        Contour sq = square({0, 0}, 20.0);
        ProfileParams p = baseParams(3.0, 3.0);
        p.side = ProfileSide::On;
        p.corners.sharpInternal = true;
        const ProfileResult r = profileToolpath({sq}, v, p);
        REQUIRE(r.ok());
        for (std::size_t i = 1; i < r.toolpath.moves.size(); ++i) {
            CHECK(r.toolpath.moves[i].to.z != 0.0);
            if (r.toolpath.moves[i].to.z < r.toolpath.moves[i - 1].to.z &&
                r.toolpath.moves[i].to.xy() == r.toolpath.moves[i - 1].to.xy()) {
                CHECK(r.toolpath.moves[i].kind == MoveKind::Plunge);
            }
        }
    }
}

TEST_CASE("V-bit sharp internal corners keep the cone inside the walls for any start depth",
          "[toolpath][profile][vbit]") {
    Contour l;
    l.closed = true;
    for (auto [a, b] : std::vector<std::pair<Vec2, Vec2>>{{{0, 0}, {40, 0}},
                                                            {{40, 0}, {40, 20}},
                                                            {{40, 20}, {20, 20}},
                                                            {{20, 20}, {20, 40}},
                                                            {{20, 40}, {0, 40}},
                                                            {{0, 40}, {0, 0}}}) {
        l.append(LineSpan{a, b});
    }
    for (double angleDeg : {60.0, 90.0}) {
        for (double diameter : {12.0, 4.0}) {  // 4 mm: the cut is deeper than the cone at 90°
            tooling::Tool v = test::endMill(diameter);
            v.type = tooling::ToolType::VBit;
            v.includedAngle = angleDeg * geometry::kPi / 180.0;
            const double tanHalf = std::tan(0.5 * v.includedAngle);
            for (ProfileSide side : {ProfileSide::Outside, ProfileSide::Inside}) {
                for (double startDepth : {0.0, 2.0}) {
                    ProfileParams p = baseParams(3.0, 3.0);
                    p.passes.startDepth = startDepth;
                    p.side = side;
                    p.corners.sharpInternal = true;
                    const ProfileResult r = profileToolpath({l}, v, p);
                    REQUIRE(r.ok());
                    INFO("angle " << angleDeg << " dia " << diameter << " side " << static_cast<int>(side)
                                  << " start " << startDepth);
                    bool reachedCorner = false;
                    for (const Move& m : r.toolpath.moves) {
                        reachedCorner = reachedCorner || m.to.z == 0.0;
                    }
                    CHECK(test::minConeClearance(r.toolpath, {l}, tanHalf, 0.5 * diameter, 0.0, 0.005) >= -1e-6);
                    CHECK(reachedCorner);
                }
            }
        }
    }
}

TEST_CASE("Separate last pass: finishing cuts follow the roughing order", "[toolpath][profile]") {
    for (int mode = 0; mode < 3; ++mode) {
        ProfileParams p = baseParams(3.0, 3.0);
        p.separateLastPass = true;
        p.lastPassAllowance = 0.3;
        p.startPosition = {100, 0};
        p.order.grid = mode == 1;
        p.order.selectionOrder = mode == 2;
        const std::vector<int> cells = cutCells(profileToolpath(grid3x3(), test::endMill(6.0), p).toolpath);
        REQUIRE(cells.size() == 18);
        INFO("mode " << mode);
        CHECK(std::vector<int>(cells.begin(), cells.begin() + 9) == std::vector<int>(cells.begin() + 9, cells.end()));
    }
}

TEST_CASE("Inside profiles starting at a corner still get their lead-in", "[toolpath][profile][leads]") {
    const Contour sq = Contour::rectangle(Box2{{0, 0}, {40, 30}});  // starts at a corner
    for (LeadType type : {LeadType::Straight, LeadType::Circular}) {
        ProfileParams p = baseParams(3.0, 3.0);
        p.side = ProfileSide::Inside;
        p.leads.type = type;
        p.leads.length = 6.0;
        p.leads.angle = geometry::kPi / 6.0;
        p.leads.radius = 4.0;
        p.leads.sweep = geometry::kPi / 2.0;
        p.leads.leadOut = true;
        const ProfileResult r = profileToolpath({sq}, test::endMill(6.0), p);
        REQUIRE(r.ok());
        INFO("type " << static_cast<int>(type));
        CHECK_FALSE(r.has(ProfileIssue::LeadDropped));
        CHECK(test::minCutDistance(r.toolpath, {sq}) >= 3.0 - 1e-3);
        // The entry is in the waste (inside the hole), away from where the cut starts, and at
        // least the tool radius from the walls (a perpendicular straight lead at a corner runs
        // along the previous edge's cut line, exactly at the tool radius).
        const Vec2 entry = r.toolpath.moves[0].to.xy();
        INFO("entry " << entry.x << ", " << entry.y);
        CHECK(test::bruteDistance(entry, {sq}) >= 3.0 - 1e-9);
        CHECK(sq.bounds().contains(entry));
        CHECK(geometry::distance(entry, r.toolpath.moves[2].to.xy()) > 1.0);
    }
}

// ------------------------------------------------------------------ T-0124: roughing tabs

namespace {

/// [first, last) move ranges of each cut (a cut starts at its rapid).
std::vector<std::pair<std::size_t, std::size_t>> cutRanges(const Toolpath& tp) {
    std::vector<std::pair<std::size_t, std::size_t>> out;
    for (std::size_t i = 0; i < tp.moves.size(); ++i) {
        if (tp.moves[i].kind == MoveKind::Rapid) {
            if (!out.empty()) out.back().second = i;
            out.push_back({i, tp.moves.size()});
        }
    }
    return out;
}

/// Total tabs left out according to the TabsNotPlaced diagnostics ("vector i: N of M ...").
std::size_t tabsLeftOut(const ProfileResult& r) {
    std::size_t n = 0;
    for (const ProfileDiagnostic& d : r.diagnostics) {
        if (d.issue == ProfileIssue::TabsNotPlaced && d.message.find(" tab(s) left out") != std::string::npos) {
            n += static_cast<std::size_t>(std::stoul(d.message.substr(d.message.find(": ") + 2)));
        }
    }
    return n;
}

/// Checks that no move of any pass goes below the tab top over tab material: the cores of the
/// finishing cut's raised runs (a tool radius + 0.05 mm in from each end) and the middle of each
/// roughing run must not lie within a tool radius of any cutting sample below the tab top.
void checkTabMaterial(const Toolpath& tp, double zTab, double r) {
    const auto cuts = cutRanges(tp);
    REQUIRE(cuts.size() >= 2);
    const test::CutCoverage deep(tp, zTab - 1e-6);
    for (std::size_t c = 0; c < cuts.size(); ++c) {
        const bool finishing = c + 1 == cuts.size();
        for (const auto& run : test::raisedRuns(tp, cuts[c].first, cuts[c].second, zTab)) {
            for (const Vec2& p : test::runCore(run, r + 0.05, !finishing)) {
                INFO("cut " << c << " tab material at " << p.x << ", " << p.y);
                CHECK_FALSE(deep.covers(p, r - 1e-3));
            }
        }
    }
}

}  // namespace

TEST_CASE("Separate last pass: every finishing tab is covered by the roughing cut", "[toolpath][profile][tabs]") {
    // Verifier repro tabmin1: inside 100 x 50, 6 mm, 12 deep in 6 mm passes, last pass 0.1,
    // 7 tabs 4 x 3; one finishing tab goes round the top-right corner and the roughing cut used
    // to run straight through it at -12.
    ProfileParams p = baseParams(12.0, 6.0);
    p.side = ProfileSide::Inside;
    p.separateLastPass = true;
    p.lastPassAllowance = 0.1;
    p.tabs.enabled = true;
    p.tabs.count = 7;
    p.tabs.length = 4.0;
    p.tabs.thickness = 3.0;
    const ProfileResult r = profileToolpath({Contour::rectangle(Box2{{0, 0}, {100, 50}})}, test::endMill(6.0), p);
    REQUIRE(r.ok());
    CHECK(r.diagnostics.empty());
    const auto cuts = cutRanges(r.toolpath);
    REQUIRE(cuts.size() == 2);
    const double zTab = -9.0;
    CHECK(test::raisedRuns(r.toolpath, cuts[1].first, cuts[1].second, zTab).size() == 7);
    CHECK(test::raisedRuns(r.toolpath, cuts[0].first, cuts[0].second, zTab).size() == 7);
    checkTabMaterial(r.toolpath, zTab, 3.0);
}

TEST_CASE("Generative: with a separate last pass no move cuts below the tab top over tab material",
          "[toolpath][profile][tabs][property]") {
    test::SplitMix rng{0x7124ULL};
    int checked = 0;
    for (int s = 0; s < 160; ++s) {
        const int kind = static_cast<int>(rng.next() % 3);
        Contour c;
        if (kind == 0) {
            c = Contour::circle({0, 0}, rng.uniform(8, 30));
        } else if (kind == 1) {
            c = Contour::rectangle(Box2{{0, 0}, {rng.uniform(20, 120), rng.uniform(15, 80)}});
        } else {
            // Chamfered rectangle: corners where tabs meet short sloped edges.
            const double w = rng.uniform(30, 100);
            const double h = rng.uniform(20, 60);
            const double k = rng.uniform(2, 8);
            const Vec2 pts[8] = {{k, 0}, {w - k, 0}, {w, k}, {w, h - k}, {w - k, h}, {k, h}, {0, h - k}, {0, k}};
            c.closed = true;
            for (int i = 0; i < 8; ++i) {
                c.append(LineSpan{pts[i], pts[(i + 1) % 8]});
            }
        }
        const double d = std::vector<double>{3.175, 6.0, 8.0, 12.0}[rng.next() % 4];
        ProfileParams p = baseParams(rng.uniform(3, 20), rng.uniform(1, 8));
        p.side = rng.uniform(0, 1) < 0.5 ? ProfileSide::Outside : ProfileSide::Inside;
        p.direction = rng.uniform(0, 1) < 0.5 ? CutDirection::Climb : CutDirection::Conventional;
        p.allowance = rng.uniform(0, 1) < 0.5 ? 0.0 : rng.uniform(0, 0.4);
        p.separateLastPass = true;
        p.lastPassAllowance = rng.uniform(0.05, 1.0);
        p.lastPassReverse = rng.uniform(0, 1) < 0.5;
        p.tabs.enabled = true;
        p.tabs.spacing = TabSpacing::ConstantNumber;
        p.tabs.count = 1 + static_cast<int>(rng.next() % 9);
        p.tabs.length = rng.uniform(2, 20);
        p.tabs.thickness = rng.uniform(0.5, std::min(5.0, p.passes.cutDepth - 0.2));
        p.tabs.threeD = rng.uniform(0, 1) < 0.3;
        p.tabs.avoidCorners = rng.uniform(0, 1) < 0.5;
        const RampType ramps[] = {RampType::None, RampType::Smooth, RampType::Spiral};
        p.ramp.type = ramps[rng.next() % 3];
        p.ramp.distance = rng.uniform(2, 40);
        p.startAt = static_cast<StartAt>(rng.next() % 3);
        p.startAnchor = static_cast<BoxAnchor>(rng.next() % 9);
        const ProfileResult r = profileToolpath({c}, test::endMill(d), p);
        if (!r.ok() || cutRanges(r.toolpath).size() != 2) {
            continue;
        }
        INFO("scene " << s);
        const double zTab = p.passes.bottomZ() + p.tabs.thickness;
        checkTabMaterial(r.toolpath, zTab, d / 2);
        // kept + left out = requested (per vector, counted on the finishing cut).
        const auto cuts = cutRanges(r.toolpath);
        const std::size_t kept = test::raisedRuns(r.toolpath, cuts[1].first, cuts[1].second, zTab).size();
        CHECK(kept + tabsLeftOut(r) == static_cast<std::size_t>(p.tabs.count));
        ++checked;
    }
    CHECK(checked > 80);
}

TEST_CASE("Tab thickness beyond the cut depth is clamped and reported", "[toolpath][profile][tabs]") {
    ProfileParams p = baseParams(6.0, 3.0);
    p.tabs.enabled = true;
    p.tabs.count = 2;
    p.tabs.thickness = 20.0;
    const ProfileResult r = profileToolpath({square({0, 0}, 40.0)}, test::endMill(6.0), p);
    REQUIRE(r.ok());
    CHECK(r.has(ProfileIssue::TabThicknessClamped));
    for (std::size_t i = 1; i + 1 < r.toolpath.moves.size(); ++i) {
        if (isCutting(r.toolpath.moves[i].kind) && r.toolpath.moves[i].kind != MoveKind::Retract) {
            CHECK(r.toolpath.moves[i].to.z <= 0.0);  // never above the top of the cut
        }
    }
}

TEST_CASE("Leads keep the tool radius plus the allowance from every vector", "[toolpath][profile][leads]") {
    const Contour sq = Contour::rectangle(Box2{{0, 0}, {40, 40}});
    const Contour near = Contour::rectangle(Box2{{-14, 15}, {-9.5, 25}});
    ProfileParams p = baseParams(3.0, 3.0);
    p.allowance = 0.5;
    p.leads.type = LeadType::Straight;
    p.leads.length = 12.0;
    p.leads.angle = geometry::kPi / 2.0;
    p.startAt = StartAt::ClosestOnBoundingBox;
    p.startAnchor = BoxAnchor::CentreLeft;
    const ProfileResult r = profileToolpath({sq, near}, test::endMill(6.0), p);
    REQUIRE(r.ok());
    CHECK(test::minCutDistance(r.toolpath, {sq, near}) >= 3.5 - 1e-3);
}

namespace {

/// Checks, with the rasterised oracle, that every kept tab (every raised run at the tab top; with
/// a separate last pass those of the finishing cut) still joins the part to the waste. Returns
/// the number of tabs checked (those with waste beyond the kerf in their window).
int checkTabsJoin(const ProfileResult& r, const std::vector<Contour>& vectors, const ProfileParams& p, double d) {
    const auto cuts = cutRanges(r.toolpath);
    const double zTab = p.passes.bottomZ() + p.tabs.thickness;
    const double rad = d / 2;
    const bool on = p.side == ProfileSide::On || std::any_of(vectors.begin(), vectors.end(), [](const Contour& v) { return !v.closed; });
    // (On the vector the kerf reaches a radius + |allowance| (+ last pass) to each side.)
    const double kerfOuter = on ? std::fabs(p.allowance) + (p.separateLastPass ? p.lastPassAllowance : 0.0) + rad + 0.05
                                : p.allowance + (p.separateLastPass ? p.lastPassAllowance : 0.0) + d;
    const double res = std::max(0.1, d / 80.0);
    std::vector<Vec2> done;
    int checked = 0;
    const std::size_t first = p.separateLastPass ? cuts.size() / 2 : 0;
    for (std::size_t c = first; c < cuts.size(); ++c) {
        for (const auto& run : test::raisedRuns(r.toolpath, cuts[c].first, cuts[c].second, zTab)) {
            double len = 0.0;
            for (std::size_t i = 1; i < run.size(); ++i) len += std::hypot(run[i].x - run[i - 1].x, run[i].y - run[i - 1].y);
            const Vec2 mid = test::runCore(run, 0.0, true).front();
            if (std::any_of(done.begin(), done.end(), [&](Vec2 q) { return std::hypot(q.x - mid.x, q.y - mid.y) < 0.5; })) {
                continue;  // the same tab on another level
            }
            done.push_back(mid);
            const test::TabJoin j = test::tabJoins(r.toolpath, vectors, p.side == ProfileSide::Inside, mid,
                                                   0.5 * len + kerfOuter + 1.0, zTab, rad, kerfOuter, res, on);
            if (!j.slug) continue;
            INFO("tab at " << mid.x << ", " << mid.y << " (raised run " << len << " mm)");
            CHECK(j.joined);
            ++checked;
        }
    }
    return checked;
}

}  // namespace

TEST_CASE("Separate last pass: a short tab on a concave curve is not cut through by the roughing cut",
          "[toolpath][profile][tabs]") {
    // Verifier repro b5circle: Inside of a circle r 19.679, 8 mm tool, allowance 0.358, last pass
    // 0.892, one 0.62 x 4.31 tab. The roughing cut runs round the concave wall on a smaller radius,
    // so the roughing tool at a merely projected end of its raised stretch cut through the outer
    // half of the tab's bridge at full depth (no diagnostic).
    const std::vector<Contour> cs{Contour::circle({0, 0}, 19.679080419874751)};
    ProfileParams p;
    p.side = ProfileSide::Inside;
    p.allowance = 0.35808273924283024;
    p.passes.startDepth = 2.5890622067618838;
    p.passes.cutDepth = 6.6827760460028767;
    p.passes.passDepth = 5.5108253589101883;
    p.separateLastPass = true;
    p.lastPassAllowance = 0.89243563469697218;
    p.tabs.enabled = true;
    p.tabs.count = 1;
    p.tabs.length = 0.62026736706984131;
    p.tabs.thickness = 4.3090084694084512;
    p.ramp.type = RampType::ZigZag;
    p.ramp.distance = 36.75464170141808;
    p.leads.type = LeadType::Circular;
    p.leads.length = 8.2550718220314288;
    p.leads.angle = 0.69261347705451748;
    p.leads.radius = 6.3947391786514576;
    p.leads.sweep = 0.15589211200284489;
    p.startAt = StartAt::Optimise;
    p.startAnchor = BoxAnchor::TopRight;
    p.startPosition = {11.026666867948485, -6.302635438741838};
    const ProfileResult r = profileToolpath(cs, test::endMill(8.0), p);
    REQUIRE(r.ok());
    const auto cuts = cutRanges(r.toolpath);
    REQUIRE(cuts.size() == 2);
    const double zTab = p.passes.bottomZ() + p.tabs.thickness;
    // The tab is kept (on both passes) and still joins the part to the waste.
    CHECK(tabsLeftOut(r) == 0);
    CHECK(test::raisedRuns(r.toolpath, cuts[1].first, cuts[1].second, zTab).size() == 1);
    CHECK(test::raisedRuns(r.toolpath, cuts[0].first, cuts[0].second, zTab).size() == 2);  // two levels
    CHECK(checkTabsJoin(r, cs, p, 8.0) == 1);
    checkTabMaterial(r.toolpath, zTab, 4.0);
}

TEST_CASE("Separate last pass: a tab whose material a lobe of the roughing cut crosses is left out",
          "[toolpath][profile][tabs]") {
    // Verifier repro b5blob: Inside of a Bezier blob, 12 mm tool, allowance 0.49, last pass
    // 0.958, one 1.44 x 3.58 tab. Another stretch of the roughing cut (a lobe coming back) ran
    // through the far part of the tab's bridge, 8-13 mm out from the vector, at full depth.
    const double B[8][8] = {
        {24.528297700458356, -0.075131575306200141, 24.589931499642784, 5.5471493574607127, 21.023356708879575, 14.160905540386025, 16.933676131720073, 17.50028364950516},
        {16.933676131720073, 17.50028364950516, 12.843995554560571, 20.839661758624295, 4.6004164157089189, 21.080710980455006, -0.009785762498662717, 19.961137079408608},
        {-0.009785762498662717, 19.961137079408608, -4.6199879407062436, 18.841563178362211, -7.0160096946899637, 14.103805991146414, -10.727536937525418, 10.782840243226778},
        {-10.727536937525418, 10.782840243226778, -14.439064180360873, 7.461874495307141, -21.381104878470005, 4.4245520465845924, -22.278949219511389, 0.035342591890786854},
        {-22.278949219511389, 0.035342591890786854, -23.176793560552774, -4.3538668628030184, -19.86863318804566, -13.108168954594763, -16.114602983773732, -15.552416484936053},
        {-16.114602983773732, -15.552416484936053, -12.360572779501807, -17.996664015277343, -5.2011807139443693, -14.516645013130244, 0.24523200612016866, -14.630142590156956},
        {0.24523200612016866, -14.630142590156956, 5.6916447261847072, -14.743640167183667, 12.516695720890473, -18.65923711623811, 16.563873336613504, -16.233401947096318},
        {16.563873336613504, -16.233401947096318, 20.611050952336534, -13.807566777954527, 24.466663901273929, -5.6974125080731124, 24.528297700458356, -0.075131575306200141}};
    Contour blob;
    blob.closed = true;
    for (const auto& b : B) {
        blob.append(geometry::BezierSpan{{b[0], b[1]}, {b[2], b[3]}, {b[4], b[5]}, {b[6], b[7]}});
    }
    const std::vector<Contour> cs{blob};
    ProfileParams p;
    p.side = ProfileSide::Inside;
    p.direction = CutDirection::Climb;
    p.allowance = 0.49190493333618557;
    p.passes.cutDepth = 18.165175165326691;
    p.passes.passDepth = 5.9478725204244043;
    p.separateLastPass = true;
    p.lastPassAllowance = 0.95809446752021477;
    p.tabs.enabled = true;
    p.tabs.count = 1;
    p.tabs.length = 1.4386404699535462;
    p.tabs.thickness = 3.576056744455637;
    p.tabs.avoidCorners = true;
    p.leads.type = LeadType::Straight;
    p.leads.length = 14.197196335998006;
    p.leads.angle = 1.1481504767893238;
    p.leads.leadOut = true;
    p.startAt = StartAt::ClosestOnBoundingBox;
    p.startAnchor = BoxAnchor::TopLeft;
    p.startPosition = {39.967753505838715, -23.008829543971238};
    p.order.leftToRight = true;
    p.order.grid = true;
    const ProfileResult r = profileToolpath(cs, test::endMill(12.0), p);
    REQUIRE(r.ok());
    const auto cuts = cutRanges(r.toolpath);
    REQUIRE(cuts.size() == 2);
    const double zTab = p.passes.bottomZ() + p.tabs.thickness;
    // Kept tabs join the part to the waste; the rest are reported.
    const std::size_t kept = test::raisedRuns(r.toolpath, cuts[1].first, cuts[1].second, zTab).size();
    CHECK(kept + tabsLeftOut(r) == 1);
    checkTabsJoin(r, cs, p, 12.0);
    checkTabMaterial(r.toolpath, zTab, 6.0);
}

TEST_CASE("Short tabs centred on sharp convex corners are kept whole or left out and reported",
          "[toolpath][profile][tabs]") {
    // Verifier N1: Outside of 40 x 30, 12 mm tool, 4 tabs 1.5 mm long starting at the bottom
    // centre, so each lies on a corner arc; the tool at the end of the tab's own raised stretch
    // cut right through the middle of the tab's material.
    const std::vector<Contour> cs{Contour::rectangle(Box2{{0, 0}, {40, 30}})};
    for (const bool separate : {false, true}) {
        for (const double length : {0.5, 1.0, 1.5, 2.0, 3.0, 4.0}) {
            ProfileParams p = baseParams(6.0, 3.0);
            p.separateLastPass = separate;
            p.lastPassAllowance = 1.0;
            p.lastPassReverse = true;
            p.tabs.enabled = true;
            p.tabs.count = 4;
            p.tabs.length = length;
            p.tabs.thickness = 1.5;
            p.tabs.avoidCorners = false;
            p.startAt = StartAt::ClosestOnBoundingBox;
            p.startAnchor = BoxAnchor::BottomCentre;
            const ProfileResult r = profileToolpath(cs, test::endMill(12.0), p);
            REQUIRE(r.ok());
            INFO("separate " << separate << ", tab length " << length);
            const auto cuts = cutRanges(r.toolpath);
            const double zTab = p.passes.bottomZ() + p.tabs.thickness;
            const std::size_t kept = test::raisedRuns(r.toolpath, cuts.back().first, cuts.back().second, zTab).size();
            CHECK(kept + tabsLeftOut(r) == 4);  // one level (-6) below the tab top
            checkTabsJoin(r, cs, p, 12.0);
        }
    }
}

TEST_CASE("Generative: every kept tab still joins the part to the waste (rasterised)",
          "[toolpath][profile][tabs][property]") {
    test::SplitMix rng{0x7125ULL};
    int tabs = 0;
    int scenes = 0;
    for (int s = 0; s < 90; ++s) {
        const int kind = static_cast<int>(rng.next() % 4);
        Contour c;
        if (kind == 0) {
            c = Contour::circle({0, 0}, rng.uniform(8, 30));
        } else if (kind == 1) {
            c = Contour::rectangle(Box2{{0, 0}, {rng.uniform(20, 70), rng.uniform(15, 50)}});
        } else {
            // Rectangle with a round bite out of the bottom edge (kind 2) or a round bump on it
            // (kind 3): concave and convex curves meeting straight edges at corners.
            const double w = rng.uniform(40, 80);
            const double h = rng.uniform(30, 60);
            const double k = rng.uniform(5, std::min(0.3 * w, 0.4 * h));
            const Vec2 m{0.5 * w, 0.0};
            c.closed = true;
            c.append(LineSpan{{0, 0}, {m.x - k, 0}});
            c.append(ArcSpan{m, k, geometry::kPi, 0.0, kind == 2 ? geometry::ArcDirection::CW : geometry::ArcDirection::CCW});
            c.append(LineSpan{{m.x + k, 0}, {w, 0}});
            c.append(LineSpan{{w, 0}, {w, h}});
            c.append(LineSpan{{w, h}, {0, h}});
            c.append(LineSpan{{0, h}, {0, 0}});
        }
        const std::vector<Contour> cs{c};
        const double d = std::vector<double>{3.175, 6.0, 8.0, 12.0}[rng.next() % 4];
        ProfileParams p = baseParams(rng.uniform(3, 15), rng.uniform(1.5, 8));
        p.side = rng.uniform(0, 1) < 0.5 ? ProfileSide::Outside : ProfileSide::Inside;
        p.direction = rng.uniform(0, 1) < 0.5 ? CutDirection::Climb : CutDirection::Conventional;
        p.allowance = rng.uniform(0, 1) < 0.5 ? 0.0 : rng.uniform(0, 0.5);
        p.separateLastPass = rng.uniform(0, 1) < 0.5;
        p.lastPassAllowance = rng.uniform(0.05, 1.0);
        p.lastPassReverse = rng.uniform(0, 1) < 0.5;
        p.tabs.enabled = true;
        p.tabs.count = 1 + static_cast<int>(rng.next() % 6);
        p.tabs.length = rng.uniform(0.3, 4.0);
        p.tabs.thickness = rng.uniform(0.5, std::min(4.0, p.passes.cutDepth - 0.2));
        p.tabs.avoidCorners = rng.uniform(0, 1) < 0.3;
        const RampType ramps[] = {RampType::None, RampType::Smooth, RampType::ZigZag, RampType::Spiral};
        p.ramp.type = ramps[rng.next() % 4];
        p.ramp.distance = rng.uniform(2, 40);
        const LeadType leads[] = {LeadType::None, LeadType::Straight, LeadType::Circular};
        p.leads.type = leads[rng.next() % 3];
        p.leads.length = rng.uniform(2, 8);
        p.leads.radius = rng.uniform(2, 8);
        p.startAt = static_cast<StartAt>(rng.next() % 3);
        p.startAnchor = static_cast<BoxAnchor>(rng.next() % 9);
        p.startPosition = {rng.uniform(-10, 60), rng.uniform(-10, 60)};
        const ProfileResult r = profileToolpath(cs, test::endMill(d), p);
        if (!r.ok() || r.toolpath.isEmpty()) {
            continue;
        }
        INFO("scene " << s);
        tabs += checkTabsJoin(r, cs, p, d);
        ++scenes;
    }
    CHECK(scenes > 60);
    CHECK(tabs > 60);
}

TEST_CASE("Tabs at an inside corner of the cut are kept joined or left out and reported", "[toolpath][profile][tabs]") {
    // Verifier repro inside_corner: Inside of a 30 x 30 square, 6 mm tool, 4 tabs starting at the
    // bottom centre, so each is centred on a corner of the cut. Below about 2.5 mm the tool at the two
    // ends of the raised stretch stands less than a diameter apart across the corner and cuts the tab
    // through; such tabs used to be kept.
    const std::vector<Contour> cs{Contour::rectangle(Box2{{0, 0}, {30, 30}})};
    int joined = 0;
    for (const bool separate : {false, true}) {
        for (const double length : {0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 6.0}) {
            ProfileParams p = baseParams(6.0, 3.0);
            p.side = ProfileSide::Inside;
            p.separateLastPass = separate;
            p.lastPassAllowance = 0.3;
            p.tabs.enabled = true;
            p.tabs.count = 4;
            p.tabs.length = length;
            p.tabs.thickness = 1.5;
            p.startAt = StartAt::ClosestOnBoundingBox;
            p.startAnchor = BoxAnchor::BottomCentre;
            const ProfileResult r = profileToolpath(cs, test::endMill(6.0), p);
            REQUIRE(r.ok());
            INFO("separate " << separate << ", tab length " << length);
            const auto cuts = cutRanges(r.toolpath);
            const double zTab = p.passes.bottomZ() + p.tabs.thickness;
            const std::size_t kept = test::raisedRuns(r.toolpath, cuts.back().first, cuts.back().second, zTab).size();
            CHECK(kept + tabsLeftOut(r) == 4);
            joined += checkTabsJoin(r, cs, p, 6.0);
        }
    }
    CHECK(joined > 0);  // the longer tabs are kept (and joined)
}

TEST_CASE("Tabs in the roots of a gear are kept joined or left out and reported", "[toolpath][profile][tabs]") {
    // Verifier repro gear_valley: Outside of a 10-tooth gear, 6 mm tool, 5 short tabs that land on
    // the sharp concave corners of the cut (tooth roots); the tool at both ends of each raised
    // stretch stood less than a diameter apart and cut every tab through.
    const std::vector<Vec2> P = {
    {0x1.3818209cdf692p+3, 0x0p+0},
    {0x1.b2a9c04745aap+3, 0x1.b74926870a02ep+0},
    {0x1.a0acbdaeb6c7fp+3, 0x1.0ec58cc07e1ddp+2},
    {0x1.1a642d6d90a64p+3, 0x1.09c43ba8926c3p+2},
    {0x1.f8faa6523f088p+2, 0x1.6ee387e227531p+2},
    {0x1.3f5fc61a14a5cp+3, 0x1.2be989c076739p+3},
    {0x1.0184e84e27044p+3, 0x1.6271d9c84d2cep+3},
    {0x1.2cb4817e85828p+2, 0x1.117d8298d838fp+3},
    {0x1.81c50b6abf3ecp+1, 0x1.28d1bb8aa8a1cp+3},
    {0x1.48615e189be9dp+1, 0x1.ae5b85bba9a8ap+3},
    {0x1.e345980e16ec2p-51, 0x1.b61e26d01c3bep+3},
    {-0x1.38ed1eb5f824fp+0, 0x1.35a21f6071b6ep+3},
    {-0x1.81c50b6abf3eap+1, 0x1.28d1bb8aa8a1cp+3},
    {-0x1.75153c96ac05bp+2, 0x1.8c6bbacfd9929p+3},
    {-0x1.0184e84e27043p+3, 0x1.6271d9c84d2cep+3},
    {-0x1.ab4953583239ap+2, 0x1.c7038a67b4affp+2},
    {-0x1.f8faa6523f087p+2, 0x1.6ee387e227532p+2},
    {-0x1.7feceffeae017p+3, 0x1.a62134078a3a4p+2},
    {-0x1.a0acbdaeb6c7fp+3, 0x1.0ec58cc07e1dfp+2},
    {-0x1.3290f283a7fd2p+3, 0x1.d3d854bfbd1f9p+0},
    {-0x1.3818209cdf692p+3, 0x1.58428616b06b5p-50},
    {-0x1.b2a9c04745aap+3, -0x1.b74926870a03p+0},
    {-0x1.a0acbdaeb6c8p+3, -0x1.0ec58cc07e1dcp+2},
    {-0x1.1a642d6d90a63p+3, -0x1.09c43ba8926c4p+2},
    {-0x1.f8faa6523f089p+2, -0x1.6ee387e22753p+2},
    {-0x1.3f5fc61a14a5cp+3, -0x1.2be989c07673ap+3},
    {-0x1.0184e84e27045p+3, -0x1.6271d9c84d2cdp+3},
    {-0x1.2cb4817e85827p+2, -0x1.117d8298d838fp+3},
    {-0x1.81c50b6abf3efp+1, -0x1.28d1bb8aa8a1cp+3},
    {-0x1.48615e189be9ap+1, -0x1.ae5b85bba9a8bp+3},
    {-0x1.6a74320a91311p-49, -0x1.b61e26d01c3bep+3},
    {0x1.38ed1eb5f8253p+0, -0x1.35a21f6071b6ep+3},
    {0x1.81c50b6abf3e8p+1, -0x1.28d1bb8aa8a1cp+3},
    {0x1.75153c96ac059p+2, -0x1.8c6bbacfd992ap+3},
    {0x1.0184e84e27043p+3, -0x1.6271d9c84d2cfp+3},
    {0x1.ab49535832399p+2, -0x1.c7038a67b4b01p+2},
    {0x1.f8faa6523f087p+2, -0x1.6ee387e227534p+2},
    {0x1.7feceffeae017p+3, -0x1.a62134078a3a6p+2},
    {0x1.a0acbdaeb6c7fp+3, -0x1.0ec58cc07e1e1p+2},
    {0x1.3290f283a7fd2p+3, -0x1.d3d854bfbd1fep+0}};
    Contour gear;
    gear.closed = true;
    for (std::size_t i = 0; i < P.size(); ++i) {
        gear.append(LineSpan{P[i], P[(i + 1) % P.size()]});
    }
    const std::vector<Contour> cs{gear};
    for (const bool asFound : {true, false}) {
        ProfileParams p = baseParams(0x1.2b8f107f8b25ap+2, 0x1.58df19fd4f982p-1);
        p.side = ProfileSide::Outside;
        p.allowance = 0x1.3251f836c5991p-2;
        p.tabs.enabled = true;
        p.tabs.count = 5;
        p.tabs.length = 0x1.8d5f26853ba7ep-2;
        p.tabs.thickness = 0x1.6ad521abda6c3p+0;
        p.tabs.avoidCorners = true;
        if (asFound) {
            p.ramp.type = RampType::Smooth;
            p.ramp.distance = 0x1.e745ae236a21bp+4;
            p.ramp.onLeadIn = true;
            p.leads.overcut = 0x1.270666784b5e4p+3;
        }
        const ProfileResult r = profileToolpath(cs, test::endMill(6.0), p);
        REQUIRE(r.ok());
        INFO("as found " << asFound);
        const auto cuts = cutRanges(r.toolpath);
        const double zTab = p.passes.bottomZ() + p.tabs.thickness;
        std::vector<Vec2> mids;
        for (const auto& run : test::raisedRuns(r.toolpath, cuts.back().first, cuts.back().second, zTab)) {
            const Vec2 m = test::runCore(run, 0.0, true).front();
            if (std::none_of(mids.begin(), mids.end(), [&](Vec2 q) { return std::hypot(q.x - m.x, q.y - m.y) < 0.5; })) {
                mids.push_back(m);
            }
        }
        CHECK(mids.size() + tabsLeftOut(r) == 5);
        checkTabsJoin(r, cs, p, 6.0);
    }
}

TEST_CASE("A tab starting at a V-bit sharp inside corner is cut flat at the tab top", "[toolpath][profile][tabs][vbit]") {
    // Verifier repro vbit_tab_slope: Inside of a D shape, 12 mm 120-degree V-bit with sharp internal
    // corners. The kept tab's raised stretch starts exactly at the corner where the corner climb is
    // inserted; the tool came back from the climb at the bottom and fed on a slope up to the tab top
    // at the far end, so the tab was never flat and was cut below its top all along.
    Contour c;
    c.closed = true;
    c.append(ArcSpan{{0, 0}, 33.069952603275667, -0.82891326789637176, -2.3126793856934218, geometry::ArcDirection::CCW});
    c.append(LineSpan{{-22.344616124565331, -24.37908725585751}, {22.344616124565327, -24.379087255857513}});
    ProfileParams p;
    p.side = ProfileSide::Inside;
    p.allowance = 0.47890384266872976;
    p.passes.cutDepth = 5.8704981320714289;
    p.passes.passDepth = 2.3961106328433783;
    p.passes.lastPassThickness = 0.69316655279558259;
    p.tabs.enabled = true;
    p.tabs.count = 5;
    p.tabs.length = 3.6268170792542103;
    p.tabs.thickness = 1.2531515480900344;
    p.tabs.avoidCorners = true;
    p.ramp.type = RampType::Spiral;
    p.ramp.distance = 29.230956653218364;
    p.leads.type = LeadType::Straight;
    p.leads.length = 8.7415901278800483;
    p.leads.angle = 0.91409126210320324;
    p.corners.sharpExternal = true;
    p.corners.sharpInternal = true;
    p.startPosition = {-1.3579110524131295, 24.403258302710057};
    tooling::Tool vbit = test::endMill(12.0);
    vbit.type = tooling::ToolType::VBit;
    vbit.includedAngle = 120.0 * geometry::kPi / 180.0;
    const ProfileResult r = profileToolpath({c}, vbit, p);
    REQUIRE(r.ok());
    const double zTab = p.passes.bottomZ() + p.tabs.thickness;
    const auto& mv = r.toolpath.moves;
    // No move climbs (or descends) along the path into or out of the tab top: every raised stretch
    // is entered and left vertically.
    for (std::size_t i = 1; i < mv.size(); ++i) {
        const Vec3 a = mv[i - 1].to;
        const Vec3 b = mv[i].to;
        const bool alongPath = (mv[i].kind == MoveKind::Feed || isArc(mv[i].kind)) && std::hypot(b.x - a.x, b.y - a.y) > 0.01;
        const bool crossesTabTop = (a.z < zTab - 1e-9 && b.z >= zTab - 1e-9) || (a.z >= zTab - 1e-9 && b.z < zTab - 1e-9);
        INFO("move " << i);
        CHECK_FALSE((alongPath && crossesTabTop && a.z > p.passes.bottomZ() - 1e-9 && std::fabs(b.z - zTab) < 1e-9));
    }
    // Every tab is kept flat (a raised run at the tab top on the last level) or reported.
    const auto cuts = cutRanges(r.toolpath);
    std::vector<Vec2> mids;
    for (const auto& run : test::raisedRuns(r.toolpath, cuts.back().first, cuts.back().second, zTab)) {
        const Vec2 m = test::runCore(run, 0.0, true).front();
        if (std::none_of(mids.begin(), mids.end(), [&](Vec2 q) { return std::hypot(q.x - m.x, q.y - m.y) < 0.5; })) {
            mids.push_back(m);
        }
    }
    CHECK(mids.size() + tabsLeftOut(r) == 5);
    // No cut below the tab top along the middle of any kept tab.
    const test::CutCoverage deep(r.toolpath, zTab - 1e-6);
    for (const auto& run : test::raisedRuns(r.toolpath, cuts.back().first, cuts.back().second, zTab)) {
        for (const Vec2& q : test::runCore(run, 0.0, true)) {
            CHECK_FALSE(deep.covers(q, 0.05));
        }
    }
}

namespace {

/// Reads a verifier scene dump (contours, then the ProfileParams fields it varies; hexadecimal
/// floats). Returns the tool diameter; `vbitDeg` is non-zero for a V-bit.
double readScene(const std::string& text, std::vector<Contour>& cs, ProfileParams& p, int& vbitDeg) {
    std::istringstream in(text);
    std::string tag;
    auto num = [&]() {
        std::string t;
        in >> t;
        return std::strtod(t.c_str(), nullptr);
    };
    auto integer = [&]() { return static_cast<int>(num()); };
    double d = 0.0;
    while (in >> tag) {
        if (tag == "C") {
            cs.emplace_back();
            cs.back().closed = integer() != 0;
        } else if (tag == "L") {
            const double a = num(), b = num(), c = num(), e = num();
            cs.back().spans.push_back(LineSpan{{a, b}, {c, e}});
        } else if (tag == "A") {
            const double a = num(), b = num(), c = num(), e = num(), f = num();
            cs.back().spans.push_back(ArcSpan{{a, b}, c, e, f, static_cast<geometry::ArcDirection>(integer())});
        } else if (tag == "B") {
            double v[8];
            for (double& x : v) x = num();
            cs.back().spans.push_back(geometry::BezierSpan{{v[0], v[1]}, {v[2], v[3]}, {v[4], v[5]}, {v[6], v[7]}});
        } else if (tag == "P") {
            d = num();
            vbitDeg = integer();
            p.allowance = num();
            p.side = static_cast<ProfileSide>(integer());
            p.direction = static_cast<CutDirection>(integer());
            p.passes.startDepth = num();
            p.passes.cutDepth = num();
            p.passes.passDepth = num();
            p.passes.lastPassThickness = num();
            p.safeZ = num();
            p.separateLastPass = integer() != 0;
            p.lastPassAllowance = num();
            p.lastPassReverse = integer() != 0;
            p.tabs.enabled = integer() != 0;
            p.tabs.length = num();
            p.tabs.thickness = num();
            p.tabs.spacing = static_cast<TabSpacing>(integer());
            p.tabs.count = integer();
            p.tabs.distance = num();
            num();
            p.tabs.avoidCorners = integer() != 0;
            p.tabs.threeD = integer() != 0;
            num();
            p.ramp.type = static_cast<RampType>(integer());
            p.ramp.distance = num();
            p.ramp.useAngle = integer() != 0;
            p.ramp.onLeadIn = integer() != 0;
            p.ramp.angle = num();
            p.leads.length = num();
            p.leads.angle = num();
            p.leads.radius = num();
            p.leads.type = static_cast<LeadType>(integer());
            p.leads.sweep = num();
            p.leads.leadOut = integer() != 0;
            integer();
            p.leads.overcut = num();
            num();
            p.startAt = static_cast<StartAt>(integer());
            p.startAnchor = static_cast<BoxAnchor>(integer());
            p.order.selectionOrder = integer() != 0;
            p.order.leftToRight = integer() != 0;
            p.order.bottomToTop = integer() != 0;
            p.order.grid = integer() != 0;
            p.startPosition.x = num();
            p.startPosition.y = num();
            p.corners.mitreLimit = num();
        } else if (tag == "K") {
            p.corners.sharpExternal = integer() != 0;
            p.corners.sharpInternal = integer() != 0;
        }
    }
    return d;
}

}  // namespace

TEST_CASE("On the vector: tabs at sharp turns of the cut are kept joined or left out and reported",
          "[toolpath][profile][tabs]") {
    // Verifier repro on_corner: the 30 x 30 square cut On the vector, 6 mm tool, 4 tabs starting
    // at the bottom centre, so each is centred on a corner. Cut::distance is |allowance| On the
    // vector, so with an allowance the tab check took the profile for an Inside/Outside one and
    // kept tabs whose raised-stretch ends stand 5 mm apart (< 2r): cut through.
    const std::vector<Contour> cs{Contour::rectangle(Box2{{0, 0}, {30, 30}})};
    int joined = 0;
    for (const double allowance : {0.3, 0.1, 0.0, -0.3}) {
        for (const bool separate : {false, true}) {
            for (const double length : {1.0, 3.0, 6.0}) {
                ProfileParams p = baseParams(6.0, 3.0);
                p.side = ProfileSide::On;
                p.allowance = allowance;
                p.separateLastPass = separate;
                p.lastPassAllowance = 0.3;
                p.tabs.enabled = true;
                p.tabs.count = 4;
                p.tabs.length = length;
                p.tabs.thickness = 1.5;
                p.startAt = StartAt::ClosestOnBoundingBox;
                p.startAnchor = BoxAnchor::BottomCentre;
                const ProfileResult r = profileToolpath(cs, test::endMill(6.0), p);
                REQUIRE(r.ok());
                INFO("allowance " << allowance << ", separate " << separate << ", tab length " << length);
                const auto cuts = cutRanges(r.toolpath);
                const double zTab = p.passes.bottomZ() + p.tabs.thickness;
                const std::size_t kept = test::raisedRuns(r.toolpath, cuts.back().first, cuts.back().second, zTab).size();
                CHECK(kept + tabsLeftOut(r) == 4);
                joined += checkTabsJoin(r, cs, p, 6.0);
            }
        }
    }
    CHECK(joined > 0);  // 6 mm tabs are kept (and joined)
}

TEST_CASE("On the vector: verifier sweep scenes keep no severed tab", "[toolpath][profile][tabs]") {
    // Scenes from the verifier's round-3 connectivity sweeps (conn6 dumps) whose kept tabs On the
    // vector were cut through, and F9 (an open curve On the vector).
    const std::vector<std::pair<std::string, std::string>> scenes{
        {"sev_D_81682_34",
         "C 1\n"
         "L 0x1.71646141fb01bp-1 0x0p+0 0x1.327114df89b15p+6 0x0p+0\n"
         "L 0x1.327114df89b15p+6 0x0p+0 0x1.3553dda20da75p+6 0x1.71646141fb01bp-1\n"
         "L 0x1.3553dda20da75p+6 0x1.71646141fb01bp-1 0x1.3553dda20da75p+6 0x1.d826ea421356p+5\n"
         "L 0x1.3553dda20da75p+6 0x1.d826ea421356p+5 0x1.327114df89b15p+6 0x1.ddec7bc71b42p+5\n"
         "L 0x1.327114df89b15p+6 0x1.ddec7bc71b42p+5 0x1.71646141fb01bp-1 0x1.ddec7bc71b42p+5\n"
         "L 0x1.71646141fb01bp-1 0x1.ddec7bc71b42p+5 0x0p+0 0x1.d826ea421356p+5\n"
         "L 0x0p+0 0x1.d826ea421356p+5 0x0p+0 0x1.71646141fb01bp-1\n"
         "L 0x0p+0 0x1.71646141fb01bp-1 0x1.71646141fb01bp-1 0x0p+0\n"
         "P 0x1.8p+3 0 0x1.0bcbffb0af359p-4 2 1 0x0p+0 0x1.97e6a5ee333bep+3 0x1.55e74e2f9400ep-1 0x0p+0 0x1.4p+2 0 0x1.0c1d5c97ee265p-1 1 1 0x1.c03232f23e528p+1 0x1.0015e1e4ddbccp+0 0 1 0x1.9ab520c4075fp+4 0x0p+0 0 0 0x0p+0 1 0x1.297928fb2b2f5p+5 0 0 0x1.eb29bd255ad36p-2 0x1.b3e5a9f605bfdp+3 0x1.47231b7cb4472p+0 0x1.d75fa7bb036f8p+1 0 0x1.59c8a64ec6127p+0 1 0 0x1.4c6628f8fdd15p+1 0x0p+0 0 5 0 1 0 0 -0x1.8a04e3acaf6p-4 0x1.13b9d863a99fap+5 0x1p+1\n"
         "K 0 0\n"},
        {"sev_D_81682_42",
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.8d9c16f36c8eap+6 0x0p+0\n"
         "L 0x1.8d9c16f36c8eap+6 0x0p+0 0x1.8d9c16f36c8eap+6 0x1.80421eef8cb07p+5\n"
         "L 0x1.8d9c16f36c8eap+6 0x1.80421eef8cb07p+5 0x0p+0 0x1.80421eef8cb07p+5\n"
         "L 0x0p+0 0x1.80421eef8cb07p+5 0x0p+0 0x0p+0\n"
         "P 0x1.8p+3 0 0x1.dc30bedbaa527p-2 2 1 0x0p+0 0x1.330d92a272917p+4 0x1.dc475c1741ac1p+0 0x0p+0 0x1.4p+2 0 0x1.2c3c9911c3ecbp-1 1 1 0x1.32061a463dce8p+1 0x1.23c1687bf0eaep+2 0 6 0x1.fb4a1d6f61bccp+5 0x0p+0 0 0 0x0p+0 0 0x1.e64be69aab16bp+4 1 0 0x1.a4a3b92a33ba9p-3 0x1.9295ab7cb33cp+2 0x1.623522270ab8p+0 0x1.4f27dc9c50dc4p+3 1 0x1.847975846d041p-1 0 0 0x0p+0 0x0p+0 2 5 0 0 0 0 -0x1.dee64998177cp+3 0x1.6490db1c7c71ep+5 0x1p+1\n"
         "K 0 0\n"},
        {"sev_J_81741_45",
         "C 1\n"
         "B 0x1.e117437138716p+3 0x1.a2f7cf99ee104p-2 0x1.d51b1b2d89a1cp+3 0x1.d73b55e8bdbbap+1 0x1.1165816022ac9p+3 0x1.82d1b3cb330bp+2 0x1.314b7e441af26p+2 0x1.0afc6dc78b7edp+3\n"
         "B 0x1.314b7e441af26p+2 0x1.0afc6dc78b7edp+3 0x1.fe5fce3f845dp-1 0x1.549001a97d782p+3 -0x1.1eb19a64de61ep+2 0x1.f2e09b3a461dfp+3 -0x1.e6ce94068d85cp+2 0x1.c68d35c87b485p+3\n"
         "B -0x1.e6ce94068d85cp+2 0x1.c68d35c87b485p+3 -0x1.5775c6d41e54dp+3 0x1.9a39d056b072bp+3 -0x1.c0f77ad0c426fp+3 0x1.1c671580912ebp+2 -0x1.bfb12dc2fff29p+3 0x1.080d1cca7ce81p-5\n"
         "B -0x1.bfb12dc2fff29p+3 0x1.080d1cca7ce81p-5 -0x1.be6ae0b53bbe3p+3 -0x1.1846e10d673b1p+2 -0x1.5bcc20c533ea3p+3 -0x1.4c140a80e42d8p+3 -0x1.d782f7615b114p+2 -0x1.8877bc0c79565p+3\n"
         "B -0x1.d782f7615b114p+2 -0x1.8877bc0c79565p+3 -0x1.eedb5a709c9c3p+1 -0x1.c4db6d980e7f2p+3 0x1.a454ee1f556p+1 -0x1.ace605ea4098ep+3 0x1.c11d61704caep+2 -0x1.694e1c28b477cp+3\n"
         "B 0x1.c11d61704caep+2 -0x1.694e1c28b477cp+3 0x1.580825e87756p+3 -0x1.25b632672856ap+3 0x1.ed136bb4e741p+3 -0x1.6e7d62024237ap+1 0x1.e117437138716p+3 0x1.a2f7cf99ee104p-2\n"
         "P 0x1p+3 0 0x1.5e377fb98562bp-2 2 0 0x0p+0 0x1.a4d082e46868p+2 0x1.a01a79d6f566ep-2 0x0p+0 0x1.4p+2 0 0x1.e3af922e8dc07p-4 1 1 0x1.f60409f1faap-1 0x1.ca0eafdb7672dp+2 1 9 0x1.7b03a6fa62c56p+5 0x0p+0 1 0 0x0p+0 0 0x1.25353c4a86512p+5 0 0 0x1.1f48e8138f3b7p-2 0x1.902af42b91e08p+2 0x1.9e9637db27c46p-1 0x1.52ad558017531p+3 1 0x1.2a331b2abefdfp+0 1 0 0x1.185c1d1f14398p+1 0x0p+0 1 6 0 0 0 0 0x1.1c8873bb08adcp+5 0x1.15b88ff0832bcp+5 0x1p+1\n"
         "K 0 0\n"},
        {"sev_C_81672_29",
         "C 1\n"
         "A 0x1.dc56dc6823964p+0 0x1.358b4651b7b4ep-1 0x1.8408163a90d43p+2 -0x1.991dfd9b8a79cp-4 0x1.74d6b71d405d4p-1 0\n"
         "A 0x1.0c842e74ac703p+2 0x1.7194be341497p+2 0x1.3bc6c2d2d16ddp+1 -0x1.e9912136c1565p-2 0x1.2e785db633755p+1 0\n"
         "A 0x1.c36714445575ap-53 0x1.8929f7a60670cp+2 0x1.65b79d4bcfcc6p+1 0x1.050c962c26e1ap-1 0x1.50dc8fb939191p+1 0\n"
         "A -0x1.3e19ff6c14257p+2 0x1.b5d4483e968b1p+2 0x1.4f034c5f90772p+1 0x1.077d870fddc18p-2 -0x1.1235ea715702cp+1 1\n"
         "A -0x1.71ad7416f507dp+2 0x1.e0764a60f284dp+0 0x1.6a2808a7743d9p+1 0x1.c9bad9b6910fdp+0 -0x1.354a2ab5bc4b7p+1 0\n"
         "A -0x1.f81fbc997897dp+2 -0x1.4799806512dc4p+1 0x1.479b40a3af9dep+1 0x1.93c7312c5600fp+0 -0x1.e5db6aeedcf41p-1 1\n"
         "A -0x1.29d5d447a6b67p+1 -0x1.99ef617f375bbp+1 0x1.13955843b9ea6p+2 -0x1.66920b1543cbfp+1 -0x1.98ce4bc7cd194p+0 0\n"
         "A -0x1.c1f3499bab82p-50 -0x1.187201ddece31p+3 0x1.5f39cd37e405p+1 0x1.5539e98e323c1p+1 0x1.e72e5db084aafp-2 1\n"
         "A 0x1.7ec45f81869e4p+0 -0x1.076ab66a02664p+1 0x1.61f1857f502dfp+2 -0x1.6635a6be3f645p+0 -0x1.f15b3181dcc3fp-2 0\n"
         "A 0x1.9c09378b08907p+2 -0x1.0bc1d9964601ap+1 0x1.4630443364df6p+1 -0x1.975bd4e5fa98ep+0 0x1.ed04b2622623dp-1 0\n"
         "P 0x1p+3 0 0x0p+0 2 1 0x1.48f3223035718p-1 0x1.1878d8010ee08p+3 0x1.327285ff20ddcp+2 0x0p+0 0x1.4p+2 0 0x1.2ab1e3dd1931bp-4 1 1 0x1.2e6bd13d43da9p+0 0x1.73797b7064cep+1 1 3 0x1.563b5a6d288bdp+5 0x0p+0 0 1 0x0p+0 3 0x1.b13d72c179452p+4 0 0 0x1.21845ec09b645p-4 0x1.02697ed8593e6p+3 0x1.93897fcc94645p-1 0x1.693fe15bb9e66p+0 1 0x1.b266a92b17d24p-3 0 0 0x1.de20fc572ab6ap+3 0x0p+0 0 3 0 0 0 0 0x1.d45fe91deee4p+4 -0x1.8f1f697b3df29p+5 0x1p+1\n"
         "K 0 0\n"},
        {"F9 (conn seed 41006 fam 5 scene 9)",
         "C 0\n"
         "B 0x0p+0 0x0p+0 0x1.d89cebd797a35p+2 0x1.218fb996f4a91p+2 0x1.d89cebd797a36p+3 0x1.77c3fced7980ep+4 0x1.6275b0e1b1ba8p+4 0x1.2f600e87bc56ap+4\n"
         "B 0x1.6275b0e1b1ba8p+4 0x1.2f600e87bc56ap+4 0x1.d89cebd797a35p+4 0x1.f0c8061601802p+4 0x1.27621366bec61p+5 0x1.b10b3cd1301d4p+4 0x1.6275b0e1b1ba8p+5 0x1.df468a85d5e78p+3\n"
         "B 0x1.6275b0e1b1ba8p+5 0x1.df468a85d5e78p+3 0x1.9d894e5ca4aefp+5 0x1.b835369f04db4p+4 0x1.d89cebd797a35p+5 0x1.7312aa33f83d8p+4 0x1.09d844a9454bep+6 0x1.550171afbcacp+3\n"
         "B 0x1.09d844a9454bep+6 0x1.550171afbcacp+3 0x1.27621366bec61p+6 0x1.6e61bc575c023p+2 0x1.44ebe22438405p+6 -0x1.a910e2320e61fp+2 0x1.6275b0e1b1ba8p+6 -0x1.b5beeca7c4308p+0\n"
         "P 0x1p+3 0 0x1.3947efce62fd3p-2 2 1 0x0p+0 0x1.aa71e0949fc39p+3 0x1.2e3005e3e1522p+2 0x0p+0 0x1.4p+2 0 0x1.d787878de93a3p-1 0 1 0x1.123e5d3488c0dp+2 0x1.648209a2b6704p-1 0 9 0x1.56687611a2e48p+6 0x0p+0 0 0 0x0p+0 3 0x1.86d28ddb82351p+4 0 0 0x1.01051c12feb17p-1 0x1.9c92f218c1be4p+1 0x1.8ab4d04cfa66cp-3 0x1.730f85bab12ep+0 0 0x1.ae74078f39455p-1 0 0 0x0p+0 0x0p+0 2 5 0 0 0 0 0x1.3a19161677f3ap+5 -0x1.7a50bc3137ba6p+4 0x1p+1\n"
         "K 0 0\n"},
    };
    for (const auto& [name, text] : scenes) {
        std::vector<Contour> cs;
        ProfileParams p;
        int vbitDeg = 0;
        const double d = readScene(text, cs, p, vbitDeg);
        REQUIRE(vbitDeg == 0);
        REQUIRE(p.side == ProfileSide::On);
        INFO("scene " << name);
        const ProfileResult r = profileToolpath(cs, test::endMill(d), p);
        REQUIRE(r.ok());
        checkTabsJoin(r, cs, p, d);
    }
}

// ------------------------------------------------------- T-0127: exact tab check On the vector

namespace {

/// Parameters of the T-0127 square repros: 30 x 30 square, cut depth 6 in two passes, 4 tabs of
/// thickness 1.5 starting at `anchor` (bottom centre: one tab centred on each corner).
ProfileParams squareTabParams(ProfileSide side, double allowance, bool separate, double length, BoxAnchor anchor) {
    ProfileParams p = baseParams(6.0, 3.0);
    p.side = side;
    p.allowance = allowance;
    p.separateLastPass = separate;
    p.lastPassAllowance = 0.3;
    p.tabs.enabled = true;
    p.tabs.count = 4;
    p.tabs.length = length;
    p.tabs.thickness = 1.5;
    p.startAt = StartAt::ClosestOnBoundingBox;
    p.startAnchor = anchor;
    return p;
}

/// Tabs kept on the last cut (raised runs at the tab top on its final level).
std::size_t keptOnLastCut(const ProfileResult& r, const ProfileParams& p) {
    const auto cuts = cutRanges(r.toolpath);
    return test::raisedRuns(r.toolpath, cuts.back().first, cuts.back().second, p.passes.bottomZ() + p.tabs.thickness).size();
}

}  // namespace

TEST_CASE("On the vector: a tab centred on a corner is kept exactly when it holds", "[toolpath][profile][tabs]") {
    // T-0127 AC1/AC2 (repro on_corner): the T-0124 guard dropped every short On tab on a sharp
    // turn. The tool at the two ends of the raised stretch (L/2 + r along each edge from the
    // corner) stands (L/2 + r)·√2 apart, so with r = 3 the tab is cut through for L < 2.49 and
    // holds above (with a neck of at least ~0.1 mm from L = 3). Every kept tab is checked with an
    // independent flood across the corner; L >= 3 must be kept for every allowance and pass mode.
    const std::vector<Contour> cs{Contour::rectangle(Box2{{0, 0}, {30, 30}})};
    for (const double allowance : {-0.3, 0.0, 0.3}) {
        for (const bool separate : {false, true}) {
            for (const double length : {1.0, 2.0, 3.0, 4.0}) {
                const ProfileParams p = squareTabParams(ProfileSide::On, allowance, separate, length, BoxAnchor::BottomCentre);
                const ProfileResult r = profileToolpath(cs, test::endMill(6.0), p);
                REQUIRE(r.ok());
                INFO("allowance " << allowance << ", separate " << separate << ", tab length " << length);
                const std::size_t kept = keptOnLastCut(r, p);
                CHECK(kept + tabsLeftOut(r) == 4);
                const double zTab = p.passes.bottomZ() + p.tabs.thickness;
                const bool held = test::floodHeld(r.toolpath, Box2{{-8, -8}, {8, 8}}, zTab, 3.0, {5, 5}, {-5, -5});
                CHECK(held == (kept == 4));
                if (length >= 3.0) {
                    CHECK(kept == 4);
                } else {
                    CHECK(kept == 0);
                }
            }
        }
    }
}

TEST_CASE("On the vector: short tabs on straight edges are kept and hold", "[toolpath][profile][tabs]") {
    // T-0127 AC6 (driver straight.cpp): the T-0124 guard dropped every On tab shorter than
    // 2|allowance| + max(0.25, 0.05 D), e.g. D 12, L 1, allowance ±0.3, on a straight edge. Two
    // tabs on the 100 x 60 rectangle land at x = 80 on the bottom edge and x = 20 on the top edge.
    const std::vector<Contour> cs{Contour::rectangle(Box2{{0, 0}, {100, 60}})};
    for (const double d : {6.0, 12.0}) {
        for (const double allowance : {-0.3, 0.0, 0.3}) {
            for (const bool threeD : {false, true}) {
                ProfileParams p = baseParams(8.0, 3.0);
                p.side = ProfileSide::On;
                p.allowance = allowance;
                p.tabs.enabled = true;
                p.tabs.count = 2;
                p.tabs.length = 1.0;
                p.tabs.thickness = 1.5;
                p.tabs.threeD = threeD;
                p.startAt = StartAt::ClosestOnBoundingBox;
                p.startAnchor = BoxAnchor::BottomLeft;
                const ProfileResult r = profileToolpath(cs, test::endMill(d), p);
                REQUIRE(r.ok());
                INFO("D " << d << ", allowance " << allowance << ", 3D " << threeD);
                CHECK(keptOnLastCut(r, p) == 2);
                CHECK(tabsLeftOut(r) == 0);
                const double zTab = p.passes.bottomZ() + p.tabs.thickness;
                const double h = 0.5 + d / 2 + 1.5;  // a window just past the raised stretch
                const double off = d / 2 + 0.3 + 1.2;
                for (const Vec2 m : {Vec2{80, 0}, Vec2{20, 60}}) {
                    INFO("tab at " << m.x << ", " << m.y);
                    CHECK(test::floodHeld(r.toolpath, Box2{{m.x - h, m.y - h}, {m.x + h, m.y + h}}, zTab, d / 2,
                                          {m.x, m.y + off}, {m.x, m.y - off}));
                }
            }
        }
    }
}

TEST_CASE("Negative allowance: a tab is kept only if the real cut leaves it joined", "[toolpath][profile][tabs]") {
    // T-0127 AC5 (verifier t0124v5, neg2_s0_da*.scn): Outside/Inside of a 30 x 30 square with a
    // 6 mm end mill and an allowance between -r and about -r + 0.5 put the tool centre right by the
    // vector; the tab check measured the material from the vector (no longer the kerf wall) and kept
    // short corner tabs that the tool cut through, with no diagnostic. Tabs centred on the corners
    // (bottom-centre start) are checked at the corner (0, 0), mid-edge tabs (bottom-left start) on
    // the bottom edge, each with an independent flood from the part to the waste.
    const std::vector<Contour> cs{Contour::rectangle(Box2{{0, 0}, {30, 30}})};
    for (const ProfileSide side : {ProfileSide::Outside, ProfileSide::Inside}) {
        const double in = side == ProfileSide::Outside ? 1.0 : -1.0;  // towards the part
        for (const double allowance : {-3.0, -2.99, -2.9, -2.5, -2.0, -1.0, -0.2}) {
            for (const bool separate : {false, true}) {
                INFO("side " << static_cast<int>(side) << ", allowance " << allowance << ", separate " << separate);
                {
                    const ProfileParams p = squareTabParams(side, allowance, separate, 1.0, BoxAnchor::BottomCentre);
                    const ProfileResult r = profileToolpath(cs, test::endMill(6.0), p);
                    REQUIRE(r.ok());
                    const std::size_t kept = keptOnLastCut(r, p);
                    CHECK(kept + tabsLeftOut(r) == 4);
                    const double zTab = p.passes.bottomZ() + p.tabs.thickness;
                    const Vec2 part = side == ProfileSide::Outside ? Vec2{6.5, 6.5} : Vec2{-6.5, -6.5};
                    const Vec2 waste = side == ProfileSide::Outside ? Vec2{-6.5, -6.5} : Vec2{6.5, 6.5};
                    const bool held = test::floodHeld(r.toolpath, Box2{{-9.5, -9.5}, {9.5, 9.5}}, zTab, 3.0, part, waste);
                    CHECK(held == (kept == 4));
                }
                {
                    ProfileParams p = squareTabParams(side, allowance, separate, 4.0, BoxAnchor::BottomLeft);
                    const ProfileResult r = profileToolpath(cs, test::endMill(6.0), p);
                    REQUIRE(r.ok());
                    CHECK(keptOnLastCut(r, p) == 4);  // mid-edge tabs hold
                    CHECK(tabsLeftOut(r) == 0);
                    const double zTab = p.passes.bottomZ() + p.tabs.thickness;
                    // The tab on the bottom edge (centre x = 15); window just past its raised stretch.
                    CHECK(test::floodHeld(r.toolpath, Box2{{15 - 6.5, -9}, {15 + 6.5, 9}}, zTab, 3.0, {15, 7.5 * in},
                                          {15, -7.5 * in}));
                }
            }
        }
    }
}

TEST_CASE("Allowance of exactly minus the radius keeps the profile side", "[toolpath][profile][tabs]") {
    // T-0127 AC5 (driver negallow.cpp): with r + allowance = 0 the cut runs on the vector and the
    // sign of the (zero) offset decided the side, so Inside turned into Outside there: the
    // direction of cut and the part side of the tabs flipped against an allowance 1e-9 larger.
    const std::vector<Contour> cs{Contour::rectangle(Box2{{0, 0}, {30, 30}}), Contour::circle({60, 15}, 10)};
    for (const ProfileSide side : {ProfileSide::Outside, ProfileSide::Inside}) {
        for (const double d : {3.175, 6.0}) {
            for (const bool separate : {false, true}) {
                INFO("side " << static_cast<int>(side) << ", D " << d << ", separate " << separate);
                ProfileParams p = squareTabParams(side, -d / 2, separate, 5.0, BoxAnchor::BottomCentre);
                const ProfileResult exact = profileToolpath(cs, test::endMill(d), p);
                p.allowance = -d / 2 + 1e-9;
                const ProfileResult above = profileToolpath(cs, test::endMill(d), p);
                REQUIRE(exact.ok());
                REQUIRE(exact.toolpath.moves.size() == above.toolpath.moves.size());
                CHECK(exact.diagnostics.size() == above.diagnostics.size());
                double worst = 0.0;
                for (std::size_t i = 0; i < exact.toolpath.moves.size(); ++i) {
                    const Move& a = exact.toolpath.moves[i];
                    const Move& b = above.toolpath.moves[i];
                    CHECK(a.kind == b.kind);
                    worst = std::max({worst, std::fabs(a.to.x - b.to.x), std::fabs(a.to.y - b.to.y), std::fabs(a.to.z - b.to.z)});
                }
                CHECK(worst < 1e-6);
            }
        }
    }
}

TEST_CASE("Generative: every kept On tab joins both sides, allowance - 0 +", "[toolpath][profile][tabs][property]") {
    // T-0127 AC1/AC2: short tabs On the vector of shapes with sharp convex and concave turns
    // (stars, notched rectangles, circles), allowances on both sides of the vector, with and
    // without a separate last pass; every kept tab is checked with the rasterised oracle and the
    // counts must add up.
    test::SplitMix rng{0x7127ULL};
    int tabs = 0;
    for (int s = 0; s < 24; ++s) {
        Contour c;
        const int kind = static_cast<int>(rng.next() % 3);
        if (kind == 0) {
            const int n = 3 + static_cast<int>(rng.next() % 4);
            const double ro = rng.uniform(15, 30);
            const double ri = ro * rng.uniform(0.35, 0.7);
            std::vector<Vec2> pts;
            for (int k = 0; k < 2 * n; ++k) {
                const double a = geometry::kPi * k / n;
                const double rr = k % 2 == 0 ? ro : ri;
                pts.push_back({rr * std::cos(a), rr * std::sin(a)});
            }
            c.closed = true;
            for (std::size_t k = 0; k < pts.size(); ++k) c.append(LineSpan{pts[k], pts[(k + 1) % pts.size()]});
        } else if (kind == 1) {
            const double w = rng.uniform(30, 60);
            const double hgt = rng.uniform(20, 40);
            const double nw = rng.uniform(4, 10);
            const double nd = rng.uniform(4, 10);
            const std::vector<Vec2> pts{{0, 0}, {0.5 * (w - nw), 0}, {0.5 * (w - nw), nd}, {0.5 * (w + nw), nd},
                                        {0.5 * (w + nw), 0}, {w, 0}, {w, hgt}, {0, hgt}};
            c.closed = true;
            for (std::size_t k = 0; k < pts.size(); ++k) c.append(LineSpan{pts[k], pts[(k + 1) % pts.size()]});
        } else {
            c = Contour::circle({0, 0}, rng.uniform(6, 25));
        }
        const std::vector<Contour> cs{c};
        const double d = std::vector<double>{3.175, 6.0, 8.0}[rng.next() % 3];
        ProfileParams p = baseParams(rng.uniform(3, 10), rng.uniform(1.5, 5));
        p.side = ProfileSide::On;
        p.direction = rng.uniform(0, 1) < 0.5 ? CutDirection::Climb : CutDirection::Conventional;
        p.allowance = rng.uniform(-0.5, 0.5);
        p.separateLastPass = rng.uniform(0, 1) < 0.5;
        p.lastPassAllowance = rng.uniform(0.05, 0.6);
        p.lastPassReverse = rng.uniform(0, 1) < 0.5;
        p.tabs.enabled = true;
        p.tabs.count = 2 + static_cast<int>(rng.next() % 7);
        p.tabs.length = rng.uniform(0.3, 4.0);
        p.tabs.thickness = rng.uniform(0.5, std::min(3.0, p.passes.cutDepth - 0.2));
        p.tabs.threeD = rng.uniform(0, 1) < 0.3;
        p.startAt = StartAt::ClosestOnBoundingBox;
        p.startAnchor = static_cast<BoxAnchor>(rng.next() % 9);
        const ProfileResult r = profileToolpath(cs, test::endMill(d), p);
        if (!r.ok() || r.toolpath.isEmpty()) continue;
        INFO("scene " << s);
        // Raised runs of the last cut on every level below the tab top (one level with a separate
        // last pass).
        const double zTab = p.passes.bottomZ() + p.tabs.thickness;
        const auto levels = p.passes.levels();
        const auto below = p.separateLastPass ? std::size_t{1}
                                              : static_cast<std::size_t>(std::count_if(levels.begin(), levels.end(),
                                                                                       [&](double z) { return z < zTab - 1e-9; }));
        const std::size_t runs = keptOnLastCut(r, p);
        CHECK(runs % below == 0);
        CHECK(runs / below + tabsLeftOut(r) == static_cast<std::size_t>(p.tabs.count));
        tabs += checkTabsJoin(r, cs, p, d);
    }
    CHECK(tabs > 30);
}

TEST_CASE("On the vector: a tab on a loop barely wider than the kerf holds the remnant or is reported",
          "[toolpath][profile][tabs]") {
    // T-0127 sweep scene n4_A_86651 #6: a circle of radius 4.1 cut On with an 8 mm tool leaves a
    // remnant 0.1 mm across at the centre. Judging the part only 0.2 mm (or a raster cell) beyond
    // the kerf took it for "nothing to hold", and the tab was kept although the remnant came free.
    // Any remnant the tools leave counts: every kept tab must hold it (independent flood from the
    // centre to the waste), the rest are reported.
    for (const double radius : {4.05, 4.1, 4.2, 4.5, 5.0, 6.0}) {
        for (const double length : {0.336, 2.0}) {
            for (const int count : {1, 2}) {
                ProfileParams p = baseParams(6.0, 3.0);
                p.side = ProfileSide::On;
                p.tabs.enabled = true;
                p.tabs.count = count;
                p.tabs.length = length;
                p.tabs.thickness = 1.2;
                const std::vector<Contour> cs{Contour::circle({0, 0}, radius)};
                const ProfileResult r = profileToolpath(cs, test::endMill(8.0), p);
                REQUIRE(r.ok());
                INFO("radius " << radius << ", tab length " << length << ", tabs " << count);
                const std::size_t kept = keptOnLastCut(r, p);
                CHECK(kept + tabsLeftOut(r) == static_cast<std::size_t>(count));
                if (kept > 0) {
                    const double zTab = p.passes.bottomZ() + p.tabs.thickness;
                    const double w = radius + 7.0;
                    CHECK(test::floodHeld(r.toolpath, Box2{{-w, -w}, {w, w}}, zTab, 4.0, {0, 0}, {-w + 0.5, -w + 0.5},
                                          0.01));
                }
            }
        }
    }
}

TEST_CASE("On the vector: tabs holding a sliver or holding a part together are kept", "[toolpath][profile][tabs]") {
    // T-0127 sweep scenes where a first version of the exact rule dropped tabs that main kept and
    // that hold the part (independent whole-scene flood from the deepest point of the part):
    // n4_D_86682 #29, a 102 x 8.09 rectangle cut On with an 8 mm tool, which leaves a 0.14 mm
    // sliver (thinner than the raster's widening of the tools: refined), and g2_E_88693 #29, a
    // six-point star cut On with a 60 degree V-bit with long tabs on the two edges of an arm (the
    // cone cuts only thickness x tan 30 at the tab top).
    struct Case {
        const char* name;
        const char* text;
        Vec2 part;
        Box2 window;
        double res;
    };
    const std::vector<Case> cases{
        {"n4_D_86682_29",
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.98c976311a084p+6 0x0p+0\n"
         "L 0x1.98c976311a084p+6 0x0p+0 0x1.98c976311a084p+6 0x1.02e54af273d0fp+3\n"
         "L 0x1.98c976311a084p+6 0x1.02e54af273d0fp+3 0x0p+0 0x1.02e54af273d0fp+3\n"
         "L 0x0p+0 0x1.02e54af273d0fp+3 0x0p+0 0x0p+0\n"
         "P 0x1p+3 0 0x1.8ae13288ebc91p-6 2 0 0x0p+0 0x1.7611033238057p+1 0x1.ebd62fb50d326p+2 0x0p+0 0x1.4p+2 1 0x1.2aa284b4d03bcp-2 0 1 0x1.39e825ef3549bp+0 0x1.7365c351900edp+0 0 3 0x1.c0ac4057d41eap+3 0x0p+0 1 0 0x0p+0 1 0x1.4d331145e1388p+0 1 0 0x1.e37fbc94cbef6p-4 0x1.f16b332963d92p+1 0x1.eca15ed8ff545p-2 0x1.65cc627f8fe82p+3 2 0x1.15fecbbc85c01p+0 1 0 0x0p+0 0x0p+0 2 5 0 0 0 1 0x1.7cd829df700a4p+5 -0x1.4a3b0bb7baaf2p+5 0x1p+1\n"
         "K 0 0\n",
         {51.0, 4.045}, Box2{{-16, -16}, {118.2, 24.1}}, 0.02},
        {"g2_E_88693_29",
         "C 1\n"
         "L 0x1.9d584f7f43213p+4 0x0p+0 0x1.253065edb4a99p+3 0x1.5efd996028e68p+2\n"
         "L 0x1.253065edb4a99p+3 0x1.5efd996028e68p+2 0x1.e13c7222173cep+3 0x1.a0c340727ec4p+4\n"
         "L 0x1.e13c7222173cep+3 0x1.a0c340727ec4p+4 0x1.f8acc2c9b7735p-3 0x1.b34141574b9f6p+2\n"
         "L 0x1.f8acc2c9b7735p-3 0x1.b34141574b9f6p+2 -0x1.d2f4f981a2accp+3 0x1.94658554e25edp+4\n"
         "L -0x1.d2f4f981a2accp+3 0x1.94658554e25edp+4 -0x1.779747d2c298ap+2 0x1.b842869113418p+1\n"
         "L -0x1.779747d2c298ap+2 0x1.b842869113418p+1 -0x1.b96e50b2a443ap+4 0x1.e6ed2c2a4cee7p-49\n"
         "L -0x1.b96e50b2a443ap+4 0x1.e6ed2c2a4cee7p-49 -0x1.9269b23f7e363p+2 -0x1.e662e40d4b917p+1\n"
         "L -0x1.9269b23f7e363p+2 -0x1.e662e40d4b917p+1 -0x1.f99dc56740d1bp+3 -0x1.b5e065e2858f1p+4\n"
         "L -0x1.f99dc56740d1bp+3 -0x1.b5e065e2858f1p+4 0x1.49dc71912c087p-2 -0x1.b4a6444e69392p+2\n"
         "L 0x1.49dc71912c087p-2 -0x1.b4a6444e69392p+2 0x1.a93cad9baa389p+3 -0x1.70441cdda0258p+4\n"
         "L 0x1.a93cad9baa389p+3 -0x1.70441cdda0258p+4 0x1.bce23dd0efcc2p+2 -0x1.1cba77b1a2416p+2\n"
         "L 0x1.bce23dd0efcc2p+2 -0x1.1cba77b1a2416p+2 0x1.9d584f7f43213p+4 0x0p+0\n"
         "P 0x1.8p+3 60 0x0p+0 2 0 0x1.e95574e5e3aa7p+0 0x1.1f7af4c40c9b1p+2 0x1.f8086afbab977p-2 0x0p+0 0x1.4p+2 0 0x1.74fb50902b036p-4 1 1 0x1.b4679d3c5fd5ep+3 0x1.e87c6cc9d2c7ap+1 0 8 0x1.eead93369f1bep+4 0x0p+0 1 0 0x0p+0 3 0x1.287d12f9a2ecp+4 0 1 0x1.c059d20304292p-3 0x1.56ecd6889086ap+2 0x1.2d56875a65c14p-3 0x1.7a4f3619eda9p+2 2 0x1.126ca7aedcb93p-1 1 0 0x0p+0 0x0p+0 2 1 0 1 0 0 0x1.c2a25a88bf77p+1 0x1.6a00498a2bff6p+5 0x1p+1\n"
         "K 0 1\n",
         {1.6106, -0.0173}, Box2{{-47.6, -47.4}, {45.9, 46.1}}, 0.04},
    };
    for (const Case& k : cases) {
        std::vector<Contour> cs;
        ProfileParams p;
        int vbitDeg = 0;
        const double d = readScene(k.text, cs, p, vbitDeg);
        tooling::Tool tool = test::endMill(d);
        double tanHalf = 0.0;
        if (vbitDeg > 0) {
            tool.type = tooling::ToolType::VBit;
            tool.includedAngle = vbitDeg * geometry::kPi / 180.0;
            tanHalf = std::tan(0.5 * tool.includedAngle);
        }
        INFO("scene " << k.name);
        const ProfileResult r = profileToolpath(cs, tool, p);
        REQUIRE(r.ok());
        const std::size_t kept = keptOnLastCut(r, p);
        CHECK(kept >= 2);
        const double zTab = p.passes.bottomZ() + std::min(p.tabs.thickness, p.passes.cutDepth);
        const double rad = 0.5 * tooling::cuttingWidthAtDepth(tool, p.passes.startDepth + p.passes.cutDepth);
        CHECK(test::floodHeld(r.toolpath, k.window, zTab, rad, k.part, k.window.min + Vec2{0.5, 0.5}, k.res, tanHalf));
    }
}

TEST_CASE("Sided tabs: a kept tab holds material, and a piece no kept tab holds is reported", "[toolpath][profile][tabs]") {
    // T-0127 round-1 verifier (whole-scene flood, scenes in harness t0127v1): a tab with nothing
    // beyond the kerf near it (the tip of a star arm the kerf consumes) was kept as "holding", while
    // the part's remnant elsewhere had no kept tab and came free, reported only as "7 of 8 left
    // out". Now such a tab is dropped ("would hold nothing"), and a loose piece is reported. Each
    // scene: flood from a point of the remnant to the frame; if it comes free a diagnostic must say
    // so, and the counts add up. The last scene (star, Outside, allowance -r; main keeps two tabs
    // that hold) stays conservative: its tabs are dropped, and the loose part is reported.
    struct Case {
        const char* name;
        const char* text;
        Vec2 piece;
        bool mustHold;
    };
    const std::vector<Case> cases{
        {"m0_271001_9",
         "C 1\n"
         "L 0x1.34a1f38971b24p+4 0x0p+0 0x1.5174d6a08e0c1p+2 0x1.f562511bd18d3p+1\n"
         "L 0x1.5174d6a08e0c1p+2 0x1.f562511bd18d3p+1 0x1.5a14ca9316cd3p+2 0x1.0a4838b5129cfp+4\n"
         "L 0x1.5a14ca9316cd3p+2 0x1.0a4838b5129cfp+4 -0x1.bd4c2d8f71bddp+0 0x1.4118d69d39ccap+2\n"
         "L -0x1.bd4c2d8f71bddp+0 0x1.4118d69d39ccap+2 -0x1.ee12f0b8a3c92p+3 0x1.66f73c36930fdp+3\n"
         "L -0x1.ee12f0b8a3c92p+3 0x1.66f73c36930fdp+3 -0x1.43f0184917db6p+2 -0x1.88f211689952dp-3\n"
         "L -0x1.43f0184917db6p+2 -0x1.88f211689952dp-3 -0x1.d1292fbae8de5p+3 -0x1.51f58bd643feap+3\n"
         "L -0x1.d1292fbae8de5p+3 -0x1.51f58bd643feap+3 -0x1.7a92a4ec7997dp+0 -0x1.41ebd337eae47p+2\n"
         "L -0x1.7a92a4ec7997dp+0 -0x1.41ebd337eae47p+2 0x1.66fe6fa05ffc9p+2 -0x1.1437a752cf50ap+4\n"
         "L 0x1.66fe6fa05ffc9p+2 -0x1.1437a752cf50ap+4 0x1.f2f0e544ecea1p+1 -0x1.4b6a3c4eae3e2p+1\n"
         "L 0x1.f2f0e544ecea1p+1 -0x1.4b6a3c4eae3e2p+1 0x1.34a1f38971b24p+4 0x0p+0\n"
         "P 0x1.8p+2 0 0x0p+0 2 0 0x0p+0 0x1.7008681aca449p+3 0x1.c4088d3a1709ap+0 0x0p+0 0x1.4p+2 0 0x1.0e77e931f0be7p-2 0 1 0x1.b525e1ecca4c5p-1 0x1.1ce791263cdeep+0 0 8 0x1.eddf34c138645p+5 0x0p+0 0 0 0x0p+0 3 0x1.1e7aacad4dfc9p+5 0 0 0x1.9854c8c7332e4p-2 0x1.4a6ed9b4361f1p+1 0x1.4b4384fb2bbf3p-2 0x1.3742ab4e8d5b8p+2 1 0x1.3337482433b6ap+0 1 0 0x1.ae8013cfbbf9ap+2 0x0p+0 0 7 0 0 0 0 0x1.35cc592bc9a84p+5 0x1.00554e6ab82e8p+4 0x1p+1\n"
         "K 0 0\n"         , {0.51, 0.76}, false},
        {"m0_271013_19",
         "C 1\n"
         "L 0x1.08f8678a78535p+4 0x0p+0 0x1.12bf1d93fadf8p+2 0x1.156baaeedbfdfp+1\n"
         "L 0x1.12bf1d93fadf8p+2 0x1.156baaeedbfdfp+1 0x1.24ab0eba30149p+3 0x1.6efe8ae26797p+3\n"
         "L 0x1.24ab0eba30149p+3 0x1.6efe8ae26797p+3 0x1.845db298d4df9p-1 0x1.0703f6a2e43d6p+2\n"
         "L 0x1.845db298d4df9p-1 0x1.0703f6a2e43d6p+2 -0x1.abc190d2be1dbp+1 0x1.d487cecd6f3eep+3\n"
         "L -0x1.abc190d2be1dbp+1 0x1.d487cecd6f3eep+3 -0x1.cf6b3df4e8999p+1 0x1.0dc147a2c5002p+2\n"
         "L -0x1.cf6b3df4e8999p+1 0x1.0dc147a2c5002p+2 -0x1.a5ddd0b303631p+3 0x1.9652036e776dbp+2\n"
         "L -0x1.a5ddd0b303631p+3 0x1.9652036e776dbp+2 -0x1.79f872793fa85p+2 -0x1.96ce0fd238b76p-3\n"
         "L -0x1.79f872793fa85p+2 -0x1.96ce0fd238b76p-3 -0x1.eb50dd027b4ap+3 -0x1.d935e3d80cf1cp+2\n"
         "L -0x1.eb50dd027b4ap+3 -0x1.d935e3d80cf1cp+2 -0x1.24af1d3466f74p+1 -0x1.669b1bde3bd97p+1\n"
         "L -0x1.24af1d3466f74p+1 -0x1.669b1bde3bd97p+1 -0x1.b263bd164a395p+1 -0x1.dbcbda6d4962ep+3\n"
         "L -0x1.b263bd164a395p+1 -0x1.dbcbda6d4962ep+3 0x1.d0bd937c47dp-1 -0x1.3464ee0f7086fp+2\n"
         "L 0x1.d0bd937c47dp-1 -0x1.3464ee0f7086fp+2 0x1.16d8fa98037c5p+3 -0x1.5da9eea62a2bap+3\n"
         "L 0x1.16d8fa98037c5p+3 -0x1.5da9eea62a2bap+3 0x1.2721ff2a1d7dp+2 -0x1.08d1b69d402bp+1\n"
         "L 0x1.2721ff2a1d7dp+2 -0x1.08d1b69d402bp+1 0x1.08f8678a78535p+4 0x0p+0\n"
         "P 0x1.8p+2 0 -0x1.641bf6b6dd6fap-3 2 1 0x0p+0 0x1.fa0aa6aa45adp+1 0x1.3e96dce8835a6p+0 0x0p+0 0x1.4p+2 0 0x1.8a8697266d717p-1 0 1 0x1.646cf324b27dbp+1 0x1.63c61e31b0bbp+1 0 8 0x1.5356ab4901232p+5 0x0p+0 0 1 0x0p+0 3 0x1.0d64a22741e48p+4 1 0 0x1.8849ab4cfad64p-5 0x1.a23508e9eaf16p+2 0x1.398e4920f8fb6p-1 0x1.2dd2f9d40981p+0 1 0x1.8f80d260029f7p-2 1 0 0x0p+0 0x0p+0 1 0 0 0 0 0 -0x1.97436ae2302d5p+4 -0x1.688afafb98c12p+5 0x1p+1\n"
         "K 0 0\n"         , {-0.13, 0.23}, false},
        {"m1_272005_0",
         "C 1\n"
         "L 0x1.705e437621aadp+4 0x0p+0 0x1.2c07bf85edc5cp+2 0x1.735e033621229p+1\n"
         "L 0x1.2c07bf85edc5cp+2 0x1.735e033621229p+1 0x1.64ad12399bf44p+3 0x1.34e3f9a9d7af4p+4\n"
         "L 0x1.64ad12399bf44p+3 0x1.34e3f9a9d7af4p+4 -0x1.d8d20dfb5afe7p-4 0x1.663de7f0d6268p+2\n"
         "L -0x1.d8d20dfb5afe7p-4 0x1.663de7f0d6268p+2 -0x1.666a1276f2ca5p+3 0x1.36655b7c25c98p+4\n"
         "L -0x1.666a1276f2ca5p+3 0x1.36655b7c25c98p+4 -0x1.e8852bf6ec0fp+2 0x1.f67d70fbf3ea1p+1\n"
         "L -0x1.e8852bf6ec0fp+2 0x1.f67d70fbf3ea1p+1 -0x1.99c87aef43bcbp+4 0x1.c40454f206c6fp-49\n"
         "L -0x1.99c87aef43bcbp+4 0x1.c40454f206c6fp-49 -0x1.a4a0261dc345p+2 -0x1.da51100c44f62p+1\n"
         "L -0x1.a4a0261dc345p+2 -0x1.da51100c44f62p+1 -0x1.69dd87cbd9053p+3 -0x1.3962746858d42p+4\n"
         "L -0x1.69dd87cbd9053p+3 -0x1.3962746858d42p+4 -0x1.8b74d9fe60866p-4 -0x1.4c52bd2ba7418p+2\n"
         "L -0x1.8b74d9fe60866p-4 -0x1.4c52bd2ba7418p+2 0x1.511fe6c747473p+3 -0x1.23f55f021ae9bp+4\n"
         "L 0x1.511fe6c747473p+3 -0x1.23f55f021ae9bp+4 0x1.7411b1ff5e1fp+2 -0x1.8aace70a7459ep+1\n"
         "L 0x1.7411b1ff5e1fp+2 -0x1.8aace70a7459ep+1 0x1.705e437621aadp+4 0x0p+0\n"
         "P 0x1p+3 0 -0x1.fb15e666a5ab6p+1 0 0 0x0p+0 0x1.0c61a4202af58p+2 0x1.5f14288d07947p+2 0x1.899abde9ad6dep-1 0x1.4p+2 0 0x1.00b21d586e8cbp-4 0 1 0x1.e177e96fd173ep-3 0x1.075235e8682d1p+1 0 7 0x1.2b3c1c42cff68p+3 0x0p+0 0 0 0x0p+0 1 0x1.cc2102c0588f6p+2 0 0 0x1.98d96914cf5dap-2 0x1.91d55a54ecf92p+3 0x1.05d74d98d04p-2 0x1.46c4b9a0e43d7p-1 2 0x1.2a327d5c59fe7p+0 1 0 0x1.7101ed687b9ecp+2 0x0p+0 2 8 0 1 0 1 -0x1.7475872b9e0cp+5 -0x1.b24e3756c77bcp+4 0x1p+1\n"
         "K 0 0\n"         , {-1.62, 0.29}, false},
        {"m1_272011_0",
         "C 1\n"
         "L 0x1.913ad60551693p+4 0x0p+0 0x1.f57ac18b48f95p+2 0x1.0518fd1a1e485p+2\n"
         "L 0x1.f57ac18b48f95p+2 0x1.0518fd1a1e485p+2 0x1.26008e2dc5456p+4 0x1.70aac46cd69dp+4\n"
         "L 0x1.26008e2dc5456p+4 0x1.70aac46cd69dp+4 0x1.b0ea279ffdf96p+0 0x1.f6ee8380d7735p+2\n"
         "L 0x1.b0ea279ffdf96p+0 0x1.f6ee8380d7735p+2 -0x1.7b81c0691a3dap+2 0x1.9fae9974966c9p+4\n"
         "L -0x1.7b81c0691a3dap+2 0x1.9fae9974966c9p+4 -0x1.f5ee86a638015p+1 0x1.40289a3dd84d4p+2\n"
         "L -0x1.f5ee86a638015p+1 0x1.40289a3dd84d4p+2 -0x1.b031c8b476c6cp+4 0x1.a0448cc6fff5ap+3\n"
         "L -0x1.b031c8b476c6cp+4 0x1.a0448cc6fff5ap+3 -0x1.afd2a902cb1c7p+2 0x1.5901578df4576p-3\n"
         "L -0x1.afd2a902cb1c7p+2 0x1.5901578df4576p-3 -0x1.bbc5536beaf34p+4 -0x1.ab6ae20c0bdfp+3\n"
         "L -0x1.bbc5536beaf34p+4 -0x1.ab6ae20c0bdfp+3 -0x1.17e3d2d76e9cap+2 -0x1.4f538b0f1dbd6p+2\n"
         "L -0x1.17e3d2d76e9cap+2 -0x1.4f538b0f1dbd6p+2 -0x1.b250d3c3359f1p+2 -0x1.dbb7239ea6783p+4\n"
         "L -0x1.b250d3c3359f1p+2 -0x1.dbb7239ea6783p+4 0x1.7a03c56a15723p+0 -0x1.fcc48b965312ep+2\n"
         "L 0x1.7a03c56a15723p+0 -0x1.fcc48b965312ep+2 0x1.08ecdc3cee83ap+4 -0x1.4c34ab3c43afbp+4\n"
         "L 0x1.08ecdc3cee83ap+4 -0x1.4c34ab3c43afbp+4 0x1.18b0511e3747ep+3 -0x1.1831b8c66eb6cp+2\n"
         "L 0x1.18b0511e3747ep+3 -0x1.1831b8c66eb6cp+2 0x1.913ad60551693p+4 0x0p+0\n"
         "P 0x1p+3 0 -0x1.de091d93c8854p+1 0 1 0x0p+0 0x1.418e84ca3a88bp+3 0x1.2fc98e728a39ep+2 0x0p+0 0x1.4p+2 1 0x1.5faffa35359e2p-1 1 1 0x1.461910187d30fp+2 0x1.312bb6d94e8f5p+0 0 7 0x1.33b8ab82a5826p+3 0x0p+0 0 0 0x0p+0 2 0x1.00056aa62b3dp+3 1 1 0x1.0f2f059e75f0bp-2 0x1.c25f6a11a0351p+3 0x1.e99afbebf8fcfp-2 0x1.d2582b750accfp+0 1 0x1.8317e5163ddf2p+0 1 0 0x1.af0389bfe4253p+3 0x0p+0 2 4 0 1 0 0 0x1.945460550504p+3 -0x1.69ce14393e3dbp+4 0x1p+1\n"
         "K 0 0\n"         , {1.94, -0.37}, false},
        {"m1_272014_40",
         "C 1\n"
         "L 0x1.317de3aa2ecdfp+4 0x0p+0 0x1.638b477e76387p+2 0x1.058884071ae6ap+2\n"
         "L 0x1.638b477e76387p+2 0x1.058884071ae6ap+2 0x1.7d3903cc48438p+2 0x1.25521a910eacdp+4\n"
         "L 0x1.7d3903cc48438p+2 0x1.25521a910eacdp+4 -0x1.9a719c7bc7ef3p+0 0x1.79c675756c82p+2\n"
         "L -0x1.9a719c7bc7ef3p+0 0x1.79c675756c82p+2 -0x1.0942d7be49bdfp+4 0x1.81726b1c10bdbp+3\n"
         "L -0x1.0942d7be49bdfp+4 0x1.81726b1c10bdbp+3 -0x1.2f6e31aea2251p+2 0x1.7c5545e24aafcp-3\n"
         "L -0x1.2f6e31aea2251p+2 0x1.7c5545e24aafcp-3 -0x1.04c11e3f31c1cp+4 -0x1.7ae5f55b91326p+3\n"
         "L -0x1.04c11e3f31c1cp+4 -0x1.7ae5f55b91326p+3 -0x1.fa9c82d6f5d41p+0 -0x1.7dec2a0ad0a61p+2\n"
         "L -0x1.fa9c82d6f5d41p+0 -0x1.7dec2a0ad0a61p+2 0x1.a535b3b771145p+2 -0x1.44166cb7cf3fcp+4\n"
         "L 0x1.a535b3b771145p+2 -0x1.44166cb7cf3fcp+4 0x1.6c83bb09f1bfap+2 -0x1.17dbee84c329cp+2\n"
         "L 0x1.6c83bb09f1bfap+2 -0x1.17dbee84c329cp+2 0x1.317de3aa2ecdfp+4 0x0p+0\n"
         "P 0x1.8p+2 0 -0x1.5842b3e655261p+1 1 1 0x0p+0 0x1.43ed5970e2fc2p+3 0x1.c96b0d958d307p-1 0x1.02f0d53beaa8ep-1 0x1.4p+2 1 0x1.41b3a3951ebecp-5 1 1 0x1.e19f0cd5d7759p+0 0x1.3286cb40eb0cbp+0 1 6 0x1.3a1372c441121p+4 0x0p+0 0 1 0x0p+0 1 0x1.0cbdf633d50ep+4 1 0 0x1.d2c9b5599b539p-3 0x1.ac605b1bf0722p+1 0x1.1a32b41084a88p-3 0x1.6ed3b0e05eed2p+1 0 0x1.6e21b2ea2a239p+0 1 0 0x0p+0 0x0p+0 2 6 0 1 0 0 0x1.fb79e6aca284cp+4 -0x1.8927674bb6b18p+5 0x1p+1\n"
         "K 0 0\n"         , {1.53, -0.31}, false},
        {"m2_273003_43",
         "C 1\n"
         "L 0x1.99c976dceb2e6p+4 0x0p+0 0x1.c7122d9b9d43bp+2 0x1.12b6ea6ef4ff1p+2\n"
         "L 0x1.c7122d9b9d43bp+2 0x1.12b6ea6ef4ff1p+2 0x1.882311feb0332p+3 0x1.5399c0ca2a1e9p+4\n"
         "L 0x1.882311feb0332p+3 0x1.5399c0ca2a1e9p+4 -0x1.dbc17a0b14788p-3 0x1.0a68b8f3b61cfp+3\n"
         "L -0x1.dbc17a0b14788p-3 0x1.0a68b8f3b61cfp+3 -0x1.86a7f1ba3c33cp+3 0x1.52516b98b7269p+4\n"
         "L -0x1.86a7f1ba3c33cp+3 0x1.52516b98b7269p+4 -0x1.f8735a170e063p+2 0x1.0ee8025061eaap+2\n"
         "L -0x1.f8735a170e063p+2 0x1.0ee8025061eaap+2 -0x1.9f6494bf2fa0bp+4 0x1.ca34714f649cp-49\n"
         "L -0x1.9f6494bf2fa0bp+4 0x1.ca34714f649cp-49 -0x1.011f5a1110ba2p+3 -0x1.0eafafe2e8355p+2\n"
         "L -0x1.011f5a1110ba2p+3 -0x1.0eafafe2e8355p+2 -0x1.9c5f9ecd24f26p+3 -0x1.65203db2fa7e9p+4\n"
         "L -0x1.9c5f9ecd24f26p+3 -0x1.65203db2fa7e9p+4 0x1.f365c3affa049p-3 -0x1.709dcfe40186cp+2\n"
         "L 0x1.f365c3affa049p-3 -0x1.709dcfe40186cp+2 0x1.81610ff12fa8cp+3 -0x1.4dbf85ae5fd6ap+4\n"
         "L 0x1.81610ff12fa8cp+3 -0x1.4dbf85ae5fd6ap+4 0x1.e8c58a75d82f8p+2 -0x1.fabe51905d1c5p+1\n"
         "L 0x1.e8c58a75d82f8p+2 -0x1.fabe51905d1c5p+1 0x1.99c976dceb2e6p+4 0x0p+0\n"
         "C 1\n"
         "L 0x1.0270008a048d4p+5 -0x1.6f4e8ec21a58ap+3 0x1.de6a0df6d6168p+5 -0x1.6f4e8ec21a58ap+3\n"
         "L 0x1.de6a0df6d6168p+5 -0x1.6f4e8ec21a58ap+3 0x1.00ebff45ea46ap+6 -0x1.c32d98dc40fafp+2\n"
         "L 0x1.00ebff45ea46ap+6 -0x1.c32d98dc40fafp+2 0x1.00ebff45ea46ap+6 0x1.79f63d74836cbp+2\n"
         "L 0x1.00ebff45ea46ap+6 0x1.79f63d74836cbp+2 0x1.de6a0df6d6168p+5 0x1.4ab2e10e3b918p+3\n"
         "L 0x1.de6a0df6d6168p+5 0x1.4ab2e10e3b918p+3 0x1.0270008a048d4p+5 0x1.4ab2e10e3b918p+3\n"
         "L 0x1.0270008a048d4p+5 0x1.4ab2e10e3b918p+3 0x1.be041fea0c2cep+4 0x1.79f63d74836cbp+2\n"
         "L 0x1.be041fea0c2cep+4 0x1.79f63d74836cbp+2 0x1.be041fea0c2cep+4 -0x1.c32d98dc40fafp+2\n"
         "L 0x1.be041fea0c2cep+4 -0x1.c32d98dc40fafp+2 0x1.0270008a048d4p+5 -0x1.6f4e8ec21a58ap+3\n"
         "P 0x1.9666666666666p+1 0 -0x1.17d49b76ddff6p+0 1 0 0x0p+0 0x1.b91e187f19406p+3 0x1.12b97fb9318bbp+0 0x0p+0 0x1.4p+2 0 0x1.f5f48e3498032p-1 0 1 0x1.2bccba39e4f99p+1 0x1.03436a8f82adep+1 1 7 0x1.ec0919d3632f3p+5 0x0p+0 0 0 0x0p+0 2 0x1.014be2f6792f3p+5 0 1 0x1.2b6de20a9aa7bp-2 0x1.a4088b8585411p+3 0x1.178bd9d2f8eecp-2 0x1.3f3125a0a6cd4p+3 2 0x1.43348e5f4a13cp-1 1 0 0x0p+0 0x0p+0 2 3 0 0 0 1 0x1.083a0b594ec4p+2 -0x1.72ab739f7d1f5p+5 0x1p+1\n"
         "K 0 0\n"         , {-0.49, 0.81}, false},
        {"m1_272004_44",
         "C 1\n"
         "L 0x1.52084112ab05p+4 0x0p+0 0x1.5f6aa3e983bedp+2 0x1.b472c64aa2511p+1\n"
         "L 0x1.5f6aa3e983bedp+2 0x1.b472c64aa2511p+1 0x1.72e4c22afa89p+3 0x1.41340971b605bp+4\n"
         "L 0x1.72e4c22afa89p+3 0x1.41340971b605bp+4 -0x1.bf4b59e6508fp-4 0x1.5e8e04a0e5814p+2\n"
         "L -0x1.bf4b59e6508fp-4 0x1.5e8e04a0e5814p+2 -0x1.53868791e92dep+3 0x1.2609a79847afbp+4\n"
         "L -0x1.53868791e92dep+3 0x1.2609a79847afbp+4 -0x1.c26d68f599997p+2 0x1.e7fdf7f474f28p+1\n"
         "L -0x1.c26d68f599997p+2 0x1.e7fdf7f474f28p+1 -0x1.9636e07868a13p+4 0x1.c01491abf9575p-49\n"
         "L -0x1.9636e07868a13p+4 0x1.c01491abf9575p-49 -0x1.6a7f148e75304p+2 -0x1.8aa871428d5cdp+1\n"
         "L -0x1.6a7f148e75304p+2 -0x1.8aa871428d5cdp+1 -0x1.4d2288b034494p+3 -0x1.2080d7449102bp+4\n"
         "L -0x1.4d2288b034494p+3 -0x1.2080d7449102bp+4 -0x1.d2fa7054baa04p-5 -0x1.d8f804adeaa82p+2\n"
         "L -0x1.d2fa7054baa04p-5 -0x1.d8f804adeaa82p+2 0x1.632573fd0fe6ap+3 -0x1.3390d2f2a26c8p+4\n"
         "L 0x1.632573fd0fe6ap+3 -0x1.3390d2f2a26c8p+4 0x1.b552ef9a3e1bap+2 -0x1.caee550fce8c5p+1\n"
         "L 0x1.b552ef9a3e1bap+2 -0x1.caee550fce8c5p+1 0x1.52084112ab05p+4 0x0p+0\n"
         "P 0x1.8p+2 0 -0x1.8p+1 0 1 0x0p+0 0x1.d729c1ad6706dp+2 0x1.daab2487a2564p+2 0x0p+0 0x1.4p+2 0 0x1.96361b08a28edp-1 0 1 0x1.371a09122eac8p+1 0x1.03811b8aa9da8p+0 0 2 0x1.3624702eacd0fp+6 0x0p+0 1 1 0x0p+0 3 0x1.31bdf447f7201p+5 0 1 0x1.81753bdf5bb83p-2 0x1.625481c0880d6p-1 0x1.ea0320036904fp-1 0x1.c842e97a78348p+1 1 0x1.168396783b9d6p-1 0 0 0x1.792d786d0f0a5p+3 0x0p+0 1 6 0 1 0 0 -0x1.7acbc949c3f8ep+5 -0x1.bcd3fc4c0b8bcp+3 0x1p+1\n"
         "K 0 0\n"         , {0.07, -0.9}, false},
    };
    for (const Case& k : cases) {
        std::vector<Contour> cs;
        ProfileParams p;
        int vbitDeg = 0;
        const double d = readScene(k.text, cs, p, vbitDeg);
        REQUIRE(vbitDeg == 0);
        INFO("scene " << k.name);
        const ProfileResult r = profileToolpath(cs, test::endMill(d), p);
        REQUIRE(r.ok());
        const auto cuts = cutRanges(r.toolpath);
        const double zTab = p.passes.bottomZ() + std::min(p.tabs.thickness, p.passes.cutDepth);
        Box2 window;
        for (const Contour& c : cs) window.expand(c.bounds());
        window = window.inflated(d + 4.0);
        const bool held = test::floodHeld(r.toolpath, window, zTab, d / 2, k.piece, window.min + Vec2{0.5, 0.5});
        const bool reported = std::any_of(r.diagnostics.begin(), r.diagnostics.end(), [](const ProfileDiagnostic& g) {
            return g.message.find("not held by any kept tab") != std::string::npos;
        });
        CHECK((held || reported));
        CHECK(held != reported);
        if (k.mustHold) {
            CHECK(held);
            CHECK(tabsLeftOut(r) == 0);
        }
    }
}

TEST_CASE("Sided tabs: survival judged by the cut at the tab layer; every loose piece reported", "[toolpath][profile][tabs]") {
    // T-0127 round-2 verifier (harness t0127v2, whole-scene flood wsf7). The first five scenes lost
    // tabs that hold (On D12 next to another vector's cut; Inside holes with a V-bit, whose cone at
    // the tab layer cuts far less than its bottom radius; an end-mill scene keeping 10 of 18),
    // because another cut coming within the bottom radius of a cross-section dropped the tab
    // outright; the flood shows all their tabs hold and no piece comes free. The last two leave 3
    // and 4 loose remnants of 0.2-0.4 mm2 between the finishing and roughing kerfs at corners
    // (V-bit, separate last pass), which a quick skip failed to report. Expected counts are the
    // flood-verified ones.
    struct Case {
        const char* name;
        const char* text;
        std::size_t left;
        std::size_t loose;
    };
    const std::vector<Case> cases{
        {"m0_291000_29",
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.18c7c0089d3ap+6 0x0p+0\n"
         "L 0x1.18c7c0089d3ap+6 0x0p+0 0x1.18c7c0089d3ap+6 0x1.5e0ab0ea33a1bp+5\n"
         "L 0x1.18c7c0089d3ap+6 0x1.5e0ab0ea33a1bp+5 0x0p+0 0x1.5e0ab0ea33a1bp+5\n"
         "L 0x0p+0 0x1.5e0ab0ea33a1bp+5 0x0p+0 0x0p+0\n"
         "C 1\n"
         "L 0x1.23864fd069c38p+2 0x1.422453aebab3ap+3 0x1.80832c8606b1p+4 0x1.422453aebab3ap+3\n"
         "L 0x1.80832c8606b1p+4 0x1.422453aebab3ap+3 0x1.80832c8606b1p+4 0x1.7b367b0a4f86bp+4\n"
         "L 0x1.80832c8606b1p+4 0x1.7b367b0a4f86bp+4 0x1.23864fd069c38p+2 0x1.7b367b0a4f86bp+4\n"
         "L 0x1.23864fd069c38p+2 0x1.7b367b0a4f86bp+4 0x1.23864fd069c38p+2 0x1.422453aebab3ap+3\n"
         "P 0x1.8p+3 0 0x0p+0 2 0 0x0p+0 0x1.9f24c53bce873p+3 0x1.9ca5f979ce331p+2 0x1.09a8a73023943p-1 0x1.4p+2 0 0x1.8c500d0a72e39p-1 1 1 0x1.2519795e4957bp+3 0x1.21126da16717bp+2 0 2 0x1.8bcdefa85b888p+4 0x0p+0 1 0 0x0p+0 2 0x1.4dda458457edcp+4 1 0 0x1.bd21758620c09p-2 0x1.9b2ec25f38dc9p+3 0x1.2eda2cd59c903p-2 0x1.6d82cb52e232fp+2 0 0x1.f676b6a6ba85ap-3 1 0 0x0p+0 0x0p+0 0 0 0 0 0 1 0x1.a03e5fefc22ap+1 -0x1.92be0b0a30a27p+4 0x1p+1\n"
         "K 0 0\n"         , 0, 0},
        {"hole1",
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.03p+6 0x0p+0\n"
         "L 0x1.03p+6 0x0p+0 0x1.03p+6 0x1.1d95810624dd3p+6\n"
         "L 0x1.03p+6 0x1.1d95810624dd3p+6 0x0p+0 0x1.1d95810624dd3p+6\n"
         "L 0x0p+0 0x1.1d95810624dd3p+6 0x0p+0 0x0p+0\n"
         "C 1\n"
         "L 0x1.ep+4 0x1.ep+4 0x1.27b851eb851ecp+5 0x1.ep+4\n"
         "L 0x1.27b851eb851ecp+5 0x1.ep+4 0x1.27b851eb851ecp+5 0x1.17p+5\n"
         "L 0x1.27b851eb851ecp+5 0x1.17p+5 0x1.ep+4 0x1.17p+5\n"
         "L 0x1.ep+4 0x1.17p+5 0x1.ep+4 0x1.ep+4\n"
         "P 0x1.8p+2 60 -0x1.7bfe5c91d14e4p+1 1 1 0x1.90a3d70a3d70ap+0 0x1.7a5e353f7ced9p+2 0x1.ecac083126e98p+2 0x1.e978d4fdf3b64p-3 0x1.4p+2 0 0x1.051eb851eb852p-2 0 1 0x1.e4dd2f1a9fbe7p-1 0x1.604189374bc6ap+0 0 1 0x1.9p+6 0x0p+0 1 0 0x0p+0 0 0x1.4p+3 0 0 0x1.657184ae74489p-4 0x1.4p+2 0x1.921fb54442d18p-1 0x1.4p+2 0 0x1.921fb54442d18p+0 0 0 0x0p+0 0x0p+0 0 0 0 0 0 0 0x0p+0 0x0p+0 0x1p+1\n"
         "K 0 0\n"         , 0, 0},
        // (13 since the merge with T-0017/T-0123: the roughing start projected onto the finishing
        // cut moved by ulps, so fitTabs kept a tab it used to drop as clashing, which then mapped
        // across the roughing start and was left out as unmatched. 12 since T-0135: a tab moved off
        // a roughing start is always kept, with room for its roughing cover, and is matched. The
        // six tabs kept all hold, whole-scene flood.)
        {"m1_292000_22",
         "C 1\n"
         "L 0x1.3c6ea3c9e402bp+3 0x0p+0 0x1.a702888c3fa3ap+3 0x1.63ae597e95724p+0\n"
         "L 0x1.a702888c3fa3ap+3 0x1.63ae597e95724p+0 0x1.9ad8cadbe3863p+3 0x1.b858257810ebp+1\n"
         "L 0x1.9ad8cadbe3863p+3 0x1.b858257810ebp+1 0x1.276a369cfda2ep+3 0x1.c598bbce60501p+1\n"
         "L 0x1.276a369cfda2ep+3 0x1.c598bbce60501p+1 0x1.1209ced5405eap+3 0x1.3c6ea3c9e402ap+2\n"
         "L 0x1.1209ced5405eap+3 0x1.3c6ea3c9e402ap+2 0x1.581b74c01436p+3 0x1.f40460dc01d21p+2\n"
         "L 0x1.581b74c01436p+3 0x1.f40460dc01d21p+2 0x1.2cc2c17ddf4b7p+3 0x1.2cc2c17ddf4b7p+3\n"
         "L 0x1.2cc2c17ddf4b7p+3 0x1.2cc2c17ddf4b7p+3 0x1.8e4642ae50889p+2 0x1.ebd3f74802a21p+2\n"
         "L 0x1.8e4642ae50889p+2 0x1.ebd3f74802a21p+2 0x1.3c6ea3c9e402cp+2 0x1.1209ced5405eap+3\n"
         "L 0x1.3c6ea3c9e402cp+2 0x1.1209ced5405eap+3 0x1.5a00b03c7d757p+2 0x1.84913fefe6e44p+3\n"
         "L 0x1.5a00b03c7d757p+2 0x1.84913fefe6e44p+3 0x1.b858257810eb7p+1 0x1.9ad8cadbe3863p+3\n"
         "L 0x1.b858257810eb7p+1 0x1.9ad8cadbe3863p+3 0x1.8c01d7c7e28e7p+0 0x1.3889504ac0585p+3\n"
         "L 0x1.8c01d7c7e28e7p+0 0x1.3889504ac0585p+3 0x1.5d0b7d62c9306p-51 0x1.3c6ea3c9e402bp+3\n"
         "L 0x1.5d0b7d62c9306p-51 0x1.3c6ea3c9e402bp+3 -0x1.63ae597e9571cp+0 0x1.a702888c3fa3bp+3\n"
         "L -0x1.63ae597e9571cp+0 0x1.a702888c3fa3bp+3 -0x1.b858257810eadp+1 0x1.9ad8cadbe3863p+3\n"
         "L -0x1.b858257810eadp+1 0x1.9ad8cadbe3863p+3 -0x1.c598bbce604fdp+1 0x1.276a369cfda2ep+3\n"
         "L -0x1.c598bbce604fdp+1 0x1.276a369cfda2ep+3 -0x1.3c6ea3c9e4029p+2 0x1.1209ced5405eap+3\n"
         "L -0x1.3c6ea3c9e4029p+2 0x1.1209ced5405eap+3 -0x1.f40460dc01d1fp+2 0x1.581b74c01436p+3\n"
         "L -0x1.f40460dc01d1fp+2 0x1.581b74c01436p+3 -0x1.2cc2c17ddf4b7p+3 0x1.2cc2c17ddf4b7p+3\n"
         "L -0x1.2cc2c17ddf4b7p+3 0x1.2cc2c17ddf4b7p+3 -0x1.ebd3f74802a2p+2 0x1.8e4642ae5088bp+2\n"
         "L -0x1.ebd3f74802a2p+2 0x1.8e4642ae5088bp+2 -0x1.1209ced5405eap+3 0x1.3c6ea3c9e402ap+2\n"
         "L -0x1.1209ced5405eap+3 0x1.3c6ea3c9e402ap+2 -0x1.84913fefe6e45p+3 0x1.5a00b03c7d752p+2\n"
         "L -0x1.84913fefe6e45p+3 0x1.5a00b03c7d752p+2 -0x1.9ad8cadbe3863p+3 0x1.b858257810eabp+1\n"
         "L -0x1.9ad8cadbe3863p+3 0x1.b858257810eabp+1 -0x1.3889504ac0584p+3 0x1.8c01d7c7e28eap+0\n"
         "L -0x1.3889504ac0584p+3 0x1.8c01d7c7e28eap+0 -0x1.3c6ea3c9e402bp+3 0x1.5d0b7d62c9306p-50\n"
         "L -0x1.3c6ea3c9e402bp+3 0x1.5d0b7d62c9306p-50 -0x1.a702888c3fa3ap+3 -0x1.63ae597e95726p+0\n"
         "L -0x1.a702888c3fa3ap+3 -0x1.63ae597e95726p+0 -0x1.9ad8cadbe3863p+3 -0x1.b858257810eb2p+1\n"
         "L -0x1.9ad8cadbe3863p+3 -0x1.b858257810eb2p+1 -0x1.276a369cfda2ep+3 -0x1.c598bbce604fcp+1\n"
         "L -0x1.276a369cfda2ep+3 -0x1.c598bbce604fcp+1 -0x1.1209ced5405ebp+3 -0x1.3c6ea3c9e4028p+2\n"
         "L -0x1.1209ced5405ebp+3 -0x1.3c6ea3c9e4028p+2 -0x1.581b74c014361p+3 -0x1.f40460dc01d1fp+2\n"
         "L -0x1.581b74c014361p+3 -0x1.f40460dc01d1fp+2 -0x1.2cc2c17ddf4b8p+3 -0x1.2cc2c17ddf4b7p+3\n"
         "L -0x1.2cc2c17ddf4b8p+3 -0x1.2cc2c17ddf4b7p+3 -0x1.8e4642ae5088dp+2 -0x1.ebd3f74802a1fp+2\n"
         "L -0x1.8e4642ae5088dp+2 -0x1.ebd3f74802a1fp+2 -0x1.3c6ea3c9e403p+2 -0x1.1209ced5405e8p+3\n"
         "L -0x1.3c6ea3c9e403p+2 -0x1.1209ced5405e8p+3 -0x1.5a00b03c7d75fp+2 -0x1.84913fefe6e42p+3\n"
         "L -0x1.5a00b03c7d75fp+2 -0x1.84913fefe6e42p+3 -0x1.b858257810ec8p+1 -0x1.9ad8cadbe3862p+3\n"
         "L -0x1.b858257810ec8p+1 -0x1.9ad8cadbe3862p+3 -0x1.8c01d7c7e28ecp+0 -0x1.3889504ac0584p+3\n"
         "L -0x1.8c01d7c7e28ecp+0 -0x1.3889504ac0584p+3 -0x1.05c89e0a16e44p-49 -0x1.3c6ea3c9e402bp+3\n"
         "L -0x1.05c89e0a16e44p-49 -0x1.3c6ea3c9e402bp+3 0x1.63ae597e95707p+0 -0x1.a702888c3fa3bp+3\n"
         "L 0x1.63ae597e95707p+0 -0x1.a702888c3fa3bp+3 0x1.b858257810ea3p+1 -0x1.9ad8cadbe3864p+3\n"
         "L 0x1.b858257810ea3p+1 -0x1.9ad8cadbe3864p+3 0x1.c598bbce60505p+1 -0x1.276a369cfda2ep+3\n"
         "L 0x1.c598bbce60505p+1 -0x1.276a369cfda2ep+3 0x1.3c6ea3c9e402cp+2 -0x1.1209ced5405eap+3\n"
         "L 0x1.3c6ea3c9e402cp+2 -0x1.1209ced5405eap+3 0x1.f40460dc01d1dp+2 -0x1.581b74c014361p+3\n"
         "L 0x1.f40460dc01d1dp+2 -0x1.581b74c014361p+3 0x1.2cc2c17ddf4b6p+3 -0x1.2cc2c17ddf4b8p+3\n"
         "L 0x1.2cc2c17ddf4b6p+3 -0x1.2cc2c17ddf4b8p+3 0x1.ebd3f74802a25p+2 -0x1.8e4642ae50885p+2\n"
         "L 0x1.ebd3f74802a25p+2 -0x1.8e4642ae50885p+2 0x1.1209ced5405e8p+3 -0x1.3c6ea3c9e403p+2\n"
         "L 0x1.1209ced5405e8p+3 -0x1.3c6ea3c9e403p+2 0x1.84913fefe6e42p+3 -0x1.5a00b03c7d76p+2\n"
         "L 0x1.84913fefe6e42p+3 -0x1.5a00b03c7d76p+2 0x1.9ad8cadbe3862p+3 -0x1.b858257810ec9p+1\n"
         "L 0x1.9ad8cadbe3862p+3 -0x1.b858257810ec9p+1 0x1.3889504ac0584p+3 -0x1.8c01d7c7e28fp+0\n"
         "L 0x1.3889504ac0584p+3 -0x1.8c01d7c7e28fp+0 0x1.3c6ea3c9e402bp+3 0x0p+0\n"
         "C 1\n"
         "L 0x1.8c41da281b482p+4 -0x1.ab593f36000cep+3 0x1.f1ed64a18e077p+5 -0x1.ab593f36000cep+3\n"
         "L 0x1.f1ed64a18e077p+5 -0x1.ab593f36000cep+3 0x1.038bf829aec86p+6 -0x1.56af106ec1e77p+3\n"
         "L 0x1.038bf829aec86p+6 -0x1.56af106ec1e77p+3 0x1.038bf829aec86p+6 0x1.3519e91240829p+3\n"
         "L 0x1.038bf829aec86p+6 0x1.3519e91240829p+3 0x1.f1ed64a18e077p+5 0x1.89c417d97ea8p+3\n"
         "L 0x1.f1ed64a18e077p+5 0x1.89c417d97ea8p+3 0x1.8c41da281b482p+4 0x1.89c417d97ea8p+3\n"
         "L 0x1.8c41da281b482p+4 0x1.89c417d97ea8p+3 0x1.61ecc2c47c357p+4 0x1.3519e91240829p+3\n"
         "L 0x1.61ecc2c47c357p+4 0x1.3519e91240829p+3 0x1.61ecc2c47c357p+4 -0x1.56af106ec1e77p+3\n"
         "L 0x1.61ecc2c47c357p+4 -0x1.56af106ec1e77p+3 0x1.8c41da281b482p+4 -0x1.ab593f36000cep+3\n"
         "P 0x1.8p+2 60 -0x1.3bb50eb711cdep+1 1 0 0x0p+0 0x1.43d9b9a17053p+2 0x1.63933f4486ce8p+1 0x0p+0 0x1.4p+2 1 0x1.fc5840336296p-1 1 1 0x1.0c4a9c12fe117p+3 0x1.67767a509549ap-1 0 9 0x1.3d160aa3a3803p+6 0x0p+0 0 0 0x0p+0 2 0x1.59e721232e2fep+4 1 1 0x1.a30619857a99cp-3 0x1.28d8e95d1bd2ap+3 0x1.d5b25836d4cfdp-1 0x1.032110acff6d6p+3 2 0x1.95699f25a558bp-1 1 0 0x0p+0 0x0p+0 1 8 0 0 0 0 -0x1.2a449fee686dep+5 -0x1.52bfbcd538f6cp+5 0x1p+1\n"
         "K 1 0\n"         , 12, 0},
        {"m1_292002_25",
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.02ffddc9af687p+6 0x0p+0\n"
         "L 0x1.02ffddc9af687p+6 0x0p+0 0x1.02ffddc9af687p+6 0x1.1d95953f29694p+6\n"
         "L 0x1.02ffddc9af687p+6 0x1.1d95953f29694p+6 0x0p+0 0x1.1d95953f29694p+6\n"
         "L 0x0p+0 0x1.1d95953f29694p+6 0x0p+0 0x0p+0\n"
         "C 1\n"
         "L 0x1.c617078517c16p+5 0x1.f1c9fd83a48d9p+4 0x1.fdce35ba8dbbp+5 0x1.f1c9fd83a48d9p+4\n"
         "L 0x1.fdce35ba8dbbp+5 0x1.f1c9fd83a48d9p+4 0x1.fdce35ba8dbbp+5 0x1.1fe538b40b426p+5\n"
         "L 0x1.fdce35ba8dbbp+5 0x1.1fe538b40b426p+5 0x1.c617078517c16p+5 0x1.1fe538b40b426p+5\n"
         "L 0x1.c617078517c16p+5 0x1.1fe538b40b426p+5 0x1.c617078517c16p+5 0x1.f1c9fd83a48d9p+4\n"
         "C 1\n"
         "L 0x1.e63c47580602p+4 0x1.5315c003e2218p+5 0x1.3fb71a902b56cp+5 0x1.5315c003e2218p+5\n"
         "L 0x1.3fb71a902b56cp+5 0x1.5315c003e2218p+5 0x1.3fb71a902b56cp+5 0x1.88b4067064c3ep+5\n"
         "L 0x1.3fb71a902b56cp+5 0x1.88b4067064c3ep+5 0x1.e63c47580602p+4 0x1.88b4067064c3ep+5\n"
         "L 0x1.e63c47580602p+4 0x1.88b4067064c3ep+5 0x1.e63c47580602p+4 0x1.5315c003e2218p+5\n"
         "P 0x1.8p+2 60 -0x1.7bffcb0b4397dp+1 1 1 0x1.9099ed9e9cffap+0 0x1.7a5de54010ebap+2 0x1.eca82bfbbb783p+2 0x1.ea14da1f33c52p-3 0x1.4p+2 0 0x1.051d57a45bd7p-2 0 1 0x1.e4d379f081e2bp-1 0x1.603100e475485p+0 0 1 0x1.25aa1e2d538aap+6 0x0p+0 1 1 0x0p+0 2 0x1.818421ee6298ap+2 0 0 0x1.bd51a925f616bp-2 0x1.28a50516f3eaep+1 0x1.e8f8ed18c8ee9p-1 0x1.4704a8b42a24bp+3 0 0x1.78fad54048bc1p-3 1 0 0x1.64e1ccb905711p+2 0x0p+0 1 1 0 1 0 1 -0x1.4859c93d17404p+3 -0x1.684eb48465b42p+5 0x1p+1\n"
         "K 1 0\n"         , 0, 0},
        {"m1_292003_7",
         "C 1\n"
         "L 0x1.c1f1858f2e7ebp+3 0x0p+0 0x1.0ac653728a875p+2 0x1.9215e4ae6351dp+1\n"
         "L 0x1.0ac653728a875p+2 0x1.9215e4ae6351dp+1 0x1.326c557a86647p+2 0x1.d7896ae24e00cp+3\n"
         "L 0x1.326c557a86647p+2 0x1.d7896ae24e00cp+3 -0x1.9a01b6c745e11p+0 0x1.356f89d8b2e21p+2\n"
         "L -0x1.9a01b6c745e11p+0 0x1.356f89d8b2e21p+2 -0x1.728beaf192496p+3 0x1.0d37c3b2d3cfcp+3\n"
         "L -0x1.728beaf192496p+3 0x1.0d37c3b2d3cfcp+3 -0x1.3e22b9f25eae6p+2 0x1.44d4d272f09b7p-3\n"
         "L -0x1.3e22b9f25eae6p+2 0x1.44d4d272f09b7p-3 -0x1.85afefc4c2423p+3 -0x1.1b1fd626b574fp+3\n"
         "L -0x1.85afefc4c2423p+3 -0x1.1b1fd626b574fp+3 -0x1.60c35c4537edp+0 -0x1.d49f99f2d55c5p+1\n"
         "L -0x1.60c35c4537edp+0 -0x1.d49f99f2d55c5p+1 0x1.2dfcb6de27e64p+2 -0x1.d0b5e0f15981cp+3\n"
         "L 0x1.2dfcb6de27e64p+2 -0x1.d0b5e0f15981cp+3 0x1.1ea76b1361b59p+2 -0x1.aff38bab5ec3fp+1\n"
         "L 0x1.1ea76b1361b59p+2 -0x1.aff38bab5ec3fp+1 0x1.c1f1858f2e7ebp+3 0x0p+0\n"
         "C 1\n"
         "L 0x1.45fefc2fdb7e6p+4 -0x1.d8b631f0db8e4p+3 0x1.f82898d76ed05p+5 -0x1.d8b631f0db8e4p+3\n"
         "L 0x1.f82898d76ed05p+5 -0x1.d8b631f0db8e4p+3 0x1.12dd3dc119fcep+6 -0x1.226ea745c6e8ap+3\n"
         "L 0x1.12dd3dc119fcep+6 -0x1.226ea745c6e8ap+3 0x1.12dd3dc119fcep+6 0x1.7e8790635e55ap+3\n"
         "L 0x1.12dd3dc119fcep+6 0x1.7e8790635e55ap+3 0x1.f82898d76ed05p+5 0x1.1a678d87397dap+4\n"
         "L 0x1.f82898d76ed05p+5 0x1.1a678d87397dap+4 0x1.45fefc2fdb7e6p+4 0x1.1a678d87397dap+4\n"
         "L 0x1.45fefc2fdb7e6p+4 0x1.1a678d87397dap+4 0x1.d5b66db4a2572p+3 0x1.7e8790635e55ap+3\n"
         "L 0x1.d5b66db4a2572p+3 0x1.7e8790635e55ap+3 0x1.d5b66db4a2572p+3 -0x1.226ea745c6e8ap+3\n"
         "L 0x1.d5b66db4a2572p+3 -0x1.226ea745c6e8ap+3 0x1.45fefc2fdb7e6p+4 -0x1.d8b631f0db8e4p+3\n"
         "P 0x1.9666666666666p+1 0 -0x1.6bb2ec7ca9ecep+0 0 0 0x0p+0 0x1.bd90def0b51e7p+3 0x1.36590b4eca72dp+2 0x0p+0 0x1.4p+2 0 0x1.acc6145041861p-2 0 1 0x1.0283c4bab757fp+2 0x1.54d56306506acp+1 0 9 0x1.0b9a86ef55023p+3 0x0p+0 1 0 0x0p+0 0 0x1.8f2f53f0a56bp+4 1 0 0x1.7c4370a0d6cp-2 0x1.de0fcda8252fbp+3 0x1.3e79ecc03ebeep-1 0x1.0ee25c4f105e8p+0 0 0x1.fd0807692e42ep-1 1 0 0x0p+0 0x0p+0 2 4 0 0 0 0 -0x1.70a1bb9f46a78p+2 0x1.16b3bc6a2d53p+1 0x1p+1\n"
         "K 0 0\n"         , 0, 0},
        {"m1_292007_6",
         "C 1\n"
         "L 0x1.baf2ff349ab6ap+4 0x0p+0 0x1.43ceb8654c3a1p+2 0x1.04aee8e3c5fa5p+3\n"
         "L 0x1.43ceb8654c3a1p+2 0x1.04aee8e3c5fa5p+3 -0x1.04d6d2a97673ap+4 0x1.c3c962cc1040ep+4\n"
         "L -0x1.04d6d2a97673ap+4 0x1.c3c962cc1040ep+4 -0x1.ed00e5adb9ac2p+2 0x1.6d78574882f97p-2\n"
         "L -0x1.ed00e5adb9ac2p+2 0x1.6d78574882f97p-2 -0x1.cbcf09ee473dfp+3 -0x1.8e34bff96efc5p+4\n"
         "L -0x1.cbcf09ee473dfp+3 -0x1.8e34bff96efc5p+4 0x1.0a1f06fbe85a3p+2 -0x1.9f135d9b90f39p+2\n"
         "L 0x1.0a1f06fbe85a3p+2 -0x1.9f135d9b90f39p+2 0x1.baf2ff349ab6ap+4 0x0p+0\n"
         "P 0x1.8p+2 120 -0x1.5b328de7f8165p+0 1 0 0x0p+0 0x1.23c8bccd6f722p+2 0x1.14e6f7e389488p+0 0x1.ddebb3d8831bep-3 0x1.4p+2 1 0x1.947d98b3c077ap-1 0 1 0x1.9e570c51275fap+1 0x1.f78f2eef958e2p-3 0 1 0x1.72d8444836cbp+4 0x0p+0 1 1 0x0p+0 1 0x1.250f4a0a7a755p+4 0 0 0x1.a32f93ac778a4p-2 0x1.117e0a571544ap+1 0x1.639cbc106d5dbp-2 0x1.0ac69272ae812p+3 1 0x1.845f084e3845fp+0 0 0 0x1.f63aca7141b86p+1 0x0p+0 2 5 0 1 0 0 0x1.1ec9ebc645206p+5 0x1.8c8b6878d10d8p+2 0x1p+1\n"
         "K 0 0\n"         , 0, 3},
        {"m2_293000_17",
         "C 1\n"
         "L 0x1.c709af18f91cdp+4 0x0p+0 0x1.35c3345553485p+2 0x1.5323e09a66f96p+2\n"
         "L 0x1.35c3345553485p+2 0x1.5323e09a66f96p+2 0x1.cfd189438947p-50 0x1.a47b66ac19dfdp+4\n"
         "L 0x1.cfd189438947p-50 0x1.a47b66ac19dfdp+4 -0x1.673cee10a3094p+2 0x1.556084a85c215p+2\n"
         "L -0x1.673cee10a3094p+2 0x1.556084a85c215p+2 -0x1.ae5c98a02419dp+4 0x1.dab766537913ep-49\n"
         "L -0x1.ae5c98a02419dp+4 0x1.dab766537913ep-49 -0x1.5c5a9da8209fp+2 -0x1.3ee74a510f1f4p+2\n"
         "L -0x1.5c5a9da8209fp+2 -0x1.3ee74a510f1f4p+2 -0x1.5a09e1ffa5487p-48 -0x1.a246964ed7fcep+4\n"
         "L -0x1.5a09e1ffa5487p-48 -0x1.a246964ed7fcep+4 0x1.27dac3b86fe27p+2 -0x1.3c0cc090758c1p+2\n"
         "L 0x1.27dac3b86fe27p+2 -0x1.3c0cc090758c1p+2 0x1.c709af18f91cdp+4 0x0p+0\n"
         "P 0x1.8p+2 60 -0x1.ec3605e72f71cp+0 1 1 0x0p+0 0x1.7358c5e35d2c3p+2 0x1.9b26c5175da51p+2 0x0p+0 0x1.4p+2 1 0x1.9ff6a36e834b4p-1 0 1 0x1.07879169c778cp+0 0x1.874c7a78c7c96p-1 0 4 0x1.9f5fe9650ef28p+5 0x0p+0 0 1 0x0p+0 0 0x1.ad01e45e5c4efp+3 0 0 0x1.0b2222517329cp-2 0x1.1b2c5d17ba15ep+3 0x1.bcf4e84861c18p-3 0x1.81c1da61e3cb8p+0 1 0x1.4bc50faaae157p+0 1 0 0x1.ec58f71c47032p+3 0x0p+0 1 8 0 0 0 0 -0x1.cf61936ff8c0ap+4 -0x1.5ce4faf98fe78p+3 0x1p+1\n"
         "K 1 0\n"         , 0, 4},
        // One enclosed piece that no kept tab is beside (the loop's one tab holds elsewhere): reported.
        {"m1_302014_9",
         "C 1\n"
         "L 0x1.31f9b584fcaeep+3 -0x1.5eb6d48d58263p+3 0x1.36c1676408ab3p+1 0x1.cae5759116b1bp+3\n"
         "A -0x1.c892b757f5083p+1 0x1.94ce250f376bfp+3 0x1.8e9fedcb343b9p+2 0x1.196e6e8a795aap-2 -0x1.6ef1e772f3a63p+1 0\n"
         "L -0x1.31f9b584fcaeep+3 0x1.5eb6d48d58263p+3 -0x1.36c1676408ab3p+1 -0x1.cae5759116b1bp+3\n"
         "A 0x1.c892b757f5083p+1 -0x1.94ce250f376bfp+3 0x1.8e9fedcb343b9p+2 -0x1.6ef1e772f3a63p+1 0x1.196e6e8a795abp-2 0\n"
         "C 1\n"
         "L 0x1.cdf2d31028ef4p+5 -0x1.81552d859e26fp+1 0x1.4d85093a260c4p+5 0x1.5448c8305c9e6p+0\n"
         "L 0x1.4d85093a260c4p+5 0x1.5448c8305c9e6p+0 0x1.29ea041e47e19p+5 0x1.11974b1b26c22p+4\n"
         "L 0x1.29ea041e47e19p+5 0x1.11974b1b26c22p+4 0x1.08ce16f3d5478p+5 0x1.22427c05a6392p+0\n"
         "L 0x1.08ce16f3d5478p+5 0x1.22427c05a6392p+0 0x1.2e62bb9475be8p+4 -0x1.81552d859e26ap+1\n"
         "L 0x1.2e62bb9475be8p+4 -0x1.81552d859e26ap+1 0x1.0585a0f858f2bp+5 -0x1.fe2a59933fb68p+2\n"
         "L 0x1.0585a0f858f2bp+5 -0x1.fe2a59933fb68p+2 0x1.29ea041e47e19p+5 -0x1.542f636465e8fp+4\n"
         "L 0x1.29ea041e47e19p+5 -0x1.542f636465e8fp+4 0x1.54a4612df075bp+5 -0x1.15d02522edfa6p+3\n"
         "L 0x1.54a4612df075bp+5 -0x1.15d02522edfa6p+3 0x1.cdf2d31028ef4p+5 -0x1.81552d859e26fp+1\n"
         "P 0x1.8p+3 0 -0x1.7c1a5dfa1a37cp+2 0 0 0x0p+0 0x1.750e33fb06dcfp+1 0x1.065a286a79a69p+0 0x0p+0 0x1.4p+2 0 0x1.371f320f45b5dp-1 1 1 0x1.2251c73b7ce62p+0 0x1.1c87d2060fb7cp+1 1 8 0x1.2b7890ff28d15p+5 0x0p+0 0 0 0x0p+0 1 0x1.c9347d512eaf1p+3 0 0 0x1.c7eacf9a01509p-2 0x1.de02f51eb055bp+3 0x1.b91ee8b07361bp-1 0x1.c2713c9dc3511p+2 0 0x1.48e201a17371cp+0 0 0 0x1.67259b1808192p+2 0x0p+0 0 5 0 0 0 0 0x1.8fd97e40b5d52p+5 -0x1.9f68161677a18p+4 0x1p+1\n"
         "K 0 0\n"         , 2, 1},
        // Two small loose pieces (about 0.05 square mm each) at two pocket corners: both reported (one
        // message each since T-0135; the de-duplication used to merge their equal rounded areas).
        {"m2_303014_28",
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.084762f0ae4fap+5 0x0p+0\n"
         "L 0x1.084762f0ae4fap+5 0x0p+0 0x1.084762f0ae4fap+5 0x1.084762f0ae4fap+5\n"
         "L 0x1.084762f0ae4fap+5 0x1.084762f0ae4fap+5 0x1.93e670831b88dp+4 0x1.084762f0ae4fap+5\n"
         "L 0x1.93e670831b88dp+4 0x1.084762f0ae4fap+5 0x1.93e670831b88dp+4 0x1.f2a155790459dp+2\n"
         "L 0x1.93e670831b88dp+4 0x1.f2a155790459dp+2 0x1.f2a155790459dp+2 0x1.f2a155790459dp+2\n"
         "L 0x1.f2a155790459dp+2 0x1.f2a155790459dp+2 0x1.f2a155790459dp+2 0x1.084762f0ae4fap+5\n"
         "L 0x1.f2a155790459dp+2 0x1.084762f0ae4fap+5 0x0p+0 0x1.084762f0ae4fap+5\n"
         "L 0x0p+0 0x1.084762f0ae4fap+5 0x0p+0 0x0p+0\n"
         "P 0x1.8p+2 0 -0x1.8e87d918b99d9p+0 1 0 0x0p+0 0x1.9da97e7631fe9p+2 0x1.4a8856dc1a03ap+0 0x0p+0 0x1.4p+2 1 0x1.a3800b0a070a8p-6 1 1 0x1.5727393b8a41bp+1 0x1.9cca2f16ecea1p+1 1 4 0x1.315ce81f0af2dp+5 0x0p+0 1 0 0x0p+0 3 0x1.26cba71f06d8p+4 1 1 0x1.09a3b1dd14939p-1 0x1.00512dbc0e0bcp+2 0x1.f0da4080b2992p-1 0x1.02472f956c8fp+3 0 0x1.10705aa6e84cbp+0 1 0 0x1.30b2522bdc40cp+4 0x0p+0 0 5 0 0 0 0 -0x1.c5ba342557c67p+4 0x1.37265a711219cp+5 0x1p+1\n"
         "K 0 0\n"         , 4, 2},
        // A roughing cover shorter than its finishing stretch (last pass 0.001 mm): only the tabs
        // that hold are kept (a quick answer once kept one the roughing cut severs).
        {"m0_301012_20",
         "C 1\n"
         "L 0x1.f04e6207aec9cp+3 -0x1.01ccffb61ac3ap+3 -0x1.ff431448768cp-2 0x1.1785212e6b132p+4\n"
         "A -0x1.00243d54f93f1p+3 0x1.986ba1097874fp+3 0x1.1b7cdf848e82fp+3 0x1.1ecb56c2daa48p-1 -0x1.4a6cdf938c286p+1 0\n"
         "L -0x1.f04e6207aec9cp+3 0x1.01ccffb61ac3ap+3 0x1.ff431448768cp-2 -0x1.1785212e6b132p+4\n"
         "A 0x1.00243d54f93f1p+3 -0x1.986ba1097874fp+3 0x1.1b7cdf848e82fp+3 -0x1.4a6cdf938c286p+1 0x1.1ecb56c2daa48p-1 0\n"
         "P 0x1.8p+3 0 0x0p+0 2 0 0x0p+0 0x1.280c513b877ddp+1 0x1.a809074ced7c8p-1 0x0p+0 0x1.4p+2 1 0x1.7d70e5d1b354p-11 0 1 0x1.53c98607de9a8p-1 0x1.65041009c716cp+0 0 9 0x1.75f122fd8ef7p+5 0x0p+0 0 1 0x0p+0 2 0x1.b1102616c7e8dp+1 1 0 0x1.96a0fa8d86107p-2 0x1.fbe7b3a1a550ap+2 0x1.29e9d0a8d6e34p-3 0x1.be64fc5511842p+2 1 0x1.4ae953775cc2fp-2 0 0 0x0p+0 0x0p+0 2 1 0 1 0 1 0x1.bfedc4d5afc14p+4 -0x1.7cbe57514a44ap+5 0x1p+1\n"
         "K 0 0\n"         , 7, 0},
    };
    for (const Case& k : cases) {
        std::vector<Contour> cs;
        ProfileParams p;
        int vbitDeg = 0;
        const double d = readScene(k.text, cs, p, vbitDeg);
        tooling::Tool tool = test::endMill(d);
        if (vbitDeg > 0) {
            tool.type = tooling::ToolType::VBit;
            tool.includedAngle = vbitDeg * geometry::kPi / 180.0;
        }
        INFO("scene " << k.name);
        const ProfileResult r = profileToolpath(cs, tool, p);
        REQUIRE(r.ok());
        const auto loose = static_cast<std::size_t>(std::count_if(r.diagnostics.begin(), r.diagnostics.end(), [](const ProfileDiagnostic& g) {
            return g.message.find("not held by any kept tab") != std::string::npos;
        }));
        CHECK(tabsLeftOut(r) == k.left);
        CHECK(loose == k.loose);
    }
}

TEST_CASE("Loose pieces: one message per piece, pieces in a notch kept apart", "[toolpath][profile][tabs]") {
    // T-0135 (a)/(b), T-0127 round-3 verifier (harness t0127v3, whole-scene flood wsf8). An H
    // On the vector (D6, allowance 0) leaves two free pieces of 0.626 and 0.621 mm2; both were
    // reported with the same text, and the de-duplication in run() kept one. Three shapes On the
    // vector: B (375 mm2) and C (64 mm2, in a concave notch of B's outline, so their boxes overlap)
    // come free, and C was taken for B. Now every piece has its own numbered message.
    struct Case {
        const char* name;
        const char* text;
        std::vector<double> areas;  // the flood's free pieces, largest first
    };
    const std::vector<Case> cases{
        {"h",
         "C 1\n"
         "L -0x1.e353f7ced9168p+2 -0x1.e353f7ced9168p+2 -0x1.1cccccccccccdp+1 -0x1.e353f7ced9168p+2\n"
         "L -0x1.1cccccccccccdp+1 -0x1.e353f7ced9168p+2 -0x1.1cccccccccccdp+1 -0x1.272b020c49ba6p+0\n"
         "L -0x1.1cccccccccccdp+1 -0x1.272b020c49ba6p+0 0x1.4fbe76c8b4396p+2 -0x1.272b020c49ba6p+0\n"
         "L 0x1.4fbe76c8b4396p+2 -0x1.272b020c49ba6p+0 0x1.4fbe76c8b4396p+2 -0x1.e353f7ced9168p+2\n"
         "L 0x1.4fbe76c8b4396p+2 -0x1.e353f7ced9168p+2 0x1.525604189374cp+3 -0x1.e353f7ced9168p+2\n"
         "L 0x1.525604189374cp+3 -0x1.e353f7ced9168p+2 0x1.525604189374cp+3 0x1.525604189374cp+3\n"
         "L 0x1.525604189374cp+3 0x1.525604189374cp+3 0x1.4fbe76c8b4396p+2 0x1.525604189374cp+3\n"
         "L 0x1.4fbe76c8b4396p+2 0x1.525604189374cp+3 0x1.4fbe76c8b4396p+2 0x1.0b22d0e560419p+2\n"
         "L 0x1.4fbe76c8b4396p+2 0x1.0b22d0e560419p+2 -0x1.1cccccccccccdp+1 0x1.0b22d0e560419p+2\n"
         "L -0x1.1cccccccccccdp+1 0x1.0b22d0e560419p+2 -0x1.1cccccccccccdp+1 0x1.525604189374cp+3\n"
         "L -0x1.1cccccccccccdp+1 0x1.525604189374cp+3 -0x1.e353f7ced9168p+2 0x1.525604189374cp+3\n"
         "L -0x1.e353f7ced9168p+2 0x1.525604189374cp+3 -0x1.e353f7ced9168p+2 -0x1.e353f7ced9168p+2\n"
         "P 0x1.8p+2 0 0x0p+0 2 0 0x0p+0 0x1.ccccccccccccdp+3 0x1.3333333333333p+0 0x0p+0 0x1.4p+2 0 0x1.999999999999ap-5 0 1 0x1.5c28f5c28f5c3p-1 0x1.0666666666666p+2 0 2 0x1.9p+6 0x0p+0 0 0 0x0p+0 0 0x1.4p+2 0 0 0x1.657184ae74489p-4 0x1.4p+2 0x1.921fb54442d18p-1 0x1.4p+2 0 0x1.921fb54442d18p+0 0 0 0x0p+0 0x0p+0 2 0 0 0 0 0 0x0p+0 0x0p+0 0x1p+1\n"
         "K 0 0\n"
         , {0.626, 0.621}},
        {"abc_2_1_2",
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.4p+5 0x0p+0\n"
         "L 0x1.4p+5 0x0p+0 0x1.4p+5 0x1.28p+4\n"
         "L 0x1.4p+5 0x1.28p+4 0x1.9p+5 0x1.28p+4\n"
         "L 0x1.9p+5 0x1.28p+4 0x1.9p+5 0x0p+0\n"
         "L 0x1.9p+5 0x0p+0 0x1.68p+6 0x0p+0\n"
         "L 0x1.68p+6 0x0p+0 0x1.68p+6 0x1.8p+3\n"
         "L 0x1.68p+6 0x1.8p+3 0x1.42p+6 0x1.8p+3\n"
         "L 0x1.42p+6 0x1.8p+3 0x1.42p+6 0x1.6p+4\n"
         "L 0x1.42p+6 0x1.6p+4 0x1.58p+6 0x1.6p+4\n"
         "L 0x1.58p+6 0x1.6p+4 0x1.58p+6 0x1.2p+5\n"
         "L 0x1.58p+6 0x1.2p+5 0x1.2p+6 0x1.2p+5\n"
         "L 0x1.2p+6 0x1.2p+5 0x1.2p+6 0x1.6p+4\n"
         "L 0x1.2p+6 0x1.6p+4 0x1.36p+6 0x1.6p+4\n"
         "L 0x1.36p+6 0x1.6p+4 0x1.36p+6 0x1.8p+3\n"
         "L 0x1.36p+6 0x1.8p+3 0x1.fp+5 0x1.8p+3\n"
         "L 0x1.fp+5 0x1.8p+3 0x1.fp+5 0x1.4p+5\n"
         "L 0x1.fp+5 0x1.4p+5 0x1.9p+5 0x1.4p+5\n"
         "L 0x1.9p+5 0x1.4p+5 0x1.9p+5 0x1.58p+4\n"
         "L 0x1.9p+5 0x1.58p+4 0x1.4p+5 0x1.58p+4\n"
         "L 0x1.4p+5 0x1.58p+4 0x1.4p+5 0x1.4p+5\n"
         "L 0x1.4p+5 0x1.4p+5 0x0p+0 0x1.4p+5\n"
         "L 0x0p+0 0x1.4p+5 0x0p+0 0x0p+0\n"
         "P 0x1.8p+2 0 0x0p+0 2 0 0x0p+0 0x1.8p+2 0x1.8p+1 0x0p+0 0x1.4p+2 0 0x1.999999999999ap-3 0 1 0x1p+2 0x1.8p+0 0 1 0x1.9p+6 0x0p+0 0 0 0x0p+0 0 0x1.4p+3 0 0 0x1.657184ae74489p-4 0x1.4p+2 0x1.921fb54442d18p-1 0x1.4p+2 0 0x1.921fb54442d18p+0 0 0 0x0p+0 0x0p+0 2 2 0 0 0 0 0x0p+0 0x0p+0 0x1p+1\n"
         "K 0 0\n"
         , {374.7, 64.4}},
    };
    for (const Case& k : cases) {
        std::vector<Contour> cs;
        ProfileParams p;
        int vbitDeg = 0;
        const double d = readScene(k.text, cs, p, vbitDeg);
        INFO("scene " << k.name);
        const ProfileResult r = profileToolpath(cs, test::endMill(d), p);
        REQUIRE(r.ok());
        std::vector<std::string> loose;
        for (const ProfileDiagnostic& g : r.diagnostics) {
            if (g.message.find("comes free") != std::string::npos) loose.push_back(g.message);
        }
        REQUIRE(loose.size() == k.areas.size());
        for (std::size_t i = 0; i < loose.size(); ++i) {
            INFO(loose[i]);
            const std::string n = "loose piece " + std::to_string(i + 1) + " of " + std::to_string(loose.size()) + ":";
            CHECK(loose[i].find(n) != std::string::npos);
            const std::size_t at = loose[i].find("(about ");
            REQUIRE(at != std::string::npos);
            const double area = std::stod(loose[i].substr(at + 7));
            CHECK(std::fabs(area - k.areas[i]) < 0.02 * k.areas[i] + 0.01);
        }
    }
}

namespace {

// The flood check behind the counts of the tab-matching scenes (T-0159): each closed vector's part
// (Outside: its deepest inside point; Inside: the window corner beyond it) stays joined to the waste
// on the other side through the kept tabs, on a 0.05 mm raster at the tab top, independently of
// the toolpath's own diagnostics. For end mills; the scenes it serves have one vector or parts
// that do not nest.
void checkPartsHeld(const ProfileResult& r, const std::vector<Contour>& cs, const ProfileParams& p, double d) {
    const double zTab = p.passes.bottomZ() + std::min(p.tabs.thickness, p.passes.cutDepth);
    geometry::Box2 all;
    for (const Contour& c : cs) all.expand(c.bounds());
    const geometry::Box2 window = all.inflated(d + 2.0);
    for (const Contour& c : cs) {
        // The point of the vector's inside furthest from it.
        Vec2 inside = c.bounds().centre();
        double best = -1.0;
        const geometry::Box2 cb = c.bounds();
        for (double x = cb.min.x; x <= cb.max.x; x += 0.5) {
            for (double y = cb.min.y; y <= cb.max.y; y += 0.5) {
                const Vec2 q{x, y};
                if (geometry::locatePoint(c, q, geometry::FillRule::EvenOdd, geometry::Tolerance{}) !=
                    geometry::PointLocation::Inside) {
                    continue;
                }
                const double dq = geometry::distanceTo(c, q);
                if (dq > best) {
                    best = dq;
                    inside = q;
                }
            }
        }
        INFO("vector bounds " << cb.min.x << "," << cb.min.y << " .. " << cb.max.x << "," << cb.max.y);
        CHECK(test::floodHeld(r.toolpath, window, zTab, 0.5 * d, inside, window.min + Vec2{0.5, 0.5}, 0.05));
    }
}

}  // namespace

TEST_CASE("Separate last pass, negative allowance: tabs that hold are matched on the roughing cut",
          "[toolpath][profile][tabs]") {
    // T-0135 (c), T-0127 round-3 verifier: Outside with allowance -1.8 (D8) / -5.1 (D12) and a separate last
    // pass, T-0127 left out tabs as "could not be matched on the roughing cut" that the T-0124
    // merge matched and that hold (whole-scene flood): the cross-sections that place the roughing
    // cover cross the vector with the kerf, so at a concave corner of the part the cover grew into
    // a neighbouring tab or start. Such a tab is now matched again with the cover placed from the
    // vector out, and kept when the exact check says it holds. Expected counts are flood-verified
    // (every kept tab holds; 15 and 4 kept, the T-0124 merge 14 and 4, T-0127 12 and 3).
    struct Case {
        const char* name;
        const char* text;
        std::size_t left;
    };
    const std::vector<Case> cases{
        {"m1_402002_27",
         "C 1\n"
         "L 0x1.dd82abb1ec249p+4 0x0p+0 0x1.142c3db02faffp+2 0x1.1892a576ee98p+2\n"
         "L 0x1.142c3db02faffp+2 0x1.1892a576ee98p+2 0x1.ce556a3e12eadp-50 0x1.a322cbd1a1937p+4\n"
         "L 0x1.ce556a3e12eadp-50 0x1.a322cbd1a1937p+4 -0x1.6c6095b99b904p+2 0x1.7c124425a5b5cp+2\n"
         "L -0x1.6c6095b99b904p+2 0x1.7c124425a5b5cp+2 -0x1.d6fe8001d646bp+4 0x1.03c4ae770ca4ep-48\n"
         "L -0x1.d6fe8001d646bp+4 0x1.03c4ae770ca4ep-48 -0x1.9aa7b9c475dcfp+2 -0x1.b1dfcb1a14ab5p+2\n"
         "L -0x1.9aa7b9c475dcfp+2 -0x1.b1dfcb1a14ab5p+2 -0x1.8e9659678c5a5p-48 -0x1.e1cb511eda43fp+4\n"
         "L -0x1.8e9659678c5a5p-48 -0x1.e1cb511eda43fp+4 0x1.9503552e872fdp+2 -0x1.a3e98032a0d17p+2\n"
         "L 0x1.9503552e872fdp+2 -0x1.a3e98032a0d17p+2 0x1.dd82abb1ec249p+4 0x0p+0\n"
         "C 1\n"
         "A 0x1.fcf7d51658504p+5 0x1.4fb90c77fd368p+1 0x1.4e8d6cf5d5f83p+4 0x1.753ae29b9caddp-7 0x1.8f353f7f0b982p+0 0\n"
         "A 0x1.7623be4fdbc44p+5 0x1.3f3e485f17135p+4 0x1.176577b8dd8dep+4 0x1.a6a97df8df288p-3 -0x1.c6f4e5035eb6ap+0 1\n"
         "A 0x1.c2d651d3371bbp+5 -0x1.292f93dd0b09p+2 0x1.e4eec06ed0ba8p+3 0x1.4fcf498ad7157p+1 -0x1.0d7eddd16b597p+0 0\n"
         "A 0x1.69db8f376ebc8p+6 -0x1.7be877deec562p+4 0x1.b425d6b092dbp+4 0x1.76076dda38f57p+1 0x1.ca5044185689ap+0 1\n"
         "P 0x1p+3 0 -0x1.cc29aed359599p+0 0 1 0x0p+0 0x1.c31079c96bff4p+1 0x1.35c38b6e26cp+0 0x0p+0 0x1.4p+2 1 0x1.2892648f4a083p-4 1 1 0x1.57de30f1a586dp+1 0x1.64b8c5a624b6ap+0 0 9 0x1.ff1d464625ca3p+5 0x0p+0 0 0 0x0p+0 2 0x1.0f94a87fe5848p+5 0 0 0x1.79275e5da79b3p-3 0x1.5a641b1ea9bc7p+3 0x1.ad8c32b3e735p-1 0x1.49787b28b0082p+2 0 0x1.ca6de50a2917ap-2 0 0 0x0p+0 0x0p+0 0 4 0 0 0 1 -0x1.6edb34ad5eba8p+5 -0x1.415749f156884p+3 0x1p+1\n"
         "K 0 0\n"
         , 3},
        {"m2_403003_37",
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.9ad18d11f539cp+5 0x0p+0\n"
         "L 0x1.9ad18d11f539cp+5 0x0p+0 0x1.9ad18d11f539cp+5 0x1.46997b693b63ep+5\n"
         "L 0x1.9ad18d11f539cp+5 0x1.46997b693b63ep+5 0x0p+0 0x1.46997b693b63ep+5\n"
         "L 0x0p+0 0x1.46997b693b63ep+5 0x0p+0 0x0p+0\n"
         "P 0x1.8p+3 0 -0x1.4513f7bd5863dp+2 0 1 0x0p+0 0x1.dd0b327940cadp+2 0x1.76b6c4f9d983ap+2 0x1.18d1720910265p-2 0x1.4p+2 1 0x1.4a1ab742e425cp-7 0 1 0x1.39a6f226e093bp+2 0x1.b147e69bff2a6p+0 0 9 0x1.59f2b258c58p+3 0x0p+0 0 0 0x0p+0 2 0x1.85d13f2a0da99p+2 1 0 0x1.77dd0c339da01p-2 0x1.b8ddd157ec117p+2 0x1.dfa3b097b2ac4p-2 0x1.a8662ad776d6p+2 0 0x1.7133141dd9a4ep+0 0 0 0x0p+0 0x0p+0 1 6 0 0 0 0 0x1.6a4a4e048d094p+5 0x1.6dc023c50f6f4p+4 0x1p+1\n"
         "K 0 0\n"
         , 5},
        // T-0135 sweep: two closed vectors (D12, Outside, allowance -5.58, last pass 0.61). The
        // inner vector's one tab holds (flood), but its check saw the material between the
        // vectors as two pieces, joined only beyond its raster window, so it was dropped as cut
        // through and its part came free. It is judged again on a wider window and kept.
        // T-0135 round 1 (Outside, D12, allowance -0.0101): a finishing tab moved off a roughing
        // start pushed its neighbour out and was then refused on the roughing cut (T-0127 kept 3
        // tabs, the first branch 2). The tabs are now fitted again without the refused one: 4 kept,
        // all holding (flood).
        {"m1_502006_8",
         "C 1\n"
         "L -0x1.18e2048cecb54p+3 -0x1.9499354fd8bcbp+2 0x1.18e2048cecb54p+3 -0x1.9499354fd8bcbp+2\n"
         "L 0x1.18e2048cecb54p+3 -0x1.9499354fd8bcbp+2 0x1.512a5717df47fp+3 -0x1.24089039f3975p+2\n"
         "L 0x1.512a5717df47fp+3 -0x1.24089039f3975p+2 0x1.512a5717df47fp+3 0x1.24089039f3975p+2\n"
         "L 0x1.512a5717df47fp+3 0x1.24089039f3975p+2 0x1.18e2048cecb54p+3 0x1.9499354fd8bcbp+2\n"
         "L 0x1.18e2048cecb54p+3 0x1.9499354fd8bcbp+2 -0x1.18e2048cecb54p+3 0x1.9499354fd8bcbp+2\n"
         "L -0x1.18e2048cecb54p+3 0x1.9499354fd8bcbp+2 -0x1.512a5717df47fp+3 0x1.24089039f3975p+2\n"
         "L -0x1.512a5717df47fp+3 0x1.24089039f3975p+2 -0x1.512a5717df47fp+3 -0x1.24089039f3975p+2\n"
         "L -0x1.512a5717df47fp+3 -0x1.24089039f3975p+2 -0x1.18e2048cecb54p+3 -0x1.9499354fd8bcbp+2\n"
         "C 1\n"
         "L 0x1.9a102fe2b8664p+5 -0x1.9e953c5ef31fep+0 0x1.5a8ec1164fbdap+5 -0x1.0c6522bab1cacp-1\n"
         "L 0x1.5a8ec1164fbdap+5 -0x1.0c6522bab1cacp-1 0x1.6cdd2920c1d6ap+5 0x1.719d24aebf8f8p+2\n"
         "L 0x1.6cdd2920c1d6ap+5 0x1.719d24aebf8f8p+2 0x1.49c41ff8eea07p+5 0x1.87a602f859cb6p+0\n"
         "L 0x1.49c41ff8eea07p+5 0x1.87a602f859cb6p+0 0x1.23cc6899772dep+5 0x1.b384e2da5f9c8p+2\n"
         "L 0x1.23cc6899772dep+5 0x1.b384e2da5f9c8p+2 0x1.3b728d663f4aap+5 -0x1.40dfeda6ea691p-1\n"
         "L 0x1.3b728d663f4aap+5 -0x1.40dfeda6ea691p-1 0x1.076289d5b7de7p+5 -0x1.9e953c5ef31f9p+0\n"
         "L 0x1.076289d5b7de7p+5 -0x1.9e953c5ef31f9p+0 0x1.332285eafdb39p+5 -0x1.a10ce438a3b06p+1\n"
         "L 0x1.332285eafdb39p+5 -0x1.a10ce438a3b06p+1 0x1.27d8311444994p+5 -0x1.255fa2d3c7207p+3\n"
         "L 0x1.27d8311444994p+5 -0x1.255fa2d3c7207p+3 0x1.498826e7cb77fp+5 -0x1.40abf1ed99c44p+2\n"
         "L 0x1.498826e7cb77fp+5 -0x1.40abf1ed99c44p+2 0x1.7062433110d32p+5 -0x1.38d6e5a7284efp+3\n"
         "L 0x1.7062433110d32p+5 -0x1.38d6e5a7284efp+3 0x1.629747e4f1f03p+5 -0x1.b2cc85240e63bp+1\n"
         "L 0x1.629747e4f1f03p+5 -0x1.b2cc85240e63bp+1 0x1.9a102fe2b8664p+5 -0x1.9e953c5ef31fep+0\n"
         "P 0x1.8p+3 0 -0x1.498b02b7734f3p-7 0 0 0x0p+0 0x1.24702d9833b6ap+3 0x1.28f7a95b508d9p+2 0x0p+0 0x1.4p+2 1 0x1.dd533d5e95dccp-1 1 1 0x1.1375b36f3725ep+3 0x1.889d50d4d6373p+1 0 9 0x1.3ee6003cef4e5p+6 0x0p+0 0 1 0x0p+0 1 0x1.252c10bc735cap+4 0 0 0x1.46094c1b2891bp-4 0x1.023a567bd261bp+2 0x1.042661f523fd1p-2 0x1.8a2c6b603c673p+2 0 0x1.991bbcc2bd5bcp-1 0 0 0x0p+0 0x0p+0 0 5 0 0 0 0 -0x1.4f96292b478bap+5 -0x1.9a092ef3c248p+4 0x1p+1\n"
         "K 0 0\n"
         , 14},
        {"m1_502001_32",
         "C 1\n"
         "A 0x0p+0 0x0p+0 0x1.040bbcb8cb282p+5 -0x1.a1bd920dee676p-1 -0x1.29b050c0c737bp+1 0\n"
         "L -0x1.645fb754ac602p+4 -0x1.7ace02ba974bap+4 0x1.645fb754ac602p+4 -0x1.7ace02ba974b9p+4\n"
         "C 1\n"
         "L 0x1.e808e06f94991p+5 -0x1.f5a63e4f6eb9ep+0 0x1.bdfae793978edp+5 0x1.3673a225341bp+1\n"
         "A 0x1.9ddc310ed4ea3p+5 -0x1.31a61fdd801e9p+1 0x1.912709a683606p+2 0x1.c04ba158c3ecp-1 -0x1.220cccee11d68p+1 0\n"
         "L 0x1.7dbd7a8a12459p+5 -0x1.ccdff0f01a2cp+2 0x1.a7cb73660f4fdp+5 -0x1.72c1a8cb47f4p+3\n"
         "A 0x1.c7ea29ead1f47p+5 -0x1.b176709535cb4p+2 0x1.912709a683606p+2 -0x1.220cccee11d68p+1 0x1.c04ba158c3ecp-1 0\n"
         "P 0x1.8p+3 0 -0x1.64e4a36e7a252p+2 0 1 0x0p+0 0x1.847f478991884p+3 0x1.349825e96f908p+1 0x0p+0 0x1.4p+2 1 0x1.3a017750c8d84p-1 1 1 0x1.5c12f409108bbp+1 0x1.ba053eac5778ap+0 1 6 0x1.c6719fb131e56p+5 0x0p+0 0 0 0x0p+0 3 0x1.b267f398cafbdp+3 0 0 0x1.b037257ccb2fdp-2 0x1.3f740aa7a3b3ap+3 0x1.2f7974e46a9bbp+0 0x1.5a962d746cb06p+3 0 0x1.51cb73a231e41p-1 0 0 0x0p+0 0x0p+0 0 8 0 1 0 0 0x1.57cb878b69a58p+2 -0x1.96a5da5f3fa54p+3 0x1p+1\n"
         "K 0 0\n"
         , 0},
    };
    for (const Case& k : cases) {
        std::vector<Contour> cs;
        ProfileParams p;
        int vbitDeg = 0;
        const double d = readScene(k.text, cs, p, vbitDeg);
        INFO("scene " << k.name);
        const ProfileResult r = profileToolpath(cs, test::endMill(d), p);
        REQUIRE(r.ok());
        for (const ProfileDiagnostic& g : r.diagnostics) {
            INFO(g.message);
            // (m1_502006_8 leaves one tab per vector out as unmatched: it cannot be covered.)
            if (std::string(k.name) != "m1_502006_8") {
                CHECK(g.message.find("could not be matched") == std::string::npos);
            }
            CHECK(g.message.find("comes free") == std::string::npos);
        }
        if (std::string(k.name) == "m1_402002_27") {
            // m1_402002_27 starts at the star's convex start vertex, the centre of the cut's round
            // join there: every point of that arc is equally near. The cut start followed the
            // rounding of a ~1e-15 radial vector, which differed between GCC and MSVC (3 vs 2 tabs
            // left out). Since T-0158 such a tie starts at the beginning of the corner arc on every
            // platform (ADR-0016 §11), so the count is exact again; the parts still hold (flood).
            CHECK(tabsLeftOut(r) == k.left);
            checkPartsHeld(r, cs, p, d);
            continue;
        }
        if (tabsLeftOut(r) == k.left) {
            continue;
        }
        // T-0159: the other scenes' counts are knife-edge (rounding of the tab fitting): GCC 15 with
        // FMA contraction keeps one more tab in m1_502006_8. Where a platform's count differs from
        // the reference, a platform-independent check applies: no more tabs left out than the
        // reference, and each vector's part stays joined to the waste through the kept tabs (flood).
        CHECK(tabsLeftOut(r) <= k.left);
        checkPartsHeld(r, cs, p, d);
    }
}

TEST_CASE("Loose pieces: slugs inside a part are pieces of their own; slivers are not", "[toolpath][profile][tabs]") {
    // T-0135 sweep (whole-scene flood wsf8, seeds 50xxxx). (1) A plate with three holes, Outside
    // with allowance -r (D2) and every tab cut through: the plate (1937.8 mm2) and the three slugs
    // (112.3, 50.2, 16.3 mm2) come free. The plate was a hole of the cut bands with the holes'
    // bands as islands inside it: reported as 2313.9 mm2 (islands included), the slugs taken for
    // the plate. Each slug is now reported against its hole. (2) On the vector, a V-bit with a
    // separate last pass leaves a 0.04 mm sliver ring between the two kerfs; a pocket of it at a
    // concave corner showed one 0.05 mm cell just clear of the tools and was reported as a loose
    // 0.14 mm2 piece, though the flood (to 0.005 mm cells) finds nothing free: small pieces are
    // checked again on finer cells, where the sliver's hairline joins them to the rest.
    {
        std::vector<Contour> cs;
        ProfileParams p;
        int vbitDeg = 0;
        const double d = readScene(
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.43845c9375423p+5 0x0p+0\n"
         "L 0x1.43845c9375423p+5 0x0p+0 0x1.43845c9375423p+5 0x1.f1dcb918ffa76p+5\n"
         "L 0x1.43845c9375423p+5 0x1.f1dcb918ffa76p+5 0x0p+0 0x1.f1dcb918ffa76p+5\n"
         "L 0x0p+0 0x1.f1dcb918ffa76p+5 0x0p+0 0x0p+0\n"
         "C 1\n"
         "A 0x1.b5bb8b27b2d8cp+4 0x1.4a6059582e2bbp+4 0x1.bea167ffc28bcp+2 0x0p+0 0x0p+0 0\n"
         "C 1\n"
         "A 0x1.b6b0bc185a1fap+4 0x1.86a2c7f6cdd5cp+5 0x1.3fc4490c2a6d7p+2 0x0p+0 0x0p+0 0\n"
         "C 1\n"
         "L 0x1.d649e1ca744d5p+3 0x1.ca9e8d680907bp+2 0x1.5f7e11e4b6a4ep+4 0x1.ca9e8d680907bp+2\n"
         "L 0x1.5f7e11e4b6a4ep+4 0x1.ca9e8d680907bp+2 0x1.5f7e11e4b6a4ep+4 0x1.883274e67f9aep+3\n"
         "L 0x1.5f7e11e4b6a4ep+4 0x1.883274e67f9aep+3 0x1.d649e1ca744d5p+3 0x1.883274e67f9aep+3\n"
         "L 0x1.d649e1ca744d5p+3 0x1.883274e67f9aep+3 0x1.d649e1ca744d5p+3 0x1.ca9e8d680907bp+2\n"
         "P 0x1p+1 0 -0x1p+0 0 1 0x0p+0 0x1.58401f06b465cp+3 0x1.201dbbf08f21dp+0 0x0p+0 0x1.4p+2 0 0x1.ac6037db2f75p-5 0 1 0x1.ac6175a101b81p-4 0x1.0daeef1938d3p+1 1 1 0x1.e108842babc05p+5 0x0p+0 0 0 0x0p+0 2 0x1.8045045c77dfap+1 0 0 0x1.59d33e31ae597p-2 0x1.52642d73e54e8p+2 0x1.5d479606a7e87p-1 0x1.8ec3a3ed23a8ap+1 0 0x1.39958a5141bd5p+0 0 0 0x0p+0 0x0p+0 2 0 0 1 0 0 0x1.fa2a78c557p-3 -0x1.e1b2b49dcc0a8p+3 0x1p+1\n"
         "K 0 0\n"
            , cs, p, vbitDeg);
        const ProfileResult r = profileToolpath(cs, test::endMill(d), p);
        REQUIRE(r.ok());
        const std::vector<std::pair<std::size_t, double>> expected{{0, 1937.8}, {1, 112.3}, {2, 50.2}, {3, 16.3}};
        std::vector<std::pair<std::size_t, double>> got;
        for (const ProfileDiagnostic& g : r.diagnostics) {
            if (g.message.find("comes free") == std::string::npos) continue;
            INFO(g.message);
            CHECK(g.message.find("loose piece 1 of 1:") != std::string::npos);
            REQUIRE(g.contour.has_value());
            got.push_back({*g.contour, std::stod(g.message.substr(g.message.find("(about ") + 7))});
        }
        REQUIRE(got.size() == expected.size());
        std::sort(got.begin(), got.end());
        for (std::size_t i = 0; i < got.size(); ++i) {
            CHECK(got[i].first == expected[i].first);
            CHECK(std::fabs(got[i].second - expected[i].second) < 0.01 * expected[i].second + 0.05);
        }
    }
    {
        std::vector<Contour> cs;
        ProfileParams p;
        int vbitDeg = 0;
        const double d = readScene(
         "C 1\n"
         "L 0x1.4521c8b10277bp+4 0x0p+0 0x1.6ed7831263aedp+4 0x1.1c923cc9e8ccp+1\n"
         "L 0x1.6ed7831263aedp+4 0x1.1c923cc9e8ccp+1 0x1.65da48cf0938ep+4 0x1.60cfa0e7732d5p+2\n"
         "L 0x1.65da48cf0938ep+4 0x1.60cfa0e7732d5p+2 0x1.32b36962c05a7p+4 0x1.afa7c9417ff44p+2\n"
         "L 0x1.32b36962c05a7p+4 0x1.afa7c9417ff44p+2 0x1.1fe3db047de2dp+4 0x1.2e3156110edb5p+3\n"
         "L 0x1.1fe3db047de2dp+4 0x1.2e3156110edb5p+3 0x1.344a9d708dfcbp+4 0x1.93f421c5f6857p+3\n"
         "L 0x1.344a9d708dfcbp+4 0x1.93f421c5f6857p+3 0x1.13df78e6fcf33p+4 0x1.e8cddfa3323f8p+3\n"
         "L 0x1.13df78e6fcf33p+4 0x1.e8cddfa3323f8p+3 0x1.bad70dc041e74p+3 0x1.dc2afbe6114ccp+3\n"
         "L 0x1.bad70dc041e74p+3 0x1.dc2afbe6114ccp+3 0x1.71645b57305d8p+3 0x1.0b940fb0a152ap+4\n"
         "L 0x1.71645b57305d8p+3 0x1.0b940fb0a152ap+4 0x1.663afefcccc81p+3 0x1.421c9d9bc9307p+4\n"
         "L 0x1.663afefcccc81p+3 0x1.421c9d9bc9307p+4 0x1.056358f560521p+3 0x1.589ca29f6faf7p+4\n"
         "L 0x1.056358f560521p+3 0x1.589ca29f6faf7p+4 0x1.55a88442560fap+2 0x1.39b63e4bf5af5p+4\n"
         "L 0x1.55a88442560fap+2 0x1.39b63e4bf5af5p+4 0x1.3985cac1d7884p+1 0x1.42c2ea7be3298p+4\n"
         "L 0x1.3985cac1d7884p+1 0x1.42c2ea7be3298p+4 0x1.1cfcb638b2bfp-1 0x1.70746e9bc01ebp+4\n"
         "L 0x1.1cfcb638b2bfp-1 0x1.70746e9bc01ebp+4 -0x1.6366ff3c8fa36p+1 0x1.6de00ca167d4p+4\n"
         "L -0x1.6366ff3c8fa36p+1 0x1.6de00ca167d4p+4 -0x1.18a2240f16369p+2 0x1.3d78e2c388afp+4\n"
         "L -0x1.18a2240f16369p+2 0x1.3d78e2c388afp+4 -0x1.cd2c6d1d077e9p+2 0x1.3000f09f88ef5p+4\n"
         "L -0x1.cd2c6d1d077e9p+2 0x1.3000f09f88ef5p+4 -0x1.46b00043d855fp+3 0x1.4a63a866aa01dp+4\n"
         "L -0x1.46b00043d855fp+3 0x1.4a63a866aa01dp+4 -0x1.a2bc11a7ae5f2p+3 0x1.2f52289389defp+4\n"
         "L -0x1.a2bc11a7ae5f2p+3 0x1.2f52289389defp+4 -0x1.a35150990152p+3 0x1.f101c773a6eb2p+3\n"
         "L -0x1.a35150990152p+3 0x1.f101c773a6eb2p+3 -0x1.e6bac3882a818p+3 0x1.af346a8804f8bp+3\n"
         "L -0x1.e6bac3882a818p+3 0x1.af346a8804f8bp+3 -0x1.2a2c5b437288fp+4 0x1.b145681182f17p+3\n"
         "L -0x1.2a2c5b437288fp+4 0x1.b145681182f17p+3 -0x1.465887d30b8f3p+4 0x1.568f1ab1d0fa3p+3\n"
         "L -0x1.465887d30b8f3p+4 0x1.568f1ab1d0fa3p+3 -0x1.2d21054826e4ep+4 0x1.ea6c1f50ea48ap+2\n"
         "L -0x1.2d21054826e4ep+4 0x1.ea6c1f50ea48ap+2 -0x1.3baf29557b1dfp+4 0x1.373c97db4554bp+2\n"
         "L -0x1.3baf29557b1dfp+4 0x1.373c97db4554bp+2 -0x1.6cb1e3b6df4ffp+4 0x1.aa06a004b1f93p+1\n"
         "L -0x1.6cb1e3b6df4ffp+4 0x1.aa06a004b1f93p+1 -0x1.708ffb4483438p+4 0x1.968c40b646f91p-49\n"
         "L -0x1.708ffb4483438p+4 0x1.968c40b646f91p-49 -0x1.439d37ded0a52p+4 -0x1.f61380601c48dp+0\n"
         "L -0x1.439d37ded0a52p+4 -0x1.f61380601c48dp+0 -0x1.3baf29557b1ep+4 -0x1.373c97db45546p+2\n"
         "L -0x1.3baf29557b1ep+4 -0x1.373c97db45546p+2 -0x1.5bab56b968536p+4 -0x1.e950a78cbba52p+2\n"
         "L -0x1.5bab56b968536p+4 -0x1.e950a78cbba52p+2 -0x1.465887d30b8f6p+4 -0x1.568f1ab1d0f9bp+3\n"
         "L -0x1.465887d30b8f6p+4 -0x1.568f1ab1d0f9bp+3 -0x1.0ff69322b8dap+4 -0x1.645a4f2ceb388p+3\n"
         "L -0x1.0ff69322b8dap+4 -0x1.645a4f2ceb388p+3 -0x1.e6bac3882a819p+3 -0x1.af346a8804f89p+3\n"
         "L -0x1.e6bac3882a819p+3 -0x1.af346a8804f89p+3 -0x1.f5fe64354b8dep+3 -0x1.0de2ffebf6338p+4\n"
         "L -0x1.f5fe64354b8dep+3 -0x1.0de2ffebf6338p+4 -0x1.a2bc11a7ae5f5p+3 -0x1.2f52289389deep+4\n"
         "L -0x1.a2bc11a7ae5f5p+3 -0x1.2f52289389deep+4 -0x1.3c047a1264c03p+3 -0x1.1c27aa99b181p+4\n"
         "L -0x1.3c047a1264c03p+3 -0x1.1c27aa99b181p+4 -0x1.cd2c6d1d077f3p+2 -0x1.3000f09f88ef4p+4\n"
         "L -0x1.cd2c6d1d077f3p+2 -0x1.3000f09f88ef4p+4 -0x1.834bd8c5a35b3p+2 -0x1.639deb9d8904dp+4\n"
         "L -0x1.834bd8c5a35b3p+2 -0x1.639deb9d8904dp+4 -0x1.6366ff3c8fa4fp+1 -0x1.6de00ca167d4p+4\n"
         "L -0x1.6366ff3c8fa4fp+1 -0x1.6de00ca167d4p+4 -0x1.f6cf5b590c819p-2 -0x1.45097b19824f5p+4\n"
         "L -0x1.f6cf5b590c819p-2 -0x1.45097b19824f5p+4 0x1.3985cac1d788dp+1 -0x1.42c2ea7be3298p+4\n"
         "L 0x1.3985cac1d788dp+1 -0x1.42c2ea7be3298p+4 0x1.3e1eaa8aa34a8p+2 -0x1.67e1268efdd35p+4\n"
         "L 0x1.3e1eaa8aa34a8p+2 -0x1.67e1268efdd35p+4 0x1.056358f56051ep+3 -0x1.589ca29f6faf7p+4\n"
         "L 0x1.056358f56051ep+3 -0x1.589ca29f6faf7p+4 0x1.203104ca37a58p+3 -0x1.2375018af6fe6p+4\n"
         "L 0x1.203104ca37a58p+3 -0x1.2375018af6fe6p+4 0x1.71645b57305cbp+3 -0x1.0b940fb0a152ep+4\n"
         "L 0x1.71645b57305cbp+3 -0x1.0b940fb0a152ep+4 0x1.db5447f44998ap+3 -0x1.19b2b3e680888p+4\n"
         "L 0x1.db5447f44998ap+3 -0x1.19b2b3e680888p+4 0x1.13df78e6fcf2dp+4 -0x1.e8cddfa332406p+3\n"
         "L 0x1.13df78e6fcf2dp+4 -0x1.e8cddfa332406p+3 0x1.07098bb4777a5p+4 -0x1.7e372e7014fffp+3\n"
         "L 0x1.07098bb4777a5p+4 -0x1.7e372e7014fffp+3 0x1.1fe3db047de2dp+4 -0x1.2e3156110edb3p+3\n"
         "L 0x1.1fe3db047de2dp+4 -0x1.2e3156110edb3p+3 0x1.555a69d1c9b05p+4 -0x1.15f74a832dce8p+3\n"
         "L 0x1.555a69d1c9b05p+4 -0x1.15f74a832dce8p+3 0x1.65da48cf0938ep+4 -0x1.60cfa0e7732d7p+2\n"
         "L 0x1.65da48cf0938ep+4 -0x1.60cfa0e7732d7p+2 0x1.41b85ca3830d5p+4 -0x1.77d2f67242d06p+1\n"
         "L 0x1.41b85ca3830d5p+4 -0x1.77d2f67242d06p+1 0x1.4521c8b10277bp+4 0x0p+0\n"
         "P 0x1.8p+2 120 0x1.e9d65f0ef5f29p+0 2 1 0x1.cc34425c031fap-3 0x1.10d23f94a1bf8p+2 0x1.90df8e8792f9p+2 0x0p+0 0x1.4p+2 1 0x1.f5b1fa0ccd4d1p-1 1 1 0x1.cac3cb958897p+0 0x1.14a44e6cc0918p-2 1 3 0x1.fe42ad649a49bp+5 0x0p+0 1 0 0x0p+0 2 0x1.5ac67fa3cfec1p+4 0 0 0x1.3f1ec3cfa6b09p-3 0x1.f0dd1b76d1bbcp+0 0x1.4d83a955091abp-3 0x1.63a3babb5a03p+1 0 0x1.6004baac90bdcp+0 0 0 0x0p+0 0x0p+0 1 5 0 0 0 0 0x1.14dd58b854b74p+4 0x1.77234e3f7a8ecp+3 0x1p+1\n"
         "K 0 1\n"
            , cs, p, vbitDeg);
        tooling::Tool tool = test::endMill(d);
        tool.type = tooling::ToolType::VBit;
        tool.includedAngle = vbitDeg * geometry::kPi / 180.0;
        const ProfileResult r = profileToolpath(cs, tool, p);
        REQUIRE(r.ok());
        for (const ProfileDiagnostic& g : r.diagnostics) {
            INFO(g.message);
            CHECK(g.message.find("comes free") == std::string::npos);
        }
    }
}

TEST_CASE("Loose pieces: material enclosed by a loop trimmed into open pieces is checked", "[toolpath][profile][tabs]") {
    // T-0135 sweep (wsf8): a blob merged with a nearby slot and trimmed near it, so its cut is a
    // chain of open pieces, Outside with allowance -0.009 (D12); the blob's part (1144.2 mm2 in
    // the flood) comes free, but only closed cuts were searched for loose pieces and no single
    // open piece's box holds it. Clusters of open cuts are searched as one region now.
    std::vector<Contour> cs;
    ProfileParams p;
    int vbitDeg = 0;
    const double d = readScene(
         "C 1\n"
         "L 0x1.a4c8213de8af8p+2 -0x1.8c44dcb817b78p+2 -0x1.77ea73a011558p-1 0x1.200b603a205c3p+3\n"
         "A -0x1.d3c56fb1eada3p+1 0x1.e62dce962c37fp+2 0x1.9ecbd16289c2p+1 0x1.cb26570f786f2p-2 -0x1.58baea6253c3ap+1 0\n"
         "L -0x1.a4c8213de8af8p+2 0x1.8c44dcb817b78p+2 0x1.77ea73a011558p-1 -0x1.200b603a205c3p+3\n"
         "A 0x1.d3c56fb1eada3p+1 -0x1.e62dce962c37fp+2 0x1.9ecbd16289c2p+1 -0x1.58baea6253c3ap+1 0x1.cb26570f786f2p-2 0\n"
         "C 1\n"
         "A 0x1.537b620bef78p+5 0x1.fe09ed9c8dfbep+2 0x1.d4545155e2629p+3 -0x1.de05c6fc3a33ep-2 0x1.04d09381a8af4p+1 0\n"
         "A 0x1.080b39d21ff76p+4 0x1.4ba8be41f0749p+4 0x1.3593137371483p+4 0x1.068cb7d60efb6p-6 -0x1.9639e8239b0d7p+0 1\n"
         "A 0x1.43e385eef9a5ep+5 0x1.814b0cb4df6abp+2 0x1.8bb684ec20c95p+4 -0x1.79fae58a24779p+1 -0x1.c26954b87f858p+0 0\n"
         "A 0x1.50c7c1f4af6aep+5 -0x1.376e2f5539a77p+2 0x1.d956bc6084f6p+3 -0x1.00e3a90468a5ep+1 0x1.be9e731239e88p-2 0\n"
         "P 0x1.8p+3 0 -0x1.28eabe9927de3p-7 0 1 0x1.71603df6554eep-2 0x1.6118ff7dea821p+1 0x1.15faf4d9a60b6p-1 0x1.db8d1a919b678p-1 0x1.4p+2 0 0x1.ab245cb50a617p-5 1 1 0x1.ad7694f6840d4p-2 0x1.d00b860952017p-2 1 9 0x1.1cd76c8879e5ep+6 0x0p+0 1 0 0x0p+0 3 0x1.e3bceab79f173p+3 0 0 0x1.4c61f2a82a7cep-2 0x1.0e5997e5ca2eap+3 0x1.a4d16453502d7p-2 0x1.cd3afe298a8bap+1 0 0x1.5ef84af2b0fbbp+0 0 0 0x0p+0 0x0p+0 1 4 0 1 0 0 -0x1.cb8b122e60e87p+4 0x1.2f3eb9dbd7b8p+1 0x1p+1\n"
         "K 0 0\n"
        , cs, p, vbitDeg);
    const ProfileResult r = profileToolpath(cs, test::endMill(d), p);
    REQUIRE(r.ok());
    std::vector<double> areas;
    for (const ProfileDiagnostic& g : r.diagnostics) {
        if (g.message.find("comes free") == std::string::npos) continue;
        INFO(g.message);
        areas.push_back(std::stod(g.message.substr(g.message.find("(about ") + 7)));
    }
    REQUIRE(areas.size() == 1);
    CHECK(std::fabs(areas.front() - 1144.2) < 5.0);
}

TEST_CASE("Loose pieces: at most 50 messages per vector, then one summing up the rest", "[toolpath][profile][tabs]") {
    // T-0135 round 1: a chain of 60 square beads (3 x 3 mm) joined by 0.5 mm necks, On the vector
    // with a 1 mm end mill: the kerf takes every neck, so 60 pieces of 4 mm2 come free (no tabs
    // asked for). 50 are listed one by one, then one message gives the other 10.
    Contour c;
    c.closed = true;
    std::vector<Vec2> pts;
    const int beads = 60;
    for (int i = 0; i < beads; ++i) {  // bottom edge, left to right
        const double x = 4.0 * i;
        pts.push_back({x, 0.0});
        pts.push_back({x + 3.0, 0.0});
        if (i + 1 < beads) {
            pts.push_back({x + 3.0, 1.25});
            pts.push_back({x + 4.0, 1.25});
        }
    }
    for (int i = beads - 1; i >= 0; --i) {  // top edge, right to left
        const double x = 4.0 * i;
        pts.push_back({x + 3.0, 3.0});
        pts.push_back({x, 3.0});
        if (i > 0) {
            pts.push_back({x, 1.75});
            pts.push_back({x - 1.0, 1.75});
        }
    }
    for (std::size_t k = 0; k < pts.size(); ++k) c.append(geometry::LineSpan{pts[k], pts[(k + 1) % pts.size()]});
    ProfileParams p;
    p.side = ProfileSide::On;
    p.passes.cutDepth = 3.0;
    p.passes.passDepth = 3.0;
    p.tabs.enabled = true;
    p.tabs.count = 0;
    p.tabs.thickness = 1.0;
    const ProfileResult r = profileToolpath({c}, test::endMill(1.0), p);
    REQUIRE(r.ok());
    std::size_t listed = 0;
    std::size_t summaries = 0;
    for (const ProfileDiagnostic& g : r.diagnostics) {
        if (g.message.find("comes free") != std::string::npos) {
            ++listed;
            CHECK(g.message.find(" of 60:") != std::string::npos);
        }
        if (g.message.find("and 10 more loose pieces") != std::string::npos) ++summaries;
    }
    CHECK(listed == 50);
    CHECK(summaries == 1);
}

TEST_CASE("T-0135 round 2: exact loose pieces; add-back and fitting never cost a holding tab",
          "[toolpath][profile][tabs]") {
    // Verifier round 2 (harness t0135v2, whole-scene flood wsf8y, leads off). The first nine: a
    // hole-linking shortcut (now removed) hid these free pieces; T-0127's raster also split or
    // missed some of them (m1_811007_4, m2_812009_22, m2_902023_44). Loose pieces are now the holes
    // of the union of the open stretches' bands, so each reports the flood's one free piece
    // (spur_12: a tab whose two sides are the same hole). The last four lost a holding tab to
    // add-back (m0_810030_3, m3_820028_27) or to fitting moved tabs last (m3_860019_2,
    // m3_860014_37); they now leave out exactly as many tabs as T-0127, all kept ones holding.
    struct Case {
        const char* name;
        const char* text;
        std::size_t left;
        std::vector<double> areas;  // the flood's free pieces
    };
    const std::vector<Case> cases{
        // (Round 3 screen: Inside, allowance 0, V-bit, separate last pass; and Inside at -r with a
        // separate last pass. The move margin put a tab T-0127 matched where it could not be
        // matched; T-0127's placement is now evaluated too and kept on a tie.)
        // (Outside, allowance -1.77, separate last pass: a tab matched only by the second try took
        // room another tab's cover needed and was cut through; T-0127's matching is a candidate.
        // T-0158: the cut starts at its convex start vertex, a tie on the round join there that
        // GCC 13 and GCC 15 (FMA) broke differently (7 and 6 left out). The tie-break rule gives 6
        // on both; the 3 kept tabs all hold (whole-scene flood).)
        {"m2_800016_13",
         "C 1\n"
         "L 0x1.1007a8608f3f4p+4 0x0p+0 0x1.f29ac1fcec938p+1 0x1.1119ba9978168p+1\n"
         "L 0x1.f29ac1fcec938p+1 0x1.1119ba9978168p+1 0x1.4b5b432d13d0fp+3 0x1.1ef6906df130ap+4\n"
         "L 0x1.4b5b432d13d0fp+3 0x1.1ef6906df130ap+4 0x1.12c7090aabdcfp-2 0x1.b61612dd087f9p+2\n"
         "L 0x1.12c7090aabdcfp-2 0x1.b61612dd087f9p+2 -0x1.1fed6ddeab54ep+3 0x1.f2b479f11374dp+3\n"
         "L -0x1.1fed6ddeab54ep+3 0x1.f2b479f11374dp+3 -0x1.f5310fd92cb2fp+1 0x1.3a58c4d684e67p+1\n"
         "L -0x1.f5310fd92cb2fp+1 0x1.3a58c4d684e67p+1 -0x1.364811d4898c7p+4 0x1.5642a3748a1cap-49\n"
         "L -0x1.364811d4898c7p+4 0x1.5642a3748a1cap-49 -0x1.e0ee9255a55e9p+1 -0x1.0b6f515c1a1e4p+1\n"
         "L -0x1.e0ee9255a55e9p+1 -0x1.0b6f515c1a1e4p+1 -0x1.499b43bbd3d39p+3 -0x1.1d729630dc66bp+4\n"
         "L -0x1.499b43bbd3d39p+3 -0x1.1d729630dc66bp+4 -0x1.2dbdfd6511d8fp-4 -0x1.4d51c6aa12456p+2\n"
         "L -0x1.2dbdfd6511d8fp-4 -0x1.4d51c6aa12456p+2 0x1.1149843771d81p+3 -0x1.d958e6aea5213p+3\n"
         "L 0x1.1149843771d81p+3 -0x1.d958e6aea5213p+3 0x1.84aee0c879f23p+2 -0x1.d554bd94a409bp+1\n"
         "L 0x1.84aee0c879f23p+2 -0x1.d554bd94a409bp+1 0x1.1007a8608f3f4p+4 0x0p+0\n"
         "P 0x1.8p+2 0 -0x1.c487ca151f37dp+0 0 0 0x0p+0 0x1.6a7ef6d1bfe66p+1 0x1.c2374880d2c0bp+1 0x0p+0 0x1.4p+2 1 0x1.b21e41f883243p-1 1 1 0x1.64499ce45e4b2p+1 0x1.465e33a88f7aap+0 0 9 0x1.eb604e132f394p+5 0x0p+0 0 1 0x0p+0 1 0x1.5df11dab4989cp+4 1 0 0x1.851a0e357e8e4p-4 0x1.1e496f0aac5edp+3 0x1.2d7e68160f324p+0 0x1.6e4bfc830cff8p+2 0 0x1.c7b304fb8e75p-1 0 0 0x0p+0 0x0p+0 0 8 0 0 0 0 -0x1.13eb499fc42e4p+5 0x1.813a4983a2c08p+2 0x1p+1\n"
         "K 0 0\n"
         , 6, {}},
        {"m3_901032_23",
         "C 1\n"
         "L -0x1.059e88f2228c2p+3 -0x1.537988c8ff595p+2 0x1.059e88f2228c2p+3 -0x1.537988c8ff595p+2\n"
         "L 0x1.059e88f2228c2p+3 -0x1.537988c8ff595p+2 0x1.1ae547522a1fcp+3 -0x1.28ec0c08f0322p+2\n"
         "L 0x1.1ae547522a1fcp+3 -0x1.28ec0c08f0322p+2 0x1.1ae547522a1fcp+3 0x1.28ec0c08f0322p+2\n"
         "L 0x1.1ae547522a1fcp+3 0x1.28ec0c08f0322p+2 0x1.059e88f2228c2p+3 0x1.537988c8ff595p+2\n"
         "L 0x1.059e88f2228c2p+3 0x1.537988c8ff595p+2 -0x1.059e88f2228c2p+3 0x1.537988c8ff595p+2\n"
         "L -0x1.059e88f2228c2p+3 0x1.537988c8ff595p+2 -0x1.1ae547522a1fcp+3 0x1.28ec0c08f0322p+2\n"
         "L -0x1.1ae547522a1fcp+3 0x1.28ec0c08f0322p+2 -0x1.1ae547522a1fcp+3 -0x1.28ec0c08f0322p+2\n"
         "L -0x1.1ae547522a1fcp+3 -0x1.28ec0c08f0322p+2 -0x1.059e88f2228c2p+3 -0x1.537988c8ff595p+2\n"
         "P 0x1.8p+2 60 0x0p+0 1 1 0x0p+0 0x1.e2c3e29e010e7p+1 0x1.46ce40eeaf792p+2 0x0p+0 0x1.4p+2 1 0x1.674198095f7b1p-1 0 1 0x1.29ac51f39b323p+0 0x1.ee42ad037b89ap-1 0 5 0x1.8678655993372p+4 0x0p+0 0 1 0x0p+0 0 0x1.a1ce7872b7d5p+4 0 0 0x1.f51f3b9a43e3dp-5 0x1.d550551a805e4p+3 0x1.0ad71e94637c2p+0 0x1.3361ee383e058p+3 0 0x1.ce0d167aa6615p-2 0 0 0x0p+0 0x0p+0 0 1 0 0 0 0 0x1.49c7ae5118c6p+0 -0x1.55381dce71b72p+5 0x1p+1\n"
         "K 0 0\n"
         , 4, {}},
        {"m3_901028_23",
         "C 1\n"
         "L -0x1.065be320df556p+3 -0x1.065be320df556p+3 0x1.6f4d712e05778p+3 -0x1.065be320df556p+3\n"
         "L 0x1.6f4d712e05778p+3 -0x1.065be320df556p+3 0x1.6f4d712e05778p+3 0x1.6f4d712e05778p+3\n"
         "L 0x1.6f4d712e05778p+3 0x1.6f4d712e05778p+3 0x1.39008198ddbd9p+2 0x1.6f4d712e05778p+3\n"
         "L 0x1.39008198ddbd9p+2 0x1.6f4d712e05778p+3 0x1.39008198ddbd9p+2 -0x1.9c7595fa45e54p+0\n"
         "L 0x1.39008198ddbd9p+2 -0x1.9c7595fa45e54p+0 -0x1.9c7595fa45e54p+0 -0x1.9c7595fa45e54p+0\n"
         "L -0x1.9c7595fa45e54p+0 -0x1.9c7595fa45e54p+0 -0x1.9c7595fa45e54p+0 0x1.6f4d712e05778p+3\n"
         "L -0x1.9c7595fa45e54p+0 0x1.6f4d712e05778p+3 -0x1.065be320df556p+3 0x1.6f4d712e05778p+3\n"
         "L -0x1.065be320df556p+3 0x1.6f4d712e05778p+3 -0x1.065be320df556p+3 -0x1.065be320df556p+3\n"
         "P 0x1.9666666666666p+1 0 -0x1.9666666666666p+0 1 0 0x0p+0 0x1.24d7fcef019c4p+3 0x1.619b2f1c87bbcp+0 0x0p+0 0x1.4p+2 1 0x1.f05649c0f584bp-1 1 1 0x1.3468979379e84p+3 0x1.329353131d9cp+1 0 8 0x1.dffea7954dc1p+4 0x0p+0 1 1 0x0p+0 1 0x1.1fe3096c1a727p+5 1 0 0x1.1442cddc7536cp-2 0x1.1311de5c13762p+3 0x1.5a06347a8d1eap-2 0x1.173d5b10d182ap+3 0 0x1.87caafeba5961p+0 0 0 0x0p+0 0x0p+0 2 4 0 1 0 0 0x1.f41471fb8102cp+4 -0x1.fe8c0ea7569cp-1 0x1p+1\n"
         "K 0 0\n"
         , 5, {}},
        {"m1_801019_13",
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.0f4aaa7320d64p+6 0x0p+0\n"
         "L 0x1.0f4aaa7320d64p+6 0x0p+0 0x1.0f4aaa7320d64p+6 0x1.6d64e3690293ap+5\n"
         "L 0x1.0f4aaa7320d64p+6 0x1.6d64e3690293ap+5 0x0p+0 0x1.6d64e3690293ap+5\n"
         "L 0x0p+0 0x1.6d64e3690293ap+5 0x0p+0 0x0p+0\n"
         "C 1\n"
         "A 0x1.ce71c2faae078p+5 0x1.b939752d2579cp+4 0x1.120433ac964e2p+3 0x0p+0 0x0p+0 0\n"
         "C 1\n"
         "A 0x1.1fd2592bb81dcp+5 0x1.01f2baa9010e4p+5 0x1.d4e43292285f9p+1 0x0p+0 0x0p+0 0\n"
         "C 1\n"
         "A 0x1.a7d97fb848e07p+5 0x1.01e09a6b306aap+5 0x1.2212ad0a247a4p+3 0x0p+0 0x0p+0 0\n"
         "P 0x1p+1 0 -0x1.d89356953a644p-1 0 1 0x1.6a4dcc292dfc1p+1 0x1.76bbfebd55c1cp+3 0x1.0f111febb5364p+1 0x0p+0 0x1.4p+2 1 0x1.cf6beaf395493p-1 1 1 0x1.8a17c8f8916fbp+1 0x1.522299a72502ap+2 0 1 0x1.0359748eda9ap+4 0x0p+0 0 0 0x0p+0 1 0x1.da63e71886b1ap+3 0 0 0x1.e367cff3db709p-4 0x1.f945a3d062877p+1 0x1.d1d68641b6c89p-4 0x1.7c44d952deef8p+1 0 0x1.525244f56c452p+0 0 0 0x0p+0 0x0p+0 2 6 0 1 0 0 0x1.7e9184d9c1d1p+4 -0x1.d469be7b4869ep+4 0x1p+1\n"
         "K 0 0\n"
         , 0, {139.9}},
        {"m2_802035_25",
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.de2f7d191f2d6p+5 0x0p+0\n"
         "L 0x1.de2f7d191f2d6p+5 0x0p+0 0x1.de2f7d191f2d6p+5 0x1.179492f2ef18ap+5\n"
         "L 0x1.de2f7d191f2d6p+5 0x1.179492f2ef18ap+5 0x0p+0 0x1.179492f2ef18ap+5\n"
         "L 0x0p+0 0x1.179492f2ef18ap+5 0x0p+0 0x0p+0\n"
         "C 1\n"
         "L 0x1.cd3614f375738p+0 0x1.6f7d69e426356p+3 0x1.7cae684ba707cp+4 0x1.6f7d69e426356p+3\n"
         "L 0x1.7cae684ba707cp+4 0x1.6f7d69e426356p+3 0x1.7cae684ba707cp+4 0x1.ae0b39d5fae31p+4\n"
         "L 0x1.7cae684ba707cp+4 0x1.ae0b39d5fae31p+4 0x1.cd3614f375738p+0 0x1.ae0b39d5fae31p+4\n"
         "L 0x1.cd3614f375738p+0 0x1.ae0b39d5fae31p+4 0x1.cd3614f375738p+0 0x1.6f7d69e426356p+3\n"
         "C 1\n"
         "A 0x1.03b83e20b60e2p+4 0x1.abaa28ebc5e76p+4 0x1.48da1cd53f226p+2 0x0p+0 0x0p+0 0\n"
         "P 0x1.9666666666666p+1 0 -0x1.6e8330eae91a7p+0 0 0 0x0p+0 0x1.aec0a69eda8bep+3 0x1.3eb0bb465cb7ap+2 0x0p+0 0x1.4p+2 1 0x1.c0bdee1bb6632p-1 0 1 0x1.0a154f78b20c7p+2 0x1.458afe1df383dp+2 0 7 0x1.efa0fd592ee2p+5 0x0p+0 0 1 0x0p+0 0 0x1.01c091bc3f4cbp+2 0 0 0x1.81292f41805cbp-2 0x1.28ae78399085ep+2 0x1.277f84c7880aap+0 0x1.7706b98b1b661p+3 0 0x1.6d66022fba911p+0 0 0 0x0p+0 0x0p+0 1 2 0 1 0 0 0x1.879c119913d7p+5 -0x1.5dea588c96eeep+4 0x1p+1\n"
         "K 0 0\n"
         , 14, {151.5}},
        {"m1_811007_4",
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.0bf1e28edf83ap+6 0x0p+0\n"
         "L 0x1.0bf1e28edf83ap+6 0x0p+0 0x1.0bf1e28edf83ap+6 0x1.5657a29fb57p+5\n"
         "L 0x1.0bf1e28edf83ap+6 0x1.5657a29fb57p+5 0x0p+0 0x1.5657a29fb57p+5\n"
         "L 0x0p+0 0x1.5657a29fb57p+5 0x0p+0 0x0p+0\n"
         "C 1\n"
         "A 0x1.516ebfe0c7c6cp+5 0x1.c1b3c6c9a137cp+4 0x1.d23a8611eb8adp+2 0x0p+0 0x0p+0 0\n"
         "C 1\n"
         "L 0x1.36fbcdd592228p+3 0x1.138eeb0020f16p+4 0x1.cd43400dc8648p+4 0x1.138eeb0020f16p+4\n"
         "L 0x1.cd43400dc8648p+4 0x1.138eeb0020f16p+4 0x1.cd43400dc8648p+4 0x1.e9990fcbd3abap+4\n"
         "L 0x1.cd43400dc8648p+4 0x1.e9990fcbd3abap+4 0x1.36fbcdd592228p+3 0x1.e9990fcbd3abap+4\n"
         "L 0x1.36fbcdd592228p+3 0x1.e9990fcbd3abap+4 0x1.36fbcdd592228p+3 0x1.138eeb0020f16p+4\n"
         "C 1\n"
         "A 0x1.0a8c7906b3712p+4 0x1.2ceb8ce4cd7ccp+4 0x1.4889e4aed00c6p+2 0x0p+0 0x0p+0 0\n"
         "P 0x1p+1 0 -0x1.fa871b4a41fd6p-1 1 1 0x0p+0 0x1.99d3c35342577p+3 0x1.06ccf69156e98p+1 0x0p+0 0x1.4p+2 1 0x1.64c090c2945e2p-1 1 1 0x1.44f88cf34dceep-1 0x1.60285b28c28d4p+1 0 3 0x1.e1c2242eba708p+5 0x0p+0 1 1 0x0p+0 0 0x1.104d9a3a8290fp+5 0 0 0x1.0602c6c661438p-3 0x1.b95acca4521fbp+2 0x1.56cffee3e14c4p+0 0x1.05d46437397dap+2 0 0x1.b9c3a4cd82b69p-1 0 0 0x0p+0 0x0p+0 1 4 0 0 0 0 0x1.cce185f2032dp+4 -0x1.4e26f4c165f5cp+3 0x1p+1\n"
         "K 0 0\n"
         , 3, {27.1}},
        {"m2_812009_22",
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.7d281a306b85ep+5 0x0p+0\n"
         "L 0x1.7d281a306b85ep+5 0x0p+0 0x1.7d281a306b85ep+5 0x1.eec4b396b671cp+5\n"
         "L 0x1.7d281a306b85ep+5 0x1.eec4b396b671cp+5 0x0p+0 0x1.eec4b396b671cp+5\n"
         "L 0x0p+0 0x1.eec4b396b671cp+5 0x0p+0 0x0p+0\n"
         "C 1\n"
         "L 0x1.d52c3176f89b4p+2 0x1.2c73f0dc95896p+5 0x1.3b8c63a9abc3ap+4 0x1.2c73f0dc95896p+5\n"
         "L 0x1.3b8c63a9abc3ap+4 0x1.2c73f0dc95896p+5 0x1.3b8c63a9abc3ap+4 0x1.71d79c3728b38p+5\n"
         "L 0x1.3b8c63a9abc3ap+4 0x1.71d79c3728b38p+5 0x1.d52c3176f89b4p+2 0x1.71d79c3728b38p+5\n"
         "L 0x1.d52c3176f89b4p+2 0x1.71d79c3728b38p+5 0x1.d52c3176f89b4p+2 0x1.2c73f0dc95896p+5\n"
         "C 1\n"
         "L 0x1.4862d8deda65bp+4 0x1.d87b43b5c37f4p+4 0x1.3c78854ecb382p+5 0x1.d87b43b5c37f4p+4\n"
         "L 0x1.3c78854ecb382p+5 0x1.d87b43b5c37f4p+4 0x1.3c78854ecb382p+5 0x1.56d5ccdda3902p+5\n"
         "L 0x1.3c78854ecb382p+5 0x1.56d5ccdda3902p+5 0x1.4862d8deda65bp+4 0x1.56d5ccdda3902p+5\n"
         "L 0x1.4862d8deda65bp+4 0x1.56d5ccdda3902p+5 0x1.4862d8deda65bp+4 0x1.d87b43b5c37f4p+4\n"
         "C 1\n"
         "A 0x1.fc3eeec94ccd2p+3 0x1.24337f9c517cdp+5 0x1.ae4325b07260bp+2 0x0p+0 0x0p+0 0\n"
         "P 0x1.9666666666666p+1 0 -0x1.81f16dfe153c7p-1 0 0 0x0p+0 0x1.53744dfc98d95p+3 0x1.0d5044e872f34p+1 0x0p+0 0x1.4p+2 1 0x1.094e64b574d5ep-1 0 1 0x1.1f2605a60c432p+2 0x1.d7b9132bc5c7fp-2 0 8 0x1.6866c20c91de8p+5 0x0p+0 1 0 0x0p+0 0 0x1.3cbf6c9359497p+4 1 0 0x1.bbdb468c68584p-3 0x1.85e47a2e1caa7p+3 0x1.828fe59791832p+0 0x1.e06baa681a215p+0 0 0x1.a3fdb7f1a25dap-4 0 0 0x0p+0 0x0p+0 1 5 0 1 0 0 -0x1.b64550c7ad085p+4 0x1.04744fa86ddc6p+5 0x1p+1\n"
         "K 0 0\n"
         , 21, {63.5}},
        {"m2_812018_9",
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.4d56dd96b016bp+5 0x0p+0\n"
         "L 0x1.4d56dd96b016bp+5 0x0p+0 0x1.4d56dd96b016bp+5 0x1.cd0150342abbep+5\n"
         "L 0x1.4d56dd96b016bp+5 0x1.cd0150342abbep+5 0x0p+0 0x1.cd0150342abbep+5\n"
         "L 0x0p+0 0x1.cd0150342abbep+5 0x0p+0 0x0p+0\n"
         "C 1\n"
         "L 0x1.4d963a7db86b1p+4 0x1.6aaecf20f51eep+5 0x1.d72e7a4b07547p+4 0x1.6aaecf20f51eep+5\n"
         "L 0x1.d72e7a4b07547p+4 0x1.6aaecf20f51eep+5 0x1.d72e7a4b07547p+4 0x1.9ad74bdc03f08p+5\n"
         "L 0x1.d72e7a4b07547p+4 0x1.9ad74bdc03f08p+5 0x1.4d963a7db86b1p+4 0x1.9ad74bdc03f08p+5\n"
         "L 0x1.4d963a7db86b1p+4 0x1.9ad74bdc03f08p+5 0x1.4d963a7db86b1p+4 0x1.6aaecf20f51eep+5\n"
         "C 1\n"
         "A 0x1.cf3d9dd6777f5p+4 0x1.88dc90ceef659p+5 0x1.b56341010fdf8p+2 0x0p+0 0x0p+0 0\n"
         "C 1\n"
         "L 0x1.54b0186f35cd8p+3 0x1.9f5ff21585436p+4 0x1.670525711e542p+4 0x1.9f5ff21585436p+4\n"
         "L 0x1.670525711e542p+4 0x1.9f5ff21585436p+4 0x1.670525711e542p+4 0x1.11b95b787d6e6p+5\n"
         "L 0x1.670525711e542p+4 0x1.11b95b787d6e6p+5 0x1.54b0186f35cd8p+3 0x1.11b95b787d6e6p+5\n"
         "L 0x1.54b0186f35cd8p+3 0x1.11b95b787d6e6p+5 0x1.54b0186f35cd8p+3 0x1.9f5ff21585436p+4\n"
         "P 0x1.9666666666666p+1 0 -0x1.4f54af28db8eap+0 1 0 0x0p+0 0x1.c0efe6247d62fp+3 0x1.82c8047e8eb28p+2 0x0p+0 0x1.4p+2 1 0x1.62c780f1be373p-2 0 1 0x1.0d8e45abed4cdp+0 0x1.afa203f67de0dp+0 1 8 0x1.24cc0a57edc96p+5 0x0p+0 1 1 0x0p+0 1 0x1.54200a89691fbp+4 0 0 0x1.bf27c089761e9p-2 0x1.ddcc7ee445182p-1 0x1.f66823d32a1b6p-1 0x1.52e5d2caa6b74p+3 0 0x1.0e8e33fbcea41p+0 0 0 0x0p+0 0x0p+0 0 6 0 1 0 0 -0x1.d1f0402c130a8p+2 0x1.1e912270b8056p+5 0x1p+1\n"
         "K 0 0\n"
         , 1, {50.2}},
        {"m3_820014_21",
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.1c87c3a6b155dp+6 0x0p+0\n"
         "L 0x1.1c87c3a6b155dp+6 0x0p+0 0x1.1c87c3a6b155dp+6 0x1.eed4db2bbb74bp+4\n"
         "L 0x1.1c87c3a6b155dp+6 0x1.eed4db2bbb74bp+4 0x0p+0 0x1.eed4db2bbb74bp+4\n"
         "L 0x0p+0 0x1.eed4db2bbb74bp+4 0x0p+0 0x0p+0\n"
         "C 1\n"
         "A 0x1.915acdb759dap+3 0x1.2b652cb1c7dccp+4 0x1.b552b838a47bcp+2 0x0p+0 0x0p+0 0\n"
         "C 1\n"
         "L 0x1.03118332be1e4p+5 0x1.349a1d47cc834p+3 0x1.8c375b1783c1cp+5 0x1.349a1d47cc834p+3\n"
         "L 0x1.8c375b1783c1cp+5 0x1.349a1d47cc834p+3 0x1.8c375b1783c1cp+5 0x1.5a4ed67dc7cp+4\n"
         "L 0x1.8c375b1783c1cp+5 0x1.5a4ed67dc7cp+4 0x1.03118332be1e4p+5 0x1.5a4ed67dc7cp+4\n"
         "L 0x1.03118332be1e4p+5 0x1.5a4ed67dc7cp+4 0x1.03118332be1e4p+5 0x1.349a1d47cc834p+3\n"
         "C 1\n"
         "L 0x1.ea7e3a9a92bdp+2 0x1.02093c2396125p+4 0x1.0156e1c20a267p+4 0x1.02093c2396125p+4\n"
         "L 0x1.0156e1c20a267p+4 0x1.02093c2396125p+4 0x1.0156e1c20a267p+4 0x1.60565cb6c37f5p+4\n"
         "L 0x1.0156e1c20a267p+4 0x1.60565cb6c37f5p+4 0x1.ea7e3a9a92bdp+2 0x1.60565cb6c37f5p+4\n"
         "L 0x1.ea7e3a9a92bdp+2 0x1.60565cb6c37f5p+4 0x1.ea7e3a9a92bdp+2 0x1.02093c2396125p+4\n"
         "P 0x1.9666666666666p+1 0 -0x1.400d78aecfb74p+0 1 0 0x0p+0 0x1.17f45c2ed6928p+3 0x1.93b4c6a559145p+2 0x0p+0 0x1.4p+2 1 0x1.e55c0d6ae2c9ap-1 1 1 0x1.6fa06e7e067d8p+1 0x1.df892f05166f2p+0 0 2 0x1.ce124f8e037e4p+5 0x0p+0 1 0 0x0p+0 0 0x1.3f8f2c27c96ffp+5 1 0 0x1.8122651620db3p-4 0x1.4b112100fe296p+3 0x1.2ec36830d363cp+0 0x1.66fe63c5a83ddp+2 0 0x1.9cb6e8440912ep-3 0 0 0x0p+0 0x0p+0 2 0 0 1 0 0 0x1.36d78ee1a381cp+3 0x1.2f765279cda5cp+5 0x1p+1\n"
         "K 0 0\n"
         , 1, {50.1}},
        {"m2_902023_44",
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.8a82b64854536p+5 0x0p+0\n"
         "L 0x1.8a82b64854536p+5 0x0p+0 0x1.8a82b64854536p+5 0x1.e2683fb1d0541p+4\n"
         "L 0x1.8a82b64854536p+5 0x1.e2683fb1d0541p+4 0x0p+0 0x1.e2683fb1d0541p+4\n"
         "L 0x0p+0 0x1.e2683fb1d0541p+4 0x0p+0 0x0p+0\n"
         "C 1\n"
         "A 0x1.1ae89703c1424p+5 0x1.922e9f32c5bf1p+3 0x1.459b9dfccbfa3p+3 0x0p+0 0x0p+0 0\n"
         "C 1\n"
         "L 0x1.11d1717a041d4p+3 0x1.3fe66113a4e59p+3 0x1.af64c61451ad4p+4 0x1.3fe66113a4e59p+3\n"
         "L 0x1.af64c61451ad4p+4 0x1.3fe66113a4e59p+3 0x1.af64c61451ad4p+4 0x1.6e16d37a23c84p+4\n"
         "L 0x1.af64c61451ad4p+4 0x1.6e16d37a23c84p+4 0x1.11d1717a041d4p+3 0x1.6e16d37a23c84p+4\n"
         "L 0x1.11d1717a041d4p+3 0x1.6e16d37a23c84p+4 0x1.11d1717a041d4p+3 0x1.3fe66113a4e59p+3\n"
         "P 0x1p+3 0 -0x1.f28efe5f8d1dp-4 2 0 0x1.4820520c0c2bfp+1 0x1.1aff5c00670a2p+2 0x1.c75ad93757a55p-1 0x0p+0 0x1.4p+2 1 0x1.f61f2431b7c15p-1 0 1 0x1.1237656902772p+2 0x1.a0fbf1ac828c4p-1 0 8 0x1.ab7a6ea6b8fd8p+3 0x0p+0 1 0 0x0p+0 1 0x1.49720af39acfep+4 0 0 0x1.9550c3beea05p-2 0x1.ef4242766275fp+2 0x1.43a000ce2ea05p+0 0x1.67f5f3795a5f6p+3 0 0x1.7c0b7832f46f9p-1 0 0 0x0p+0 0x0p+0 2 5 0 0 0 0 0x1.9d2ebe9eb448cp+3 0x1.2084bb487e77ap+5 0x1p+1\n"
         "K 0 0\n"
         , 19, {155.6}},
        {"m2_902033_10",
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.9e7f5bbf79ba4p+5 0x0p+0\n"
         "L 0x1.9e7f5bbf79ba4p+5 0x0p+0 0x1.9e7f5bbf79ba4p+5 0x1.25dffcac754bcp+6\n"
         "L 0x1.9e7f5bbf79ba4p+5 0x1.25dffcac754bcp+6 0x0p+0 0x1.25dffcac754bcp+6\n"
         "L 0x0p+0 0x1.25dffcac754bcp+6 0x0p+0 0x0p+0\n"
         "C 1\n"
         "A 0x1.27ebbf15bf0d3p+5 0x1.7b649d83e5634p+5 0x1.5311c1673de38p+3 0x0p+0 0x0p+0 0\n"
         "C 1\n"
         "L 0x1.6328a6c82f84fp+4 0x1.d41345cac49abp+5 0x1.d6cc0647167f7p+4 0x1.d41345cac49abp+5\n"
         "L 0x1.d6cc0647167f7p+4 0x1.d41345cac49abp+5 0x1.d6cc0647167f7p+4 0x1.fc8c7403fbd8dp+5\n"
         "L 0x1.d6cc0647167f7p+4 0x1.fc8c7403fbd8dp+5 0x1.6328a6c82f84fp+4 0x1.fc8c7403fbd8dp+5\n"
         "L 0x1.6328a6c82f84fp+4 0x1.fc8c7403fbd8dp+5 0x1.6328a6c82f84fp+4 0x1.d41345cac49abp+5\n"
         "C 1\n"
         "L 0x1.3a87342f62e66p+4 0x1.993fbacfdc556p+4 0x1.41a64b3cf008bp+5 0x1.993fbacfdc556p+4\n"
         "L 0x1.41a64b3cf008bp+5 0x1.993fbacfdc556p+4 0x1.41a64b3cf008bp+5 0x1.3fb1bfcecd2cfp+5\n"
         "L 0x1.41a64b3cf008bp+5 0x1.3fb1bfcecd2cfp+5 0x1.3a87342f62e66p+4 0x1.3fb1bfcecd2cfp+5\n"
         "L 0x1.3a87342f62e66p+4 0x1.3fb1bfcecd2cfp+5 0x1.3a87342f62e66p+4 0x1.993fbacfdc556p+4\n"
         "P 0x1.9666666666666p+1 0 -0x1.73200c4936966p+0 0 0 0x0p+0 0x1.e76b876963ba7p+1 0x1.a1345e01e74c1p+1 0x0p+0 0x1.4p+2 1 0x1.7dd5224a04b87p-1 1 1 0x1.feefeb99e2dbfp+0 0x1.4c7604383b7ffp+0 0 6 0x1.ce77a1b9f7edcp+4 0x0p+0 1 1 0x0p+0 1 0x1.ff9b53c8e9999p+4 1 0 0x1.0d3e05fc5407fp-4 0x1.8a760c859c8bep-1 0x1.64c52d39dd253p-1 0x1.a2ee6e068678fp+2 0 0x1.59d29d6bcd9d4p-2 0 0 0x0p+0 0x0p+0 1 3 0 0 0 1 -0x1.590c495c114a2p+4 -0x1.6cfd3c5b5e6ddp+5 0x1p+1\n"
         "K 0 0\n"
         , 12, {190.6}},
        {"spur_12",
         "C 1\n"
         "L 0x1.18p+6 0x1.4p+3 0x1.18p+6 0x1.4p+4\n"
         "L 0x1.18p+6 0x1.4p+4 0x1.ep+4 0x1.4p+4\n"
         "L 0x1.ep+4 0x1.4p+4 0x1.ep+4 0x1.5p+3\n"
         "L 0x1.ep+4 0x1.5p+3 0x1.4p+4 0x1.5p+3\n"
         "L 0x1.4p+4 0x1.5p+3 0x1.4p+4 0x1.4p+4\n"
         "L 0x1.4p+4 0x1.4p+4 0x0p+0 0x1.4p+4\n"
         "L 0x0p+0 0x1.4p+4 0x0p+0 0x0p+0\n"
         "L 0x0p+0 0x0p+0 0x1.4p+4 0x0p+0\n"
         "L 0x1.4p+4 0x0p+0 0x1.4p+4 0x1.3p+3\n"
         "L 0x1.4p+4 0x1.3p+3 0x1.ep+4 0x1.3p+3\n"
         "L 0x1.ep+4 0x1.3p+3 0x1.ep+4 0x0p+0\n"
         "L 0x1.ep+4 0x0p+0 0x1.18p+6 0x0p+0\n"
         "L 0x1.18p+6 0x0p+0 0x1.18p+6 0x1.4p+3\n"
         "C 0\n"
         "L 0x1.9p+5 0x0p+0 0x1.9p+5 0x1.8p+3\n"
         "P 0x1p+1 0 0x0p+0 2 0 0x0p+0 0x1.8p+1 0x1.8p+1 0x0p+0 0x1.4p+2 0 0x1.999999999999ap-3 0 1 0x1p+1 0x1p+0 0 1 0x1.9p+6 0x0p+0 0 0 0x0p+0 0 0x1.4p+3 0 0 0x1.657184ae74489p-4 0x1.4p+2 0x1.921fb54442d18p-1 0x1.4p+2 0 0x1.921fb54442d18p+0 0 0 0x0p+0 0x0p+0 0 0 0 0 0 0 0x0p+0 0x0p+0 0x1p+1\n"
         "K 0 0\n"
         , 0, {665.3}},
        {"m0_810030_3",
         "C 1\n"
         "A -0x1.743a97f96e6p-2 -0x1.743a97f96e6p-2 0x1.842074c8edb65p+3 0x1.eb1a0a16b193ap-6 0x1.8a734d1be80b3p+0 0\n"
         "A -0x1.de5cc8fe336e9p+2 0x1.de5cc8fe336eap+2 0x1.13b530eb5b5a1p+3 0x1.0a8d131569c0fp-1 -0x1.0bb31f677bd9p+1 1\n"
         "A -0x1.dee77fe9db1b4p+1 -0x1.dee77fe9db1b2p+1 0x1.1b26d9eb479adp+3 0x1.5a3e2f77da0e4p+1 -0x1.225ca9ab714b1p+0 0\n"
         "A 0x1.6c1579781e74p+3 -0x1.6c1579781e742p+3 0x1.6c4a198008f91p+3 -0x1.8dd2d4dd37121p+1 0x1.8985f4762b529p+0 1\n"
         "P 0x1p+3 0 -0x1.16be713c9aee8p-2 2 0 0x0p+0 0x1.6bc47be4b3298p+3 0x1.2cff7065bcebcp+2 0x0p+0 0x1.4p+2 1 0x1.8db7618b64909p-4 0 1 0x1.1ef10eb1cfe4cp+2 0x1.0d580e205fbd8p-2 0 9 0x1.9fc97e0711d11p+5 0x0p+0 0 0 0x0p+0 1 0x1.2d92c9da2b536p+4 1 0 0x1.c680c10edc961p-2 0x1.48cc2bfbda608p+3 0x1.89afe79c681a6p+0 0x1.ed583b9786f7cp-1 0 0x1.54cdfdce9ef8ep-1 0 0 0x0p+0 0x0p+0 2 4 0 0 0 0 -0x1.8ada2e6c1e202p+5 -0x1.525393073c304p+3 0x1p+1\n"
         "K 0 0\n"
         , 8, {2.13}},
        {"m3_820028_27",
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.8c17e6ec4107ep+6 0x0p+0\n"
         "L 0x1.8c17e6ec4107ep+6 0x0p+0 0x1.8c17e6ec4107ep+6 0x1.a003b5ea63dc8p+5\n"
         "L 0x1.8c17e6ec4107ep+6 0x1.a003b5ea63dc8p+5 0x0p+0 0x1.a003b5ea63dc8p+5\n"
         "L 0x0p+0 0x1.a003b5ea63dc8p+5 0x0p+0 0x0p+0\n"
         "C 1\n"
         "A 0x1.51832df12149cp+6 0x1.40b7da9c7d646p+3 0x1.3f414c9aeb85p+2 0x0p+0 0x0p+0 0\n"
         "C 1\n"
         "A 0x1.32a688ac6ff26p+6 0x1.ea7ca8885266ap+3 0x1.fb319868fb336p+1 0x0p+0 0x0p+0 0\n"
         "C 1\n"
         "L 0x1.480f064b06e7bp+4 0x1.afca9ce5c2737p+2 0x1.448a911be554ep+5 0x1.afca9ce5c2737p+2\n"
         "L 0x1.448a911be554ep+5 0x1.afca9ce5c2737p+2 0x1.448a911be554ep+5 0x1.4caa212bf9a4ap+4\n"
         "L 0x1.448a911be554ep+5 0x1.4caa212bf9a4ap+4 0x1.480f064b06e7bp+4 0x1.4caa212bf9a4ap+4\n"
         "L 0x1.480f064b06e7bp+4 0x1.4caa212bf9a4ap+4 0x1.480f064b06e7bp+4 0x1.afca9ce5c2737p+2\n"
         "P 0x1.8p+3 0 0x1.00f22deffb0e2p-4 2 0 0x0p+0 0x1.078aecf3d49eap+3 0x1.6bb31622054c4p+0 0x0p+0 0x1.4p+2 1 0x1.628277aa9a4c2p-4 1 1 0x1.afc4bed3d115bp+2 0x1.c854be589418p-1 0 9 0x1.86d21921169dp+5 0x0p+0 1 0 0x0p+0 2 0x1.2ac23d3e52f9bp+4 0 0 0x1.9be365656b15cp-2 0x1.d098f7bf5053fp+2 0x1.2491de18a33c9p+0 0x1.f6f8d43c57afp+2 0 0x1.b33a7b2be59bp-3 0 0 0x0p+0 0x0p+0 2 5 0 1 0 0 -0x1.8ff536e895514p+4 0x1.93ac4359b5854p+4 0x1p+1\n"
         "K 0 0\n"
         , 27, {}},
        {"m3_860019_2",
         "C 1\n"
         "B 0x1.4d7ec005c167p+3 -0x1.ac07ab2f018f9p-3 0x1.4fa02e5a7c70ap+3 0x1.fd490d4914fcbp+0 0x1.3375fc871debp+3 0x1.485e7932afc64p+2 0x1.039b04379c1b8p+3 0x1.a6506431ae97p+2\n"
         "B 0x1.039b04379c1b8p+3 0x1.a6506431ae97p+2 0x1.a78017d03498p+2 0x1.0221279856b3ep+3 0x1.c0e960baf619p+1 0x1.10aa9b91150d6p+3 0x1.72e77145b4503p+0 0x1.1325a250406cp+3\n"
         "B 0x1.72e77145b4503p+0 0x1.1325a250406cp+3 -0x1.3807bdd507236p-1 0x1.15a0a90f6bcaap+3 -0x1.3e5eeabd6058ep+1 0x1.febaddaa70b43p+2 -0x1.0f2aefd91b9bep+2 0x1.c414b527b706bp+2\n"
         "B -0x1.0f2aefd91b9bep+2 0x1.c414b527b706bp+2 -0x1.7f266a53870b5p+2 0x1.896e8ca4fd593p+2 -0x1.02177bb706588p+3 0x1.3a5a3898a2662p+2 -0x1.219581468bc46p+3 0x1.8ccca3204d8e3p+1\n"
         "B -0x1.219581468bc46p+3 0x1.8ccca3204d8e3p+1 -0x1.411386d611304p+3 0x1.49c9aa1eaca04p+0 -0x1.590a2813d9df8p+3 -0x1.d6b25cd8579f4p+0 -0x1.44899949ae551p+3 -0x1.e7456a165d674p+1\n"
         "B -0x1.44899949ae551p+3 -0x1.e7456a165d674p+1 -0x1.30090a7f82caap+3 -0x1.7198d2e0477f7p+2 -0x1.d0fbafea1ef5ep+2 -0x1.de6338d31aa58p+2 -0x1.4d2451130d0bap+2 -0x1.16af30b736ffep+3\n"
         "B -0x1.4d2451130d0bap+2 -0x1.16af30b736ffep+3 -0x1.9299e477f642cp+1 -0x1.3e2cc504e0adp+3 0x1.0b350d5e56ep-4 -0x1.720d79cf0a7ap+3 0x1.1bf20cee1da6ap+1 -0x1.66c2d4579168ap+3\n"
         "B 0x1.1bf20cee1da6ap+1 -0x1.66c2d4579168ap+3 0x1.17c538b8a44b2p+2 -0x1.5b782ee018574p+3 0x1.96241d88606edp+2 -0x1.0d9d18dc2ed28p+3 0x1.eda4dc7673c34p+2 -0x1.a5de9fd4c12efp+2\n"
         "B 0x1.eda4dc7673c34p+2 -0x1.a5de9fd4c12efp+2 0x1.2292cdb2438bep+3 -0x1.30830df124b8dp+2 0x1.4b5d51b1065d6p+3 -0x1.34257c0a6ab05p+1 0x1.4d7ec005c167p+3 -0x1.ac07ab2f018f9p-3\n"
         "P 0x1.8p+3 0 0x1.9542d8ec37af5p-4 2 0 0x1.f9355b39bc46cp+0 0x1.8ae2323e998e9p+3 0x1.3f32a2ae8db12p+2 0x0p+0 0x1.4p+2 1 0x1.caa1b891b564bp-2 0 1 0x1.0b73447adf8eap-1 0x1.4f351faea46a9p+1 0 9 0x1.c5e903c5b1d97p+5 0x0p+0 0 0 0x0p+0 0 0x1.3ce2e930c3931p+5 0 0 0x1.8a16b3d6dae01p-2 0x1.212c7d150dd6cp+2 0x1.caea5442d7a79p-1 0x1.228b256ad5fbdp+1 1 0x1.42275a71018bp+0 1 0 0x0p+0 0x0p+0 2 0 0 0 0 0 0x1.fe3ec66fe1168p+2 -0x1.8b10abb7906d4p+3 0x1p+1\n"
         "K 0 0\n"
         , 8, {}},
        {"m3_860014_37",
         "C 1\n"
         "A 0x0p+0 0x0p+0 0x1.38bc522d4a8a4p+4 -0x1.7182b2c7ecdfdp-1 -0x1.35bf08924799ap+1 0\n"
         "L -0x1.d587ecde031bep+3 -0x1.9d399f674a822p+3 0x1.d587ecde031bdp+3 -0x1.9d399f674a824p+3\n"
         "P 0x1.8p+3 0 -0x1.8p+2 2 0 0x1.2b4acc5369d88p-2 0x1.a14311fabb86dp+2 0x1.c2f51dceed99ep+1 0x0p+0 0x1.4p+2 1 0x1.8026fa6ffe052p-8 1 1 0x1.3356dcf8644aep+0 0x1.ac6604d249681p+0 0 5 0x1.fbf1c32bce21bp+4 0x0p+0 0 1 0x0p+0 1 0x1.dc0909f8156b1p+3 0 0 0x1.ef25828904c94p-2 0x1.a80f27193df55p+3 0x1.5ff88bae4bd54p-1 0x1.79ef7fa081749p+3 1 0x1.1c1816c128edbp-3 1 0 0x1.6384efa3966d1p+3 0x0p+0 2 2 0 1 0 0 -0x1.8e7d4b189e3e8p+2 0x1.4717a22b28012p+5 0x1p+1\n"
         "K 0 0\n"
         , 3, {}}
    };
    for (const Case& k : cases) {
        std::vector<Contour> cs;
        ProfileParams p;
        int vbitDeg = 0;
        const double d = readScene(k.text, cs, p, vbitDeg);
        p.leads.type = LeadType::None;
        p.leads.overcut = 0.0;
        p.leads.leadOut = false;
        p.ramp.onLeadIn = false;
        tooling::Tool tool = test::endMill(d);
        if (vbitDeg > 0) {
            tool.type = tooling::ToolType::VBit;
            tool.includedAngle = vbitDeg * geometry::kPi / 180.0;
        }
        INFO("scene " << k.name);
        const ProfileResult r = profileToolpath(cs, tool, p);
        REQUIRE(r.ok());
        std::vector<double> areas;
        for (const ProfileDiagnostic& g : r.diagnostics) {
            if (g.message.find("comes free") == std::string::npos) continue;
            areas.push_back(std::stod(g.message.substr(g.message.find("(about ") + 7)));
        }
        // The counts were taken with GCC 13 and MSVC, and some scenes are knife-edge: the tab fitting
        // follows the last bits of the offset (GCC 15 with FMA contraction kept one more tab in
        // m2_800016_13, T-0159; that one was the start-point tie fixed by T-0158). Where a
        // platform's count differs, what the scenes guard is checked instead: no tab lost against
        // the reference, and every part still held through the kept tabs (raster flood). Scenes
        // with loose pieces keep the exact count.
        if (tabsLeftOut(r) != k.left && k.areas.empty()) {
            CHECK(tabsLeftOut(r) < k.left);
            checkPartsHeld(r, cs, p, d);
        } else {
            CHECK(tabsLeftOut(r) == k.left);
        }
        REQUIRE(areas.size() == k.areas.size());
        for (std::size_t i = 0; i < areas.size(); ++i) CHECK(std::fabs(areas[i] - k.areas[i]) < 0.02 * k.areas[i] + 0.1);
    }
}

namespace {

// The loose-piece messages of a result: (vector, area) pairs, sorted.
std::vector<std::pair<std::size_t, double>> loosePieces(const ProfileResult& r) {
    std::vector<std::pair<std::size_t, double>> out;
    for (const ProfileDiagnostic& g : r.diagnostics) {
        if (g.message.find("comes free") == std::string::npos) continue;
        out.push_back({g.contour.value_or(9999), std::stod(g.message.substr(g.message.find("(about ") + 7))});
    }
    std::sort(out.begin(), out.end());
    return out;
}

// Whole-scene raster flood at the tab-top layer, independent of the planner (T-0140): every
// cutting sample of `tp` below `zTab` (every 0.05 mm) cuts the cells (of `res` mm) whose centres
// lie within `r` of it; the areas of the uncut 4-connected components that do not reach the
// frame (the cut's box grown by r + 1 mm) are the pieces that come free.
std::vector<double> floodPieces(const Toolpath& tp, double zTab, double r, double res,
                                const std::function<double(double)>& radiusAt) {
    Box2 box;
    test::sampleCuttingMoves3(
        tp, 0.05, [&](Vec2 q, double z) { if (z < zTab - 1e-6) box.expand(q); }, zTab - 1e-6);
    if (box.isEmpty()) return {};
    box = box.inflated(r + 1.0);
    const int nx = static_cast<int>(std::ceil(box.width() / res));
    const int ny = static_cast<int>(std::ceil(box.height() / res));
    auto at = [&](int i, int j) { return static_cast<std::size_t>(j) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(i); };
    std::vector<char> cut(static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny), 0);
    test::sampleCuttingMoves3(
        tp, 0.05,
        [&](Vec2 c, double z) {
            if (z >= zTab - 1e-6) return;
            const double rz = radiusAt(z);  // the disc the tool cuts at the layer there
            if (!(rz > 0.0)) return;
            const int j0 = std::max(0, static_cast<int>(std::floor((c.y - rz - box.min.y) / res)));
            const int j1 = std::min(ny - 1, static_cast<int>(std::ceil((c.y + rz - box.min.y) / res)));
            for (int j = j0; j <= j1; ++j) {
                const double dy = box.min.y + (j + 0.5) * res - c.y;
                const double w2 = rz * rz - dy * dy;
                if (w2 <= 0.0) continue;
                const double w = std::sqrt(w2);
                const int i0 = std::max(0, static_cast<int>(std::ceil((c.x - w - box.min.x) / res - 0.5)));
                const int i1 = std::min(nx - 1, static_cast<int>(std::floor((c.x + w - box.min.x) / res - 0.5)));
                for (int i = i0; i <= i1; ++i) cut[at(i, j)] = 1;
            }
        },
        zTab - 1e-6);
    std::vector<double> out;
    std::vector<char> seen(cut.size(), 0);
    for (std::size_t s = 0; s < cut.size(); ++s) {
        if (cut[s] || seen[s]) continue;
        std::vector<std::size_t> st{s};
        seen[s] = 1;
        std::size_t cells = 0;
        bool frame = false;
        while (!st.empty()) {
            const std::size_t k = st.back();
            st.pop_back();
            ++cells;
            const int i = static_cast<int>(k % static_cast<std::size_t>(nx));
            const int j = static_cast<int>(k / static_cast<std::size_t>(nx));
            if (i == 0 || j == 0 || i == nx - 1 || j == ny - 1) frame = true;
            const int di[4] = {1, -1, 0, 0};
            const int dj[4] = {0, 0, 1, -1};
            for (int e = 0; e < 4; ++e) {
                const int ii = i + di[e];
                const int jj = j + dj[e];
                if (ii < 0 || jj < 0 || ii >= nx || jj >= ny) continue;
                const std::size_t kk = at(ii, jj);
                if (seen[kk] || cut[kk]) continue;
                seen[kk] = 1;
                st.push_back(kk);
            }
        }
        if (!frame) out.push_back(static_cast<double>(cells) * res * res);
    }
    std::sort(out.begin(), out.end());
    return out;
}

std::vector<double> rasterFreePieces(const Toolpath& tp, double zTab, double r, double res) {
    return floodPieces(tp, zTab, r, res, [r](double) { return r; });
}

// The same for a V-bit: the disc at the layer is the cone's, (zTab - tip height) * tan(half angle)
// at most `rMax`.
std::vector<double> coneFreePieces(const Toolpath& tp, double zTab, double tanHalf, double rMax, double res) {
    return floodPieces(tp, zTab, rMax, res,
                       [=](double z) { return std::min(rMax, (zTab - z) * tanHalf); });
}

// A circle of `radius` about the origin drawn as cubic Beziers.
Contour bezierCircle(double radius) {
    Contour c;
    c.closed = true;
    for (const geometry::BezierSpan& b : ArcSpan::fullCircle({0, 0}, radius).toBeziers()) c.append(b);
    return c;
}

ProfileParams looseParams(ProfileSide side, double allowance) {
    ProfileParams p;
    p.side = side;
    p.allowance = allowance;
    p.passes.cutDepth = 3.0;
    p.passes.passDepth = 1.5;
    p.tabs.enabled = true;
    p.tabs.count = 2;
    p.tabs.length = 30.0;  // fits no vector below: every tab is left out
    p.tabs.thickness = 1.0;
    p.leads.type = LeadType::None;
    return p;
}

}  // namespace

TEST_CASE("Loose pieces (T-0140): Inside/Outside with allowance >= 0 report what no kept tab holds",
          "[toolpath][profile][tabs]") {
    // Before T-0140 only sided cuts (On, or a negative allowance) were checked: these pieces came
    // free unreported. A 2 mm end mill: the cut at the tab layer is 1 mm wide each side of the
    // path, plus the 2 um the bands carry (reach 1.002 mm).
    // With a separate last pass the roughing cut runs 0.3 mm further into the waste: inside a
    // hole that takes 0.3 mm more off the slug; outside a part it does not touch the part.
    struct Case {
        const char* what;
        Contour vector;
        ProfileSide side;
        double allowance;
        std::function<double(double)> area;  // of the extra allowance; 0: nothing reported
    };
    const double reach = 1.002;
    const double lpa = 0.3;
    auto slug = [&](double radius, double allowance) {
        return [=](double extra) {
            const double s = radius - 1.0 - allowance - extra - reach;
            const double a = s > 0.0 ? geometry::kPi * s * s : 0.0;
            return a >= 0.04 ? a : 0.0;
        };
    };
    const std::vector<Case> cases{
        {"slug of a hole, allowance 0", Contour::circle({0, 0}, 4.0), ProfileSide::Inside, 0.0, slug(4.0, 0.0)},
        {"slug of a hole, allowance 0.5", Contour::circle({0, 0}, 4.0), ProfileSide::Inside, 0.5, slug(4.0, 0.5)},
        {"slug just over 0.04 mm2", Contour::circle({0, 0}, 2.15), ProfileSide::Inside, 0.0, slug(2.15, 0.0)},
        {"slug under 0.04 mm2", Contour::circle({0, 0}, 2.1), ProfileSide::Inside, 0.0, slug(2.1, 0.0)},
        {"rectangular slug (sharp inner corners)", Contour::rectangle(Box2{{0, 0}, {10, 8}}), ProfileSide::Inside, 0.0,
         [=](double extra) { return (8.0 - 2.0 * (reach + extra)) * (6.0 - 2.0 * (reach + extra)); }},
        {"small part, Outside (path arcs tighter than the band)", Contour::rectangle(Box2{{0, 0}, {6, 6}}),
         ProfileSide::Outside, 0.0, [=](double) { return (6.0 - 2.0 * (reach - 1.0)) * (6.0 - 2.0 * (reach - 1.0)); }},
        {"small round part, Outside, allowance -0.0", Contour::circle({0, 0}, 1.5), ProfileSide::Outside, -0.0,
         [=](double) { return geometry::kPi * (1.5 + 1.0 - reach) * (1.5 + 1.0 - reach); }},
        {"slug of a hole drawn with Beziers (densely polygonised cut)", bezierCircle(4.0), ProfileSide::Inside, 0.0,
         slug(4.0, 0.0)},
    };
    for (const Case& k : cases) {
        for (const bool sep : {false, true}) {
            INFO(k.what << (sep ? ", separate last pass" : ""));
            ProfileParams p = looseParams(k.side, k.allowance);
            p.separateLastPass = sep;
            p.lastPassAllowance = lpa;
            const ProfileResult r = profileToolpath({k.vector}, test::endMill(2.0), p);
            REQUIRE(r.ok());
            const auto got = loosePieces(r);
            const double area = k.area(sep ? lpa : 0.0);
            if (area == 0.0) {
                CHECK(got.empty());
                continue;
            }
            REQUIRE(got.size() == 1);
            CHECK(got.front().first == 0);
            CHECK(std::fabs(got.front().second - area) <= 0.01 * area + 0.01);
        }
    }
}

TEST_CASE("Loose pieces (T-0140): held parts and slugs are not reported", "[toolpath][profile][tabs]") {
    // A plate with a round and a square hole, Inside or Outside, allowance >= 0, end mill or
    // V-bit, with or without a separate last pass: a part or slug comes free (and is reported)
    // exactly when its vector keeps no tab (the V-bit's tabs in the square hole are cut through,
    // inside it), and nothing is reported where a tab is kept.
    const std::vector<Contour> cs{Contour::rectangle(Box2{{0, 0}, {60, 40}}), Contour::circle({18, 20}, 8.0),
                                  Contour::rectangle(Box2{{36, 10}, {50, 30}})};
    for (const ProfileSide side : {ProfileSide::Outside, ProfileSide::Inside}) {
        for (const double allowance : {0.0, 0.4}) {
            for (const bool sep : {false, true}) {
                for (const bool vbit : {false, true}) {
                    INFO("side " << static_cast<int>(side) << " allowance " << allowance << " sep " << sep << " vbit "
                                 << vbit);
                    ProfileParams p = looseParams(side, allowance);
                    p.tabs.length = 3.0;
                    p.tabs.count = 3;
                    p.separateLastPass = sep;
                    tooling::Tool tool = test::endMill(3.175);
                    if (vbit) {
                        tool.type = tooling::ToolType::VBit;
                        tool.diameter = 6.0;
                        tool.includedAngle = geometry::kPi / 2.0;
                    }
                    const ProfileResult r = profileToolpath(cs, tool, p);
                    REQUIRE(r.ok());
                    std::vector<std::size_t> lost;  // vectors that keep no tab
                    for (const ProfileDiagnostic& g : r.diagnostics) {
                        if (g.issue != ProfileIssue::TabsNotPlaced || !g.contour) continue;
                        std::istringstream in(g.message.substr(g.message.find(": ") + 2));  // "N of M tab(s)"
                        int left = -1;
                        int of = -2;
                        std::string word;
                        in >> left >> word >> of;
                        if (word == "of" && left == of) lost.push_back(*g.contour);
                    }
                    std::vector<std::size_t> loose;
                    for (const auto& g : loosePieces(r)) loose.push_back(g.first);
                    std::sort(lost.begin(), lost.end());
                    CHECK(loose == lost);
                    if (!vbit) CHECK(loose.empty());
                }
            }
        }
    }
}

TEST_CASE("Loose pieces (T-0140): waste the kerfs enclose between vectors is reported", "[toolpath][profile][tabs]") {
    // A square frame cut as four bars, Outside, 4 mm apart (more than the tool, less than two
    // kerfs): the tool runs through the gaps and the kerfs close round the waste in the middle.
    // With 2 mm tabs some land on the inner edges and hold it: nothing comes free. With tabs that
    // fit no bar, the bars and (where the kerfs close in the gaps) the waste in the middle (inside
    // no vector, which before T-0140 nothing looked at, even for sided cuts) come free. Checked
    // against a raster flood.
    const std::vector<Contour> cs{Contour::rectangle(Box2{{0, 0}, {30, 5}}), Contour::rectangle(Box2{{0, 25}, {30, 30}}),
                                  Contour::rectangle(Box2{{0, 9}, {5, 21}}), Contour::rectangle(Box2{{25, 9}, {30, 21}})};
    int withMiddle = 0;  // the bars and the middle come free
    for (const double tabLength : {2.0, 40.0}) {
        for (const double allowance : {0.0, 0.3}) {
            for (const bool sep : {false, true}) {
                INFO("tab length " << tabLength << " allowance " << allowance << " sep " << sep);
                ProfileParams p = looseParams(ProfileSide::Outside, allowance);
                p.tabs.length = tabLength;
                p.tabs.count = 2;
                p.separateLastPass = sep;
                const ProfileResult r = profileToolpath(cs, test::endMill(3.175), p);
                REQUIRE(r.ok());
                std::vector<double> got;
                for (const auto& g : loosePieces(r)) got.push_back(g.second);
                std::sort(got.begin(), got.end());
                const std::vector<double> flood =
                    rasterFreePieces(r.toolpath, p.passes.bottomZ() + p.tabs.thickness, 1.5875, 0.04);
                std::vector<double> big;
                for (double a : flood) {
                    if (a >= 0.1) big.push_back(a);
                }
                CHECK((tabLength > 30.0 ? big.size() >= 4 : big.empty()));
                if (big.size() == 5) ++withMiddle;
                REQUIRE(got.size() == big.size());
                for (std::size_t k = 0; k < big.size(); ++k) {
                    CHECK(std::fabs(got[k] - big[k]) <= 0.02 * big[k] + 0.05);
                }
            }
        }
    }
    CHECK(withMiddle >= 2);
}

TEST_CASE("Generative (T-0140): Inside/Outside with allowance >= 0, every free piece reported, nothing else",
          "[toolpath][profile][tabs][property]") {
    // Random small scenes (one or two vectors, some tabs too long to fit), checked against a
    // whole-scene raster flood of the moves at the tab layer (0.04 mm cells): every free piece of
    // 0.3 mm2 or more is reported with its area, and every reported piece is a free piece (pieces
    // under 0.3 mm2 are left to the cell resolution). Leads off.
    test::SplitMix rng{0x7140};
    int withFree = 0;
    for (int s = 0; s < 24; ++s) {
        std::vector<Contour> cs;
        const int kind = static_cast<int>(rng.uniform(0.0, 4.0));
        const double w = rng.uniform(4.0, 20.0);
        const double h = rng.uniform(4.0, 16.0);
        if (kind == 0) {
            cs.push_back(Contour::rectangle(Box2{{0, 0}, {w, h}}));
        } else if (kind == 1) {
            cs.push_back(Contour::circle({0, 0}, 0.5 * h));
        } else if (kind == 2) {  // a plate with a hole
            cs.push_back(Contour::rectangle(Box2{{0, 0}, {w + 8.0, h + 8.0}}));
            cs.push_back(Contour::circle({0.5 * w + 4.0, 0.5 * h + 4.0}, rng.uniform(1.5, 0.5 * std::min(w, h))));
        } else {  // two parts close together
            cs.push_back(Contour::rectangle(Box2{{0, 0}, {w, h}}));
            const double gap = rng.uniform(0.5, 6.0);
            cs.push_back(Contour::circle({w + gap + 0.4 * h, 0.5 * h}, 0.4 * h));
        }
        ProfileParams p = looseParams(rng.uniform(0.0, 1.0) < 0.5 ? ProfileSide::Outside : ProfileSide::Inside,
                                      rng.uniform(0.0, 1.0) < 0.4 ? 0.0 : rng.uniform(0.0, 1.0));
        p.tabs.count = 1 + static_cast<int>(rng.uniform(0.0, 3.0));
        p.tabs.length = rng.uniform(0.0, 1.0) < 0.3 ? 40.0 : rng.uniform(0.5, 4.0);
        p.tabs.thickness = rng.uniform(0.5, 2.0);
        p.separateLastPass = rng.uniform(0.0, 1.0) < 0.5;
        p.lastPassAllowance = rng.uniform(0.05, 0.6);
        const double d = rng.uniform(0.0, 1.0) < 0.5 ? 2.0 : 3.175;
        INFO("scene " << s << " side " << static_cast<int>(p.side) << " allowance " << p.allowance << " sep "
                      << p.separateLastPass << " tab length " << p.tabs.length << " d " << d);
        const ProfileResult r = profileToolpath(cs, test::endMill(d), p);
        REQUIRE(r.ok());
        std::vector<double> got;
        for (const auto& g : loosePieces(r)) got.push_back(g.second);
        std::sort(got.begin(), got.end());
        const std::vector<double> flood =
            rasterFreePieces(r.toolpath, p.passes.bottomZ() + p.tabs.thickness, 0.5 * d, 0.04);
        if (!flood.empty() && flood.back() >= 0.3) ++withFree;
        std::vector<bool> used(got.size(), false);
        for (double a : flood) {
            if (a < 0.3) continue;
            INFO("free piece " << a);
            bool found = false;
            for (std::size_t i = 0; i < got.size() && !found; ++i) {
                if (!used[i] && std::fabs(got[i] - a) <= 0.03 * a + 0.1) used[i] = found = true;
            }
            CHECK(found);
        }
        for (std::size_t i = 0; i < got.size(); ++i) {
            if (used[i]) continue;
            INFO("reported piece " << got[i]);
            const bool small = got[i] < 0.3 + 0.1;
            const bool inFlood = std::any_of(flood.begin(), flood.end(), [&](double a) {
                return std::fabs(got[i] - a) <= 0.03 * a + 0.1;
            });
            CHECK((small || inFlood));
        }
    }
    CHECK(withFree >= 4);  // the scenes do exercise the report
}

TEST_CASE("Loose pieces (T-0140): a piece counts when it is thicker than 0.05 mm somewhere",
          "[toolpath][profile][tabs]") {
    // A part (radius 20 with a hole of radius 12) and a round part of radius 8 - t in its hole,
    // Outside, 2 mm end mill, every tab left out: the waste between the hole's kerf and the round
    // part's kerf is a ring about t - 4 um wide (each band reaches 2 um past the tool), of a few
    // square millimetres. Below about 0.055 mm it is not reported (it holds no 0.05 mm disc);
    // above, it is. The two parts come free too.
    const double reach = 1.002;
    for (const double t : {0.03, 0.045, 0.08, 0.2}) {
        INFO("t " << t);
        const std::vector<Contour> cs{Contour::circle({0, 0}, 20.0), Contour::circle({0, 0}, 12.0),
                                      Contour::circle({0, 0}, 12.0 - 4.0 - t)};
        ProfileParams p = looseParams(ProfileSide::Outside, 0.0);
        p.tabs.length = 200.0;
        const ProfileResult r = profileToolpath(cs, test::endMill(2.0), p);
        REQUIRE(r.ok());
        std::vector<double> got;
        for (const auto& g : loosePieces(r)) got.push_back(g.second);
        std::sort(got.begin(), got.end());
        std::vector<double> want;
        const double inner = 12.0 - 1.0 - reach;      // the hole's kerf reaches in to here
        const double outer = 8.0 - t + 1.0 + reach;  // the round part's kerf reaches out to here
        if (t > 0.06) want.push_back(geometry::kPi * (inner * inner - outer * outer));
        const double disc = 12.0 - 4.0 - t - (reach - 1.0);
        want.push_back(geometry::kPi * disc * disc);
        const double a = 20.0 - (reach - 1.0);
        const double b = 12.0 + (reach - 1.0);
        want.push_back(geometry::kPi * (a * a - b * b));
        REQUIRE(got.size() == want.size());
        // The band polygons run outside the arcs, up to 0.005 mm (on average about two thirds of
        // that) beyond them: each piece is up to that much smaller along its curved edges.
        const double perimeter[3] = {2.0 * geometry::kPi * (inner + outer), 2.0 * geometry::kPi * disc,
                                     2.0 * geometry::kPi * (a + b)};
        for (std::size_t k = 0; k < want.size(); ++k) {
            const double edges = perimeter[want.size() == 3 ? k : k + 1];
            INFO("piece " << k << ": " << got[k] << " mm2, exact " << want[k]);
            CHECK(got[k] <= want[k] + 0.01);
            CHECK(got[k] >= want[k] - 0.005 * edges - 0.01);
        }
    }
}

TEST_CASE("Loose pieces (T-0140): each piece is reported against its own vector, with its exact area",
          "[toolpath][profile][tabs]") {
    // A 24 x 20 plate with a 12 x 8 hole, Outside (the tool runs outside the outline and inside
    // the hole), 2 mm end mill, every tab left out: the plate (vector 0) and the hole's slug
    // (vector 1) come free, each reported against its own vector. The plate's area: the outline
    // less 2 um (each band reaches 2 um past the tool) less what the hole's cut takes, the hole
    // grown by 2 um with its corners rounded at the band's 1.002 mm. Those corners are the wedges
    // of the band at the sharp corners of the hole's path: without them each corner would keep a
    // quarter disc of material, 3.15 mm2 in all.
    const double w = 1.002;
    const double e = w - 1.0;
    const std::vector<Contour> cs{Contour::rectangle(Box2{{0, 0}, {24, 20}}), Contour::rectangle(Box2{{6, 6}, {18, 14}})};
    for (const bool sep : {false, true}) {
        INFO("separate last pass " << sep);
        ProfileParams p = looseParams(ProfileSide::Outside, 0.0);
        p.tabs.length = 200.0;
        p.separateLastPass = sep;
        p.lastPassAllowance = 0.3;
        const ProfileResult r = profileToolpath(cs, test::endMill(2.0), p);
        REQUIRE(r.ok());
        const auto got = loosePieces(r);
        REQUIRE(got.size() == 2);
        CHECK(got[0].first == 0);
        CHECK(got[1].first == 1);
        const double plate =
            (24.0 - 2.0 * e) * (20.0 - 2.0 * e) - ((12.0 + 2.0 * e) * (8.0 + 2.0 * e) - (4.0 - geometry::kPi) * w * w);
        CHECK(std::fabs(got[0].second - plate) <= 0.1);  // chords of the corners' arcs
        // The slug: the hole less the path's 1 mm and the band's 1.002 mm (sharp corners), and
        // less the roughing cut's 0.3 mm with a separate last pass.
        const double in = 1.0 + w + (sep ? 0.3 : 0.0);
        CHECK(std::fabs(got[1].second - (12.0 - 2.0 * in) * (8.0 - 2.0 * in)) <= 0.02);
    }
}

TEST_CASE("Loose pieces (T-0140): a piece is reported against the innermost closed cut round it",
          "[toolpath][profile][tabs]") {
    // A 40 x 30 plate with a 1 x 26 slot near its left side, Inside (the tool runs inside the
    // outline and outside the slot), 2 mm end mill, every tab left out: the slot's kerf reaches
    // the outline's from top to bottom and cuts the plate in two. Both parts lie inside the
    // outline's cut, so both are reported against vector 0, though the larger lies nearer the
    // slot's cut (its left edge is the slot's kerf); the slot's slug comes free against vector 1.
    // Areas: what the bands leave (2 um past the tool), the parts' within 1 % (the slot's kerf
    // ends round where it meets the outline's).
    const std::vector<Contour> cs{Contour::rectangle(Box2{{0, 0}, {40, 30}}), Contour::rectangle(Box2{{8, 2}, {9, 28}})};
    ProfileParams p = looseParams(ProfileSide::Inside, 0.0);
    p.tabs.length = 200.0;
    const ProfileResult r = profileToolpath(cs, test::endMill(2.0), p);
    REQUIRE(r.ok());
    auto got = loosePieces(r);
    std::sort(got.begin(), got.end(), [](const auto& a, const auto& b) { return a.second < b.second; });
    REQUIRE(got.size() == 3);
    const double e = 0.002;
    CHECK(got[0].first == 1);
    CHECK(std::fabs(got[0].second - (1.0 - 2.0 * e) * (26.0 - 2.0 * e)) <= 0.01);
    CHECK(got[1].first == 0);
    CHECK(std::fabs(got[1].second - (4.0 - 2.0 * e) * (26.0 - 2.0 * e)) <= 0.01 * got[1].second);
    CHECK(got[2].first == 0);
    CHECK(std::fabs(got[2].second - (27.0 - 2.0 * e) * (26.0 - 2.0 * e)) <= 0.01 * got[2].second);
}

TEST_CASE("Loose pieces (T-0140): sweep scenes, Inside/Outside with allowance >= 0", "[toolpath][profile][tabs]") {
    // T-0140 sweep (whole-scene flood of the moves, as T-0127's wsf, leads off): each reports
    // exactly the flood's free pieces (before T-0140, none of them).
    struct Case {
        const char* name;
        const char* text;
        std::vector<double> areas;  // the flood's free pieces
    };
    const std::vector<Case> cases{
        // 1480018_45: End mill 6 mm, Inside, 3 of 6 tabs kept.
        {"1480018_45",
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.11dee7492925ep+6 0x0p+0\n"
         "L 0x1.11dee7492925ep+6 0x0p+0 0x1.11dee7492925ep+6 0x1.9a55ef147285ap+5\n"
         "L 0x1.11dee7492925ep+6 0x1.9a55ef147285ap+5 0x0p+0 0x1.9a55ef147285ap+5\n"
         "L 0x0p+0 0x1.9a55ef147285ap+5 0x0p+0 0x0p+0\n"
         "C 1\n"
         "A 0x1.36d1f33058beap+3 0x1.0588c5c3c4e92p+4 0x1.86fb407958683p+1 0x0p+0 0x0p+0 0\n"
         "C 1\n"
         "L 0x1.a0291cd4cf192p+5 0x1.70e930c21b2bap+3 0x1.0504965061d6dp+6 0x1.70e930c21b2bap+3\n"
         "L 0x1.0504965061d6dp+6 0x1.70e930c21b2bap+3 0x1.0504965061d6dp+6 0x1.4cae481830cc3p+4\n"
         "L 0x1.0504965061d6dp+6 0x1.4cae481830cc3p+4 0x1.a0291cd4cf192p+5 0x1.4cae481830cc3p+4\n"
         "L 0x1.a0291cd4cf192p+5 0x1.4cae481830cc3p+4 0x1.a0291cd4cf192p+5 0x1.70e930c21b2bap+3\n"
         "C 1\n"
         "A 0x1.066fce86129ecp+4 0x1.f62647890e1fap+3 0x1.57c098cf58252p+2 0x0p+0 0x0p+0 0\n"
         "P 0x1.8p+2 0 -0x0p+0 1 1 0x1.80488ff57c4f3p-1 0x1.a2c598ad639dcp+3 0x1.ff14789ad15cap+1 0x0p+0 0x1.4p+2 0 0x1.b35090cb55462p-2 1 1 0x1.4babe86f79a44p+3 0x1.0aba2c5a7ce1fp+2 1 7 0x1.6ca0106feaeb5p+5 0x0p+0 1 0 0x0p+0 3 0x1.a294af2d8bc4bp+4 0 0 0x1.6036f0fd682b2p-3 0x1.3c237149c7994p+3 0x1.3833a72a02555p-2 0x1.4a6f8d576f31ap+3 0 0x1.1d518ffecc526p+0 0 0 0x0p+0 0x0p+0 1 6 0 0 0 0 0x1.824b6d493446cp+5 -0x1.79886279e068p-1 0x1p+1\n"
         "K 0 0\n"
         , {5.381, 115.729}},
        // 1480030_22: V-bit 90 deg, Inside, separate last pass, 1 of 18 tabs kept.
        {"1480030_22",
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.5271f282072cap+5 0x0p+0\n"
         "L 0x1.5271f282072cap+5 0x0p+0 0x1.5271f282072cap+5 0x1.f9ddffa56c388p+5\n"
         "L 0x1.5271f282072cap+5 0x1.f9ddffa56c388p+5 0x0p+0 0x1.f9ddffa56c388p+5\n"
         "L 0x0p+0 0x1.f9ddffa56c388p+5 0x0p+0 0x0p+0\n"
         "C 1\n"
         "A 0x1.69df856d7e314p+4 0x1.1c12aab9cbac5p+4 0x1.18e9658f99ed2p+3 0x0p+0 0x0p+0 0\n"
         "C 1\n"
         "A 0x1.8e0914a444915p+4 0x1.88983c56d3ca2p+4 0x1.629acd20900b5p+2 0x0p+0 0x0p+0 0\n"
         "C 1\n"
         "A 0x1.8438b336e3508p+4 0x1.13320b0e475e4p+5 0x1.1fbc8c47ef97ep+3 0x0p+0 0x0p+0 0\n"
         "P 0x1.8p+3 90 0x1.ba8afe274656bp-7 1 0 0x0p+0 0x1.fbf4cac1dd3aep+1 0x1.4153f2f586151p+2 0x0p+0 0x1.4p+2 1 0x1.b95de59d82075p-1 1 1 0x1.46bc3c95c92c2p+2 0x1.cf785e546d5afp+0 0 9 0x1.f68b9a0f53044p+4 0x0p+0 1 0 0x0p+0 1 0x1.ccf5e91833176p+4 0 0 0x1.591cd880c2e17p-4 0x1.aeec7bde8f6fdp+2 0x1.1ec980e4c8f2fp-1 0x1.53bdd16ccc752p+3 0 0x1.5b2c2dab53651p-2 0 0 0x0p+0 0x0p+0 0 4 0 0 0 1 0x1.2624989f77a6cp+5 -0x1.91acc2c9cc09ap+4 0x1p+1\n"
         "K 1 0\n"
         , {2.979, 719.753}},
        // 1480009_31: End mill 12 mm, Inside, separate last pass, every tab left out.
        {"1480009_31",
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.26ff204c96d2fp+6 0x0p+0\n"
         "L 0x1.26ff204c96d2fp+6 0x0p+0 0x1.26ff204c96d2fp+6 0x1.a0ed9fb6bf71p+5\n"
         "L 0x1.26ff204c96d2fp+6 0x1.a0ed9fb6bf71p+5 0x0p+0 0x1.a0ed9fb6bf71p+5\n"
         "L 0x0p+0 0x1.a0ed9fb6bf71p+5 0x0p+0 0x0p+0\n"
         "C 1\n"
         "A 0x1.ceb6c36499b28p+4 0x1.cd0564f90d69ap+4 0x1.507bfaa436fcep+2 0x0p+0 0x0p+0 0\n"
         "C 1\n"
         "A 0x1.bb51d271b874ap+5 0x1.08954ddad4e5p+4 0x1.bb1ff62b5a692p+2 0x0p+0 0x0p+0 0\n"
         "P 0x1.8p+3 0 -0x0p+0 1 0 0x1.dcc65ceb812f8p+0 0x1.57464770436f1p+2 0x1.810978af7ca54p+2 0x0p+0 0x1.4p+2 1 0x1.a3ad7fd907972p-4 1 1 0x1.984be1a7906cap-2 0x1.02a7acea4c8ebp+1 0 3 0x1.4d06f4fc2e0bp+3 0x0p+0 1 0 0x0p+0 2 0x1.19968e04519d9p+5 0 0 0x1.d46b3552b7149p-4 0x1.aa0845a448826p+3 0x1.d28b3aa50a5eap-3 0x1.afc09a0dd89a7p+2 0 0x1.9278979cc36cap-1 0 0 0x0p+0 0x0p+0 2 4 0 0 0 0 0x1.44eabb3f2cae4p+4 -0x1.1ac89f34e137p+1 0x1p+1\n"
         "K 0 0\n"
         , {47.734, 1.73, 86.838, 90.987, 9.901}},
        // Round 3 (T-0140 round-2 verifier). 1600019_46: End mill 12 mm, Inside, separate last
        // pass, every tab left out: one 187.8 mm2 slug about the origin, which leaf by leaf was
        // reported as two of 93.9 mm2 (Clipper2 truncated the crossings at the leaf side x = 0).
        {"1600019_46",
         "C 1\n"
         "L 0x1.3c46a9e6fb9p+4 0x0p+0 0x1.88c0e58753ca1p+4 0x1.f1a5fa36d211bp+1\n"
         "L 0x1.88c0e58753ca1p+4 0x1.f1a5fa36d211bp+1 0x1.6f614652cf2bbp+4 0x1.3058f860ab96bp+3\n"
         "L 0x1.6f614652cf2bbp+4 0x1.3058f860ab96bp+3 0x1.0dab6deefdd1ep+4 0x1.4a81ee5e4756p+3\n"
         "L 0x1.0dab6deefdd1ep+4 0x1.4a81ee5e4756p+3 0x1.bf482764830cbp+3 0x1.bf482764830cap+3\n"
         "L 0x1.bf482764830cbp+3 0x1.bf482764830cap+3 0x1.d376f684a7968p+3 0x1.41b47993e4372p+4\n"
         "L 0x1.d376f684a7968p+3 0x1.41b47993e4372p+4 0x1.3058f860ab96bp+3 0x1.6f614652cf2bbp+4\n"
         "L 0x1.3058f860ab96bp+3 0x1.6f614652cf2bbp+4 0x1.27552b470853dp+2 0x1.33898c5e93bb5p+4\n"
         "L 0x1.27552b470853dp+2 0x1.33898c5e93bb5p+4 0x1.5cdf64c1aef09p-50 0x1.3c46a9e6fb9p+4\n"
         "L 0x1.5cdf64c1aef09p-50 0x1.3c46a9e6fb9p+4 -0x1.f1a5fa36d2118p+1 0x1.88c0e58753ca1p+4\n"
         "L -0x1.f1a5fa36d2118p+1 0x1.88c0e58753ca1p+4 -0x1.3058f860ab96ap+3 0x1.6f614652cf2bbp+4\n"
         "L -0x1.3058f860ab96ap+3 0x1.6f614652cf2bbp+4 -0x1.4a81ee5e4756p+3 0x1.0dab6deefdd1fp+4\n"
         "L -0x1.4a81ee5e4756p+3 0x1.0dab6deefdd1fp+4 -0x1.bf482764830cap+3 0x1.bf482764830cbp+3\n"
         "L -0x1.bf482764830cap+3 0x1.bf482764830cbp+3 -0x1.41b47993e4371p+4 0x1.d376f684a796ap+3\n"
         "L -0x1.41b47993e4371p+4 0x1.d376f684a796ap+3 -0x1.6f614652cf2bbp+4 0x1.3058f860ab96cp+3\n"
         "L -0x1.6f614652cf2bbp+4 0x1.3058f860ab96cp+3 -0x1.33898c5e93bb5p+4 0x1.27552b470853ep+2\n"
         "L -0x1.33898c5e93bb5p+4 0x1.27552b470853ep+2 -0x1.3c46a9e6fb9p+4 0x1.5cdf64c1aef09p-49\n"
         "L -0x1.3c46a9e6fb9p+4 0x1.5cdf64c1aef09p-49 -0x1.88c0e58753ca1p+4 -0x1.f1a5fa36d2114p+1\n"
         "L -0x1.88c0e58753ca1p+4 -0x1.f1a5fa36d2114p+1 -0x1.6f614652cf2bcp+4 -0x1.3058f860ab969p+3\n"
         "L -0x1.6f614652cf2bcp+4 -0x1.3058f860ab969p+3 -0x1.0dab6deefdd1fp+4 -0x1.4a81ee5e4755ep+3\n"
         "L -0x1.0dab6deefdd1fp+4 -0x1.4a81ee5e4755ep+3 -0x1.bf482764830ccp+3 -0x1.bf482764830cap+3\n"
         "L -0x1.bf482764830ccp+3 -0x1.bf482764830cap+3 -0x1.d376f684a796ap+3 -0x1.41b47993e4371p+4\n"
         "L -0x1.d376f684a796ap+3 -0x1.41b47993e4371p+4 -0x1.3058f860ab972p+3 -0x1.6f614652cf2b9p+4\n"
         "L -0x1.3058f860ab972p+3 -0x1.6f614652cf2b9p+4 -0x1.27552b4708536p+2 -0x1.33898c5e93bb6p+4\n"
         "L -0x1.27552b4708536p+2 -0x1.33898c5e93bb6p+4 -0x1.05a78b9143346p-48 -0x1.3c46a9e6fb9p+4\n"
         "L -0x1.05a78b9143346p-48 -0x1.3c46a9e6fb9p+4 0x1.f1a5fa36d211p+1 -0x1.88c0e58753ca1p+4\n"
         "L 0x1.f1a5fa36d211p+1 -0x1.88c0e58753ca1p+4 0x1.3058f860ab96ep+3 -0x1.6f614652cf2bap+4\n"
         "L 0x1.3058f860ab96ep+3 -0x1.6f614652cf2bap+4 0x1.4a81ee5e4755bp+3 -0x1.0dab6deefdd2p+4\n"
         "L 0x1.4a81ee5e4755bp+3 -0x1.0dab6deefdd2p+4 0x1.bf482764830c9p+3 -0x1.bf482764830ccp+3\n"
         "L 0x1.bf482764830c9p+3 -0x1.bf482764830ccp+3 0x1.41b47993e4371p+4 -0x1.d376f684a796bp+3\n"
         "L 0x1.41b47993e4371p+4 -0x1.d376f684a796bp+3 0x1.6f614652cf2b9p+4 -0x1.3058f860ab973p+3\n"
         "L 0x1.6f614652cf2b9p+4 -0x1.3058f860ab973p+3 0x1.33898c5e93bb6p+4 -0x1.27552b4708537p+2\n"
         "L 0x1.33898c5e93bb6p+4 -0x1.27552b4708537p+2 0x1.3c46a9e6fb9p+4 0x0p+0\n"
         "P 0x1.8p+3 0 0x0p+0 1 0 0x1.6d6af6acacabdp+0 0x1.d12ff60c786bep+2 0x1.3640fe9e96358p-1 0x1.ee9416e0bdac6p-1 0x1.4p+2 1 0x1.aa0c068266688p-4 0 1 0x1.5dc2f5105e997p-1 0x1.56ebf6bdd7657p+0 1 4 0x1.0c80006e24b14p+5 0x0p+0 1 0 0x0p+0 2 0x1.939a7dc0cd1d4p+0 1 0 0x1.c789c16dea568p-3 0x1.3b8dbb61aa981p+1 0x1.001e1ead90ac2p+0 0x1.3cfc31460d16cp+3 0 0x1.0c59b55030b1ap-2 0 0 0x0p+0 0x0p+0 2 8 0 0 0 0 0x1.76a5bc2bfae14p+3 -0x1.67d6546797b24p+5 0x1p+1\n"
         "K 0 0\n"
         , {187.815}},
        // 1510007_2: End mill 3.175 mm, Inside, separate last pass: a thick 0.46 mm2 piece that
        // the bands' chords (inside their arcs) left a channel out of; the band polygons now
        // run outside the arcs.
        {"1510007_2",
         "C 1\n"
         "L 0x1.a788e5da275dcp+4 0x0p+0 0x1.263c9a2ad5a6cp+2 0x1.2c70b697e4093p+2\n"
         "L 0x1.263c9a2ad5a6cp+2 0x1.2c70b697e4093p+2 0x1.ea87c5811dbcdp-50 0x1.bcb2b6459260bp+4\n"
         "L 0x1.ea87c5811dbcdp-50 0x1.bcb2b6459260bp+4 -0x1.0d0d5cb556e17p+2 0x1.159bdcfc4171ap+2\n"
         "L -0x1.0d0d5cb556e17p+2 0x1.159bdcfc4171ap+2 -0x1.acafbe57e4a68p+4 0x1.d8de590e2892p-49\n"
         "L -0x1.acafbe57e4a68p+4 0x1.d8de590e2892p-49 -0x1.1d695cbe18942p+2 -0x1.141aed2083757p+2\n"
         "L -0x1.1d695cbe18942p+2 -0x1.141aed2083757p+2 -0x1.2c27061171414p-48 -0x1.6acf849ab361p+4\n"
         "L -0x1.2c27061171414p-48 -0x1.6acf849ab361p+4 0x1.86c514434a77ap+2 -0x1.ae20209e8b857p+2\n"
         "L 0x1.86c514434a77ap+2 -0x1.ae20209e8b857p+2 0x1.a788e5da275dcp+4 0x0p+0\n"
         "C 0\n"
         "A -0x1.873fdb8a55328p+2 -0x1.e97603ef43bd8p+2 0x1.268f05fd11957p+2 0x1.921fb54442d18p+1 0x0p+0 1\n"
         "A 0x1.8bbc60df9bf0cp+1 -0x1.e97603ef43bd8p+2 0x1.268f05fd11957p+2 0x1.921fb54442d18p+1 0x1.921fb54442d18p+2 0\n"
         "L 0x1.ec6d366cdf8ddp+2 -0x1.e97603ef43bd9p+2 0x1.803347f2b7b1ap+3 -0x1.d7a63750c43c8p-1\n"
         "P 0x1.9666666666666p+1 0 0x0p+0 1 1 0x0p+0 0x1.d8b0fbe697c9dp+3 0x1.50713984a16bcp+2 0x0p+0 0x1.4p+2 1 0x1.a9c167b35c11ep-1 1 1 0x1.b5c5b7983cef7p+1 0x1.4e79c0960fb86p+2 0 5 0x1.26320116ae0cdp+6 0x0p+0 1 0 0x0p+0 2 0x1.512b03305f45ep+4 1 0 0x1.633ff25ad215ep-2 0x1.8b6f1873e7a18p+1 0x1.a1c9a7b7e86c2p-5 0x1.567cad99adbd2p+2 0 0x1.29528ad81da29p+0 0 0 0x0p+0 0x0p+0 0 2 0 0 0 1 0x1.21b31fb511f5p+2 -0x1.ebf5cd3fd257p+2 0x1p+1\n"
         "K 0 0\n"
         , {0.869, 0.46}},
    };
    for (const Case& k : cases) {
        INFO(k.name);
        std::vector<Contour> cs;
        ProfileParams p;
        int vbitDeg = 0;
        const double d = readScene(k.text, cs, p, vbitDeg);
        tooling::Tool tool = test::endMill(d);
        if (vbitDeg > 0) {
            tool.type = tooling::ToolType::VBit;
            tool.includedAngle = vbitDeg * geometry::kPi / 180.0;
        }
        const ProfileResult r = profileToolpath(cs, tool, p);
        REQUIRE(r.ok());
        std::vector<double> got;
        for (const auto& g : loosePieces(r)) got.push_back(g.second);
        std::vector<double> want = k.areas;
        std::sort(got.begin(), got.end());
        std::sort(want.begin(), want.end());
        REQUIRE(got.size() == want.size());
        for (std::size_t i = 0; i < want.size(); ++i) {
            CHECK(std::fabs(got[i] - want[i]) <= 0.03 * want[i] + 0.1);
        }
    }
}

// T-0156 (harness t0135v3 vfree, the exact flood of the moves at the tab layer, leads off).
namespace {
struct FloodScene {
    std::vector<Contour> cs;
    ProfileParams p;
    tooling::Tool tool;
    double d{0.0};
    int vbitDeg{0};
};

FloodScene loadFloodScene(const std::string& text) {
    FloodScene f;
    f.d = readScene(text, f.cs, f.p, f.vbitDeg);
    f.tool = test::endMill(f.d);
    if (f.vbitDeg > 0) {
        f.tool.type = tooling::ToolType::VBit;
        f.tool.includedAngle = f.vbitDeg * geometry::kPi / 180.0;
    }
    f.p.leads.type = LeadType::None;
    f.p.leads.overcut = 0.0;
    f.p.leads.leadOut = false;
    f.p.ramp.onLeadIn = false;
    return f;
}

// The flood's free pieces of 0.1 mm2 or more at the tab layer.
std::vector<double> floodOf(const FloodScene& f, const ProfileResult& r) {
    const double zTab = f.p.passes.bottomZ() + f.p.tabs.thickness;
    const std::vector<double> all =
        f.vbitDeg > 0 ? coneFreePieces(r.toolpath, zTab, std::tan(0.5 * f.vbitDeg * geometry::kPi / 180.0),
                                       0.5 * f.d, 0.04)
                      : rasterFreePieces(r.toolpath, zTab, 0.5 * f.d, 0.04);
    std::vector<double> big;
    for (double a : all) {
        if (a >= 0.1) big.push_back(a);
    }
    return big;
}

std::vector<double> reportedAreas(const ProfileResult& r) {
    std::vector<double> got;
    for (const auto& g : loosePieces(r)) got.push_back(g.second);
    std::sort(got.begin(), got.end());
    return got;
}
}  // namespace

TEST_CASE("Loose pieces (T-0156 c): waste enclosed by several cuts, inside none of their boxes", "[toolpath][profile][tabs]") {
    // Sided tabs searched the inside of each closed cut (and clusters of open ones) only, so a
    // piece between vectors was never seen: the waste between the four bars of a "#" (On, 2 mm
    // tool, kerfs overlapping at the crossings) and between three circles whose kerfs overlap.
    struct Case {
        const char* name;
        const char* text;
        double area;  // the flood's piece
    };
    const std::vector<Case> cases{
        {"c8_hash2",
         "C 1\n"
         "L -0x1.4p+2 0x0p+0 0x1.68p+5 0x0p+0\n"
         "L 0x1.68p+5 0x0p+0 0x1.68p+5 0x1p+1\n"
         "L 0x1.68p+5 0x1p+1 -0x1.4p+2 0x1p+1\n"
         "L -0x1.4p+2 0x1p+1 -0x1.4p+2 0x0p+0\n"
         "C 1\n"
         "L -0x1.4p+2 0x1.3p+5 0x1.68p+5 0x1.3p+5\n"
         "L 0x1.68p+5 0x1.3p+5 0x1.68p+5 0x1.4p+5\n"
         "L 0x1.68p+5 0x1.4p+5 -0x1.4p+2 0x1.4p+5\n"
         "L -0x1.4p+2 0x1.4p+5 -0x1.4p+2 0x1.3p+5\n"
         "C 1\n"
         "L 0x0p+0 -0x1.4p+2 0x1p+1 -0x1.4p+2\n"
         "L 0x1p+1 -0x1.4p+2 0x1p+1 0x1.68p+5\n"
         "L 0x1p+1 0x1.68p+5 0x0p+0 0x1.68p+5\n"
         "L 0x0p+0 0x1.68p+5 0x0p+0 -0x1.4p+2\n"
         "C 1\n"
         "L 0x1.3p+5 -0x1.4p+2 0x1.4p+5 -0x1.4p+2\n"
         "L 0x1.4p+5 -0x1.4p+2 0x1.4p+5 0x1.68p+5\n"
         "L 0x1.4p+5 0x1.68p+5 0x1.3p+5 0x1.68p+5\n"
         "L 0x1.3p+5 0x1.68p+5 0x1.3p+5 -0x1.4p+2\n"
         "P 0x1p+1 0 0x0p+0 2 0 0x0p+0 0x1.8p+1 0x1.8p+1 0x0p+0 0x1.4p+2 0 0x1.999999999999ap-3 0 1 0x1p+1 0x1p+0 0 4 0x1.9p+6 0x0p+0 0 0 0x0p+0 0 0x1.4p+3 0 0 0x1.657184ae74489p-4 0x1.4p+2 0x1.921fb54442d18p-1 0x1.4p+2 0 0x1.921fb54442d18p+0 0 0 0x0p+0 0x0p+0 0 0 0 0 0 0 0x0p+0 0x0p+0 0x1p+1\n"
         "K 0 0\n"         , 1155.932},
        {"c9_tri",
         "C 1\n"
         "A 0x0p+0 0x0p+0 0x1.4p+3 0x0p+0 0x0p+0 0\n"
         "C 1\n"
         "A 0x1.48p+4 0x0p+0 0x1.4p+3 0x0p+0 0x0p+0 0\n"
         "C 1\n"
         "A 0x1.48p+3 0x1.1c0e6bcd8911dp+4 0x1.4p+3 0x0p+0 0x0p+0 0\n"
         "P 0x1p+1 0 0x0p+0 2 0 0x0p+0 0x1.8p+1 0x1.8p+1 0x0p+0 0x1.4p+2 0 0x1.999999999999ap-3 0 1 0x1p+0 0x1p+0 0 3 0x1.9p+6 0x0p+0 0 0 0x0p+0 0 0x1.4p+3 0 0 0x1.657184ae74489p-4 0x1.4p+2 0x1.921fb54442d18p-1 0x1.4p+2 0 0x1.921fb54442d18p+0 0 0 0x0p+0 0x0p+0 0 0 0 0 0 0 0x0p+0 0x0p+0 0x1p+1\n"
         "K 0 0\n"         , 3.963},
    };
    for (const Case& k : cases) {
        INFO(k.name);
        const FloodScene f = loadFloodScene(k.text);
        const ProfileResult r = profileToolpath(f.cs, f.tool, f.p);
        REQUIRE(r.ok());
        const std::vector<double> got = reportedAreas(r);
        const std::vector<double> flood = floodOf(f, r);
        REQUIRE(flood.size() == 1);
        REQUIRE(got.size() == 1);
        CHECK(std::fabs(flood[0] - k.area) <= 0.02 * k.area + 0.1);
        CHECK(std::fabs(got[0] - flood[0]) <= 0.02 * flood[0] + 0.1);
    }
}

TEST_CASE("Loose pieces (T-0156 b): a roughing loop whose finishing cut vanished is searched", "[toolpath][profile][tabs]") {
    // m0_1310036_24: V-bit 60 deg, On, separate last pass, negative allowance: the finishing cut
    // of a small circle vanishes, its roughing loop was never searched, and the 2.1 mm2 slug it
    // encloses came free unreported.
    const FloodScene f = loadFloodScene(
         "C 1\n"
         "A 0x0p+0 0x0p+0 0x1.c901c55efb26ep+4 -0x1.a20dffe67453bp+0 -0x1.82316aa2114f5p+0 0\n"
         "L 0x1.c6bccbb533944p+0 -0x1.c81f4fe8fe3b7p+4 -0x1.c6bccbb533937p+0 -0x1.c81f4fe8fe3b7p+4\n"
         "C 1\n"
         "A 0x1.e9fe88f4c1164p+4 -0x1.8beb61939a208p-1 0x1.037699f2579cp+2 0x0p+0 0x0p+0 1\n"
         "P 0x1.8p+3 60 -0x1.60f47b703f68p-4 2 1 0x0p+0 0x1.2ecc5831a5034p+2 0x1.6626a76f03b6ep+0 0x1.d73c24266808cp-3 0x1.4p+2 1 0x1.a1015f8d80095p-1 0 1 0x1.94db45ecd7725p+1 0x1.a92f8daaabafcp-1 0 1 0x1.790a4a7a416f8p+5 0x0p+0 0 0 0x0p+0 0 0x1.8a29a87cee017p+4 0 0 0x1.deba6c4ec550bp-2 0x1.67610ef31eac2p-1 0x1.11644e9eac71ap-2 0x1.8a8d3f43184d2p+2 0 0x1.f18e739e6c4e2p-4 0 0 0x0p+0 0x0p+0 2 7 0 0 0 1 0x1.b8011a1005d9cp+3 0x1.29070e915234p+3 0x1p+1\n"
         "K 0 0\n"        );
    const ProfileResult r = profileToolpath(f.cs, f.tool, f.p);
    REQUIRE(r.ok());
    const std::vector<double> flood = floodOf(f, r);
    const std::vector<double> got = reportedAreas(r);
    REQUIRE(flood.size() == 1);
    REQUIRE(got.size() == 1);
    CHECK(std::fabs(got[0] - flood[0]) <= 0.03 * flood[0] + 0.1);
    CHECK(got[0] > 1.5);
}

TEST_CASE("V-bit corner moves (T-0156 a): a tab is not cut away along its stretch", "[toolpath][profile][tabs]") {
    // m2_1310149_15: Inside, V-bit 120 deg, sharp internal corners, a tab whose raised stretch
    // ends exactly at an inside corner. The stretch was walked down along a slope to the corner's
    // excursion, cutting the tab away: a 134 mm2 part came free with no message.
    FloodScene f = loadFloodScene(
         "C 1\n"
         "L -0x1.26f7786300082p+3 -0x1.05eb2f5031d1ap+2 0x1.0042cf8407cb2p+2 -0x1.05eb2f5031d1ap+2\n"
         "L 0x1.0042cf8407cb2p+2 -0x1.05eb2f5031d1ap+2 0x1.0042cf8407cb2p+2 0x1.26f7786300082p+3\n"
         "L 0x1.0042cf8407cb2p+2 0x1.26f7786300082p+3 -0x1.26f7786300082p+3 0x1.26f7786300082p+3\n"
         "L -0x1.26f7786300082p+3 0x1.26f7786300082p+3 -0x1.26f7786300082p+3 -0x1.05eb2f5031d1ap+2\n"
         "C 1\n"
         "A 0x1.69a5e9b07f7bbp+5 -0x1.0f1908603f4fcp+1 0x1.dc5012ed50206p+4 -0x1.9fb4841cc329ep-1 -0x1.2a32943d12071p+1 0\n"
         "L 0x1.8b8b2e0a3aa8bp+4 -0x1.7b8177ea3c68ap+4 0x1.06c31e2df0d18p+6 -0x1.7b8177ea3c68ap+4\n"
         "P 0x1.8p+2 120 -0x1.7a8d406f423e5p+1 1 0 0x0p+0 0x1.092462a6d8d2ep+1 0x1.7e8e2181c4ee6p+1 0x0p+0 0x1.4p+2 1 0x1.6603ee050b0d6p-1 0 1 0x1.483dc8921eda6p+1 0x1.e1235a0569712p-3 0 1 0x1.0e29adaf2fa42p+5 0x0p+0 1 0 0x0p+0 3 0x1.3ff06f490c026p+5 1 0 0x1.c227d133a64dfp-3 0x1.1de6dab3d0b5ep+3 0x1.54210283d1f68p-3 0x1.297c825cc2327p+3 0 0x1.c000673c3b294p-3 0 0 0x0p+0 0x0p+0 0 4 0 0 0 0 0x1.c6c79d2ba938p+4 -0x1.4dfdf5bd5e2b9p+5 0x1p+1\n"
         "K 1 1\n"        );
    REQUIRE(f.p.corners.sharpInternal);
    const ProfileResult r = profileToolpath(f.cs, f.tool, f.p);
    REQUIRE(r.ok());
    CHECK(floodOf(f, r).empty());
    CHECK(reportedAreas(r).empty());
}

TEST_CASE("Loose pieces (T-0156): sweep scenes, sided tabs, whole scene", "[toolpath][profile][tabs]") {
    // T-0156 screen (harness vfree, exact flood of the moves, leads off, V-bit corners on), scenes
    // where the region search of sided tabs disagreed with the flood: a piece it did not see
    // (m0_12_762, On, V-bit 90 deg, separate last pass: 5.0 mm2), nine false reports of 0.05 mm2
    // (m0_14_434, its sampled allowance over the bands) and a second 0.07 mm2 piece (m0_13_269).
    struct Case {
        const char* name;
        const char* text;
        std::vector<double> want;
    };
    const std::vector<Case> cases{
        {"m0_12_762",
         "C 1\n"
         "A 0x0p+0 0x0p+0 0x1.1150b51bef2f2p+3 -0x1.ce5273b1ee07dp+0 -0x1.55ecf6d6979b5p+0 0\n"
         "L 0x1.fd6f0c6e4e5ddp+0 -0x1.09cb2374b3b41p+3 -0x1.fd6f0c6e4e5eap+0 -0x1.09cb2374b3b41p+3\n"
         "P 0x1.9p+4 90 -0x1.60a167cd39c47p-3 2 1 0x0p+0 0x1.193bf54a9470cp+1 0x1.374fddc3177c4p+2 0x0p+0 0x1.4p+2 1 0x1.d5a8009e0f573p-1 1 1 0x1.948c74f5074cp+1 0x1.2a5c983a6d08bp-2 0 6 0x1.4df3dcb52e265p+5 0x0p+0 1 0 0x0p+0 0 0x1.ea98e6f3232ecp+3 0 0 0x1.68ccab3b6a17p-4 0x1.623528cc2adfbp+2 0x1.ff0ae9c08d542p-1 0x1.0539b8a15182cp+1 0 0x1.028cf4e64e1e3p-1 0 0 0x0p+0 0x0p+0 0 2 0 0 0 1 0x1.5076157013014p+4 -0x1.0e3b6746174f2p+5 0x1p+1\n"
         "K 1 1\n"
         , {5.026}},
        {"m0_14_434",
         "C 1\n"
         "L -0x1.d434df2dcf0fap+3 -0x1.d434df2dcf0fap+3 -0x1.af7236e5f8779p+2 -0x1.d434df2dcf0fap+3\n"
         "L -0x1.af7236e5f8779p+2 -0x1.d434df2dcf0fap+3 -0x1.af7236e5f8779p+2 -0x1.04cda9d533366p+0\n"
         "L -0x1.af7236e5f8779p+2 -0x1.04cda9d533366p+0 0x1.930174b88242p+3 -0x1.04cda9d533366p+0\n"
         "L 0x1.930174b88242p+3 -0x1.04cda9d533366p+0 0x1.930174b88242p+3 -0x1.d434df2dcf0fap+3\n"
         "L 0x1.930174b88242p+3 -0x1.d434df2dcf0fap+3 0x1.47be9c39aa8afp+4 -0x1.d434df2dcf0fap+3\n"
         "L 0x1.47be9c39aa8afp+4 -0x1.d434df2dcf0fap+3 0x1.47be9c39aa8afp+4 0x1.47be9c39aa8afp+4\n"
         "L 0x1.47be9c39aa8afp+4 0x1.47be9c39aa8afp+4 0x1.930174b88242p+3 0x1.47be9c39aa8afp+4\n"
         "L 0x1.930174b88242p+3 0x1.47be9c39aa8afp+4 0x1.930174b88242p+3 0x1.b7c41d0058da2p+2\n"
         "L 0x1.930174b88242p+3 0x1.b7c41d0058da2p+2 -0x1.af7236e5f8779p+2 0x1.b7c41d0058da2p+2\n"
         "L -0x1.af7236e5f8779p+2 0x1.b7c41d0058da2p+2 -0x1.af7236e5f8779p+2 0x1.47be9c39aa8afp+4\n"
         "L -0x1.af7236e5f8779p+2 0x1.47be9c39aa8afp+4 -0x1.d434df2dcf0fap+3 0x1.47be9c39aa8afp+4\n"
         "L -0x1.d434df2dcf0fap+3 0x1.47be9c39aa8afp+4 -0x1.d434df2dcf0fap+3 -0x1.d434df2dcf0fap+3\n"
         "P 0x1.9p+4 90 -0x1.b32b37a48338p+0 2 1 0x0p+0 0x1.efa5c0a9f5f7p+0 0x1.7399e6b692b6ep+1 0x0p+0 0x1.4p+2 1 0x1.e51e83baec8c3p-1 1 1 0x1.c0e7af9a671f5p+1 0x1.deb8c7cecbf45p-2 0 9 0x1.2459efab33777p+6 0x0p+0 1 0 0x0p+0 0 0x1.10c2cb4a49611p+3 0 0 0x1.dbda8d25822dep-3 0x1.12f3395f6b8fcp+2 0x1.9606eaecd0fc4p-1 0x1.5b8c9aebfe8cfp+2 0 0x1.11feffb3fe8f5p-1 0 0 0x0p+0 0x0p+0 2 1 0 0 0 0 0x1.dd3fcd631e198p+4 0x1.4024d33443154p+5 0x1p+1\n"
         "K 1 1\n"
         , {}},
        {"m0_13_269",
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.3ad3d1fecf5e5p+6 0x0p+0\n"
         "L 0x1.3ad3d1fecf5e5p+6 0x0p+0 0x1.3ad3d1fecf5e5p+6 0x1.9ed923be885bep+5\n"
         "L 0x1.3ad3d1fecf5e5p+6 0x1.9ed923be885bep+5 0x0p+0 0x1.9ed923be885bep+5\n"
         "L 0x0p+0 0x1.9ed923be885bep+5 0x0p+0 0x0p+0\n"
         "C 1\n"
         "L 0x1.0d0dc474a4429p+6 0x1.1edb9cd63536ap+1 0x1.2658b583b64d3p+6 0x1.1edb9cd63536ap+1\n"
         "L 0x1.2658b583b64d3p+6 0x1.1edb9cd63536ap+1 0x1.2658b583b64d3p+6 0x1.aab52713e4ac5p+2\n"
         "L 0x1.2658b583b64d3p+6 0x1.aab52713e4ac5p+2 0x1.0d0dc474a4429p+6 0x1.aab52713e4ac5p+2\n"
         "L 0x1.0d0dc474a4429p+6 0x1.aab52713e4ac5p+2 0x1.0d0dc474a4429p+6 0x1.1edb9cd63536ap+1\n"
         "C 1\n"
         "A 0x1.f87a7d33c4419p+4 0x1.ab078e1700a8fp+4 0x1.23da4c34a8f6ep+3 0x0p+0 0x0p+0 0\n"
         "C 1\n"
         "A 0x1.d400778d06edp+4 0x1.474ef767a5597p+5 0x1.32c4122ff5f5fp+3 0x0p+0 0x0p+0 0\n"
         "P 0x1.8p+2 120 0x1.2b758fde1a7acp-2 2 1 0x0p+0 0x1.27c9bafa0a632p+1 0x1.1bfc614592edp+2 0x1.5ae23c54fa6d5p-1 0x1.4p+2 1 0x1.a4dcefc8a8aa5p-5 1 1 0x1.13e4c32fd9c8fp+0 0x1.0551e7a262d42p+0 0 7 0x1.a82c7d662d3dcp+4 0x0p+0 1 0 0x0p+0 1 0x1.1547cf757e267p+4 0 0 0x1.6c3a976efa766p-3 0x1.4ad066cdc3a75p+1 0x1.b77835d71834cp-3 0x1.2a4390125aa4p+1 0 0x1.629c7301cc5a6p+0 0 0 0x0p+0 0x0p+0 1 7 0 1 0 0 -0x1.63b3070ec1276p+5 0x1.133826212e25ep+5 0x1p+1\n"
         "K 1 1\n"
         , {0.424, 0.076}},
    };
    for (const Case& k : cases) {
        INFO(k.name);
        const FloodScene f = loadFloodScene(k.text);
        const ProfileResult r = profileToolpath(f.cs, f.tool, f.p);
        REQUIRE(r.ok());
        const std::vector<double> got = reportedAreas(r);
        REQUIRE(got.size() == k.want.size());
        for (std::size_t i = 0; i < got.size(); ++i) {
            CHECK(std::fabs(got[i] - k.want[k.want.size() - 1 - i]) <= 0.03 * k.want[0] + 0.1);
        }
    }
}

TEST_CASE("V-bit corner moves (T-0156 round 2): bands and tab check decide outcomes", "[toolpath][profile][tabs][vbit]") {
    // Verifier scenes (harness t0156v1/repro). bandhull_7100_569: L-shape, Inside, V-bit 60 deg,
    // allowance 0, every tab left out: the corner moves' tapered bands close six pieces round the
    // part, which the cut paths alone close only two of. tabcheck_580009_6: a tab that holds,
    // dropped without the corner moves' tool positions in the tab check (they make the exposed
    // corner part of the tab's own material). tabdrop_9001_11 and tabdrop_570014_182: tabs that
    // hold (even with the tools widened) and were dropped by a first version of the corner-move
    // samples; they stay, as on main.
    auto leftOut = [](const ProfileResult& r, const char* vec) {
        std::size_t n = 0;
        for (const ProfileDiagnostic& g : r.diagnostics) {
            if (g.message.find(vec) == 0 && g.message.find("left out") != std::string::npos) ++n;
        }
        return n;
    };
    {
        const FloodScene f = loadFloodScene(
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.4p+5 0x0p+0\n"
         "L 0x1.4p+5 0x0p+0 0x1.4p+5 0x1.4p+3\n"
         "L 0x1.4p+5 0x1.4p+3 0x1.4p+3 0x1.4p+3\n"
         "L 0x1.4p+3 0x1.4p+3 0x1.4p+3 0x1.ep+4\n"
         "L 0x1.4p+3 0x1.ep+4 0x0p+0 0x1.ep+4\n"
         "L 0x0p+0 0x1.ep+4 0x0p+0 0x0p+0\n"
         "P 0x1.8p+3 60 0x0p+0 1 0 0x0p+0 0x1.ce1bbed3a5b3fp+1 0x1.fb39036a2811p+0 0x0p+0 0x1.4p+2 1 0x1.b02e6d64af859p-1 0 1 0x1.5e289ae2ad04bp+2 0x1.3b3f74a9ec15bp-1 0 5 0x1.bc5c5878cae23p+4 0x0p+0 0 0 0x0p+0 2 0x1.0c6db42fdb06cp+3 0 0 0x1.e6a15aefae611p-3 0x1.75a5f42def378p+3 0x1.4c6ee4fc98ca1p-2 0x1.df102a4b4471fp+2 0 0x1.71737e6ebda8ep-1 0 0 0x0p+0 0x0p+0 0 0 0 0 0 1 0x1.ccb3df7b43ec8p+3 -0x1.1b4c0c1533557p+5 0x1p+1\n"
         "K 1 1\n"        );
        const ProfileResult r = profileToolpath(f.cs, f.tool, f.p);
        REQUIRE(r.ok());
        const std::vector<double> got = reportedAreas(r);
        CHECK(got.size() == 6);
        CHECK(floodOf(f, r).size() == 6);
    }
    {
        const FloodScene f = loadFloodScene(
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.4d307451f47afp+3 0x0p+0\n"
         "L 0x1.4d307451f47afp+3 0x0p+0 0x1.4d307451f47afp+3 0x1.6e711d2247c54p+3\n"
         "L 0x1.4d307451f47afp+3 0x1.6e711d2247c54p+3 0x1.6e711d2247c54p+4 0x1.6e711d2247c54p+3\n"
         "L 0x1.6e711d2247c54p+4 0x1.6e711d2247c54p+3 0x1.6e711d2247c54p+4 0x0p+0\n"
         "L 0x1.6e711d2247c54p+4 0x0p+0 0x1.0a84aba5a1016p+5 0x0p+0\n"
         "L 0x1.0a84aba5a1016p+5 0x0p+0 0x1.0a84aba5a1016p+5 0x1.0a84aba5a1016p+5\n"
         "L 0x1.0a84aba5a1016p+5 0x1.0a84aba5a1016p+5 0x1.6e711d2247c54p+4 0x1.0a84aba5a1016p+5\n"
         "L 0x1.6e711d2247c54p+4 0x1.0a84aba5a1016p+5 0x1.6e711d2247c54p+4 0x1.5dd0c8ba1e202p+4\n"
         "L 0x1.6e711d2247c54p+4 0x1.5dd0c8ba1e202p+4 0x1.4d307451f47afp+3 0x1.5dd0c8ba1e202p+4\n"
         "L 0x1.4d307451f47afp+3 0x1.5dd0c8ba1e202p+4 0x1.4d307451f47afp+3 0x1.0a84aba5a1016p+5\n"
         "L 0x1.4d307451f47afp+3 0x1.0a84aba5a1016p+5 0x0p+0 0x1.0a84aba5a1016p+5\n"
         "L 0x0p+0 0x1.0a84aba5a1016p+5 0x0p+0 0x0p+0\n"
         "C 1\n"
         "L 0x1.9p+5 0x0p+0 0x1.4128c0b3c8545p+6 0x0p+0\n"
         "L 0x1.4128c0b3c8545p+6 0x0p+0 0x1.4128c0b3c8545p+6 0x1.e4a302cf21513p+4\n"
         "L 0x1.4128c0b3c8545p+6 0x1.e4a302cf21513p+4 0x1.1f7bf6e4bcdecp+6 0x1.e4a302cf21513p+4\n"
         "L 0x1.1f7bf6e4bcdecp+6 0x1.e4a302cf21513p+4 0x1.1f7bf6e4bcdecp+6 0x1.0d664e785bacap+3\n"
         "L 0x1.1f7bf6e4bcdecp+6 0x1.0d664e785bacap+3 0x1.d359939e16eb2p+5 0x1.0d664e785bacap+3\n"
         "L 0x1.d359939e16eb2p+5 0x1.0d664e785bacap+3 0x1.d359939e16eb2p+5 0x1.e4a302cf21513p+4\n"
         "L 0x1.d359939e16eb2p+5 0x1.e4a302cf21513p+4 0x1.9p+5 0x1.e4a302cf21513p+4\n"
         "L 0x1.9p+5 0x1.e4a302cf21513p+4 0x1.9p+5 0x0p+0\n"
         "P 0x1.9p+4 60 -0x1.1cb6c21fed81p-6 0 0 0x0p+0 0x1.2ee58b4445d76p+1 0x1.a6e059d800ccp-1 0x0p+0 0x1.4p+2 1 0x1.c965ce0cc9445p-1 1 1 0x1.aa621b7197b37p+1 0x1.a09e7c7411d3ep-1 0 7 0x1.a622e4302aadep+5 0x0p+0 1 0 0x0p+0 0 0x1.ce94608ef08fep+4 1 1 0x1.b3cffa5ce1e6bp-2 0x1.cd8d18e775e8dp+1 0x1.93452c908fd9dp-1 0x1.1e0231e2f628cp+3 0 0x1.6ded7913916b9p+0 1 0 0x1.22dc77c14194dp+4 0x0p+0 2 6 0 0 0 0 -0x1.9a46b116ceafdp+4 -0x1.0c6379cb93d0fp+5 0x1p+1\n"
         "K 1 1\n"        );
        const ProfileResult r = profileToolpath(f.cs, f.tool, f.p);
        REQUIRE(r.ok());
        CHECK(leftOut(r, "vector 0") == 0);
        CHECK(floodOf(f, r).empty());
    }
    {
        const FloodScene f = loadFloodScene(
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.848035381fa5p+4 0x0p+0\n"
         "L 0x1.848035381fa5p+4 0x0p+0 0x1.848035381fa5p+4 0x1.cd5b3e814d4bdp+2\n"
         "L 0x1.848035381fa5p+4 0x1.cd5b3e814d4bdp+2 0x1.cd5b3e814d4bdp+2 0x1.cd5b3e814d4bdp+2\n"
         "L 0x1.cd5b3e814d4bdp+2 0x1.cd5b3e814d4bdp+2 0x1.cd5b3e814d4bdp+2 0x1.0ff358741626bp+5\n"
         "L 0x1.cd5b3e814d4bdp+2 0x1.0ff358741626bp+5 0x0p+0 0x1.0ff358741626bp+5\n"
         "L 0x0p+0 0x1.0ff358741626bp+5 0x0p+0 0x0p+0\n"
         "C 1\n"
         "L 0x1.1449b1fe19d37p+5 0x1.ff3c65701e698p-1 0x1.a01b4959b7ebap+5 0x1.ff3c65701e698p-1\n"
         "L 0x1.a01b4959b7ebap+5 0x1.ff3c65701e698p-1 0x1.a01b4959b7ebap+5 0x1.279d11e2bd23ap+4\n"
         "L 0x1.a01b4959b7ebap+5 0x1.279d11e2bd23ap+4 0x1.73ec0d02d6666p+5 0x1.279d11e2bd23ap+4\n"
         "L 0x1.73ec0d02d6666p+5 0x1.279d11e2bd23ap+4 0x1.73ec0d02d6666p+5 0x1.a1616f650ff71p+2\n"
         "L 0x1.73ec0d02d6666p+5 0x1.a1616f650ff71p+2 0x1.4078ee54fb58bp+5 0x1.a1616f650ff71p+2\n"
         "L 0x1.4078ee54fb58bp+5 0x1.a1616f650ff71p+2 0x1.4078ee54fb58bp+5 0x1.279d11e2bd23ap+4\n"
         "L 0x1.4078ee54fb58bp+5 0x1.279d11e2bd23ap+4 0x1.1449b1fe19d37p+5 0x1.279d11e2bd23ap+4\n"
         "L 0x1.1449b1fe19d37p+5 0x1.279d11e2bd23ap+4 0x1.1449b1fe19d37p+5 0x1.ff3c65701e698p-1\n"
         "P 0x1.8p+2 60 -0x1.71e7fc117af62p-4 1 0 0x0p+0 0x1.22461f2caa1d2p+1 0x1.c5f546ba04691p+2 0x0p+0 0x1.4p+2 1 0x1.fd2deb71121d5p-1 0 1 0x1.5ece6c5e3a856p+1 0x1.6a11774c8e17ap-1 0 6 0x1.f6d934eaae276p+5 0x0p+0 0 0 0x0p+0 2 0x1.bd21dfd4656f2p+2 0 0 0x1.9382d0e27a694p-2 0x1.37b86b6a00a1fp+2 0x1.42a4dbd2f772dp+0 0x1.85c5749eea1f6p+1 2 0x1.fe9eb92ba9d66p-1 0 0 0x0p+0 0x0p+0 1 7 0 0 0 0 -0x1.04725a0b445aep+5 0x1.9fd95f7954d54p+4 0x1p+1\n"
         "K 1 1\n"        );
        const ProfileResult r = profileToolpath(f.cs, f.tool, f.p);
        REQUIRE(r.ok());
        CHECK(leftOut(r, "vector 0") == 0);
        CHECK(reportedAreas(r).size() == floodOf(f, r).size());
    }
    {
        const FloodScene f = loadFloodScene(
         "C 1\n"
         "L 0x0p+0 0x0p+0 0x1.859a6c4f4c58fp+3 0x0p+0\n"
         "L 0x1.859a6c4f4c58fp+3 0x0p+0 0x1.859a6c4f4c58fp+3 0x1.73f0dfd17bbaap+3\n"
         "L 0x1.859a6c4f4c58fp+3 0x1.73f0dfd17bbaap+3 0x1.73f0dfd17bbaap+4 0x1.73f0dfd17bbaap+3\n"
         "L 0x1.73f0dfd17bbaap+4 0x1.73f0dfd17bbaap+3 0x1.73f0dfd17bbaap+4 0x0p+0\n"
         "L 0x1.73f0dfd17bbaap+4 0x0p+0 0x1.1b5f0afc90f39p+5 0x0p+0\n"
         "L 0x1.1b5f0afc90f39p+5 0x0p+0 0x1.1b5f0afc90f39p+5 0x1.1b5f0afc90f39p+5\n"
         "L 0x1.1b5f0afc90f39p+5 0x1.1b5f0afc90f39p+5 0x1.73f0dfd17bbaap+4 0x1.1b5f0afc90f39p+5\n"
         "L 0x1.73f0dfd17bbaap+4 0x1.1b5f0afc90f39p+5 0x1.73f0dfd17bbaap+4 0x1.7cc5a6106409dp+4\n"
         "L 0x1.73f0dfd17bbaap+4 0x1.7cc5a6106409dp+4 0x1.859a6c4f4c58fp+3 0x1.7cc5a6106409dp+4\n"
         "L 0x1.859a6c4f4c58fp+3 0x1.7cc5a6106409dp+4 0x1.859a6c4f4c58fp+3 0x1.1b5f0afc90f39p+5\n"
         "L 0x1.859a6c4f4c58fp+3 0x1.1b5f0afc90f39p+5 0x0p+0 0x1.1b5f0afc90f39p+5\n"
         "L 0x0p+0 0x1.1b5f0afc90f39p+5 0x0p+0 0x0p+0\n"
         "C 1\n"
         "L 0x1.4344808245b48p+5 0x1.c951d8e902534p+0 0x1.c834b9584086cp+5 0x1.c951d8e902534p+0\n"
         "L 0x1.c834b9584086cp+5 0x1.c951d8e902534p+0 0x1.c834b9584086cp+5 0x1.26758f3a85c9cp+4\n"
         "L 0x1.c834b9584086cp+5 0x1.26758f3a85c9cp+4 0x1.a26abc7819ep+5 0x1.26758f3a85c9cp+4\n"
         "L 0x1.a26abc7819ep+5 0x1.26758f3a85c9cp+4 0x1.a26abc7819ep+5 0x1.a0a45d3b75ca9p+2\n"
         "L 0x1.a26abc7819ep+5 0x1.a0a45d3b75ca9p+2 0x1.690e7d626c5b4p+5 0x1.a0a45d3b75ca9p+2\n"
         "L 0x1.690e7d626c5b4p+5 0x1.a0a45d3b75ca9p+2 0x1.690e7d626c5b4p+5 0x1.26758f3a85c9cp+4\n"
         "L 0x1.690e7d626c5b4p+5 0x1.26758f3a85c9cp+4 0x1.4344808245b48p+5 0x1.26758f3a85c9cp+4\n"
         "L 0x1.4344808245b48p+5 0x1.26758f3a85c9cp+4 0x1.4344808245b48p+5 0x1.c951d8e902534p+0\n"
         "P 0x1.9p+4 90 -0x1.d72e7d0453c5ap-7 1 0 0x0p+0 0x1.28fe28d31c6f2p+2 0x1.a707b18611261p+0 0x1.df77762acb7ap-2 0x1.4p+2 0 0x1.15aa24c0b579cp-4 0 1 0x1.8d417a28341bp+0 0x1.67ce49d533255p+0 0 8 0x1.8e4cad575a5a7p+3 0x0p+0 0 0 0x0p+0 3 0x1.fedc533b70657p+4 1 0 0x1.b8ef2d04c6b5p-4 0x1.7d046ee6d601dp+3 0x1.91327c64e537fp-2 0x1.35dd7177d9d15p+3 2 0x1.1bc19756d2b52p-3 1 0 0x0p+0 0x0p+0 1 1 0 1 0 0 -0x1.5b79b2473848p+5 -0x1.2b7c22465ac12p+5 0x1p+1\n"
         "K 1 1\n"        );
        const ProfileResult r = profileToolpath(f.cs, f.tool, f.p);
        REQUIRE(r.ok());
        CHECK(leftOut(r, "vector 0") == 0);
        CHECK(reportedAreas(r).size() == floodOf(f, r).size());
    }
}
