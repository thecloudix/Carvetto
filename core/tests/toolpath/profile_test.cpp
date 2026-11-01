// 2D Profile toolpath (T-0120): acceptance criteria 1-5 and the Goal features. The golden files
// (AC1, AC6 G-code) are in profile_golden_test.cpp; the randomized gouge test in
// profile_property_test.cpp.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iterator>
#include <sstream>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/contour.hpp"
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
    // A remnant the tools leave counts (one thinner than the raster's finest widening, about
    // 0.15 mm across for this tool, counts as nothing): every kept tab must hold it (independent
    // flood from the centre to the waste), the rest are reported.
    for (const double radius : {4.1, 4.2, 4.5, 5.0, 6.0}) {
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
    // six-point star cut On with a 60 degree V-bit whose two long tabs on the two edges of an arm
    // only hold the part together (and the cone cuts only thickness x tan 30 at the tab top).
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
