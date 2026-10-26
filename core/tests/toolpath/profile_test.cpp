// 2D Profile toolpath (T-0120): acceptance criteria 1-5 and the Goal features. The golden files
// (AC1, AC6 G-code) are in profile_golden_test.cpp; the randomized gouge test in
// profile_property_test.cpp.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <utility>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "profile_test_util.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using carvetto::geometry::ArcSpan;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::LineSpan;
using carvetto::geometry::Vec2;
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
