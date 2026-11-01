// Toolpath API alignment (T-0123, ADR-0018): one ramp implementation (AC1), Tool + cutting data
// and one diagnostics type for Profile and Pocket (AC2), the spatial index behind the vector-pair
// checks (AC3), rest patterns built over the rest areas (AC4) and links routed through cleared
// area for Raster and rest toolpaths (AC5).

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

#include "../geometry/test_rng.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/tooling/cutting_data.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/box_index.hpp"
#include "carvetto/toolpath/clearance.hpp"
#include "carvetto/toolpath/operation.hpp"
#include "carvetto/toolpath/path_walk.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"  // with pocket.hpp in one unit: the shared types do not clash
#include "carvetto/toolpath/ramp_entry.hpp"
#include "pocket_oracle.hpp"
#include "pocket_test_util.hpp"

using namespace carvetto::toolpath;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::kPi;
using carvetto::geometry::Polyline;
using carvetto::geometry::Tolerance;
using carvetto::geometry::Vec2;
using carvetto::geometry::Vec3;
using carvetto::test::SplitMix;
using carvetto::tooling::CuttingData;
using carvetto::tooling::Tool;
using carvetto::tooling::ToolType;
using Catch::Matchers::WithinAbs;
namespace orc = carvetto::test::pocket;

namespace {

Contour rect(double x0, double y0, double x1, double y1) {
    return Contour::rectangle(Box2{Vec2{x0, y0}, Vec2{x1, y1}});
}

Tool endMillTool(double diameter, double stepover, double passDepth) {
    Tool t;
    t.id = "em";
    t.name = "EM " + std::to_string(diameter);
    t.diameter = diameter;
    t.defaultCuttingData.stepover = stepover;
    t.defaultCuttingData.passDepth = passDepth;
    t.defaultCuttingData.feedRate = 1000.0;
    t.defaultCuttingData.plungeRate = 300.0;
    t.defaultCuttingData.spindleSpeed = 18000.0;
    t.defaultCuttingData.toolNumber = 3;
    return t;
}

bool sameMoves(const Toolpath& a, const Toolpath& b) {
    if (a.moves.size() != b.moves.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.moves.size(); ++i) {
        if (!(a.moves[i] == b.moves[i])) {
            return false;
        }
    }
    return true;
}

}  // namespace

// --------------------------------------------------------------------------------------- AC1

TEST_CASE("AC1: zigZagLegs descends back and forth and ends at the start at the level",
          "[toolpath][ramp][T-0123]") {
    RampOptions ramp;
    ramp.type = RampType::ZigZag;
    const std::vector<PathLeg> two = zigZagLegs(2.0, 1.0, 8.0, 0.0, -3.0, ramp);
    REQUIRE(two.size() == 2);
    CHECK(two[0].u0 == 2.0);
    CHECK(two[0].u1 == 10.0);
    CHECK(two[1].u0 == 10.0);
    CHECK(two[1].u1 == 2.0);
    CHECK(two[0].z.after(0.0) == 0.0);
    CHECK(two[0].z.before(8.0) == -1.5);
    CHECK(two[1].z.after(0.0) == -1.5);
    CHECK(two[1].z.before(8.0) == -3.0);
    // Backwards (open path walked from its far end).
    const std::vector<PathLeg> back = zigZagLegs(20.0, -1.0, 5.0, -1.0, -2.0, ramp);
    REQUIRE(back.size() == 2);
    CHECK(back[0].u1 == 15.0);
    CHECK(back[1].u1 == 20.0);
    // Angle mode: no leg descends more than length * tan(angle).
    ramp.useAngle = true;
    ramp.angle = std::atan(0.25);  // 2 mm per 8 mm leg
    const std::vector<PathLeg> many = zigZagLegs(0.0, 1.0, 8.0, 0.0, -9.0, ramp);
    REQUIRE(many.size() == 6);  // 9 mm / 2 mm per leg -> 5 legs -> rounded up to pairs
    for (const PathLeg& leg : many) {
        CHECK(leg.z.after(0.0) - leg.z.before(8.0) <= 2.0 + 1e-12);
    }
    CHECK(many.back().u1 == 0.0);
    CHECK(many.back().z.before(8.0) == -9.0);
}

TEST_CASE("AC1: Profile's planPass and Pocket's ramp plunge share the zig-zag legs", "[toolpath][ramp][T-0123]") {
    RampOptions ramp;
    ramp.type = RampType::ZigZag;
    ramp.distance = 6.0;
    const Tolerance tol = Tolerance::defaults();
    // Closed path: planPass = the zig-zag legs, then one revolution at the level.
    const Contour square = rect(0, 0, 10, 10);
    const ArcLengthPath walk(square, tol);
    const PassLegs pass = planPass(walk, 3.0, -1.0, -4.0, ramp);
    const std::vector<PathLeg> legs = zigZagLegs(3.0, 1.0, 6.0, -1.0, -4.0, ramp);
    REQUIRE(pass.legs.size() == legs.size() + 1);
    for (std::size_t i = 0; i < legs.size(); ++i) {
        CHECK(pass.legs[i].u0 == legs[i].u0);
        CHECK(pass.legs[i].u1 == legs[i].u1);
        CHECK(pass.legs[i].z.knots == legs[i].z.knots);
    }

    // Pocket's polyline ramp is exactly those legs walked with appendLeg on the same path.
    for (const bool closed : {true, false}) {
        INFO("closed " << closed);
        const Polyline pl{{{0, 0}, {4, 0}, {4, 0}, {4, 3}, {9, 3}, {9, 8}}, closed};
        Toolpath viaPocket;
        viaPocket.append(Move::rapid(Vec3{pl.points.front(), -1.0}));
        REQUIRE(appendPathRamp(pl, -1.0, -4.0, 7.5, 0.01, tol, viaPocket));
        Toolpath viaLegs;
        viaLegs.append(Move::rapid(Vec3{pl.points.front(), -1.0}));
        const Polyline clean{{{0, 0}, {4, 0}, {4, 3}, {9, 3}, {9, 8}}, closed};  // zero-length segment dropped
        const ArcLengthPath pw(Contour::fromPolyline(clean), tol);
        for (const PathLeg& leg : zigZagLegs(0.0, 1.0, 7.5, -1.0, -4.0, ramp)) {
            appendLeg(pw, leg.u0, leg.u1, leg.z, viaLegs);
        }
        CHECK(sameMoves(viaPocket, viaLegs));
        // Forward to the midway height at 7.5 mm along, back to the start exactly at the level.
        CHECK(viaPocket.moves.back().to == Vec3{pl.points.front(), -4.0});
        bool reachedMid = false;
        for (const Move& m : viaPocket.moves) {
            CHECK(m.kind != MoveKind::Plunge);
            // 7.5 mm along: 4 + 3 to (4, 3), then 0.5 towards (9, 3).
            reachedMid = reachedMid || (m.to.z == -2.5 && distance(m.to.xy(), Vec2{4.5, 3.0}) < 1e-12);
        }
        CHECK(reachedMid);
    }
    // Too short for a ramp: one vertical plunge.
    Toolpath plunge;
    plunge.append(Move::rapid(Vec3{{0, 0}, -1.0}));
    CHECK_FALSE(appendPathRamp(Polyline{{{0, 0}, {0.005, 0}}, false}, -1.0, -4.0, 7.5, 0.01, tol, plunge));
    REQUIRE(plunge.moves.size() == 2);
    CHECK(plunge.moves.back().kind == MoveKind::Plunge);
    CHECK(plunge.moves.back().to == Vec3{{0, 0}, -4.0});
}

// --------------------------------------------------------------------------------------- AC2

static_assert(std::is_same_v<decltype(ProfileResult::diagnostics), decltype(PocketResult::diagnostics)>,
              "Profile and Pocket report the same diagnostics type");
static_assert(std::is_same_v<ProfileDiagnostic, ToolpathDiagnostic> && std::is_same_v<ProfileIssue, ToolpathIssue>);
static_assert(std::is_same_v<decltype(ProfileParams::direction), decltype(PocketParams::direction)>,
              "one CutDirection for both operations");
static_assert(std::is_same_v<decltype(ProfileParams::cuttingData), decltype(PocketTool::cuttingData)>,
              "cutting data is given the same way");

TEST_CASE("AC2: Tool + cutting data -> pass tool conversion", "[toolpath][api][T-0123]") {
    Tool em = endMillTool(6.0, 2.4, 3.0);
    const PassTool p = passTool(em, em.defaultCuttingData, 12.0);
    CHECK(p.radius == 3.0);
    CHECK(p.stepover == 2.4);
    CHECK(p.passDepth == 3.0);
    CHECK(p.snapshot.name == em.name);
    CHECK(p.snapshot.diameter == 6.0);
    CHECK(p.snapshot.toolNumber == 3);
    CHECK(p.snapshot.feedRate == 1000.0);
    CHECK(p.snapshot.plungeRate == 300.0);
    CHECK(p.snapshot.spindleSpeed == 18000.0);
    CHECK(p.snapshot == toolSnapshot(em, em.defaultCuttingData));

    // A V-bit cuts its width at the bottom of the cut (as Profile's offset radius).
    Tool v = em;
    v.type = ToolType::VBit;
    v.diameter = 12.0;
    v.includedAngle = kPi / 2.0;  // width = 2 * depth
    CHECK_THAT(passTool(v, v.defaultCuttingData, 2.0).radius, WithinAbs(2.0, 1e-12));
    CHECK_THAT(passTool(v, v.defaultCuttingData, 50.0).radius, WithinAbs(6.0, 1e-12));  // capped at D/2
    ProfileParams pp;
    pp.passes.cutDepth = 2.0;
    CHECK(passTool(v, v.defaultCuttingData, 2.0).radius == profileToolRadius(v, pp.passes));

    // effectiveCuttingData: the given data wins, else the tool's default.
    CuttingData other = em.defaultCuttingData;
    other.feedRate = 2500.0;
    CHECK(&effectiveCuttingData(em, std::nullopt) == &em.defaultCuttingData);
    CHECK(effectiveCuttingData(em, other).feedRate == 2500.0);
}

TEST_CASE("AC2: Profile and Pocket take the cutting data the same way", "[toolpath][api][T-0123]") {
    const Tool em = endMillTool(6.0, 2.4, 3.0);
    CuttingData fast = em.defaultCuttingData;
    fast.feedRate = 2400.0;
    fast.plungeRate = 600.0;
    fast.toolNumber = 7;
    fast.passDepth = 2.0;
    fast.stepover = 3.0;

    ProfileParams pp;
    pp.passes.cutDepth = 6.0;
    const ProfileResult byDefault = profileToolpath({rect(0, 0, 40, 30)}, em, pp);
    pp.cuttingData = fast;
    const ProfileResult byData = profileToolpath({rect(0, 0, 40, 30)}, em, pp);
    CHECK(byDefault.toolpath.tool == toolSnapshot(em, em.defaultCuttingData));
    CHECK(byData.toolpath.tool == toolSnapshot(em, fast));
    CHECK(byDefault.toolpath.count(MoveKind::Plunge) == 2);  // 6 mm in 3 mm passes
    CHECK(byData.toolpath.count(MoveKind::Plunge) == 3);     // 6 mm in 2 mm passes

    PocketParams kp;
    kp.cutDepth = 6.0;
    const PocketResult pDefault = pocketToolpath({rect(0, 0, 40, 30)}, {PocketTool{em, std::nullopt}}, kp);
    const PocketResult pData = pocketToolpath({rect(0, 0, 40, 30)}, {PocketTool{em, fast}}, kp);
    CHECK(pDefault.toolpaths[0].tool == toolSnapshot(em, em.defaultCuttingData));
    CHECK(pData.toolpaths[0].tool == toolSnapshot(em, fast));
    CHECK(pDefault.plans[0].levels.size() == 2);
    CHECK(pData.plans[0].levels.size() == 3);
    // The stepover comes from the cutting data too: wider stepover, fewer rings.
    CHECK(pData.plans[0].paths.size() < pDefault.plans[0].paths.size());
    CHECK(pDefault.diagnostics.empty());
    CHECK(pDefault.ok());
}

TEST_CASE("AC2: Pocket reports through the shared diagnostics type", "[toolpath][api][T-0123]") {
    PocketParams p;
    p.cutDepth = 3.0;
    const PocketTool small = orc::endMill("S", 1, 3.0, 1.2, 3.0);
    const PocketTool huge = orc::endMill("H", 2, 80.0, 30.0, 3.0);

    const PocketResult none = pocketToolpath({rect(0, 0, 50, 30)}, {}, p);
    CHECK(none.has(ToolpathIssue::NoTool));
    CHECK_FALSE(none.ok());

    Contour open = rect(0, 0, 10, 10);
    open.closed = false;
    const PocketResult withOpen = pocketToolpath({rect(0, 0, 50, 30), open}, {small}, p);
    REQUIRE(withOpen.diagnostics.size() == 1);
    CHECK(withOpen.diagnostics[0].issue == ToolpathIssue::OpenContourIgnored);
    CHECK_FALSE(withOpen.diagnostics[0].tool.has_value());
    CHECK(withOpen.ok());

    const PocketResult tooLarge = pocketToolpath({rect(0, 0, 50, 30)}, {huge}, p);
    REQUIRE(tooLarge.diagnostics.size() == 1);
    CHECK(tooLarge.diagnostics[0].issue == ToolpathIssue::ToolTooLarge);
    CHECK(tooLarge.diagnostics[0].tool == std::optional<std::size_t>{0});
    CHECK(tooLarge.diagnostics[0].message.find("too large") != std::string::npos);

    PocketTool badStep = small;
    orc::cuttingOf(badStep).stepover = 0.0;
    const PocketResult invalid = pocketToolpath({rect(0, 0, 50, 30)}, {small, badStep}, p);
    REQUIRE(invalid.diagnostics.size() == 1);
    CHECK(invalid.diagnostics[0].issue == ToolpathIssue::InvalidTool);
    CHECK(invalid.diagnostics[0].tool == std::optional<std::size_t>{1});

    // The same tool again finds nothing it can reach.
    const PocketResult same = pocketToolpath({rect(0, 0, 50, 30)}, {small, small}, p);
    CHECK((same.has(ToolpathIssue::NoRestMaterial) || same.has(ToolpathIssue::RestUnreachable)));
    CHECK(same.toolpaths[1].isEmpty());
    REQUIRE(same.diagnostics.size() == 1);
    CHECK(same.diagnostics[0].tool == std::optional<std::size_t>{1});

    PocketParams bad = p;
    bad.cutDepth = -1.0;
    CHECK(pocketToolpath({rect(0, 0, 50, 30)}, {small}, bad).has(ToolpathIssue::InvalidParameters));

    PocketTool tinyPass = small;
    orc::cuttingOf(tinyPass).passDepth = 1e-9;
    CHECK(pocketToolpath({rect(0, 0, 50, 30)}, {tinyPass}, p).has(ToolpathIssue::WorkLimitExceeded));

    std::vector<ToolpathDiagnostic> centreNotes;
    CHECK(pocketCentreRegion({rect(0, 0, 50, 30), open}, 3.0, p, &centreNotes).size() == 1);
    REQUIRE(centreNotes.size() == 1);
    CHECK(centreNotes[0].issue == ToolpathIssue::OpenContourIgnored);

    // Profile uses the same type and codes.
    const ProfileResult prof = profileToolpath({Contour::circle({0, 0}, 2.0)}, endMillTool(6.0, 2.4, 3.0), [] {
        ProfileParams pp;
        pp.side = ProfileSide::Inside;
        pp.passes.cutDepth = 3.0;
        return pp;
    }());
    CHECK(prof.has(ToolpathIssue::ToolTooLarge));
    CHECK(hasIssue(prof.diagnostics, ProfileIssue::ToolTooLarge));
}

// ------------------------------------------------------------------- AC3: spatial index pieces

TEST_CASE("AC3: BoxIndex returns exactly the intersecting boxes, ascending", "[toolpath][index][T-0123]") {
    SplitMix rng{0x7123'0001ULL};
    std::vector<Box2> boxes;
    for (int i = 0; i < 1500; ++i) {
        const Vec2 a{rng.uniform(-100, 100), rng.uniform(-100, 100)};
        const double w = i % 97 == 0 ? rng.uniform(50, 300) : rng.uniform(0.0, 6.0);  // a few frames
        const double h = i % 89 == 0 ? rng.uniform(50, 300) : rng.uniform(0.0, 6.0);
        boxes.push_back(i % 101 == 0 ? Box2{} : Box2{a, a + Vec2{w, h}});  // and some empty boxes
    }
    const BoxIndex index(boxes);
    for (int q = 0; q < 400; ++q) {
        const Vec2 a{rng.uniform(-150, 150), rng.uniform(-150, 150)};
        const Box2 query = q % 3 == 0 ? Box2{a, a} : Box2{a, a + Vec2{rng.uniform(0, 40), rng.uniform(0, 40)}};
        std::vector<std::size_t> brute;
        for (std::size_t i = 0; i < boxes.size(); ++i) {
            if (boxes[i].intersects(query)) {
                brute.push_back(i);
            }
        }
        CHECK(index.query(query) == brute);
        if (q % 3 == 0) {
            CHECK(index.containing(a) == brute);
        }
    }
    CHECK(BoxIndex{}.query(Box2{{0, 0}, {1, 1}}).empty());
    CHECK(BoxIndex(std::vector<Box2>{Box2{}}).query(Box2{{0, 0}, {1, 1}}).empty());
}

TEST_CASE("AC3: BoxIndex stays exact with huge and non-finite boxes", "[toolpath][index][T-0123]") {
    const double big = 1e308;
    const double inf = std::numeric_limits<double>::infinity();
    const std::vector<Box2> boxes{
        Box2{{-big, -big}, {-big + 1e300, -big + 1e300}},
        Box2{{big - 1e300, big - 1e300}, {big, big}},  // bounds of these two overflow a double
        Box2{{-big, 0}, {big, 1}},                     // finite corners, infinite width
        Box2{{0, 0}, {1, 1}},
        Box2{{-inf, -1}, {inf, 1}},
    };
    const BoxIndex index(boxes);
    const std::vector<Box2> queries{Box2{{0.5, 0.5}, {0.5, 0.5}}, Box2{{-big, -big}, {big, big}},
                                    Box2{{5e307, 0.5}, {5e307, 0.5}}, Box2{{2, -0.5}, {3, 0.5}},
                                    Box2{{-inf, -inf}, {inf, inf}}};
    for (const Box2& q : queries) {
        std::vector<std::size_t> brute;
        for (std::size_t i = 0; i < boxes.size(); ++i) {
            if (boxes[i].intersects(q)) {
                brute.push_back(i);
            }
        }
        CHECK(index.query(q) == brute);
    }
    // Only one ordinary box beside a box whose width overflows: still found outside the others.
    const BoxIndex two(std::vector<Box2>{Box2{{0, 0}, {1, 1}}, Box2{{-big, 5}, {big, 6}}});
    CHECK(two.containing({1e300, 5.5}) == std::vector<std::size_t>{1});
}

TEST_CASE("AC3: indexed ClearanceField equals the brute-force minimum, with exclusions",
          "[toolpath][index][T-0123]") {
    SplitMix rng{0x7123'0002ULL};
    std::vector<Contour> obstacles;
    for (int i = 0; i < 300; ++i) {
        const Vec2 c{rng.uniform(0, 200), rng.uniform(0, 200)};
        obstacles.push_back(i % 2 == 0 ? Contour::circle(c, rng.uniform(0.5, 3.0))
                                       : rect(c.x, c.y, c.x + rng.uniform(0.5, 5), c.y + rng.uniform(0.5, 5)));
    }
    obstacles.push_back(rect(-20, -20, 220, 220));  // a frame round everything
    const ClearanceField field(obstacles);
    const std::vector<std::size_t> skip{3, 7, 300, 4000};  // out-of-range indices are ignored
    const ClearanceField fewer = field.excluding(skip);
    CHECK(field.size() == obstacles.size());
    CHECK(fewer.size() == obstacles.size() - 3);
    for (int q = 0; q < 500; ++q) {
        const Vec2 p{rng.uniform(-50, 250), rng.uniform(-50, 250)};
        double all = std::numeric_limits<double>::infinity();
        double some = all;
        for (std::size_t i = 0; i < obstacles.size(); ++i) {
            const double d = carvetto::geometry::distanceTo(obstacles[i], p);
            all = std::min(all, d);
            if (std::find(skip.begin(), skip.end(), i) == skip.end()) {
                some = std::min(some, d);
            }
        }
        CHECK(field.distance(p) == all);
        CHECK(fewer.distance(p) == some);
    }
    CHECK(ClearanceField{}.distance({0, 0}) == std::numeric_limits<double>::infinity());
    CHECK(ClearanceField({obstacles[0]}).excluding({0}).isEmpty());
}

// --------------------------------------------------------------------------------------- AC4

TEST_CASE("AC4: rest tools build their pattern only over the rest areas", "[toolpath][pocket][rest][T-0123]") {
    // 100 x 60 pocket, 12 mm clearance tool, 2 mm rest tool: rest material in the four corners.
    for (const PocketStrategy s : {PocketStrategy::Offset, PocketStrategy::Raster}) {
        INFO("strategy " << static_cast<int>(s));
        PocketParams p;
        p.cutDepth = 6.0;
        p.strategy = s;
        const PocketResult res = pocketToolpath(
            {rect(0, 0, 100, 60)}, {orc::endMill("B", 1, 12.0, 4.8, 3.0), orc::endMill("S", 2, 2.0, 0.8, 3.0)}, p);
        REQUIRE(res.diagnostics.empty());
        const PocketToolPlan& rest = res.plans[1];
        REQUIRE(rest.restMaterial.size() == 4);
        std::vector<orc::Poly> material;
        for (const Contour& c : rest.restMaterial) {
            material.push_back(c.polygonise(Tolerance::defaults()).points);
        }
        // One cluster per corner; every path point lies where the tool disc reaches rest material
        // (the need region, thinned by at most the region tolerance) and nowhere else.
        CHECK(rest.paths.back().cluster == 3);
        for (const PocketPath& pp : rest.paths) {
            for (const Vec2 q : pp.path.points) {
                const bool inMaterial = orc::insideEvenOdd(material, q);
                CHECK((inMaterial || orc::boundaryDist(material, q) <= 1.0 + 0.01));
            }
        }
        const orc::GougeReport g = orc::gougeCheck(res.toolpaths[1], {orc::rectPoly(0, 0, 100, 60)}, 0.0, p.safeZ);
        CHECK(g.allInside);
        CHECK(g.minDistance >= 1.0);
    }
}

// --------------------------------------------------------------------------------------- AC5

TEST_CASE("AC5: raster pockets split by islands retract only between disconnected regions",
          "[toolpath][pocket][linking][T-0123]") {
    const std::vector<Contour> islands{rect(0, 0, 100, 40), rect(15, 10, 25, 30), rect(45, 10, 55, 30),
                                       rect(75, 10, 85, 30)};
    const std::vector<orc::Poly> walls{orc::rectPoly(0, 0, 100, 40), orc::rectPoly(15, 10, 25, 30),
                                       orc::rectPoly(45, 10, 55, 30), orc::rectPoly(75, 10, 85, 30)};
    for (const ProfilePass pass : {ProfilePass::First, ProfilePass::Last}) {
        for (const double angle : {0.0, 0.3, kPi / 2}) {
            INFO("pass " << static_cast<int>(pass) << " angle " << angle);
            PocketParams p;
            p.cutDepth = 6.0;
            p.strategy = PocketStrategy::Raster;
            p.profilePass = pass;
            p.rasterAngle = angle;
            const PocketResult res = pocketToolpath(islands, {orc::endMill("A", 1, 6.0, 2.4, 3.0)}, p);
            const Toolpath& tp = res.toolpaths[0];
            REQUIRE_FALSE(tp.isEmpty());
            CHECK(tp.count(MoveKind::Retract) == 1);  // one connected region: one retract (at the end)
            const orc::GougeReport g = orc::gougeCheck(tp, walls, 0.0, p.safeZ);
            CHECK(g.allInside);
            CHECK(g.minDistance >= 3.0);
            // Travel lifts only to the floor of the previous level (or the surface).
            for (std::size_t i = 0; i < tp.moves.size(); ++i) {
                const Move& mv = tp.moves[i];
                if (mv.kind == MoveKind::Feed && mv.to.z > tp.startOf(i).z) {
                    CHECK((mv.to.z == 0.0 || mv.to.z == -3.0));
                }
            }
        }
    }
    // Two separate pockets: one retract each.
    PocketParams p;
    p.cutDepth = 6.0;
    p.strategy = PocketStrategy::Raster;
    const PocketResult two =
        pocketToolpath({rect(0, 0, 40, 40), rect(60, 0, 100, 40)}, {orc::endMill("A", 1, 6.0, 2.4, 3.0)}, p);
    CHECK(two.toolpaths[0].count(MoveKind::Retract) == 2);
    // Without a boundary pass the floor keeps wall scallops: such toolpaths still never travel.
    p.profilePass = ProfilePass::None;
    const PocketResult scalloped = pocketToolpath(islands, {orc::endMill("A", 1, 6.0, 2.4, 3.0)}, p);
    CHECK(scalloped.toolpaths[0].count(MoveKind::Retract) > 1);
}

TEST_CASE("AC5: rest areas are cut through all levels each and retract between areas",
          "[toolpath][pocket][linking][rest][T-0123]") {
    const PocketTool big = orc::endMill("B", 1, 12.0, 4.8, 3.0);
    const PocketTool small = orc::endMill("S", 2, 2.0, 0.8, 3.0);
    for (const PocketStrategy s : {PocketStrategy::Offset, PocketStrategy::Raster}) {
        for (const bool ramp : {false, true}) {
            INFO("strategy " << static_cast<int>(s) << " ramp " << ramp);
            PocketParams p;
            p.cutDepth = 6.0;
            p.startDepth = 1.0;
            p.strategy = s;
            p.rampPlunge = ramp;
            p.rampDistance = 3.0;
            // Four corners of one pocket: owner decision 2026-11-01, the tool retracts to safe Z
            // between rest areas (even within one connected region) and only there: each corner
            // is cut through both levels without a retract.
            const PocketResult corners = pocketToolpath({rect(0, 0, 100, 60)}, {big, small}, p);
            REQUIRE(corners.diagnostics.empty());
            const Toolpath& rest = corners.toolpaths[1];
            CHECK(corners.plans[1].paths.back().cluster == 3);
            CHECK(corners.plans[1].levels.size() == 2);
            CHECK(rest.count(MoveKind::Retract) == 4);
            CHECK(rest.count(MoveKind::Rapid) == 4);
            const orc::GougeReport g = orc::gougeCheck(rest, {orc::rectPoly(0, 0, 100, 60)}, -1.0, p.safeZ);
            CHECK(g.allInside);
            CHECK(g.minDistance >= 1.0);
            // Every rapid between areas starts from safe Z, right after a retract.
            for (std::size_t i = 1; i < rest.moves.size(); ++i) {
                if (rest.moves[i].kind == MoveKind::Rapid) {
                    CHECK(rest.moves[i - 1].kind == MoveKind::Retract);
                }
            }
            // Two separate pockets (four corners each): one retract per rest area.
            const PocketResult two = pocketToolpath({rect(0, 0, 40, 40), rect(60, 0, 100, 40)}, {big, small}, p);
            CHECK(two.plans[1].paths.back().cluster == 7);
            CHECK(two.toolpaths[1].count(MoveKind::Retract) == 8);
            const orc::GougeReport g2 = orc::gougeCheck(
                two.toolpaths[1], {orc::rectPoly(0, 0, 40, 40), orc::rectPoly(60, 0, 100, 40)}, -1.0, p.safeZ);
            CHECK(g2.allInside);
            CHECK(g2.minDistance >= 1.0);
        }
    }
}
