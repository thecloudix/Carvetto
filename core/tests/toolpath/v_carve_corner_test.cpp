// T-0160: V-Carve corner sharpening (ADR-0025 §6). With a flat depth and clearance end mills, each
// convex corner of a flat area keeps a sloped floor between the wall ring's corner and the end
// mill's fillet; the carving tool clears it with a groove along the bisector and passes either
// side at its stepover, so the corner is no rougher than the rest of the floor. Checked here:
//  - the leftover in every sharpened corner, by the exact oracle (v_carve_leftover.hpp), is no
//    higher than the floor's ridge height h(s/2) for the same stepover, and the cone never
//    crosses a wall (v_carve_oracle.hpp);
//  - detection from the boundary (convex junctions, flat-area corners, an end mill reaching in,
//    the automatic threshold: predicted leftover above the chordal tolerance);
//  - output without a flat depth or without clearance tools (or switched off) is unchanged, and
//    otherwise only complete corner cuts at the flat depth are added.

#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/transform2.hpp"
#include "carvetto/toolpath/move.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/v_carve.hpp"
#include "v_carve_corners.hpp"
#include "v_carve_leftover.hpp"
#include "v_carve_oracle.hpp"
#include "v_carve_test_util.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using geometry::Box2;
using geometry::Contour;
using geometry::Vec2;
namespace orc = carvetto::test::v_carve;

namespace {

constexpr double kSafeZ = 5.0;

struct Job {
    std::string what;
    std::vector<Contour> region;
    tooling::Tool tool;
    std::vector<PocketTool> clearance;
    VCarvingParams params;
    std::vector<Vec2> corners;  // convex corners whose floor is measured
};

double limitOf(const Job& job) {
    return std::min(job.params.flatDepth, vCarvingFullWidthDepth(job.tool));
}

/// The floor's ridge height for the carving tool's stepover (clamped as for its rest clearing).
double ridgeOf(const Job& job) {
    const double rho = orc::footprintAt(job.tool, limitOf(job));
    const double s = std::min(job.tool.defaultCuttingData.stepover, 2.0 * rho);
    return vCarvingDepthForRadius(job.tool, 0.5 * s);
}

struct Leftover {
    double worst{0.0};
    Vec2 at{};
    std::size_t points{0};
};

/// Highest material left on the floor (points at least ρ + Pocket's chordal from every wall)
/// within `radius` of `corner`, by the exact oracle over every toolpath of `r`, on a grid.
Leftover floorLeftover(const Job& job, const VCarvingResult& r, Vec2 corner, double radius, double step) {
    const VCarvingParams& p = job.params;
    const double limit = limitOf(job);
    const double floorFrom = orc::footprintAt(job.tool, limit) + p.tolerance.chordal / 3.0 + 1e-6;
    std::vector<orc::CarvedSurface> surfaces{orc::CarvedSurface(r.toolpath, job.tool, p.startDepth)};
    for (std::size_t t = 0; t < r.clearance.size(); ++t) {
        surfaces.emplace_back(r.clearance[t], job.clearance[t].tool, p.startDepth);
    }
    Leftover out;
    const auto n = static_cast<int>(std::ceil(radius / step));
    for (int i = -n; i <= n; ++i) {
        for (int k = -n; k <= n; ++k) {
            const Vec2 q{corner.x + i * step, corner.y + k * step};
            if (geometry::distance(q, corner) > radius || orc::boundaryDistance(job.region, q) < floorFrom ||
                !orc::insideRegion(job.region, q)) {
                continue;
            }
            ++out.points;
            double c = 0.0;
            for (const orc::CarvedSurface& s : surfaces) {
                c = std::max(c, s.depthAt(q));
            }
            if (limit - c > out.worst) {
                out.worst = limit - c;
                out.at = q;
            }
        }
    }
    return out;
}

VCarvingResult run(const Job& job, bool sharpen) {
    VCarvingParams p = job.params;
    p.cornerSharpen = sharpen;
    return vCarvingToolpath(job.region, job.tool, std::nullopt, job.clearance, p);
}

/// The corner-sharpening inputs vCarvingToolpath derives for `job`, with the wall rings it cuts.
struct CornerSetup {
    detail::CornerSpec spec;
    std::vector<std::vector<Vec2>> rings;
};

CornerSetup cornerSetup(const Job& job) {
    CornerSetup out;
    const VCarvingParams& p = job.params;
    const double limit = limitOf(job);
    const double rho = orc::footprintAt(job.tool, limit);
    const double inner = p.tolerance.chordal / 3.0;
    out.spec.tool = &job.tool;
    out.spec.limit = limit;
    out.spec.limitRadius = rho;
    out.spec.pocketChordal = inner;
    out.spec.ringInset = rho + inner;
    for (const PocketTool& t : job.clearance) {
        out.spec.millRadii.push_back(0.5 * t.tool.diameter);
    }
    out.spec.stepover = std::min(job.tool.defaultCuttingData.stepover, 2.0 * rho);
    out.spec.chordal = p.tolerance.chordal;
    out.spec.distance = p.tolerance.distance;
    PocketParams walls;
    walls.tolerance = p.tolerance;
    walls.tolerance.chordal = inner;
    walls.tolerance.distance = std::min(p.tolerance.distance, inner);
    for (const Contour& ring : pocketCentreRegion(job.region, 2.0 * rho, walls)) {
        out.rings.emplace_back();
        for (const geometry::Span& sp : ring.spans) {
            out.rings.back().push_back(geometry::startPoint(sp));
        }
    }
    return out;
}

std::vector<detail::SharpCorner> cornersOf(const Job& job) {
    const CornerSetup setup = cornerSetup(job);
    const detail::RingRegion walls(setup.rings, 1e-9 * 100.0);
    return detail::sharpCorners(job.region, walls, setup.spec);
}

Job squareJob() {
    Job j;
    j.what = "square, 90-degree V-bit, 6 mm end mill";
    j.region = {Contour::rectangle(Box2{Vec2{0, 0}, Vec2{20, 20}})};
    j.tool = orc::vBit(90.0, 12.7, "V 90");
    j.clearance = {{orc::endMill(6.0, "EM 6", 4), std::nullopt}};
    j.params.flatDepth = 1.0;
    j.params.tolerance.chordal = 0.05;
    j.corners = {{0, 0}, {20, 0}, {20, 20}, {0, 20}};
    return j;
}

/// True when `with` is `without` plus inserted complete cuts (rapid at safe Z, plunge, feeds at
/// the flat depth, retract): corner sharpening adds moves and changes nothing else.
bool onlyCornerCutsAdded(const Toolpath& with, const Toolpath& without, double flatZ, std::size_t& added) {
    std::size_t j = 0;
    added = 0;
    std::size_t i = 0;
    while (i < with.moves.size()) {
        if (j < without.moves.size() && with.moves[i] == without.moves[j]) {
            ++i;
            ++j;
            continue;
        }
        // An inserted cut.
        if (with.moves[i].kind != MoveKind::Rapid || with.moves[i].to.z != kSafeZ) {
            return false;
        }
        ++i;
        if (i >= with.moves.size() || with.moves[i].kind != MoveKind::Plunge || with.moves[i].to.z != flatZ) {
            return false;
        }
        ++i;
        while (i < with.moves.size() && with.moves[i].kind == MoveKind::Feed) {
            if (with.moves[i].to.z != flatZ) {
                return false;
            }
            ++i;
        }
        if (i >= with.moves.size() || with.moves[i].kind != MoveKind::Retract || with.moves[i].to.z != kSafeZ) {
            return false;
        }
        ++i;
        ++added;
    }
    return j == without.moves.size();
}

}  // namespace

TEST_CASE("T-0160: predicted corner leftover, min(F, h(R (1 - sin(a/2))))", "[toolpath][v_carve][corner]") {
    const Job j = squareJob();
    CornerSetup s = cornerSetup(j);
    // The card's example: 90-degree corner, 6 mm end mill, 90-degree V-bit: 3 (1 - sin 45°) = 0.879.
    CHECK(std::fabs(detail::predictedCornerLeftover(s.spec, geometry::kPi / 2, 3.0) - 3.0 * (1.0 - std::sqrt(0.5))) <
          1e-12);
    // Never above the flat depth.
    s.spec.limit = 0.5;
    CHECK(detail::predictedCornerLeftover(s.spec, geometry::kPi / 2, 3.0) == 0.5);
    // A ball nose: its height above the tip at that distance.
    const tooling::Tool ball = orc::ballNose(6.0);
    s.spec.tool = &ball;
    s.spec.limit = 1.0;
    const double d = 2.0 * (1.0 - std::sin(geometry::kPi / 6));  // 60-degree corner, 4 mm end mill
    CHECK(std::fabs(detail::predictedCornerLeftover(s.spec, geometry::kPi / 3, 2.0) - (3.0 - std::sqrt(9.0 - d * d))) <
          1e-12);
}

TEST_CASE("T-0160: corners are found from the boundary: convex flat-area corners an end mill reaches",
          "[toolpath][v_carve][corner]") {
    SECTION("a square: its four corners, bisectors into the region") {
        const std::vector<detail::SharpCorner> cs = cornersOf(squareJob());
        REQUIRE(cs.size() == 4);
        for (const detail::SharpCorner& c : cs) {
            CHECK(std::fabs(c.angle - geometry::kPi / 2) < 1e-12);
            CHECK(c.millRadius == 3.0);
            CHECK(std::fabs(c.predicted - 3.0 * (1.0 - std::sqrt(0.5))) < 1e-12);
            const Vec2 inward = Vec2{10, 10} - c.vertex;
            CHECK(std::fabs(c.bisector.dot(inward) / inward.length() - 1.0) < 1e-12);
        }
    }
    SECTION("an L: five convex corners, the reflex one is not a corner; the same for a reversed outline") {
        Job j = squareJob();
        j.region = {orc::polygon({{0, 0}, {30, 0}, {30, 12}, {12, 12}, {12, 30}, {0, 30}})};
        CHECK(cornersOf(j).size() == 5);
        j.region.front().reverse();
        const std::vector<detail::SharpCorner> cs = cornersOf(j);
        REQUIRE(cs.size() == 5);
        for (const detail::SharpCorner& c : cs) {
            CHECK_FALSE(c.vertex == Vec2{12, 12});
        }
    }
    SECTION("a hole: its reflex vertex is a convex corner of the region, its convex ones are not") {
        Job j = squareJob();
        Contour hole = orc::polygon({{10, 10}, {20, 10}, {20, 14}, {14, 14}, {14, 20}, {10, 20}});
        hole.reverse();
        j.region = {Contour::rectangle(Box2{Vec2{0, 0}, Vec2{30, 30}}), hole};
        const std::vector<detail::SharpCorner> cs = cornersOf(j);
        REQUIRE(cs.size() == 5);
        CHECK(cs.back().vertex == Vec2{14, 14});
        CHECK(std::fabs(cs.back().bisector.x - std::sqrt(0.5)) < 1e-12);
        CHECK(std::fabs(cs.back().bisector.y - std::sqrt(0.5)) < 1e-12);
    }
    SECTION("the automatic threshold: sharpened exactly when the predicted leftover exceeds chordal") {
        // A hexagon (120-degree corners), 90-degree V-bit: R (1 - sin 60°) against chordal 0.05,
        // i.e. R = 0.3732 at the threshold.
        Job j = squareJob();
        std::vector<Vec2> pts;
        for (int i = 0; i < 6; ++i) {
            pts.push_back(Vec2::fromAngle(geometry::kPi / 3 * i, 10.0));
        }
        j.region = {orc::polygon(pts)};
        j.clearance = {{orc::endMill(0.74, "EM 0.74", 4), std::nullopt}};
        CHECK(cornersOf(j).empty());
        j.clearance = {{orc::endMill(0.76, "EM 0.76", 4), std::nullopt}};
        CHECK(cornersOf(j).size() == 6);
        // Below the threshold the output is exactly that without sharpening.
        j.clearance = {{orc::endMill(0.74, "EM 0.74", 4), std::nullopt}};
        CHECK(run(j, true).toolpath.moves == run(j, false).toolpath.moves);
        j.clearance = {{orc::endMill(0.76, "EM 0.76", 4), std::nullopt}};
        CHECK(run(j, true).toolpath.moves.size() > run(j, false).toolpath.moves.size());
    }
    SECTION("the smallest end mill reaching in bounds the leftover; none reaching, no sharpening") {
        Job j = squareJob();
        j.clearance = {{orc::endMill(6.0, "EM 6", 4), std::nullopt}, {orc::endMill(2.0, "EM 2", 5), std::nullopt}};
        std::vector<detail::SharpCorner> cs = cornersOf(j);
        REQUIRE(cs.size() == 4);
        CHECK(cs.front().millRadius == 1.0);
        // A 5 mm square: the 6 mm end mill's centre cannot reach any corner (4.02 mm from both
        // walls), so the floor there is the carving tool's own rest clearing: left alone.
        j.region = {Contour::rectangle(Box2{Vec2{0, 0}, Vec2{5, 5}})};
        j.clearance = {{orc::endMill(6.0, "EM 6", 4), std::nullopt}};
        CHECK(cornersOf(j).empty());
        CHECK(run(j, true).toolpath.moves == run(j, false).toolpath.moves);
    }
    SECTION("rounded corners have no junction: not corners") {
        Job j = squareJob();
        j.region = {orc::roundedRect(Box2{Vec2{0, 0}, Vec2{20, 20}}, 0.5)};
        CHECK(cornersOf(j).empty());
    }
}

TEST_CASE("T-0160: sharpened corners keep no more than the floor's ridge height (exact oracle); the cone stays inside",
          "[toolpath][v_carve][corner][oracle]") {
    std::vector<Job> jobs;
    jobs.push_back(squareJob());
    {
        Job j;
        j.what = "triangle, 60-degree V-bit, 4 mm and 1 mm end mills";
        j.region = {orc::polygon({{0, 0}, {24, 0}, {12, 20.78}})};
        j.tool = orc::vBit(60.0, 12.7, "V 60");
        j.clearance = {{orc::endMill(4.0, "EM 4", 4), std::nullopt}, {orc::endMill(1.0, "EM 1", 5), std::nullopt}};
        j.params.flatDepth = 1.5;
        j.params.tolerance.chordal = 0.02;
        j.corners = {{0, 0}, {24, 0}};
        jobs.push_back(j);
    }
    {
        Job j;
        j.what = "30-degree wedge, 6 mm ball nose, 3 mm end mill";
        j.region = {orc::polygon({{0, 0}, {30, -8.04}, {30, 8.04}})};
        j.tool = orc::ballNose(6.0, "Ball 6");
        j.clearance = {{orc::endMill(3.0, "EM 3", 4), std::nullopt}};
        j.params.flatDepth = 1.0;
        j.params.tolerance.chordal = 0.02;
        j.corners = {{30, 8.04}};
        jobs.push_back(j);
    }
    {
        Job j;
        j.what = "rotated square with an L-shaped hole, 120-degree V-bit, start depth, raster";
        const geometry::Transform2 rot = geometry::Transform2::rotation(0.3);
        Contour hole = orc::polygon({{10, 10}, {20, 10}, {20, 14}, {14, 14}, {14, 20}, {10, 20}});
        hole.reverse();
        j.region = {Contour::rectangle(Box2{Vec2{0, 0}, Vec2{30, 30}}).transformed(rot), hole.transformed(rot)};
        j.tool = orc::vBit(120.0, 20.0, "V 120");
        j.tool.defaultCuttingData.stepover = 0.6;
        j.clearance = {{orc::endMill(5.0, "EM 5", 4), std::nullopt}};
        j.params.flatDepth = 0.8;
        j.params.startDepth = 1.5;
        j.params.clearanceStrategy = PocketStrategy::Raster;
        j.params.clearanceRasterAngle = 0.2;
        j.params.tolerance.chordal = 0.05;
        j.corners = {rot.apply({0, 0}), rot.apply({14, 14})};
        jobs.push_back(j);
    }
    for (const Job& j : jobs) {
        INFO(j.what);
        const VCarvingResult r = run(j, true);
        REQUIRE(r.ok());
        CHECK(r.diagnostics.empty());
        const orc::ConeReport cone = orc::coneCheck(r.toolpath, j.region, j.tool, j.params.startDepth, kSafeZ, 1e-6);
        CHECK(cone.minClearance >= 0.0);
        CHECK(cone.allInside);
        CHECK(cone.rapidsAtSafeZ);
        const double ridge = ridgeOf(j);
        for (const Vec2 c : j.corners) {
            INFO("corner (" << c.x << ", " << c.y << "), floor ridge " << ridge);
            const Leftover l = floorLeftover(j, r, c, 6.0, 0.04);
            INFO("leftover " << l.worst << " at (" << l.at.x << ", " << l.at.y << ") over " << l.points << " points");
            CHECK(l.points > 1000);
            CHECK(l.worst <= ridge + 1e-9);
        }
    }
    // Without sharpening the oracle sees the corners' tents (0.49 mm against a 0.125 mm ridge).
    const Job sq = squareJob();
    const Leftover before = floorLeftover(sq, run(sq, false), {0, 0}, 6.0, 0.04);
    INFO("leftover without sharpening " << before.worst);
    CHECK(before.worst > 0.45);
}

TEST_CASE("T-0160: corner sharpening only adds corner cuts; without a flat depth or clearance tools nothing changes",
          "[toolpath][v_carve][corner]") {
    std::size_t sharpenedCases = 0;
    std::size_t addedCuts = 0;
    for (std::uint64_t k = 0; k < 60; ++k) {
        const orc::SweepCase sc = orc::sweepCase(k);
        INFO("case " << k << ": " << sc.what);
        VCarvingParams on = sc.params;
        on.cornerSharpen = true;
        VCarvingParams off = sc.params;
        off.cornerSharpen = false;
        const VCarvingResult a = vCarvingToolpath(sc.region, sc.tool, std::nullopt, sc.clearance, on);
        const VCarvingResult b = vCarvingToolpath(sc.region, sc.tool, std::nullopt, sc.clearance, off);
        REQUIRE(a.clearance.size() == b.clearance.size());
        for (std::size_t t = 0; t < a.clearance.size(); ++t) {
            CHECK(a.clearance[t].moves == b.clearance[t].moves);
        }
        CHECK(a.diagnostics == b.diagnostics);
        if (!(sc.params.flatDepth > 0.0) || sc.clearance.empty()) {
            CHECK(a.toolpath.moves == b.toolpath.moves);
            continue;
        }
        const double limit = std::min(sc.params.flatDepth, vCarvingFullWidthDepth(sc.tool));
        std::size_t added = 0;
        CHECK(onlyCornerCutsAdded(a.toolpath, b.toolpath, -(sc.params.startDepth + limit), added));
        if (added > 0) {
            ++sharpenedCases;
            addedCuts += added;
        }
    }
    INFO(sharpenedCases << " cases sharpened, " << addedCuts << " corner cuts");
    CHECK(sharpenedCases >= 3);
}

TEST_CASE("T-0160: corner sharpening is on by default", "[toolpath][v_carve][corner]") {
    CHECK(VCarvingParams{}.cornerSharpen);
    const Job j = squareJob();
    const VCarvingResult r = vCarvingToolpath(j.region, j.tool, std::nullopt, j.clearance, j.params);
    CHECK(r.toolpath.moves == run(j, true).toolpath.moves);
    CHECK(r.toolpath.moves.size() > run(j, false).toolpath.moves.size());
}
