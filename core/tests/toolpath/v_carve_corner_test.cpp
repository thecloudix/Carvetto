// T-0160: V-Carve corner sharpening (ADR-0025 §6). With a flat depth and clearance end mills, every
// convex stretch of a flat area's walls the end mills cannot reach into (sharp corners, small
// fillets and chamfers, junctions of curved walls) keeps a sloped floor between the wall ring and
// the end mills' swept area; the carving tool clears it with a groove along the stretch's bisector
// and passes either side at its stepover, so it is no rougher than the rest of the floor. Checked:
//  - the leftover in every sharpened stretch, by the exact oracle (v_carve_leftover.hpp), is no
//    higher than the floor's ridge height h(s/2) for the same stepover (acute corners at small
//    chordal tolerances, curved walls, rounded and chamfered corners), and the cone never crosses
//    a wall (v_carve_oracle.hpp), links at depth included;
//  - detection from the actual wall rings and end-mill moves (stretches next to the swept area,
//    the automatic threshold: predicted leftover above the chordal tolerance);
//  - the passes reach the chordal tolerance into the swept area;
//  - output without a flat depth or without clearance tools (or switched off) is unchanged, and
//    otherwise only complete corner cuts at the flat depth are added, every other move in place.

#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
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
    std::vector<Vec2> corners;  // convex corners (or the corners of rounded ones) whose floor is measured
    double radius{6.0};         // how far round each corner the floor is measured
};

double limitOf(const Job& job) {
    return std::min(job.params.flatDepth, vCarvingFullWidthDepth(job.tool));
}

double stepoverOf(const Job& job) {
    const double rho = orc::footprintAt(job.tool, limitOf(job));
    const double s = job.tool.defaultCuttingData.stepover;
    return s > 0.0 ? std::min(s, 2.0 * rho) : rho;
}

/// The floor's ridge height for the carving tool's stepover (clamped as for its rest clearing).
double ridgeOf(const Job& job) {
    return vCarvingDepthForRadius(job.tool, 0.5 * stepoverOf(job));
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

/// The corner-sharpening inputs vCarvingToolpath derives for `job`: the wall rings it cuts and
/// where the end mills swept the floor.
struct CornerSetup {
    detail::CornerSpec spec;
    std::vector<std::vector<Vec2>> rings;
    std::vector<detail::SweptPath> swept;
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
    out.spec.stepover = stepoverOf(job);
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
    // The end mills' level moves at the flat depth, from the toolpaths themselves.
    const VCarvingResult r = run(job, false);
    const double floorZ = -(p.startDepth + limit);
    for (std::size_t t = 0; t < r.clearance.size(); ++t) {
        detail::SweptPath cur;
        cur.radius = 0.5 * job.clearance[t].tool.diameter;
        const std::vector<Move>& m = r.clearance[t].moves;
        for (std::size_t i = 1; i <= m.size(); ++i) {
            const bool level = i < m.size() && m[i].kind == MoveKind::Feed &&
                               std::fabs(m[i - 1].to.z - floorZ) < 1e-9 && std::fabs(m[i].to.z - floorZ) < 1e-9;
            if (level) {
                if (cur.points.empty()) {
                    cur.points.push_back(Vec2{m[i - 1].to.x, m[i - 1].to.y});
                }
                cur.points.push_back(Vec2{m[i].to.x, m[i].to.y});
            } else if (!cur.points.empty()) {
                out.swept.push_back(cur);
                cur.points.clear();
            }
        }
    }
    return out;
}

std::vector<detail::SharpStretch> stretchesOf(const Job& job) {
    const CornerSetup setup = cornerSetup(job);
    const detail::RingRegion walls(setup.rings, 1e-10);
    return detail::sharpStretches(walls, setup.swept, setup.spec);
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

/// A D: the left side a line, the right a half circle of radius 10 (90-degree junctions of a line
/// and a convex arc), counter-clockwise.
Contour dShape() {
    Contour c;
    c.closed = true;
    c.append(geometry::ArcSpan{{0, 0}, 10.0, -geometry::kPi / 2, geometry::kPi / 2, geometry::ArcDirection::CCW});
    c.append(geometry::LineSpan{{0, 10}, {0, -10}});
    return c;
}

/// A triangle whose sides are concave arcs bowing 2.5 mm into it: cusps of about 13 degrees.
Contour concaveTriangle() {
    Contour c;
    c.closed = true;
    std::vector<Vec2> t;
    for (int i = 0; i < 3; ++i) {
        t.push_back(Vec2::fromAngle(geometry::kPi / 2 + geometry::kTwoPi * i / 3, 14.0));
    }
    for (std::size_t i = 0; i < 3; ++i) {
        const Vec2 a = t[i];
        const Vec2 e = t[(i + 1) % 3];
        const Vec2 m = (a + e) * 0.5;
        const Vec2 out = m / m.length();
        const double sag = 2.5;
        const double l = geometry::distance(a, e) / 2;
        const double r = (l * l + sag * sag) / (2 * sag);
        const Vec2 o = m + out * (r - sag);
        c.append(geometry::ArcSpan{o, r, std::atan2(a.y - o.y, a.x - o.x), std::atan2(e.y - o.y, e.x - o.x),
                                   geometry::ArcDirection::CW});
    }
    return c;
}

/// A square of side 20 at the origin with every corner chamfered by `ch` (counter-clockwise).
Contour chamferedSquare(double ch) {
    const double d = ch / std::sqrt(2.0);
    return orc::polygon({{d, 0}, {20 - d, 0}, {20, d}, {20, 20 - d}, {20 - d, 20}, {d, 20}, {0, 20 - d}, {0, d}});
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

/// The feed moves the corner cuts add (start and end), in order.
std::vector<std::pair<Vec2, Vec2>> addedFeeds(const Toolpath& with, const Toolpath& without) {
    std::vector<std::pair<Vec2, Vec2>> out;
    std::size_t j = 0;
    for (std::size_t i = 0; i < with.moves.size(); ++i) {
        if (j < without.moves.size() && with.moves[i] == without.moves[j]) {
            ++j;
            continue;
        }
        if (with.moves[i].kind == MoveKind::Feed && i > 0) {
            const geometry::Vec3 a = with.moves[i - 1].to;
            const geometry::Vec3 b = with.moves[i].to;
            out.emplace_back(Vec2{a.x, a.y}, Vec2{b.x, b.y});
        }
    }
    return out;
}

}  // namespace

TEST_CASE("T-0160: stretches are found from the wall rings and the end mills' swept area",
          "[toolpath][v_carve][corner]") {
    SECTION("a square: one stretch per corner, along its bisector, predicted from the end mill's fillet") {
        const Job j = squareJob();
        const std::vector<detail::SharpStretch> cs = stretchesOf(j);
        REQUIRE(cs.size() == 4);
        const double rho = 1.0;
        const double c = 0.05 / 3.0;
        const double delta = 0.05;  // the V-bit's footprint at depth chordal (below half the stepover)
        for (const detail::SharpStretch& s : cs) {
            // The corner it belongs to.
            const Vec2 v{s.root.x < 10 ? 0.0 : 20.0, s.root.y < 10 ? 0.0 : 20.0};
            const Vec2 inward = Vec2{10, 10} - v;
            INFO("corner (" << v.x << ", " << v.y << ")");
            CHECK(s.direction.dot(inward) / inward.length() > 1.0 - 1e-6);
            // The root: the wall ring's corner moved δ off both rings, on the bisector.
            const Vec2 root = v + inward / inward.length() * ((rho + c + delta) * std::sqrt(2.0));
            CHECK(geometry::distance(s.root, root) < 1e-4);
            // The fillet's point on the bisector is R (1 − sin 45°) from the ring's edges.
            CHECK(std::fabs(s.farthest - 3.0 * (1.0 - std::sqrt(0.5))) < 2e-3);
            CHECK(std::fabs(s.predicted - s.farthest) < 1e-12);  // a 90-degree V-bit: h(d) = d
        }
    }
    SECTION("an L: five convex corners, not the reflex one; the same for a reversed outline") {
        Job j = squareJob();
        j.region = {orc::polygon({{0, 0}, {30, 0}, {30, 12}, {12, 12}, {12, 30}, {0, 30}})};
        CHECK(stretchesOf(j).size() == 5);
        j.region.front().reverse();
        const std::vector<detail::SharpStretch> cs = stretchesOf(j);
        REQUIRE(cs.size() == 5);
        for (const detail::SharpStretch& s : cs) {
            CHECK(geometry::distance(s.root, Vec2{12, 12}) > 3.0);
        }
    }
    SECTION("a hole: its reflex vertex is a convex corner of the region, its convex ones are not") {
        Job j = squareJob();
        Contour hole = orc::polygon({{10, 10}, {20, 10}, {20, 14}, {14, 14}, {14, 20}, {10, 20}});
        hole.reverse();
        j.region = {Contour::rectangle(Box2{Vec2{0, 0}, Vec2{30, 30}}), hole};
        const std::vector<detail::SharpStretch> cs = stretchesOf(j);
        REQUIRE(cs.size() == 5);
        std::size_t atHole = 0;
        for (const detail::SharpStretch& s : cs) {
            if (geometry::distance(s.root, Vec2{14, 14}) < 2.0) {
                ++atHole;
                CHECK(s.direction.x > std::sqrt(0.5) - 1e-4);
                CHECK(s.direction.y > std::sqrt(0.5) - 1e-4);
            }
        }
        CHECK(atHole == 1);
    }
    SECTION("the automatic threshold: sharpened exactly when the predicted leftover exceeds chordal") {
        // A hexagon (120-degree corners), 90-degree V-bit, stepover 0.04 (so δ = 0.02 and the
        // threshold, not δ, decides): R (1 − sin 60°) against chordal 0.05, R = 0.3732 at the
        // threshold.
        Job j = squareJob();
        std::vector<Vec2> pts;
        for (int i = 0; i < 6; ++i) {
            pts.push_back(Vec2::fromAngle(geometry::kPi / 3 * i, 10.0));
        }
        j.region = {orc::polygon(pts)};
        j.tool.defaultCuttingData.stepover = 0.04;
        j.clearance = {{orc::endMill(0.73, "EM 0.73", 4), std::nullopt}};
        CHECK(stretchesOf(j).empty());
        CHECK(run(j, true).toolpath.moves == run(j, false).toolpath.moves);
        j.clearance = {{orc::endMill(0.77, "EM 0.77", 4), std::nullopt}};
        const std::vector<detail::SharpStretch> cs = stretchesOf(j);
        REQUIRE(cs.size() == 6);
        for (const detail::SharpStretch& s : cs) {
            CHECK(s.predicted > 0.05);
            CHECK(s.predicted < 0.0525);
        }
        CHECK(run(j, true).toolpath.moves.size() > run(j, false).toolpath.moves.size());
        // A pentagon with two 90-degree and three 120-degree corners and a 0.6 mm end mill: the
        // right-angled corners keep R (1 − sin 45°) = 0.088 > chordal and are sharpened; the
        // others keep 0.040 (patches farther than δ but not than r(chordal)) and are left alone.
        j.region = {orc::polygon({{0, 0}, {10, 0}, {15, 8.660254}, {10, 17.320508}, {0, 17.320508}})};
        j.clearance = {{orc::endMill(0.6, "EM 0.6", 4), std::nullopt}};
        const std::vector<detail::SharpStretch> mixed = stretchesOf(j);
        REQUIRE(mixed.size() == 2);
        for (const detail::SharpStretch& s : mixed) {
            CHECK(s.root.x < 2.0);
            CHECK(s.predicted > 0.05);
        }
    }
    SECTION("rounded and chamfered corners the end mill cannot reach into are stretches too") {
        Job j = squareJob();
        for (const double r : {0.01, 0.5, 1.6}) {
            INFO("fillet " << r);
            j.region = {orc::roundedRect(Box2{Vec2{0, 0}, Vec2{20, 20}}, r)};
            CHECK(stretchesOf(j).size() == 4);
        }
        for (const double ch : {0.0005, 0.0015, 0.2}) {
            INFO("chamfer " << ch);
            j.region = {chamferedSquare(ch)};
            CHECK(stretchesOf(j).size() == 4);
        }
        // A fillet the end mill fits (radius ≥ R + ρ + c): nothing left in the corners.
        j.region = {orc::roundedRect(Box2{Vec2{0, 0}, Vec2{20, 20}}, 4.1)};
        CHECK(stretchesOf(j).empty());
    }
    SECTION("curved walls: a line meeting an arc, cusps between concave arcs") {
        Job j = squareJob();
        j.region = {dShape()};
        CHECK(stretchesOf(j).size() == 2);
        j.region = {concaveTriangle()};
        j.clearance = {{orc::endMill(3.0, "EM 3", 4), std::nullopt}};
        CHECK(stretchesOf(j).size() == 3);
    }
    SECTION("floor no end mill reaches is the carving tool's own rest clearing: left alone") {
        // A 5 mm square: the 6 mm end mill fits nowhere.
        Job j = squareJob();
        j.region = {Contour::rectangle(Box2{Vec2{0, 0}, Vec2{5, 5}})};
        CHECK(stretchesOf(j).empty());
        CHECK(run(j, true).toolpath.moves == run(j, false).toolpath.moves);
        // A 20 mm square with a 5 mm lobe behind a 1 mm neck: the lobe's floor is a flat area of
        // its own that the end mill never enters; the square's four corners are stretches (and
        // the spike its wall ring makes at the neck's mouth), the lobe's corners are not.
        j.region = {orc::polygon({{0, 0}, {20, 0}, {20, 9.5}, {24, 9.5}, {24, 7.5}, {29, 7.5}, {29, 12.5}, {24, 12.5},
                                  {24, 10.5}, {20, 10.5}, {20, 20}, {0, 20}})};
        std::size_t inSquare = 0;
        for (const detail::SharpStretch& s : stretchesOf(j)) {
            CHECK(s.root.x < 20.0);
            if (s.root.x < 10.0 || s.root.y < 5.0 || s.root.y > 15.0) {
                ++inSquare;
            }
        }
        CHECK(inSquare == 4);
    }
}

TEST_CASE("T-0160: sharpened stretches keep no more than the floor's ridge height (exact oracle); the cone stays "
          "inside",
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
    for (const double apex : {30.0, 45.0}) {
        // Acute corners at a small chordal tolerance: the end mill's fillet is steep between the
        // outer passes (round 1 left up to 1.18 × the ridge here).
        Job j;
        j.what = std::to_string(static_cast<int>(apex)) + "-degree corner, 60-degree V-bit, chordal 0.002";
        const double h = 20.0;
        const double hw = h * std::tan(apex * geometry::kPi / 360.0);
        j.region = {orc::polygon({{0, 0}, {h, -hw}, {h + 4, -hw}, {h + 4, hw}, {h, hw}})};
        j.tool = orc::vBit(60.0, 12.7, "V 60");
        j.tool.defaultCuttingData.stepover = 0.3;
        j.clearance = {{orc::endMill(6.0, "EM 6", 4), std::nullopt}};
        j.params.flatDepth = 1.0;
        j.params.tolerance.chordal = 0.002;
        j.corners = {{0, 0}};
        j.radius = 13.0;
        jobs.push_back(j);
    }
    {
        Job j = squareJob();
        j.what = "D: a line meeting a convex arc";
        j.region = {dShape()};
        j.corners = {{0, 10}, {0, -10}};
        jobs.push_back(j);
    }
    {
        Job j = squareJob();
        j.what = "triangle of concave arcs, 3 mm end mill";
        j.region = {concaveTriangle()};
        j.clearance = {{orc::endMill(3.0, "EM 3", 4), std::nullopt}};
        j.corners = {Vec2::fromAngle(geometry::kPi / 2, 14.0)};
        j.radius = 9.0;
        jobs.push_back(j);
    }
    {
        Job j = squareJob();
        j.what = "square with 0.5 mm fillets";
        j.region = {orc::roundedRect(Box2{Vec2{0, 0}, Vec2{20, 20}}, 0.5)};
        j.corners = {{0, 0}, {20, 20}};
        jobs.push_back(j);
    }
    {
        Job j = squareJob();
        j.what = "square with 0.2 mm chamfers";
        j.region = {chamferedSquare(0.2)};
        j.corners = {{20, 0}};
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
        const VCarvingResult off = run(j, false);
        for (const Vec2 c : j.corners) {
            INFO("corner (" << c.x << ", " << c.y << "), floor ridge " << ridge);
            const Leftover l = floorLeftover(j, r, c, j.radius, 0.04);
            INFO("leftover " << l.worst << " at (" << l.at.x << ", " << l.at.y << ") over " << l.points << " points");
            CHECK(l.points > 1000);
            CHECK(l.worst <= ridge + 1e-6);
            // Without sharpening the same floor keeps more than the ridge (a coarser grid will do).
            const Leftover before = floorLeftover(j, off, c, j.radius, 0.1);
            INFO("without sharpening " << before.worst);
            CHECK(before.worst > ridge);
        }
    }
}

TEST_CASE("T-0160: the passes reach the chordal tolerance into the end mill's swept area",
          "[toolpath][v_carve][corner]") {
    const Job j = squareJob();
    const VCarvingResult on = run(j, true);
    const VCarvingResult off = run(j, false);
    // The end mill's centre stops at the corner of its centre region, inset by R + ρ + c.
    const double inset = 3.0 + 1.0 + 0.05 / 3.0;
    const Vec2 centre{inset, inset};
    // The groove along the corner's bisector: the farthest of its points from the corner.
    double far = -1.0;
    Vec2 end{};
    for (const auto& [a, b] : addedFeeds(on.toolpath, off.toolpath)) {
        for (const Vec2 p : {a, b}) {
            if (std::fabs(p.x - p.y) < 1e-3 && p.x < 10.0 && p.x + p.y > far) {
                far = p.x + p.y;
                end = p;
            }
        }
    }
    REQUIRE(far > 0.0);
    INFO("groove ends at (" << end.x << ", " << end.y << ")");
    CHECK(geometry::distance(end, centre) <= 3.0 - 0.05 + 1e-6);
}

TEST_CASE("T-0160: corner cuts join at depth only inside the wall rings", "[toolpath][v_carve][corner]") {
    // A 30-degree corner the 6 mm end mill reaches, with small holes in its tent: the passes are
    // split by the holes' wall rings, and the links between them must not cross the holes.
    for (const double hy : {0.0, 0.15}) {
        INFO("hole at y " << hy);
        Job j = squareJob();
        const double h = 60.0;
        const double hw = h * std::tan(15.0 * geometry::kPi / 180.0);
        Contour hole = orc::polygon({{8.0, hy}, {8.1, hy}, {8.1, hy + 0.1}, {8.0, hy + 0.1}});
        hole.reverse();
        j.region = {orc::polygon({{0, 0}, {h, -hw}, {h, hw}}), hole};
        const VCarvingResult on = run(j, true);
        const VCarvingResult off = run(j, false);
        REQUIRE(on.ok());
        std::size_t added = 0;
        CHECK(onlyCornerCutsAdded(on.toolpath, off.toolpath, -1.0, added));
        CHECK(added >= 2);
        const orc::ConeReport cone = orc::coneCheck(on.toolpath, j.region, j.tool, 0.0, kSafeZ, 1e-6);
        CHECK(cone.minClearance >= 0.0);
        CHECK(cone.allInside);
        // Every added feed keeps the carving tool's centre ρ + c/2 from the walls (Pocket's margin).
        const double margin = 1.0 + 0.5 * 0.05 / 3.0;
        for (const auto& [a, b] : addedFeeds(on.toolpath, off.toolpath)) {
            for (int k = 0; k <= 64; ++k) {
                const Vec2 p = a + (b - a) * (k / 64.0);
                CHECK(orc::boundaryDistance(j.region, p) >= margin - 1e-9);
            }
        }
    }
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

TEST_CASE("T-0160: the next region's cuts keep their order after a region's corner cuts",
          "[toolpath][v_carve][corner]") {
    // A square with a flat area, then a plus sign too narrow for one: the square's corner cuts
    // leave the tool elsewhere than its own last cut, and the plus's two cuts must still come in
    // the order they have without sharpening (round 1 ordered them from where the corner cuts
    // ended).
    for (const Vec2 centre : {Vec2{26, 10}, Vec2{30, -4}, Vec2{26, 2}}) {
        INFO("plus at (" << centre.x << ", " << centre.y << ")");
        Job j = squareJob();
        const double w = 0.75;
        const double l = 4.0;
        std::vector<Vec2> plus{{-w, -l}, {w, -l}, {w, -w}, {l, -w}, {l, w},   {w, w},
                               {w, l},   {-w, l}, {-w, w}, {-l, w}, {-l, -w}, {-w, -w}};
        for (Vec2& q : plus) {
            q = q + centre;
        }
        j.region.push_back(orc::polygon(plus));
        const VCarvingResult on = run(j, true);
        const VCarvingResult off = run(j, false);
        REQUIRE(on.ok());
        std::size_t added = 0;
        CHECK(onlyCornerCutsAdded(on.toolpath, off.toolpath, -1.0, added));
        CHECK(added >= 4);
    }
}

TEST_CASE("T-0160: far from the origin the corner cuts keep the cone off the walls", "[toolpath][v_carve][corner]") {
    // Round 1 kept a piece outside the wall ring at 1e6 mm (points within 1e-9 × the coordinates
    // of a ring counted as on it, more than the margin there).
    Job j = squareJob();
    const geometry::Transform2 move =
        geometry::Transform2::translation({1e6, -7e5}) * geometry::Transform2::rotation(0.3);
    j.region = {j.region.front().transformed(move)};
    j.params.tolerance.chordal = 0.001;
    j.params.tolerance.distance = 0.001;
    const VCarvingResult on = run(j, true);
    const VCarvingResult off = run(j, false);
    REQUIRE(on.ok());
    std::size_t added = 0;
    CHECK(onlyCornerCutsAdded(on.toolpath, off.toolpath, -1.0, added));
    CHECK(added >= 4);
    const double margin = 1.0 + 0.5 * 0.001 / 3.0;
    for (const auto& [a, b] : addedFeeds(on.toolpath, off.toolpath)) {
        for (int k = 0; k <= 16; ++k) {
            const Vec2 p = a + (b - a) * (k / 16.0);
            CHECK(orc::boundaryDistance(j.region, p) >= margin - 1e-7);
        }
    }
}

TEST_CASE("T-0160: corner sharpening is on by default", "[toolpath][v_carve][corner]") {
    CHECK(VCarvingParams{}.cornerSharpen);
    const Job j = squareJob();
    const VCarvingResult r = vCarvingToolpath(j.region, j.tool, std::nullopt, j.clearance, j.params);
    CHECK(r.toolpath.moves == run(j, true).toolpath.moves);
    CHECK(r.toolpath.moves.size() > run(j, false).toolpath.moves.size());
}
