// T-0265: V-carve inlay (vcarve_inlay.hpp, ADR-0054): the V-carved pocket and the V-carved plug of
// the same vector mate within the glue gap (V-bit flank geometry), the plug is mirrored (and centred
// on another sheet), every plug boundary option, refusals.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/move.hpp"
#include "carvetto/toolpath/vcarve_inlay.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using Catch::Approx;
using geometry::Box2;
using geometry::Contour;
using geometry::LineSpan;
using geometry::Vec2;

namespace {

constexpr double kPi = 3.14159265358979323846;

/// 90 degree V-bit, 6 mm wide: full width at depth 3, flank slope 1.
tooling::Tool vbit(double angle = kPi / 2.0) {
    tooling::Tool t;
    t.id = "v";
    t.name = "V90";
    t.type = tooling::ToolType::VBit;
    t.diameter = 6.0;
    t.includedAngle = angle;
    t.defaultCuttingData.toolNumber = 1;
    t.defaultCuttingData.feedRate = 1000.0;
    t.defaultCuttingData.plungeRate = 300.0;
    t.defaultCuttingData.spindleSpeed = 16000.0;
    t.defaultCuttingData.passDepth = 5.0;
    t.defaultCuttingData.stepover = 0.1;
    return t;
}

tooling::Tool endMill() {
    tooling::Tool t = vbit();
    t.id = "em";
    t.name = "End mill";
    t.type = tooling::ToolType::EndMill;
    t.diameter = 3.0;
    t.defaultCuttingData.passDepth = 2.0;
    return t;
}

Contour rect(double x0, double y0, double x1, double y1) {
    return Contour::rectangle(Box2{Vec2{x0, y0}, Vec2{x1, y1}});
}

Contour polygon(const std::vector<Vec2>& pts) {
    Contour c;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        c.append(LineSpan{pts[i], pts[(i + 1) % pts.size()]});
    }
    c.closed = true;
    return c;
}

VCarveInlayParams plugParams() {
    VCarveInlayParams p;
    p.pocketDepth = 3.0;
    p.glueGap = 0.2;
    p.boundaryOffset = 10.0;
    return p;
}

/// Samples of the V-bit's tip along a toolpath: (x, y, depth below the surface), every <= 0.05 mm.
struct Tip {
    double x, y, depth;
};
std::vector<Tip> tips(const Toolpath& tp) {
    std::vector<Tip> out;
    Vec3 at{0, 0, 5};
    for (const Move& m : tp.moves) {
        REQUIRE_FALSE(isArc(m.kind));  // the V-carve polygonises everything
        if (isCutting(m.kind)) {
            const double len = std::hypot(m.to.x - at.x, m.to.y - at.y);
            const int n = std::max(1, static_cast<int>(std::ceil(std::max(len, std::fabs(m.to.z - at.z)) / 0.05)));
            for (int i = 1; i <= n; ++i) {
                const double f = static_cast<double>(i) / n;
                out.push_back({at.x + f * (m.to.x - at.x), at.y + f * (m.to.y - at.y), -(at.z + f * (m.to.z - at.z))});
            }
        }
        at = m.to;
    }
    return out;
}

/// Depth the (90 degree, slope 1) V-bit's cone has cut at (x, y): max over the tip samples of
/// depth - distance (0 where nothing was cut).
double carved(const std::vector<Tip>& samples, double x, double y) {
    double best = 0.0;
    for (const Tip& s : samples) {
        best = std::max(best, s.depth - std::hypot(x - s.x, y - s.y));
    }
    return best;
}

bool near(Vec2 a, Vec2 b, double eps = 1e-6) {
    return std::hypot(a.x - b.x, a.y - b.y) < eps;
}

}  // namespace

TEST_CASE("T-0265: only V-bits carve a V-carve inlay", "[toolpath][vcarve_inlay]") {
    CHECK(vCarveInlayToolAllowed(tooling::ToolType::VBit));
    CHECK_FALSE(vCarveInlayToolAllowed(tooling::ToolType::EndMill));
    CHECK_FALSE(vCarveInlayToolAllowed(tooling::ToolType::BallNose));
    VCarveInlayParams p;
    VCarvingResult r = vCarveInlayToolpath({rect(0, 0, 20, 14)}, endMill(), std::nullopt, {}, p);
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::InvalidTool));
}

TEST_CASE("T-0265: the pocket and the plug of one vector mate within the glue gap", "[toolpath][vcarve_inlay]") {
    const std::vector<Contour> shape{rect(0, 0, 30, 14)};
    const tooling::Tool tool = vbit();
    VCarveInlayParams p = plugParams();

    p.piece = VCarveInlayPiece::Pocket;
    const VCarvingResult pocket = vCarveInlayToolpath(shape, tool, std::nullopt, {}, p);
    REQUIRE(pocket.ok());
    p.piece = VCarveInlayPiece::Plug;
    const VCarvingResult plug = vCarveInlayToolpath(shape, tool, std::nullopt, {}, p);
    REQUIRE(plug.ok());
    const std::vector<Tip> pocketTips = tips(pocket.toolpath);
    const std::vector<Tip> plugTips = tips(plug.toolpath);

    const double pd = 3.0;
    const double gap = p.glueGap;
    double worstLow = 1e9;
    double worstHigh = -1e9;
    // Along the middle of the rectangle: the left flank (x from the wall), then the flat floor.
    for (double x = 0.25; x <= 14.75; x += 0.25) {
        const double s = std::min(x, 7.0);  // distance to the nearest wall
        const double dp = carved(pocketTips, x, 7.0);
        // The pocket's floor follows the V-bit's flank, flat at the pocket depth.
        CHECK(dp == Approx(std::min(s, pd)).margin(0.08));
        const double dc = carved(plugTips, x, 7.0);  // the plug stock's carved depth (plug coordinates == own: symmetric)
        // The plug's tip face sits (pocket depth - glue gap) down; the carved depth lowers its surface.
        const double plugSurface = (pd - gap) - dc;  // depth of the plug's surface when seated, in the pocket
        const double vertical = dp - plugSurface;
        worstLow = std::min(worstLow, vertical);
        worstHigh = std::max(worstHigh, vertical);
    }
    // No interference, and nowhere looser than the glue gap (flanks touch, the floor is the gap).
    CHECK(worstLow >= -0.08);
    CHECK(worstHigh <= gap + 0.08);
    CHECK(worstHigh >= gap - 0.08);
}

TEST_CASE("T-0265: the glue gap sets the plug's inset", "[toolpath][vcarve_inlay]") {
    const std::vector<Contour> shape{rect(0, 0, 30, 14)};
    VCarveInlayParams p = plugParams();
    for (const double gap : {0.0, 0.2, 1.0}) {
        p.glueGap = gap;
        const VCarvePlug g = vCarvePlugGeometry(shape, vbit(), p, Box2{});
        REQUIRE_FALSE(g.region.empty());
        Box2 top;
        for (const Contour& c : g.top) {
            top.expand(c.bounds());
        }
        const double inset = 3.0 - gap;  // (pocket depth - gap) x tan(45 degrees)
        CHECK(top.width() == Approx(30.0 - 2.0 * inset).margin(1e-3));
        CHECK(top.height() == Approx(14.0 - 2.0 * inset).margin(1e-3));
    }
    // A narrower V-bit angle makes the flank steeper: the inset is depth x tan(half angle).
    p.glueGap = 0.0;
    const VCarvePlug narrow = vCarvePlugGeometry(shape, vbit(kPi / 3.0), p, Box2{});
    REQUIRE_FALSE(narrow.region.empty());
    Box2 top;
    for (const Contour& c : narrow.top) {
        top.expand(c.bounds());
    }
    CHECK(top.width() == Approx(30.0 - 2.0 * 3.0 * std::tan(kPi / 6.0)).margin(1e-3));
}

TEST_CASE("T-0265: the plug is mirrored, in place or centred on another sheet", "[toolpath][vcarve_inlay]") {
    // A right triangle: its right angle is bottom-left; mirrored it is bottom-right.
    const std::vector<Contour> shape{polygon({{0, 0}, {40, 0}, {0, 20}})};
    VCarveInlayParams p = plugParams();
    p.glueGap = 0.0;
    p.pocketDepth = 1.0;  // a small inset so the triangle keeps a top face
    const VCarvePlug same = vCarvePlugGeometry(shape, vbit(), p, Box2{});
    REQUIRE_FALSE(same.top.empty());
    // Mirrored about x = 20: the right angle is at (40, 0). A point near it is inside, a point near (0, 20)'s old corner is not.
    CHECK(geometry::isInside(same.top, Vec2{38.0, 2.0}, geometry::FillRule::EvenOdd, p.tolerance));
    CHECK_FALSE(geometry::isInside(same.top, Vec2{2.0, 2.0}, geometry::FillRule::EvenOdd, p.tolerance));
    Box2 around;
    for (const Contour& c : same.boundary) {
        around.expand(c.bounds());
    }
    CHECK(around.centre().x == Approx(20.0).margin(1e-6));  // same place: the mirror axis is the vectors' middle
    CHECK(around.centre().y == Approx(10.0).margin(1e-6));

    // Another sheet: centred on it.
    p.plugSheet = 7;
    const Box2 sheet{Vec2{1000, 500}, Vec2{1300, 800}};
    const VCarvePlug other = vCarvePlugGeometry(shape, vbit(), p, sheet);
    REQUIRE_FALSE(other.top.empty());
    Box2 placed;
    for (const Contour& c : other.boundary) {
        placed.expand(c.bounds());
    }
    CHECK(placed.centre().x == Approx(1150.0).margin(1e-6));
    CHECK(placed.centre().y == Approx(650.0).margin(1e-6));
    // Still mirrored: the right angle's corner is at the right and bottom.
    CHECK(geometry::isInside(other.top, Vec2{1150.0 + 18.0, 650.0 - 8.0}, geometry::FillRule::EvenOdd, p.tolerance));
    CHECK_FALSE(geometry::isInside(other.top, Vec2{1150.0 - 18.0, 650.0 - 8.0}, geometry::FillRule::EvenOdd, p.tolerance));
    // Without the sheet's bounds a plug on another sheet is refused.
    const VCarvePlug unknown = vCarvePlugGeometry(shape, vbit(), p, Box2{});
    CHECK(unknown.region.empty());
    CHECK_FALSE(unknown.diagnostics.empty());
}

TEST_CASE("T-0265: plug outer boundary options", "[toolpath][vcarve_inlay]") {
    const std::vector<Contour> shape{rect(100, 100, 130, 114)};
    VCarveInlayParams p = plugParams();
    const auto boundsOf = [](const std::vector<Contour>& cs) {
        Box2 b;
        for (const Contour& c : cs) {
            b.expand(c.bounds());
        }
        return b;
    };

    p.plugBoundary = PlugBoundary::BoundaryOffset;
    p.boundaryOffset = 12.0;
    VCarvePlug g = vCarvePlugGeometry(shape, vbit(), p, Box2{});
    REQUIRE_FALSE(g.region.empty());
    Box2 b = boundsOf(g.boundary);
    CHECK(b.min.x == Approx(88.0));
    CHECK(b.max.x == Approx(142.0));
    CHECK(b.min.y == Approx(88.0));
    CHECK(b.max.y == Approx(126.0));
    CHECK(boundsOf(g.region) == g.boundary.front().bounds());  // the carved region fills the boundary

    p.plugBoundary = PlugBoundary::VectorOffset;
    g = vCarvePlugGeometry(shape, vbit(), p, Box2{});
    REQUIRE_FALSE(g.region.empty());
    b = boundsOf(g.boundary);
    CHECK(b.min.x == Approx(88.0).margin(1e-3));
    CHECK(b.max.x == Approx(142.0).margin(1e-3));
    // A rounded outline: its corner is cut off where the box has one.
    CHECK_FALSE(geometry::isInside(g.boundary, Vec2{88.5, 88.5}, geometry::FillRule::EvenOdd, p.tolerance));
    CHECK(geometry::isInside(g.boundary, Vec2{115.0, 89.0}, geometry::FillRule::EvenOdd, p.tolerance));

    p.plugBoundary = PlugBoundary::SheetLimits;
    const Box2 sheet{Vec2{0, 0}, Vec2{300, 200}};
    g = vCarvePlugGeometry(shape, vbit(), p, sheet);
    REQUIRE_FALSE(g.region.empty());
    CHECK(boundsOf(g.boundary) == sheet);
    p.plugBoundary = PlugBoundary::SheetLimits;
    g = vCarvePlugGeometry(shape, vbit(), p, Box2{});  // sheet unknown
    CHECK(g.region.empty());
    // The sheet is too small for the vectors: the flank cannot reach full depth.
    g = vCarvePlugGeometry(shape, vbit(), p, Box2{Vec2{100, 100}, Vec2{130, 114}});
    CHECK(g.region.empty());
    CHECK(g.diagnostics.front().issue == ToolpathIssue::ToolTooLarge);
}

TEST_CASE("T-0265: a boundary that is too tight is refused", "[toolpath][vcarve_inlay]") {
    const std::vector<Contour> shape{rect(0, 0, 30, 14)};
    VCarveInlayParams p = plugParams();
    p.plugBoundary = PlugBoundary::BoundaryOffset;
    // The top face is inset 2.8 mm; the flank needs 2 x 3 = 6 mm beyond it: an offset of 3.2 just fits.
    p.boundaryOffset = 3.1;
    CHECK(vCarvePlugGeometry(shape, vbit(), p, Box2{}).region.empty());
    p.boundaryOffset = 3.3;
    CHECK_FALSE(vCarvePlugGeometry(shape, vbit(), p, Box2{}).region.empty());
    p.boundaryOffset = 3.1;
    p.piece = VCarveInlayPiece::Plug;
    const VCarvingResult r = vCarveInlayToolpath(shape, vbit(), std::nullopt, {}, p);
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::ToolTooLarge));
}

TEST_CASE("T-0265: depth limits, glue gap, thin parts and surface clearance", "[toolpath][vcarve_inlay]") {
    const std::vector<Contour> shape{rect(0, 0, 30, 14)};
    VCarveInlayParams p = plugParams();

    // Deeper than the V-bit's full width (3 mm): limited, with a diagnostic, for both pieces.
    p.pocketDepth = 5.0;
    p.glueGap = 0.2;
    for (const VCarveInlayPiece piece : {VCarveInlayPiece::Pocket, VCarveInlayPiece::Plug}) {
        p.piece = piece;
        const VCarvingResult r = vCarveInlayToolpath(shape, vbit(), std::nullopt, {}, p);
        REQUIRE(r.ok());
        CHECK(r.has(ToolpathIssue::CarveDepthLimited));
        double deepest = 0.0;
        for (const Move& m : r.toolpath.moves) {
            deepest = std::max(deepest, -m.to.z);
        }
        CHECK(deepest == Approx(3.0).margin(1e-6));
    }

    // Surface clearance: the plug stock is carved deeper than the pocket (up to the full width).
    p.pocketDepth = 2.0;
    p.glueGap = 0.0;
    p.piece = VCarveInlayPiece::Plug;
    p.surfaceClearance = 0.0;
    VCarvingResult flush = vCarveInlayToolpath(shape, vbit(), std::nullopt, {}, p);
    p.surfaceClearance = 0.5;
    VCarvingResult cleared = vCarveInlayToolpath(shape, vbit(), std::nullopt, {}, p);
    REQUIRE(flush.ok());
    REQUIRE(cleared.ok());
    const auto deepestOf = [](const Toolpath& tp) {
        double d = 0.0;
        for (const Move& m : tp.moves) {
            d = std::max(d, -m.to.z);
        }
        return d;
    };
    CHECK(deepestOf(flush.toolpath) == Approx(2.0).margin(1e-6));
    CHECK(deepestOf(cleared.toolpath) == Approx(2.5).margin(1e-6));
    // The pocket ignores it.
    p.piece = VCarveInlayPiece::Pocket;
    CHECK(deepestOf(vCarveInlayToolpath(shape, vbit(), std::nullopt, {}, p).toolpath) == Approx(2.0).margin(1e-6));

    // The glue gap must stay below the pocket depth.
    p.piece = VCarveInlayPiece::Plug;
    p.surfaceClearance = 0.0;
    p.glueGap = 2.0;
    CHECK_FALSE(vCarveInlayToolpath(shape, vbit(), std::nullopt, {}, p).ok());

    // A shape thinner than twice the inset has no plug; a dumbbell keeps its ends and warns.
    p.glueGap = 0.0;
    p.pocketDepth = 3.0;
    VCarvePlug thin = vCarvePlugGeometry({rect(0, 0, 30, 5)}, vbit(), p, Box2{});
    CHECK(thin.region.empty());
    CHECK(thin.diagnostics.front().issue == ToolpathIssue::ToolTooLarge);
    p.boundaryOffset = 12.0;
    VCarvePlug dumbbell = vCarvePlugGeometry({polygon({{0, 0}, {20, 0}, {20, 8}, {40, 8}, {40, 0}, {60, 0}, {60, 20},
                                                       {40, 20}, {40, 12}, {20, 12}, {20, 20}, {0, 20}})},
                                             vbit(), p, Box2{});
    REQUIRE_FALSE(dumbbell.region.empty());
    CHECK(std::any_of(dumbbell.diagnostics.begin(), dumbbell.diagnostics.end(),
                      [](const ToolpathDiagnostic& d) { return d.issue == ToolpathIssue::RegionTrimmed; }));
    // A plain rectangle wide enough everywhere does not warn (its corners are not "thin").
    const VCarvePlug fine = vCarvePlugGeometry(shape, vbit(), p, Box2{});
    CHECK(std::none_of(fine.diagnostics.begin(), fine.diagnostics.end(),
                       [](const ToolpathDiagnostic& d) { return d.issue == ToolpathIssue::RegionTrimmed; }));
}

TEST_CASE("T-0265: clearance tools clear the flat areas of both pieces", "[toolpath][vcarve_inlay]") {
    const std::vector<Contour> shape{rect(0, 0, 40, 30)};
    VCarveInlayParams p = plugParams();
    const std::vector<PocketTool> clearance{PocketTool{endMill(), std::nullopt}};
    for (const VCarveInlayPiece piece : {VCarveInlayPiece::Pocket, VCarveInlayPiece::Plug}) {
        p.piece = piece;
        const VCarvingResult r = vCarveInlayToolpath(shape, vbit(), std::nullopt, clearance, p);
        REQUIRE(r.ok());
        REQUIRE(r.clearance.size() == 1);
        CHECK_FALSE(r.clearance[0].isEmpty());
        double deepest = 0.0;
        for (const Move& m : r.clearance[0].moves) {
            deepest = std::max(deepest, -m.to.z);
        }
        CHECK(deepest == Approx(3.0).margin(1e-6));  // the flat floor
    }
}

TEST_CASE("T-0265: refusals", "[toolpath][vcarve_inlay]") {
    const std::vector<Contour> shape{rect(0, 0, 30, 14)};
    const double inf = std::numeric_limits<double>::infinity();
    const auto run = [&](VCarveInlayParams p, const std::vector<Contour>& v = {rect(0, 0, 30, 14)}) {
        return vCarveInlayToolpath(v, vbit(), std::nullopt, {}, p);
    };
    VCarveInlayParams base = plugParams();
    for (const VCarveInlayPiece piece : {VCarveInlayPiece::Pocket, VCarveInlayPiece::Plug}) {
        base.piece = piece;
        REQUIRE(run(base).ok());
        VCarveInlayParams p = base;
        p.pocketDepth = 0.0;
        CHECK_FALSE(run(p).ok());
        p = base;
        p.glueGap = -0.1;
        CHECK_FALSE(run(p).ok());
        p = base;
        p.surfaceClearance = inf;
        CHECK_FALSE(run(p).ok());
        p = base;
        p.safeZ = 0.0;
        CHECK_FALSE(run(p).ok());
        p = base;
        p.boundaryOffset = -1.0;
        CHECK_FALSE(run(p).ok());
        p = base;
        p.tolerance.chordal = 5.0;
        CHECK_FALSE(run(p).ok());
        // No closed vector.
        Contour open = rect(0, 0, 30, 14);
        open.closed = false;
        const VCarvingResult r = run(base, {open});
        CHECK_FALSE(r.ok());
        CHECK(r.has(ToolpathIssue::OpenContourIgnored));
        CHECK_FALSE(run(base, {}).ok());
    }
}

TEST_CASE("T-0265: deterministic", "[toolpath][vcarve_inlay]") {
    VCarveInlayParams p = plugParams();
    p.piece = VCarveInlayPiece::Plug;
    const std::vector<Contour> shape{polygon({{0, 0}, {30, 0}, {10, 12}})};
    const VCarvingResult a = vCarveInlayToolpath(shape, vbit(), std::nullopt, {}, p);
    const VCarvingResult b = vCarveInlayToolpath(shape, vbit(), std::nullopt, {}, p);
    REQUIRE(a.ok());
    CHECK(a.toolpath.moves == b.toolpath.moves);
    // every rapid at safe Z
    for (const Move& m : a.toolpath.moves) {
        if (m.kind == MoveKind::Rapid) {
            CHECK(m.to.z == Approx(p.safeZ));
        }
    }
    CHECK(near({0, 0}, {0, 0}));
}
