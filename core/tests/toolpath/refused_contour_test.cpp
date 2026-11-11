// T-0183: a vector the clippers would silently leave out (its polygon cannot meet the chordal
// tolerance within the work bounds, ADR-0005 A2) is reported by every operation that polygonises
// (ToolpathIssue::ContourRefused, one diagnostic per vector, ADR-0018) and the other vectors are
// cut exactly as without it. Drill, Chamfer, Thread Mill and Engrave Outline never polygonise
// through the clippers: a huge arc is cut as an arc and no diagnostic is raised.
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <optional>
#include <vector>

#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/toolpath/engrave.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "carvetto/toolpath/v_carve.hpp"
#include "pocket_test_util.hpp"
#include "profile_test_util.hpp"
#include "v_carve_test_util.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using geometry::ArcDirection;
using geometry::ArcSpan;
using geometry::BezierSpan;
using geometry::Box2;
using geometry::Contour;
using geometry::Tolerance;
using geometry::Vec2;

namespace {

/// A closed circle of radius `radius` through the origin whose polygon needs far more than 2^20 chords.
Contour hugeCircle(double radius) {
    Contour c;
    c.closed = true;
    c.append(ArcSpan{{0.0, -radius}, radius, 0.0, 2.0 * std::numbers::pi, ArcDirection::CCW});
    return c;
}

std::vector<std::size_t> refusedIndices(const std::vector<ToolpathDiagnostic>& diagnostics) {
    std::vector<std::size_t> out;
    for (const ToolpathDiagnostic& d : diagnostics) {
        if (d.issue == ToolpathIssue::ContourRefused) {
            REQUIRE(d.contour.has_value());
            out.push_back(*d.contour);
        }
    }
    return out;
}

}  // namespace

TEST_CASE("refusedByClippers decides what the clippers decide", "[toolpath][refused]") {
    const Tolerance tol = Tolerance::defaults();
    const Contour ok = Contour::circle({0, 0}, 10.0);
    const Contour huge = hugeCircle(1e10);
    Contour nan;
    nan.closed = true;
    nan.append(ArcSpan{{0, 0}, std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0, ArcDirection::CCW});
    Contour nanBezier;
    nanBezier.closed = false;
    nanBezier.append(BezierSpan{{0, 0}, {1, 1}, {2, std::numeric_limits<double>::quiet_NaN()}, {3, 0}});
    const std::vector<Contour> all{ok, huge, nan, nanBezier, Contour::rectangle(Box2{{0, 0}, {5, 5}})};
    Tolerance fineTol = tol;
    fineTol.chordal = 1e-4;
    Tolerance coarseTol = tol;
    coarseTol.chordal = 0.5;
    for (const Tolerance& t : {tol, fineTol, coarseTol}) {
        for (const Contour& c : all) {
            const geometry::BooleanResult r = geometry::booleanOp(geometry::BooleanOp::Union, {c}, {}, t);
            CHECK(geometry::refusedByClippers(c, t) == (r.refusedContours == 1));
        }
    }
    CHECK(geometry::refusedContourIndices(all, tol) == std::vector<std::size_t>{1, 2, 3});
    CHECK_FALSE(geometry::refusedByClippers(Contour::circle({0, 0}, 1e7), tol));
}

TEST_CASE("Profile reports a refused vector and cuts the others as before", "[toolpath][refused][profile]") {
    ProfileParams p;
    p.passes.cutDepth = 6.0;
    p.passes.passDepth = 3.0;
    const tooling::Tool tool = test::endMill(6.0);
    const Contour a = Contour::circle({0, 0}, 10.0);
    const Contour b = Contour::rectangle(Box2{{100, 0}, {130, 20}});
    const ProfileResult base = profileToolpath({a, b}, tool, p);
    REQUIRE(base.ok());
    CHECK(refusedIndices(base.diagnostics).empty());
    // Profile's own input limits (coordinates within 1e5 mm, ADR-0016 section 19) refuse a job with a
    // huge arc as a whole before the polygon screen is reached, with a diagnostic and no partial result.
    const ProfileResult r = profileToolpath({a, hugeCircle(1e11), b}, tool, p);
    CHECK_FALSE(r.ok());
    CHECK_FALSE(r.diagnostics.empty());
    CHECK(r.toolpath.moves.empty());
}

TEST_CASE("Pocket reports a refused vector and cuts the others as before", "[toolpath][refused][pocket]") {
    PocketParams p;
    p.cutDepth = 3.0;
    const PocketTool tool = test::pocket::endMill("em", 1, 6.0, 2.4, 3.0);
    const Contour a = Contour::rectangle(Box2{{0, 0}, {50, 30}});
    const Contour b = Contour::circle({100, 0}, 15.0);
    const PocketResult base = pocketToolpath({a, b}, {tool}, p);
    REQUIRE_FALSE(base.toolpaths[0].isEmpty());
    CHECK(refusedIndices(base.diagnostics).empty());
    const PocketResult r = pocketToolpath({a, hugeCircle(1e11), b}, {tool}, p);
    CHECK(refusedIndices(r.diagnostics) == std::vector<std::size_t>{1});
    CHECK(r.toolpaths[0].moves == base.toolpaths[0].moves);
    const PocketResult two = pocketToolpath({hugeCircle(1e12), a, hugeCircle(1e11), b}, {tool}, p);
    CHECK(refusedIndices(two.diagnostics) == std::vector<std::size_t>{0, 2});
    CHECK(two.toolpaths[0].moves == base.toolpaths[0].moves);
    // An open vector is ignored as before, not reported as refused.
    Contour open = hugeCircle(1e10);
    open.closed = false;
    const PocketResult withOpen = pocketToolpath({a, open}, {tool}, p);
    CHECK(refusedIndices(withOpen.diagnostics).empty());
    CHECK(hasIssue(withOpen.diagnostics, ToolpathIssue::OpenContourIgnored));
}

TEST_CASE("V-Carve reports a refused vector and carves the others as before", "[toolpath][refused][v_carve]") {
    namespace vc = test::v_carve;
    const tooling::Tool tool = vc::vBit(60.0, 6.35);
    VCarvingParams p;
    const Contour a = Contour::rectangle(Box2{{0, 0}, {20, 8}});
    const Contour b = Contour::circle({60, 0}, 6.0);
    const VCarvingResult base = vCarvingToolpath({a, b}, tool, std::nullopt, {}, p);
    REQUIRE(base.ok());
    CHECK(refusedIndices(base.diagnostics).empty());
    const VCarvingResult r = vCarvingToolpath({a, hugeCircle(1e11), b}, tool, std::nullopt, {}, p);
    REQUIRE(r.ok());
    CHECK(refusedIndices(r.diagnostics) == std::vector<std::size_t>{1});
    CHECK(r.toolpath.moves == base.toolpath.moves);
    const VCarvingResult two = vCarvingToolpath({hugeCircle(1e12), a, hugeCircle(1e11), b}, tool, std::nullopt, {}, p);
    CHECK(refusedIndices(two.diagnostics) == std::vector<std::size_t>{0, 2});
    CHECK(two.toolpath.moves == base.toolpath.moves);
}

TEST_CASE("Engrave Fill reports a refused vector, Outline cuts the huge arc as an arc", "[toolpath][refused][engrave]") {
    tooling::Tool tool = test::endMill(2.0);
    tool.defaultCuttingData.stepover = 0.8;
    EngraveParams p;
    p.depth = 0.5;
    p.passes = 1;
    p.strategy = EngraveStrategy::Fill;
    const Contour a = Contour::rectangle(Box2{{0, 0}, {30, 20}});
    const EngraveResult base = engraveToolpath({a}, tool, std::nullopt, p);
    REQUIRE(base.ok());
    const EngraveResult r = engraveToolpath({hugeCircle(1e11), a}, tool, std::nullopt, p);
    REQUIRE(r.ok());
    CHECK(refusedIndices(r.diagnostics) == std::vector<std::size_t>{0});
    CHECK(r.toolpath.moves == base.toolpath.moves);
    // Outline does not polygonise through the clippers: a small arc is cut, nothing is refused.
    p.strategy = EngraveStrategy::Outline;
    const EngraveResult outline = engraveToolpath({a, Contour::circle({60, 0}, 5.0)}, tool, std::nullopt, p);
    REQUIRE(outline.ok());
    CHECK(refusedIndices(outline.diagnostics).empty());
}
