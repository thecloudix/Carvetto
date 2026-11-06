// T-0175: Pocket keeps the tool away from islands whose curves lie within the polygon tolerance of
// their chords (a thin lens, a crescent, a flat tab, flat Bezier lenses), measured by exact distance
// to the ORIGINAL curves (pocket_curve_oracle.hpp).
//
// Root cause: Pocket polygonises at chordal / 2 (ADR-0017 §3), and an arc or Bezier within that of
// its chord became the bare chord. An island made of such a curve and a line back along the chord
// (or two such curves) became a polygon of zero area, which Clipper2's union drops, so the region
// had no island there and the tool cut straight through it (verifier T-0169: centre on the island,
// clearance − radius −0.5 for sagitta 0.04 at chordal 0.1). A closed contour's curves now always
// keep vertices off their chords (`polygoniseRegionInto`), so the island keeps a polygon of
// non-zero area within the tolerance of the curves.

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "../geometry/test_rng.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/transform2.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "pocket_curve_oracle.hpp"
#include "pocket_test_util.hpp"

using namespace carvetto::toolpath;
using carvetto::geometry::ArcDirection;
using carvetto::geometry::ArcSpan;
using carvetto::geometry::BezierSpan;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::LineSpan;
using carvetto::geometry::Transform2;
using carvetto::geometry::Vec2;
using carvetto::test::SplitMix;
namespace oc = carvetto::test::pocket_curve;

namespace {

/// The documented numeric slack of ADR-0017 §3: tool centres keep at least
/// radius + allowance − this from the true walls.
constexpr double kSlack = 1e-5;

/// Arc from P to Q with sagitta s (s > 0 bulges to the left of P→Q).
ArcSpan arcWithSagitta(Vec2 P, Vec2 Q, double s) {
    const Vec2 d = Q - P;
    const double h = 0.5 * d.length();
    const double radius = (h * h + s * s) / (2.0 * std::fabs(s));
    const Vec2 n = (d * (1.0 / d.length())).perp();
    const Vec2 centre = (P + Q) * 0.5 - n * ((radius - std::fabs(s)) * (s > 0.0 ? 1.0 : -1.0));
    const double a0 = std::atan2(P.y - centre.y, P.x - centre.x);
    const double a1 = std::atan2(Q.y - centre.y, Q.x - centre.x);
    return ArcSpan{centre, radius, a0, a1, s > 0.0 ? ArcDirection::CW : ArcDirection::CCW};
}

/// The verifier's repro (t0169v1 repro/lens.cpp): a 20 mm square with a lens island on a 4 mm
/// chord at the centre, sagitta `s` (an arc and the line back along its chord).
std::vector<Contour> squareWithLens(double s) {
    Contour island;
    island.closed = true;
    island.append(arcWithSagitta({-2, 0}, {2, 0}, s));
    island.append(LineSpan{{2, 0}, {-2, 0}});
    return {Contour::rectangle(Box2{{-10, -10}, {10, 10}}), island};
}

enum class IslandKind { Lens, DoubleLens, Crescent, Tab, BezierLens, SBezierLens, CrescentTab, Count };

const char* kindName(IslandKind k) {
    switch (k) {
        case IslandKind::Lens:
            return "lens";
        case IslandKind::DoubleLens:
            return "double lens";
        case IslandKind::Crescent:
            return "crescent";
        case IslandKind::Tab:
            return "tab";
        case IslandKind::BezierLens:
            return "Bezier lens";
        case IslandKind::SBezierLens:
            return "S-Bezier lens";
        case IslandKind::CrescentTab:
            return "crescent tab";
        case IslandKind::Count:
            break;
    }
    return "?";
}

/// An island on the chord A→B (length L, direction u, normal v = u⊥, centred at c) whose curves lie
/// within |s| of the chord: a lens (arc + line back), a double lens (two arcs either side), a
/// crescent (two arcs the same side), a 3 × 3 square with a lens tab (the tab's arc and the line
/// back along it), a C-shaped or S-shaped Bezier lens, and a square with a crescent tab.
Contour thinIsland(IslandKind kind, Vec2 c, Vec2 u, double L, double s) {
    const Vec2 v = u.perp();
    auto at = [&](double x, double y) { return c + u * x + v * y; };
    const Vec2 A = at(-0.5 * L, 0.0);
    const Vec2 B = at(0.5 * L, 0.0);
    Contour is;
    is.closed = true;
    auto squareWithTab = [&](auto tab) {
        const Vec2 a = at(-0.5 * L - 3.0, -1.5);
        const Vec2 b = at(-0.5 * L, -1.5);
        const Vec2 t = at(-0.5 * L, 1.5);
        const Vec2 d = at(-0.5 * L - 3.0, 1.5);
        is.append(LineSpan{a, b});
        is.append(LineSpan{b, A});
        tab();
        is.append(LineSpan{A, t});
        is.append(LineSpan{t, d});
        is.append(LineSpan{d, a});
    };
    switch (kind) {
        case IslandKind::Lens:
            is.append(arcWithSagitta(A, B, s));
            is.append(LineSpan{B, A});
            break;
        case IslandKind::DoubleLens:
            is.append(arcWithSagitta(A, B, 0.5 * s));
            is.append(arcWithSagitta(B, A, 0.5 * s));
            break;
        case IslandKind::Crescent:
            is.append(arcWithSagitta(A, B, s));
            is.append(arcWithSagitta(B, A, -0.5 * s));
            break;
        case IslandKind::Tab:
            squareWithTab([&] {
                is.append(arcWithSagitta(A, B, s));
                is.append(LineSpan{B, A});
            });
            break;
        case IslandKind::BezierLens:
            is.append(BezierSpan{A, at(-L / 6.0, 4.0 / 3.0 * s), at(L / 6.0, 4.0 / 3.0 * s), B});
            is.append(LineSpan{B, A});
            break;
        case IslandKind::SBezierLens:
            is.append(BezierSpan{A, at(-L / 6.0, 2.0 * s), at(L / 6.0, -2.0 * s), B});
            is.append(LineSpan{B, A});
            break;
        case IslandKind::CrescentTab:
            squareWithTab([&] {
                is.append(arcWithSagitta(A, B, s));
                is.append(arcWithSagitta(B, A, -s / 3.0));
            });
            break;
        case IslandKind::Count:
            break;
    }
    return is;
}

}  // namespace

TEST_CASE("T-0175 regression: the verifier's thin lens islands are not cut through",
          "[toolpath][pocket][gouge][t0175]") {
    // repro/lens.cpp: `lens 0.04 0.1` (−0.5: the tool centre on the island) and `lens 0.004 0.01`
    // (−0.306) before the fix. Both sagittas are below half the chordal tolerance.
    struct Lens {
        double sagitta;
        double chordal;
    };
    for (const Lens l : {Lens{0.04, 0.1}, Lens{0.004, 0.01}}) {
        INFO("sagitta " << l.sagitta << " chordal " << l.chordal);
        const std::vector<Contour> walls = squareWithLens(l.sagitta);
        PocketParams p;
        p.cutDepth = 1.0;
        p.tolerance.chordal = l.chordal;
        p.tolerance.distance = std::min(l.chordal, 0.001);
        p.strategy = PocketStrategy::Raster;
        const PocketResult r = pocketToolpath(walls, {carvetto::test::pocket::endMill("E", 1, 1.0, 0.4, 3.0)}, p);
        REQUIRE(r.toolpaths.size() == 1);
        REQUIRE_FALSE(r.toolpaths[0].isEmpty());
        // The tool-centre region has the island's hole.
        REQUIRE(r.plans.size() == 1);
        CHECK(r.plans[0].centreRegion.size() == 2);
        const oc::ClearanceReport rep = oc::wallClearance(r.toolpaths[0], walls, 0.5, -kSlack, 0.0);
        INFO("min clearance " << rep.minClearance << " at move " << rep.worstMove);
        CHECK(rep.certified);
        CHECK(rep.plungesInside);
        CHECK(rep.minClearance >= -kSlack);
    }
}

TEST_CASE("T-0175 regression: the 0.005 mm wide island of the scale-0.01 sweep is kept",
          "[toolpath][pocket][gouge][t0175]") {
    // Verifier T-0169 sweep, scale 0.01, family 0, case 1099 (repro/tinyisland.cpp): the island's
    // arc (radius 0.0267, 70.7°) is within chordal / 2 of its chord, so the island vanished and an
    // offset ring crossed it (clearance − radius −0.0112 before the fix).
    Contour outer;
    outer.closed = true;
    outer.append(LineSpan{{0.07888121189718042, 0.061463439617646873}, {-0.061463439617646866, 0.07888121189718042}});
    outer.append(BezierSpan{{-0.061463439617646866, 0.07888121189718042},
                            {-0.086818921077338837, 0.027227544795219251},
                            {-0.092624845170516681, -0.019554005709723172},
                            {-0.07888121189718042, -0.061463439617646866}});
    outer.append(ArcSpan{{0.034117576012081416, 0.27490423224630828}, 0.35484072024521046, -1.8948895269718793,
                         -1.4936546610286749, ArcDirection::CCW});
    outer.append(LineSpan{{0.061463439617646894, -0.078881211897180406}, {0.07888121189718042, 0.061463439617646873}});
    Contour island;
    island.closed = true;
    island.append(ArcSpan{{-0.021471844566363221, 0.0035042484277013979}, 0.026669627815965203, 0.454985045315216,
                          -0.77853662250020017, ArcDirection::CW});
    island.append(
        LineSpan{{-0.0024846180074038745, -0.015224186516010634}, {0.0024846180074038745, 0.015224186516010634}});
    const std::vector<Contour> walls{outer, island};
    const double d = 0.022430994229998218;
    PocketParams p;
    p.cutDepth = 0.01;
    p.safeZ = 0.05;
    p.tolerance.chordal = 0.01;
    p.tolerance.distance = 1e-5;
    p.strategy = PocketStrategy::Offset;
    p.profilePass = ProfilePass::None;
    for (const double stepover : {0.3 * d, 0.6 * d, 0.9 * d}) {
        INFO("stepover " << stepover);
        const PocketResult r = pocketToolpath(walls, {carvetto::test::pocket::endMill("E", 1, d, stepover, 0.02)}, p);
        REQUIRE(r.toolpaths.size() == 1);
        REQUIRE_FALSE(r.toolpaths[0].isEmpty());
        const oc::ClearanceReport rep = oc::wallClearance(r.toolpaths[0], walls, 0.5 * d, -kSlack, 0.0);
        INFO("min clearance " << rep.minClearance << " at move " << rep.worstMove);
        CHECK(rep.certified);
        CHECK(rep.minClearance >= -kSlack);
    }
}

TEST_CASE("T-0175 property: thin islands keep the tool away at every scale and far from the origin",
          "[toolpath][pocket][gouge][t0175]") {
    // Lenses, crescents, tabs and Bezier lenses whose curves lie within 0.0002–0.5 × chordal of
    // their chords (below the chordal / 2 the region is polygonised at), in a 20 mm square scaled
    // by 0.01, 1 or 100 and translated 0 or 1e4 mm; sagittas of at least 1e-5 mm (ten grid units
    // of ADR-0006: thinner islands are below the boolean grid's resolution, §11). Chordal 1e-4–0.1
    // (0.001–0.1 at scale 100), distance chordal or chordal / 10; both strategies; allowances.
#ifdef NDEBUG
    const std::uint64_t cases = 160;
#else
    const std::uint64_t cases = 20;
#endif
    std::size_t checked = 0;
    std::size_t perKind[static_cast<int>(IslandKind::Count)] = {};
    double worst = 1e300;
    for (const double scale : {0.01, 1.0, 100.0}) {
        for (const double far : {0.0, 1e4}) {
            for (std::uint64_t k = 1; k <= cases; ++k) {
                SplitMix rng{0x7175'0000ULL + k + static_cast<std::uint64_t>(scale * 1000.0)};
                const double chords[] = {1e-4, 3e-4, 0.001, 0.003, 0.01, 0.03, 0.1};
                PocketParams p;
                // (Not below 0.001 at scale 100: a 2 m pocket at 0.1 µm takes seconds per case.)
                p.tolerance.chordal = chords[scale > 10.0 ? 2 + rng.next() % 5 : rng.next() % 7];
                p.tolerance.distance = std::max(1e-6, p.tolerance.chordal * (rng.next() % 2 == 0 ? 1.0 : 0.1));
                auto kind = static_cast<IslandKind>(rng.next() % static_cast<int>(IslandKind::Count));
                // In model units (before scaling): sagitta below chordal / 2 after scaling, ≥ 1e-5 mm.
                const double fraction = 0.5 * std::pow(10.0, rng.uniform(-3.7, 0.0));
                double sAbs = std::max(fraction * p.tolerance.chordal, 1e-5) / scale;
                const double L = rng.uniform(0.3, 6.0);
                const double d = rng.uniform(0.2, 2.0) * scale;
                if (kind == IslandKind::SBezierLens) {
                    // The S-curve crosses its chord at an angle of about 3·s / L, and the region
                    // loses the part near the crossing that is thinner than the grid, about 2e-6 /
                    // angle mm long (ADR-0006 §11): the tool may come about length² / (8·radius)
                    // closer there. Keep that below 5e-6 mm, or use a lens.
                    const double angle = 2e-6 / std::sqrt(4e-5 * 0.5 * d);
                    sAbs = std::max(sAbs, angle * L / 3.0);
                    if (sAbs * scale > 0.5 * p.tolerance.chordal) {
                        kind = IslandKind::Lens;
                    }
                }
                const double s = rng.next() % 2 == 0 ? sAbs : -sAbs;
                const Vec2 centre{rng.uniform(-3.0, 3.0), rng.uniform(-3.0, 3.0)};
                const Vec2 u = Vec2::fromAngle(rng.uniform(0.0, 6.283));
                std::vector<Contour> walls{Contour::rectangle(Box2{{-10, -10}, {10, 10}}),
                                           thinIsland(kind, centre, u, L, s)};
                const Transform2 m = Transform2::translation({far, -0.7 * far}) * Transform2::scaling(scale, scale);
                for (Contour& w : walls) {
                    w = w.transformed(m);
                }
                p.strategy = rng.next() % 2 == 0 ? PocketStrategy::Raster : PocketStrategy::Offset;
                p.rasterAngle = rng.uniform(0.0, 3.2);
                p.allowance = rng.next() % 3 == 0 ? rng.uniform(0.0, 0.2) * d : 0.0;
                p.cutDepth = scale;
                p.safeZ = 5.0 * scale;
                const PocketTool tool = carvetto::test::pocket::endMill("E", 1, d, rng.uniform(0.2, 0.9) * d, 3.0 * scale);
                const PocketResult r = pocketToolpath(walls, {tool}, p);
                REQUIRE(r.toolpaths.size() == 1);
                if (r.toolpaths[0].isEmpty()) {
                    continue;
                }
                const oc::ClearanceReport rep =
                    oc::wallClearance(r.toolpaths[0], walls, 0.5 * d, p.allowance - kSlack, 0.0);
                INFO("scale " << scale << " far " << far << " case " << k << " " << kindName(kind) << " sagitta "
                              << s * scale << " chord " << L * scale << " chordal " << p.tolerance.chordal
                              << " distance " << p.tolerance.distance << " d " << d << " strategy "
                              << static_cast<int>(p.strategy) << ": clearance - allowance "
                              << rep.minClearance - p.allowance << " at move " << rep.worstMove);
                CHECK(rep.certified);
                CHECK(rep.plungesInside);
                worst = std::min(worst, rep.minClearance - p.allowance);
                ++checked;
                ++perKind[static_cast<int>(kind)];
            }
        }
    }
    INFO("worst clearance - allowance " << worst);
    CHECK(worst >= -kSlack);
    CHECK(checked >= 4 * cases);  // (coarse chordal tolerances leave no room at scale 0.01)
    for (std::size_t i = 0; i < static_cast<std::size_t>(IslandKind::Count); ++i) {
        INFO(kindName(static_cast<IslandKind>(i)));
        CHECK(perKind[i] >= cases / 16);
    }
}
