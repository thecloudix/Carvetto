// T-0169: Pocket's tool centres keep radius + allowance from the walls where curves meet lines,
// measured by exact distance to the ORIGINAL curves (pocket_curve_oracle.hpp), not polygons.
//
// Root cause (ADR-0017 §8a): the inside test of links, travel, routes and bridges accepted a
// segment with the user's distance tolerance, so a raster link between a point on a round join of
// the centre region and a point on the straight edge next to it cut the corner between them,
// leaving the region by up to about twice that tolerance (45 nm under an "A"'s crossbar at
// distance = chordal / 3 inside V-Carve; up to 0.029 mm with distance up to chordal).

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "../geometry/test_rng.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/v_carve.hpp"
#include "pocket_curve_oracle.hpp"
#include "pocket_test_util.hpp"

using namespace carvetto::toolpath;
using carvetto::geometry::ArcDirection;
using carvetto::geometry::ArcSpan;
using carvetto::geometry::BezierSpan;
using carvetto::geometry::Contour;
using carvetto::geometry::kTwoPi;
using carvetto::geometry::LineSpan;
using carvetto::geometry::Span;
using carvetto::geometry::Vec2;
using carvetto::test::SplitMix;
namespace oc = carvetto::test::pocket_curve;

namespace {

/// The documented numeric slack of ADR-0017 §3 / §8a: tool centres keep at least
/// radius + allowance − this from the true walls.
constexpr double kSlack = 1e-5;

Contour polygon(const std::vector<Vec2>& pts, Vec2 offset) {
    Contour c;
    c.closed = true;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        c.append(LineSpan{pts[i] + offset, pts[(i + 1) % pts.size()] + offset});
    }
    return c;
}

/// The "A" of the verifier's repro2/nest_frame.txt (T-0153 round 3) with its triangular hole.
std::vector<Contour> letterA(Vec2 offset) {
    return {polygon({{0, 0}, {1.125, 0}, {1.5, 1.5}, {3, 1.5}, {3.375, 0}, {4.5, 0}, {2.8125, 6}, {1.6875, 6}}, offset),
            polygon({{1.7999999999999998, 2.4375}, {2.25, 4.3125}, {2.7000000000000002, 2.4375}}, offset)};
}

/// Edge P→Q as a line, an arc of sagitta s (s > 0 bulges to the left of P→Q), or a Bezier whose
/// inner control points sit 4s/3 to the left (moved along the chord by `skew`).
Span edge(Vec2 P, Vec2 Q, int kind, double s, double skew) {
    if (kind == 0 || s == 0.0) {
        return LineSpan{P, Q};
    }
    const Vec2 d = Q - P;
    const double len = d.length();
    const Vec2 n = (d * (1.0 / len)).perp();
    if (kind == 1) {
        const double h = 0.5 * len;
        const double radius = (h * h + s * s) / (2.0 * std::fabs(s));
        const Vec2 centre = (P + Q) * 0.5 - n * ((radius - std::fabs(s)) * (s > 0.0 ? 1.0 : -1.0));
        const double a0 = std::atan2(P.y - centre.y, P.x - centre.x);
        const double a1 = std::atan2(Q.y - centre.y, Q.x - centre.x);
        const Vec2 apex = (P + Q) * 0.5 + n * s;
        const ArcSpan ccw{centre, radius, a0, a1, ArcDirection::CCW};
        const ArcSpan cw{centre, radius, a0, a1, ArcDirection::CW};
        return distance(ccw.pointAt(0.5), apex) < distance(cw.pointAt(0.5), apex) ? ccw : cw;
    }
    return BezierSpan{P, P + d * (1.0 / 3.0 + skew) + n * (4.0 / 3.0 * s),
                      P + d * (2.0 / 3.0 + skew) + n * (4.0 / 3.0 * s), Q};
}

struct JunctionCase {
    std::vector<Contour> contours;
    std::vector<PocketTool> tools;
    PocketParams params;
    std::string what;
};

/// A polygon or star (reflex corners) whose edges are lines, arcs and Beziers bulging either way,
/// so curves meet lines (and each other) at many angles, optionally with a D-shaped island (a
/// curve and a line); chordal 0.001–0.1, distance up to the chordal tolerance; both strategies,
/// every profile pass, allowances, rest tools; `far` translates the case.
JunctionCase junctionCase(std::uint64_t seed, double far) {
    SplitMix rng{seed};
    JunctionCase c;
    const int n = 4 + static_cast<int>(rng.next() % 4);
    const bool star = rng.next() % 2 == 0;
    const double inner = star ? rng.uniform(0.55, 0.8) : 1.0;
    const double phase = rng.uniform(0.0, kTwoPi);
    const Vec2 o{far, -0.7 * far};
    std::vector<Vec2> v;
    const int m = star ? 2 * n : n;
    for (int i = 0; i < m; ++i) {
        const double a = phase + kTwoPi * i / m;
        const double r = (star && i % 2 == 1) ? 10.0 * inner : 10.0;
        v.push_back(o + Vec2{r * std::cos(a), r * std::sin(a)});
    }
    Contour outer;
    outer.closed = true;
    for (int i = 0; i < m; ++i) {
        const Vec2 P = v[static_cast<std::size_t>(i)];
        const Vec2 Q = v[static_cast<std::size_t>((i + 1) % m)];
        const int kind = static_cast<int>(rng.next() % 3);
        const double s = rng.uniform(star ? -0.08 : -0.12, star ? 0.08 : 0.2) * distance(P, Q);
        outer.append(edge(P, Q, kind, s, rng.uniform(-0.1, 0.1)));
    }
    c.contours.push_back(outer);
    if (rng.next() % 2 == 0) {
        const double ir = rng.uniform(1.0, 2.5) * inner;
        const double ia = rng.uniform(0.0, kTwoPi);
        const Vec2 P = o + Vec2::fromAngle(ia, ir);
        const Vec2 Q = o - Vec2::fromAngle(ia, ir);
        Contour island;
        island.closed = true;
        const int kind = 1 + static_cast<int>(rng.next() % 2);
        island.append(edge(P, Q, kind, rng.uniform(0.3, 1.0) * ir, rng.uniform(-0.1, 0.1)));
        island.append(edge(Q, P, static_cast<int>(rng.next() % 3), rng.uniform(-0.3, 0.3) * ir, 0.0));
        c.contours.push_back(island);
    }
    const double chords[] = {0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1};
    const double distanceFraction[] = {1.0, 1.0, 0.5, 0.1};
    PocketParams& p = c.params;
    p.tolerance.chordal = chords[rng.next() % 7];
    p.tolerance.distance = p.tolerance.chordal * distanceFraction[rng.next() % 4];
    const double d = rng.uniform(0.4, 3.0) * inner;
    c.tools.push_back(carvetto::test::pocket::endMill("T1", 1, d, rng.uniform(0.2, 0.9) * d, 2.0));
    if (rng.next() % 3 == 0) {
        const double d2 = rng.uniform(0.2, 0.7) * d;
        c.tools.push_back(carvetto::test::pocket::endMill("T2", 2, d2, rng.uniform(0.2, 0.9) * d2, 2.0));
    }
    p.strategy = rng.next() % 2 == 0 ? PocketStrategy::Offset : PocketStrategy::Raster;
    p.direction = rng.next() % 2 == 0 ? CutDirection::Climb : CutDirection::Conventional;
    p.rasterAngle = rng.uniform(0.0, 3.2);
    p.profilePass = static_cast<ProfilePass>(rng.next() % 3);
    p.allowance = rng.next() % 2 == 0 ? 0.0 : rng.uniform(0.0, 0.3);
    p.cutDepth = rng.next() % 2 == 0 ? 1.0 : 3.0;  // one level, or two (travel at depth)
    p.safeZ = 5.0;
    c.what = std::string(star ? "star" : "polygon") + std::to_string(n);
    return c;
}

/// The A's flat-area clearing as V-Carve asks Pocket for it in the repro (60° V-bit at 0.3 mm:
/// cutting radius 0.3·tan 30°; chordal and distance 0.001/3; raster at 0.66490 rad).
PocketResult letterAPocket(Vec2 offset, double& radius) {
    radius = 0.3 * std::tan(kTwoPi / 12.0);
    PocketParams p;
    p.cutDepth = 0.3;
    p.strategy = PocketStrategy::Raster;
    p.rasterAngle = 0.66490102198018897;
    p.profilePass = ProfilePass::Last;
    p.maintainExactStep = true;
    p.lastPassThickness = 0.0;
    p.safeZ = 5.0;
    p.tolerance.chordal = 0.001 / 3.0;
    p.tolerance.distance = 0.001 / 3.0;
    const PocketTool tool = carvetto::test::pocket::endMill("E", 1, 2.0 * radius, 0.1732050807568877, 1.0);
    return pocketToolpath(letterA(offset), {tool}, p);
}

}  // namespace

TEST_CASE("T-0169 regression: the raster link under an A's crossbar keeps the tool radius",
          "[toolpath][pocket][gouge][t0169]") {
    for (const Vec2 offset : {Vec2{0, 0}, Vec2{1e4, -7e3}}) {
        INFO("offset " << offset.x << ", " << offset.y);
        double radius = 0.0;
        const PocketResult res = letterAPocket(offset, radius);
        REQUIRE(res.toolpaths.size() == 1);
        REQUIRE_FALSE(res.toolpaths[0].isEmpty());
        const oc::ClearanceReport rep = oc::wallClearance(res.toolpaths[0], letterA(offset), radius, -kSlack, 0.0);
        INFO("min clearance " << rep.minClearance << " at move " << rep.worstMove);
        CHECK(rep.checkedMoves > 100);
        CHECK(rep.certified);
        CHECK(rep.plungesInside);
        // Before the fix: −4.59e-5 (the link from the join arc to the crossbar's offset). Now the
        // margin's half left by the join chords (chordal / 6 ≈ 1.67e-4) is kept.
        CHECK(rep.minClearance > 0.0);
    }
}

TEST_CASE("T-0169 regression: V-Carve's flat-area moves in the A keep the footprint at the flat depth",
          "[toolpath][v_carve][pocket][t0169]") {
    // The verifier's case (repro2/nest_frame.txt, VC_FIX "0 6 60 0.001 0.3 0", raster at
    // 0.66490 rad), with only the A: every carving-tool move at the flat depth keeps the
    // footprint there (0.3·tan 30°) from the walls.
    carvetto::tooling::Tool vbit;
    vbit.id = "V";
    vbit.name = "V";
    vbit.type = carvetto::tooling::ToolType::VBit;
    vbit.diameter = 6.0;
    vbit.includedAngle = kTwoPi / 6.0;
    vbit.defaultCuttingData.toolNumber = 1;
    vbit.defaultCuttingData.feedRate = 1000.0;
    vbit.defaultCuttingData.plungeRate = 300.0;
    vbit.defaultCuttingData.spindleSpeed = 18000.0;
    vbit.defaultCuttingData.passDepth = 1.1244448085417738;
    vbit.defaultCuttingData.stepover = 0.1732050807568877;
    VCarvingParams p;
    p.safeZ = 5.2367691924120914;
    p.tolerance.chordal = 0.001;
    p.tolerance.distance = 0.001;
    p.flatDepth = 0.3;
    p.clearanceStrategy = PocketStrategy::Raster;
    p.clearanceRasterAngle = 0.66490102198018897;
    const std::vector<Contour> walls = letterA({0, 0});
    const VCarvingResult r = vCarvingToolpath(walls, vbit, std::nullopt, {}, p);
    REQUIRE(r.ok());
    Toolpath flat;
    flat.tool = r.toolpath.tool;
    const double radius = 0.3 * std::tan(kTwoPi / 12.0);
    std::size_t atFlat = 0;
    for (std::size_t i = 0; i < r.toolpath.moves.size(); ++i) {
        const Move& m = r.toolpath.moves[i];
        const carvetto::geometry::Vec3 s = r.toolpath.startOf(i);
        if (m.kind == MoveKind::Rapid || std::fabs(s.z + 0.3) > 1e-12 || std::fabs(m.to.z + 0.3) > 1e-12) {
            continue;
        }
        // One feed per flat move, each from a rapid at its start (no plunge: only the
        // horizontal moves are measured here).
        flat.moves.push_back(Move::rapid({s.x, s.y, p.safeZ}));
        flat.moves.push_back(Move::feed({s.x, s.y, s.z}));
        flat.moves.push_back(m);
        ++atFlat;
    }
    REQUIRE(atFlat > 1000);
    const oc::ClearanceReport rep = oc::wallClearance(flat, walls, radius, -kSlack, 0.0);
    INFO("min clearance " << rep.minClearance << " at move " << rep.worstMove);
    CHECK(rep.certified);
    CHECK(rep.minClearance > 0.0);  // before the fix: −4.59e-5
}

TEST_CASE("T-0169 property: tool centres keep radius + allowance from curve/line junctions",
          "[toolpath][pocket][gouge][t0169]") {
#ifdef NDEBUG
    const std::uint64_t cases = 240;
#else
    const std::uint64_t cases = 60;
#endif
    std::size_t checked = 0, withCurves = 0, coarseDistance = 0;
    double worst = 1e300;
    for (const double far : {0.0, 1e4}) {
        for (std::uint64_t k = 1; k <= cases; ++k) {
            const JunctionCase c = junctionCase(0x7169'0000ULL + k, far);
            const PocketResult res = pocketToolpath(c.contours, c.tools, c.params);
            for (std::size_t ti = 0; ti < res.toolpaths.size(); ++ti) {
                const Toolpath& tp = res.toolpaths[ti];
                if (tp.isEmpty()) {
                    continue;
                }
                const double radius = 0.5 * c.tools[ti].tool.diameter;
                const oc::ClearanceReport rep = oc::wallClearance(tp, c.contours, radius,
                                                                  c.params.allowance - kSlack, -c.params.startDepth);
                INFO("case " << k << " far " << far << " " << c.what << " tool " << ti << " d " << 2.0 * radius
                             << " chordal " << c.params.tolerance.chordal << " distance "
                             << c.params.tolerance.distance << " strategy " << static_cast<int>(c.params.strategy)
                             << " pass " << static_cast<int>(c.params.profilePass) << " allowance "
                             << c.params.allowance << ": clearance - allowance "
                             << rep.minClearance - c.params.allowance << " at move " << rep.worstMove);
                CHECK(rep.certified);
                CHECK(rep.plungesInside);
                worst = std::min(worst, rep.minClearance - c.params.allowance);
                ++checked;
                coarseDistance += c.params.tolerance.distance > 0.5 * c.params.tolerance.chordal ? 1u : 0u;
                withCurves += c.contours.size() > 1 ? 1u : 0u;
            }
        }
    }
    INFO("worst clearance - allowance " << worst);
    CHECK(worst >= -kSlack);
    // The generator must exercise toolpaths with islands and with distance close to chordal (the
    // case that showed the defect).
    CHECK(checked >= cases);
    CHECK(withCurves >= cases / 4);
    CHECK(coarseDistance >= cases / 4);
}
