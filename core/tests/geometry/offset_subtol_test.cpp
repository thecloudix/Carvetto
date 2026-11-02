// T-0009: open offsets of inputs that hold sub-tolerance spans. Every span of positive length now
// takes part in the offset, so the distance, Round and coverage properties hold for the input as
// given, not for a flattened copy that left those spans out.

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <variant>
#include <vector>

#include "carvetto/geometry/offset.hpp"
#include "offset_chain_checks.hpp"
#include "test_rng.hpp"

using namespace carvetto::geometry;
using carvetto::test::SplitMix;

namespace {

const Tolerance kTol{};  // distance 0.001, chordal 0.01
const JoinType kJoins[] = {JoinType::Round, JoinType::Sharp, JoinType::Square};

double distanceToSpan(Vec2 p, const Span& s) {
    return carvetto::geometry::testing::distanceToSpanExact(p, s);
}

// Distance from `p` to the input's spans other than `skipA` and `skipB` (indices).
double distanceToInput(Vec2 p, const Contour& in, std::size_t skipA = SIZE_MAX,
                       std::size_t skipB = SIZE_MAX) {
    double best = 1e300;
    for (std::size_t i = 0; i < in.spans.size(); ++i) {
        if (i != skipA && i != skipB && length(in.spans[i]) > 0.0) {
            best = std::min(best, distanceToSpan(p, in.spans[i]));
        }
    }
    return best;
}

double distanceToOutput(Vec2 p, const std::vector<Contour>& out) {
    double best = 1e300;
    for (const Contour& c : out) {
        for (const Span& s : c.spans) {
            best = std::min(best, distanceToSpan(p, s));
        }
    }
    return best;
}

// How far an offset point may lie inside d (or, for Round, beyond it): the documented bounds of
// offset.hpp (bridge sagitta tol^2 / (8 d), snapping allowance), plus rounding.
double allowance(double d, Vec2 at) {
    const double t = kTol.distance;
    return std::max(t * t / (8.0 * d), std::min(5e-7 * std::max(1.0, d), 0.1 * t * t / d)) +
           1e-12 * std::max({1.0, d, std::fabs(at.x), std::fabs(at.y)});
}

struct Report {
    double under{0.0};      // worst (d - distance) beyond the allowance, 0 if none
    double over{0.0};       // Round: worst (distance - d) beyond the allowance
    std::size_t misses{0};  // Round: valid offset points farther than the tolerance from the result
    double worstMiss{0.0};
    std::size_t backward{0};
    std::size_t lineGaps{0};  // junctions with a line on either side that are not bit-identical
    std::string first;
};

// Independent check of one offset against the input as given (sub-tolerance spans included).
//  - Every result point is at least d (and for Round at most d) from the input, within the
//    allowance: each span sampled at 400 points.
//  - Round: every point of the requested-side offset of an input span, and of a Round corner arc
//    about a vertex, that is at least d from the rest of the input lies within the tolerance of
//    the result. Points within d + tol / 2 of another span are left out: there the offset may run
//    along a strip or loop narrower than the tolerance, which offset.hpp documents as dropped.
//  - Junctions are exact where a line is involved and no span steps back (shared checker).
Report checkOffset(const Contour& in, const std::vector<Contour>& out, double d, OffsetSide side,
                   JoinType join) {
    Report r;
    for (const Contour& c : out) {
        const std::size_t n = c.spans.size();
        for (std::size_t k = 0; k < n; ++k) {
            const Span& s = c.spans[k];
            for (int q = 0; q <= 400; ++q) {
                const Vec2 p = pointAt(s, q / 400.0);
                const double m = distanceToInput(p, in);
                const double a = allowance(d, p);
                if (d - m > a && d - m > r.under) {
                    r.under = d - m;
                }
                if (join == JoinType::Round && m - d > a && m - d > r.over) {
                    r.over = m - d;
                }
            }
            const bool hasNext = c.closed ? n > 1 : k + 1 < n;
            if (hasNext) {
                const Span& b = c.spans[(k + 1) % n];
                const bool line = std::holds_alternative<LineSpan>(s) || std::holds_alternative<LineSpan>(b);
                if (line && !(endPoint(s) == startPoint(b))) {
                    ++r.lineGaps;
                }
            }
        }
        r.backward += carvetto::geometry::testing::countBackwardSteps(c, kTol, &r.first);
    }
    if (join != JoinType::Round) {
        return r;
    }
    const double sgn = side == OffsetSide::Left ? 1.0 : -1.0;
    const double tie = d + 0.5 * kTol.distance;
    auto expect = [&](Vec2 q) {
        const double miss = distanceToOutput(q, out);
        if (miss > kTol.distance) {
            if (r.misses == 0) {
                r.first = "coverage miss at (" + std::to_string(q.x) + ", " + std::to_string(q.y) + ")";
            }
            ++r.misses;
            r.worstMiss = std::max(r.worstMiss, miss);
        }
    };
    std::vector<std::size_t> real;  // spans of positive length, in order
    for (std::size_t i = 0; i < in.spans.size(); ++i) {
        if (length(in.spans[i]) > 0.0) {
            real.push_back(i);
        }
    }
    for (const std::size_t i : real) {
        const Span& s = in.spans[i];
        for (int k = 1; k <= 9; ++k) {
            const Vec2 p = pointAt(s, k / 10.0);
            const Vec2 q = p + tangentAt(s, k / 10.0).perp() * (sgn * d);
            if (std::fabs(distanceToSpan(q, s) - d) > 1e-9 * std::max(1.0, d)) {
                continue;  // an arc whose offset radius on this side is negative
            }
            if (distanceToInput(q, in, i) >= tie) {
                expect(q);
            }
        }
    }
    for (std::size_t k = 0; k < real.size(); ++k) {
        if (k + 1 == real.size() && !in.closed) {
            break;
        }
        const std::size_t a = real[k];
        const std::size_t b = real[(k + 1) % real.size()];
        const Vec2 t1 = tangentAt(in.spans[a], 1.0);
        const Vec2 t2 = tangentAt(in.spans[b], 0.0);
        if (sgn * t1.cross(t2) >= -1e-9 || t1.dot(t2) < -0.999999) {
            continue;  // not an outside corner on this side (or an exact reversal, either way round)
        }
        const double a1 = (t1.perp() * sgn).angle();
        const double sweep = std::remainder((t2.perp() * sgn).angle() - a1, kTwoPi);
        const Vec2 v = endPoint(in.spans[a]);
        for (int k2 = 1; k2 <= 4; ++k2) {
            const Vec2 q = v + Vec2::fromAngle(a1 + sweep * k2 / 5.0, d);
            if (distanceToInput(q, in) >= d - 1e-9 && distanceToInput(q, in, a, b) >= tie) {
                expect(q);
            }
        }
    }
    return r;
}

void requireClean(const Report& r) {
    CAPTURE(r.first, r.under, r.over, r.misses, r.worstMiss);
    CHECK(r.under == 0.0);
    CHECK(r.over == 0.0);
    CHECK(r.misses == 0);
    CHECK(r.backward == 0);
    CHECK(r.lineGaps == 0);
}

Contour openContour(std::vector<Span> spans) {
    Contour c;
    c.spans = std::move(spans);
    return c;
}

std::vector<Contour> offsetOf(const Contour& c, double d, OffsetSide side, JoinType join,
                              double mitre = 2.0) {
    return offsetOpenContour(c, d, side, join, kTol, mitre);
}

}  // namespace

TEST_CASE("T-0009: the card's seed 0x60660ace keeps its offset next to a sub-tolerance jog",
          "[geometry][offset][open][t0009]") {
    // Legacy fuzz case with sub-tolerance arcs and jogs: main left them out and lost ~1 mm of
    // valid Round offset next to them.
    const Contour in = openContour({
        ArcSpan{{0.0020306425702524894, 0.00010962654193881519}, 0.0020335995738638172, -3.0876588756250412,
                -2.7344190280260237, ArcDirection::CCW},
        LineSpan{{0.00016330241588326222, -0.00069571063881183816}, {0.17376376847426217, 0.057130027393132804}},
        LineSpan{{0.17376376847426217, 0.057130027393132804}, {0.32361761516990273, 0.10704584547156627}},
        LineSpan{{0.32361761516990273, 0.10704584547156627}, {0.45437010339105466, 0.15059906455312491}},
        LineSpan{{0.45437010339105466, 0.15059906455312491}, {0.4544597540676647, 0.13495592798416783}},
        ArcSpan{{0.3095688314725813, 0.19640672434481865}, 0.15738354368806359, -0.4011230444980024,
                0.92206674763072061, ArcDirection::CCW},
        ArcSpan{{0.40380861836309595, 0.32362332932023846}, 0.0019941895244542898, -1.1319368024216574,
                -1.2498162249523173, ArcDirection::CW},
        LineSpan{{0.40443777869043246, 0.32173098972546454}, {0.31655257216128529, 0.29251113644932847}},
        LineSpan{{0.31655257216128529, 0.29251113644932847}, {0.27658780546147355, 0.36986357200407521}},
        LineSpan{{0.27658780546147355, 0.36986357200407521}, {0.25675460395845823, 0.5245297798894778}},
        ArcSpan{{0.18543488152036808, 0.48456556851473193}, 0.081753538146379867, 0.51075691203995655,
                -2.6189320181608235, ArcDirection::CW},
        LineSpan{{0.11459592365897481, 0.44375523834499014}, {0.0702955928999211, 0.52065217230346983}},
        ArcSpan{{0.065911731496961987, 0.51583694834312166}, 0.0065118831829714058, 0.83225573560767785,
                3.8157085035583802, ArcDirection::CCW},
        ArcSpan{{0.021643297680159761, 0.59999446786435606}, 0.096531440396138979, -1.1528459313607338,
                0.72402535650950073, ArcDirection::CCW},
        LineSpan{{0.093959380952946259, 0.66393743530438609}, {-0.016947094923074814, 0.7893667399806763}},
        ArcSpan{{-0.048574168351571724, 0.86016712658195882}, 0.077543320257405832, -1.1506837027202041,
                0.88764023592255636, ArcDirection::CCW},
        ArcSpan{{-0.11291952322524115, 0.9227248104008503}, 0.11331987296393178, -0.021324446172148723,
                -3.0817053934075926, ArcDirection::CW},
        ArcSpan{{-0.21915008557801605, 0.91828188717856729}, 0.0072727015060290697, -2.8140959219343746,
                1.6552158969584325, ArcDirection::CCW},
        ArcSpan{{-0.24113414026160154, 1.1780771631966107}, 0.2534510682228206, -1.4863767566313564,
                -4.2495765516048394, ArcDirection::CW},
        ArcSpan{{-0.35271083526304131, 1.4016972661307472}, 0.0035404655991661611, 2.0336087555747544,
                0.86902865449723965, ArcDirection::CW},
        ArcSpan{{-0.35476608934584408, 1.4018044876140037}, 0.0050582281511076712, 0.5390835340650143,
                2.1601444588284222, ArcDirection::CCW},
    });
    const double d = 2.2872756845348943;
    const auto out = offsetOf(in, d, OffsetSide::Right, JoinType::Round);
    requireClean(checkOffset(in, out, d, OffsetSide::Right, JoinType::Round));
}

TEST_CASE("T-0009: an offset next to a sub-tolerance jog keeps its distance (Round and Square)",
          "[geometry][offset][open][t0009]") {
    SECTION("Round, a 1e-3 mm jog at the end (main: 9.4e-4 mm short of d)") {
        const Contour in = openContour({
            LineSpan{{0, 0}, {-0.28046638629581044, -0.045968562618332175}},
            LineSpan{{-0.28046638629581044, -0.045968562618332175}, {-0.33539542168715858, -0.32481852884031737}},
            LineSpan{{-0.33539542168715858, -0.32481852884031737}, {-0.23344836125159363, -0.58975689940138476}},
            LineSpan{{-0.23344836125159363, -0.58975689940138476}, {-0.23420549417269379, -0.59040298087138721}},
        });
        const double d = 2.5034656442171062;
        const auto out = offsetOf(in, d, OffsetSide::Right, JoinType::Round, 3.5273060135135204);
        requireClean(checkOffset(in, out, d, OffsetSide::Right, JoinType::Round));
    }
    SECTION("Square, small arcs and jogs (main: 9.2e-4 mm short of d)") {
        const Contour in = openContour({
            ArcSpan{{0.15614828362369507, -0.015873043134954332}, 0.15695298651822476, 3.0402869923214291,
                    3.8648056554279497, ArcDirection::CCW},
            LineSpan{{0.038483258901462258, -0.11974403010487958}, {0.22348858847784209, -0.32931801523176041}},
            ArcSpan{{0.21703586016653731, -0.33059117797897541}, 0.006577130532413821, 0.19480394796412612,
                    -1.5824165141262765, ArcDirection::CW},
            ArcSpan{{0.21712178877156973, -0.3231967414749507}, 0.013972066295300206, -1.5824165141262798,
                    -5.8860483192487179, ArcDirection::CW},
            ArcSpan{{0.28355969270299336, -0.316549381196505}, 0.053567683003654075, -3.1183816490455971,
                    -1.409479047586659, ArcDirection::CCW},
            ArcSpan{{0.27124647048041456, -0.30518897519071869}, 0.067552609254920307, -1.2559788988549525,
                    2.0296572895295744, ArcDirection::CCW},
            LineSpan{{0.24132558362574816, -0.24462417066714909}, {0.065190137540337256, -0.33115621898287839}},
            LineSpan{{0.065190137540337256, -0.33115621898287839}, {0.065993370086495667, -0.33124370761693966}},
            ArcSpan{{0.074995004895387254, -0.33475248337755914}, 0.0096613113277111279, 2.7699161173044486,
                    4.3617617614139794, ArcDirection::CCW},
            LineSpan{{0.071676470704912246, -0.34382597594644393}, {0.13093367898866434, -0.22560510172064829}},
            LineSpan{{0.13093367898866434, -0.22560510172064829}, {0.13120194817450465, -0.22614750704354611}},
            ArcSpan{{0.18772275394870097, -0.2547375115026565}, 0.0633402702894168, 2.6732907279712146,
                    1.8925741649658034, ArcDirection::CW},
            LineSpan{{0.16769116230555059, -0.1946482015001407}, {0.46604297651815468, -0.054188310958694266}},
            LineSpan{{0.46604297651815468, -0.054188310958694266}, {0.46668972835659911, -0.054927470795677752}},
        });
        const double d = 4.1856033022989365;
        const auto out = offsetOf(in, d, OffsetSide::Right, JoinType::Square);
        requireClean(checkOffset(in, out, d, OffsetSide::Right, JoinType::Square));
    }
}

TEST_CASE("T-0009: circles about a cluster of vertices 2e-10 mm apart are split where they enter a neighbour",
          "[geometry][offset][open][t0009]") {
    // A zig-zag of 2e-10 mm lines before a long line. The Round arc about one of its vertices
    // passed 9e-10 mm inside the start of the long line's offset without crossing it, so no split
    // separated its stretch 0.03 d inside the long line's strip (7.2e-3 mm short of d, before
    // boundary end points next to sub-tolerance spans also split the raw elements).
    const Contour in = openContour({
        LineSpan{{-96.53346049035666, 41.650084079704996}, {-96.533460490434933, 41.650084079887336}},
        LineSpan{{-96.533460490434933, 41.650084079887336}, {-96.533460490587714, 41.650084079760717}},
        LineSpan{{-96.533460490587714, 41.650084079760717}, {-96.533460490420225, 41.650084079867121}},
        LineSpan{{-96.533460490420225, 41.650084079867121}, {-96.533460490283971, 41.650084079722859}},
        LineSpan{{-96.533460490283971, 41.650084079722859}, {-96.533460489762774, 41.650084080499767}},
        LineSpan{{-96.533460489762774, 41.650084080499767}, {-96.005429171563179, 40.943443114531021}},
    });
    const double d = 0.26036274431406109;
    for (const JoinType join : kJoins) {
        CAPTURE(static_cast<int>(join));
        const auto out = offsetOf(in, d, OffsetSide::Right, join, 3.3416559231094602);
        requireClean(checkOffset(in, out, d, OffsetSide::Right, join));
    }
}

TEST_CASE("T-0009: an arc of rounding-size radius turns a Round offset like a vertex",
          "[geometry][offset][open][t0009]") {
    SECTION("at the end of a path (0x9100043f: main and a plain inverted offset lost the corner arc)") {
        const Contour in = openContour({
            LineSpan{{-9097.9158396131352, -2070.1922381857262}, {-9098.8203772678589, -2070.217229104856}},
            ArcSpan{{-9098.8203772678607, -2070.2172291048555}, 1.1445631869570066e-12, -0.46364760900080609,
                    -1.4418744363253388, ArcDirection::CW},
        });
        const double d = 0.86384381011008371;
        const auto out = offsetOf(in, d, OffsetSide::Right, JoinType::Round);
        requireClean(checkOffset(in, out, d, OffsetSide::Right, JoinType::Round));
    }
    SECTION("its Round joins are kept within the pruning margin (0x92300adc)") {
        // The join about the 2.9e-9 mm arc's start lies up to 3e-9 mm inside the circle about its
        // end; judged by three samples it went as a whole. Spans 0 and 6 are 2d apart on one
        // carrier: their offsets coincide and are dropped (zero width), as before.
        const Contour in = openContour({
            LineSpan{{2.9211724141671391, 0}, {6.6450270926116302, 0}},
            LineSpan{{6.6450270926116302, 0}, {6.6454477960576694, 0}},
            LineSpan{{6.6454477960576694, 0}, {6.0822088117320936, 40.96960674706294}},
            LineSpan{{6.0822088117320936, 40.96960674706294}, {6.0821999245532776, 40.97025319418627}},
            LineSpan{{6.0821999245532776, 40.97025319418627}, {6.7318941690832848, 41.303955006040731}},
            LineSpan{{6.7318941690832848, 41.303955006040731}, {6.6454477960576694, 4.0280336346821652}},
            LineSpan{{6.6454477960576694, 4.0280336346821652}, {2.921172418292227, 4.0280336346821652}},
            ArcSpan{{2.921172416229683, 4.0280336325858892}, 2.9408265166838671e-09, 0.79350886335790971,
                    2.3480837883302508, ArcDirection::CCW},
        });
        const double d = 2.0140168173410826;
        const auto out = offsetOf(in, d, OffsetSide::Left, JoinType::Round);
        requireClean(checkOffset(in, out, d, OffsetSide::Left, JoinType::Round));
    }
}

TEST_CASE("T-0009: Bezier polygons keep their sub-tolerance segments and tiny Beziers are not dropped",
          "[geometry][offset][open][t0009]") {
    Contour c;
    c.spans.emplace_back(LineSpan{{0.0, 0.0}, {5.0, 0.0}});
    c.spans.emplace_back(BezierSpan{{5.0, 0.0}, {5.0001, 0.0002}, {5.0002, 0.0004}, {5.0003, 0.0004}});
    c.spans.emplace_back(LineSpan{{5.0003, 0.0004}, {9.0, 3.0}});
    // Zero distance returns the flattened input: continuous, the tiny Bezier as its chord.
    const auto flat = offsetOpenContour(c, 0.0, OffsetSide::Left, JoinType::Round, kTol);
    REQUIRE(flat.size() == 1);
    REQUIRE(flat.front().spans.size() == 3);
    for (std::size_t k = 1; k < flat.front().spans.size(); ++k) {
        CHECK(endPoint(flat.front().spans[k - 1]) == startPoint(flat.front().spans[k]));
    }
    for (const JoinType join : kJoins) {
        for (const OffsetSide side : {OffsetSide::Left, OffsetSide::Right}) {
            CAPTURE(static_cast<int>(join), static_cast<int>(side));
            const auto out = offsetOf(c, 0.5, side, join);
            requireClean(checkOffset(flat.front(), out, 0.5, side, join));
        }
    }
}

namespace {

// Sub-tolerance length: mostly 0.001..1 x tol, sometimes far smaller, down to rounding.
double tinyLength(SplitMix& g) {
    const double u = g.uniform(0.0, 1.0);
    if (u < 0.1) {
        return g.logUniform(1e-12, 1e-6) * kTol.distance;
    }
    return g.uniform(0.001, 1.0) * kTol.distance;
}

// Fixed-seed base path: a walk of lines, or lines and arcs, possibly closed.
Contour basePath(SplitMix& g) {
    Contour c;
    Vec2 p{g.uniform(-5.0, 5.0), g.uniform(-5.0, 5.0)};
    double h = g.uniform(-kPi, kPi);
    const int n = static_cast<int>(g.uniform(2.0, 14.0));
    const bool arcs = g.uniform(0.0, 1.0) < 0.5;
    for (int i = 0; i < n; ++i) {
        if (arcs && g.uniform(0.0, 1.0) < 0.5) {
            const double r = g.logUniform(0.03, 2.0);
            const double sweep = g.uniform(0.2, 3.0) * (g.uniform(0.0, 1.0) < 0.5 ? -1.0 : 1.0);
            const Vec2 left = Vec2::fromAngle(h + kPi / 2.0);
            const Vec2 centre = sweep > 0.0 ? p + left * r : p - left * r;
            const double a0 = (p - centre).angle();
            const ArcSpan arc{centre, r, a0, a0 + sweep, sweep > 0.0 ? ArcDirection::CCW : ArcDirection::CW};
            if (auto* line = c.spans.empty() ? nullptr : std::get_if<LineSpan>(&c.spans.back())) {
                line->end = arc.startPoint();
            }
            c.spans.emplace_back(arc);
            p = arc.endPoint();
            h += sweep;
        } else {
            const Vec2 e = p + Vec2::fromAngle(h, g.uniform(0.1, 2.0));
            c.spans.emplace_back(LineSpan{p, e});
            p = e;
        }
        h += g.uniform(-2.8, 2.8);
    }
    if (c.spans.size() >= 3 && g.uniform(0.0, 1.0) < 0.25 && distance(p, c.startPoint()) > 0.01) {
        c.spans.emplace_back(LineSpan{p, c.startPoint()});
        c.closed = true;
    }
    return c;
}

// Puts sub-tolerance spans into a continuous path: a jog in a random direction at the end of a
// line, a collinear split, a tiny arc at the end of a line, a run of tiny lines, a tiny piece cut
// off an arc on its own circle.
Contour withSubTolerance(const Contour& in, SplitMix& g) {
    Contour o;
    o.closed = in.closed;
    for (std::size_t i = 0; i < in.spans.size(); ++i) {
        const Span& s = in.spans[i];
        if (g.uniform(0.0, 1.0) > 0.4 || length(s) <= 8.0 * kTol.distance) {
            o.spans.push_back(s);
            continue;
        }
        const double tl = tinyLength(g);
        const int kind = static_cast<int>(g.uniform(0.0, 4.0));
        if (const auto* line = std::get_if<LineSpan>(&s)) {
            const Vec2 dir = (line->end - line->start).normalized();
            if (kind == 0) {
                const Vec2 mid = line->end - Vec2::fromAngle(dir.angle() + g.uniform(-3.1, 3.1), tl);
                o.spans.emplace_back(LineSpan{line->start, mid});
                o.spans.emplace_back(LineSpan{mid, line->end});
            } else if (kind == 1) {
                const Vec2 mid = g.uniform(0.0, 1.0) < 0.5 ? line->start + dir * tl : line->end - dir * tl;
                o.spans.emplace_back(LineSpan{line->start, mid});
                o.spans.emplace_back(LineSpan{mid, line->end});
            } else if (kind == 2) {
                // Tiny arc from `mid` to the line's end, sweeping up to 3 rad either way.
                double sweep = g.uniform(-3.0, 3.0);
                if (std::fabs(sweep) < 1e-3) {
                    sweep = 1e-3;
                }
                const double chord = 0.6 * tl;
                const Vec2 mid = line->end - dir * chord;
                const double r = chord / (2.0 * std::sin(std::fabs(sweep) / 2.0));
                const double h = std::sqrt(std::max(0.0, r * r - chord * chord / 4.0));
                const Vec2 centre = (mid + line->end) * 0.5 + dir.perp() * (sweep > 0.0 ? h : -h);
                const double a0 = (mid - centre).angle();
                const ArcSpan arc{centre, r, a0, a0 + sweep,
                                  sweep > 0.0 ? ArcDirection::CCW : ArcDirection::CW};
                o.spans.emplace_back(LineSpan{line->start, arc.startPoint()});
                o.spans.emplace_back(arc);
                if (arc.endPoint() != line->end) {
                    o.spans.emplace_back(LineSpan{arc.endPoint(), line->end});
                }
            } else {
                const int k = static_cast<int>(g.uniform(2.0, 6.0));
                Vec2 q = line->end - dir * (tl * k);
                o.spans.emplace_back(LineSpan{line->start, q});
                for (int j = 0; j < k; ++j) {
                    const Vec2 e =
                        j + 1 == k ? line->end : q + Vec2::fromAngle(dir.angle() + g.uniform(-3.1, 3.1), tl);
                    o.spans.emplace_back(LineSpan{q, e});
                    q = e;
                }
            }
        } else if (const auto* arc = std::get_if<ArcSpan>(&s)) {
            const double cut = tl / arc->radius * (arc->sweepAngle() > 0.0 ? 1.0 : -1.0);
            ArcSpan first = *arc;
            ArcSpan second = *arc;
            const double at = g.uniform(0.0, 1.0) < 0.5 ? arc->startAngle + cut : arc->endAngle - cut;
            first.endAngle = at;
            second.startAngle = at;
            o.spans.emplace_back(first);
            o.spans.emplace_back(second);
        }
    }
    return o;
}

}  // namespace

TEST_CASE("T-0009 AC2 generative: inputs with sub-tolerance spans, all joins, both sides, open and closed",
          "[geometry][offset][open][t0009]") {
    // A small fixed-seed version of the AC2 fuzz (the full run is in ADR-0012 A2): distance and
    // Round bounds for every join, coverage for Round, exact junctions and no backward step.
    SplitMix g{0x7009'5ab0'7001ULL};
    int cases = 0;
    int withSubTol = 0;
    for (int trial = 0; trial < 240; ++trial) {
        const Contour base = basePath(g);
        const Contour in = withSubTolerance(base, g);
        const double d = g.logUniform(0.05, 4.0);
        const OffsetSide side = g.uniform(0.0, 1.0) < 0.5 ? OffsetSide::Left : OffsetSide::Right;
        const JoinType join = kJoins[static_cast<int>(g.uniform(0.0, 3.0))];
        const double mitre = g.uniform(0.0, 1.0) < 0.5 ? 2.0 : g.uniform(1.0, 4.0);
        bool sub = false;
        for (const Span& s : in.spans) {
            sub = sub || length(s) <= kTol.distance;
        }
        withSubTol += sub ? 1 : 0;
        ++cases;
        CAPTURE(trial, d, static_cast<int>(side), static_cast<int>(join), mitre, in.closed);
        const auto out = offsetOpenContour(in, d, side, join, kTol, mitre);
        requireClean(checkOffset(in, out, d, side, join));
    }
    CHECK(withSubTol > cases / 2);
}
