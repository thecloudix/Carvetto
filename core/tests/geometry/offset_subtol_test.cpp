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
#include "cpu_time.hpp"
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
//    the result. A miss is exempt only where the region beyond d there is thinner than the
//    tolerance (a strip or loop narrower than the tolerance, documented as dropped), and the loci of
//    a sub-tolerance near-reversal are not required (owner decision (c), see below).
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
    const double valid = d + 1e-6;  // farther than this from every other span: a point of the boundary
    // A miss is exempt only where the region beyond d is thinner than the tolerance along the outward
    // normal (the documented drop of strips and loops narrower than the tolerance).
    auto thin = [&](Vec2 q, Vec2 normal) {
        for (int k = 1; k <= 32; ++k) {
            const Vec2 t = q + normal * (kTol.distance * k / 32.0);
            if (distanceToInput(t, in) < d - 1e-9 * std::max(1.0, d)) {
                return true;
            }
        }
        return false;
    };
    // Owner decision (c): a turn of more than 135 degrees at a span no longer than the tolerance never
    // flips the offset to the other side, so the corner arcs and span offsets of such a sub-tolerance
    // near-reversal and of the sub-tolerance spans within 2 tol of it are not part of the requested side.
    std::vector<Vec2> reversals;
    for (std::size_t i = 0; i < in.spans.size(); ++i) {
        const std::size_t j = i + 1 == in.spans.size() ? 0 : i + 1;
        if ((j == 0 && !in.closed) || length(in.spans[i]) <= 0.0 || length(in.spans[j]) <= 0.0) {
            continue;
        }
        if (length(in.spans[i]) > kTol.distance && length(in.spans[j]) > kTol.distance) {
            continue;
        }
        const Vec2 t1 = tangentAt(in.spans[i], 1.0);
        const Vec2 t2 = tangentAt(in.spans[j], 0.0);
        if (std::fabs(std::atan2(t1.cross(t2), t1.dot(t2))) > 0.75 * kPi) {
            reversals.push_back(endPoint(in.spans[i]));
        }
    }
    auto nearReversal = [&](Vec2 v) {
        return std::any_of(reversals.begin(), reversals.end(),
                           [&](Vec2 w) { return distance(v, w) <= 2.0 * kTol.distance; });
    };
    auto expect = [&](Vec2 q, Vec2 normal) {
        const double miss = distanceToOutput(q, out);
        if (miss > kTol.distance && !thin(q, normal)) {
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
        if (length(s) <= kTol.distance && (nearReversal(startPoint(s)) || nearReversal(endPoint(s)))) {
            continue;
        }
        for (int k = 1; k <= 9; ++k) {
            const Vec2 p = pointAt(s, k / 10.0);
            const Vec2 normal = tangentAt(s, k / 10.0).perp() * sgn;
            const Vec2 q = p + normal * d;
            if (std::fabs(distanceToSpan(q, s) - d) > 1e-9 * std::max(1.0, d)) {
                continue;  // an arc whose offset radius on this side is negative
            }
            if (distanceToInput(q, in, i) >= valid) {
                expect(q, normal);
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
        const Vec2 v = endPoint(in.spans[a]);
        if (nearReversal(v)) {
            continue;
        }
        const double a1 = (t1.perp() * sgn).angle();
        const double sweep = std::remainder((t2.perp() * sgn).angle() - a1, kTwoPi);
        for (int k2 = 1; k2 <= 4; ++k2) {
            const Vec2 u = Vec2::fromAngle(a1 + sweep * k2 / 5.0);
            const Vec2 q = v + u * d;
            if (distanceToInput(q, in) >= d - 1e-9 && distanceToInput(q, in, a, b) >= valid) {
                expect(q, u);
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

namespace {

double maxArcSweep(const std::vector<Contour>& out) {
    double sweep = 0.0;
    for (const Contour& c : out) {
        for (const Span& s : c.spans) {
            if (const auto* arc = std::get_if<ArcSpan>(&s)) {
                sweep = std::max(sweep, std::fabs(arc->sweepAngle()));
            }
        }
    }
    return sweep;
}

double minDistanceToInput(const std::vector<Contour>& out, const Contour& in) {
    double best = 1e300;
    for (const Contour& c : out) {
        for (const Span& s : c.spans) {
            for (int k = 0; k <= 400; ++k) {
                best = std::min(best, distanceToInput(pointAt(s, k / 400.0), in));
            }
        }
    }
    return best;
}

}  // namespace

TEST_CASE("T-0009 round 2: a rounding-radius arc between nearly collinear lines is not wrapped into a circle",
          "[geometry][offset][open][t0009]") {
    // Verifier seeds 0xc9200e83, 0xc90008db, 0xc9201371: with Sharp the arc's offset, extended at
    // both corners nearly all the way round, kept pieces at either end of its sweep whose ends lie
    // within the tolerance of each other; re-merged across the gap they gave a circle of radius d
    // through the input (1e-8 mm from it against d = 0.069).
    struct Case {
        Contour in;
        double d;
        double mitre;
    };
    const Case cases[] = {
        {openContour({LineSpan{{-286.64931065391545, -667.2793053148431}, {-292.45208432818686, -664.71106545508371}},
                      ArcSpan{{-292.45208432818669, -664.71106545508337}, 3.9653404153571554e-13,
                              -2.0344439357957027, -6.8817730429172244, ArcDirection::CW},
                      LineSpan{{-292.45208432818634, -664.7110654550836}, {-312.65154256130393, -655.77102114319712}}}),
         0.068653772818644987, 2.0},
        {openContour({LineSpan{{3.6681549236583715, 9.7507350321250676}, {3.6385756065220707, 5.9441539321690033}},
                      ArcSpan{{3.6385756065220711, 5.9441539321690033}, 5.1267333246122462e-16, 3.1415926535897931,
                              8.8705911338854726, ArcDirection::CCW},
                      LineSpan{{3.6385756065220707, 5.9441539321690033}, {3.592169221517957, -0.027913213936717238}}}),
         1.7780116989105186, 2.5774050735653171},
        {openContour({LineSpan{{23.024066380607042, 4.3987616220873802}, {15.760295181442771, 13.776157000264629}},
                      ArcSpan{{15.760295181442773, 13.776157000264629}, 1.3435550662105094e-15, 3.1415926535897931,
                              -1.8930167051923847, ArcDirection::CW},
                      LineSpan{{15.760295181442773, 13.776157000264627}, {12.098151102817612, 18.503917830764067}}}),
         0.14125543441318877, 3.9089467380180274},
    };
    for (const Case& c : cases) {
        for (const JoinType join : kJoins) {
            for (const OffsetSide side : {OffsetSide::Left, OffsetSide::Right}) {
                CAPTURE(c.d, static_cast<int>(join), static_cast<int>(side));
                const auto out = offsetOf(c.in, c.d, side, join, c.mitre);
                CHECK(minDistanceToInput(out, c.in) >= c.d - allowance(c.d, endPoint(c.in.spans[0])));
                CHECK(maxArcSweep(out) < kPi);
                requireClean(checkOffset(c.in, out, c.d, side, join));
            }
        }
    }
}

TEST_CASE("T-0009 round 2 generative: rounding-radius arcs of any sweep between nearly collinear lines",
          "[geometry][offset][open][t0009]") {
    // The neighbourhood of such a path is that of the two lines and a disc about the joint, so no
    // offset arc sweeps half a turn or more and no point comes inside d (the full sweep: 18,000
    // offsets in ADR-0012 A2).
    SplitMix g{0x7009'0b1d'0002ULL};
    for (int trial = 0; trial < 120; ++trial) {
        const double h = g.uniform(-kPi, kPi);
        const double r = g.logUniform(1e-16, 1e-8);
        const double sweep = g.uniform(0.01, kTwoPi - 0.01) * (g.uniform(0.0, 1.0) < 0.5 ? -1.0 : 1.0);
        const Vec2 shift = g.uniform(0.0, 1.0) < 0.5 ? Vec2{} : Vec2{g.uniform(-1e3, 1e3), g.uniform(-1e3, 1e3)};
        const Vec2 v = shift + Vec2{g.uniform(-5.0, 5.0), g.uniform(-5.0, 5.0)};
        const Vec2 a = v - Vec2::fromAngle(h, g.uniform(0.5, 10.0));
        const Vec2 left = Vec2::fromAngle(h + kPi / 2.0);
        const Vec2 centre = sweep > 0.0 ? v + left * r : v - left * r;
        const double a0 = (v - centre).angle();
        const ArcSpan arc{centre, r, a0, a0 + sweep, sweep > 0.0 ? ArcDirection::CCW : ArcDirection::CW};
        const double h2 = h + (g.uniform(0.0, 1.0) < 0.3 ? 0.0 : g.uniform(-1e-3, 1e-3));
        const Contour in = openContour({LineSpan{a, arc.startPoint()}, arc,
                                        LineSpan{arc.endPoint(), arc.endPoint() + Vec2::fromAngle(h2, g.uniform(0.5, 10.0))}});
        const double d = g.logUniform(0.01, 5.0);
        const double mitre = g.uniform(1.0, 4.0);
        for (const JoinType join : kJoins) {
            for (const OffsetSide side : {OffsetSide::Left, OffsetSide::Right}) {
                CAPTURE(trial, r, sweep, d, static_cast<int>(join), static_cast<int>(side));
                const auto out = offsetOf(in, d, side, join, mitre);
                CHECK(minDistanceToInput(out, in) >= d - allowance(d, v));
                CHECK(maxArcSweep(out) < kPi);
            }
        }
    }
}

namespace {

// Verifier probes (round 1, B2): n sub-tolerance spans of a 1e-7 mm random walk then a 5 mm line,
// and a zig-zag of 2e-4 x 3e-4 mm steps.
Contour microCluster(int n) {
    SplitMix g{12345};
    Contour c;
    Vec2 p{0.0, 0.0};
    for (int i = 0; i < n; ++i) {
        const Vec2 q = p + Vec2::fromAngle(g.uniform(0.0, kTwoPi), 1e-7);
        c.spans.emplace_back(LineSpan{p, q});
        p = q;
    }
    c.spans.emplace_back(LineSpan{p, p + Vec2{5.0, 1.0}});
    return c;
}

Contour tinyZigZag(int n) {
    Contour c;
    Vec2 p{0.0, 0.0};
    for (int i = 0; i < n; ++i) {
        const Vec2 q = p + Vec2{2e-4, (i % 2 ? 1.0 : -1.0) * 3e-4};
        c.spans.emplace_back(LineSpan{p, q});
        p = q;
    }
    return c;
}

// Worst CPU time over the joins and sides at d = 1 (best of `repeats`).
double worstSeconds(const Contour& in, int repeats) {
    double worst = 0.0;
    for (const JoinType join : kJoins) {
        for (const OffsetSide side : {OffsetSide::Left, OffsetSide::Right}) {
            double best = 1e300;
            for (int r = 0; r < repeats; ++r) {
                const double t0 = carvetto::test::processCpuSeconds();
                const auto out = offsetOpenContour(in, 1.0, side, join, kTol);
                best = std::min(best, carvetto::test::processCpuSeconds() - t0);
                CHECK_FALSE(out.empty());
            }
            worst = std::max(worst, best);
        }
    }
    return worst;
}

}  // namespace

TEST_CASE("T-0009 round 2: dense sub-tolerance input offsets in about linear time (CPU time; bounds in Release)",
          "[geometry][offset][open][t0009][perf]") {
    // Round 1 grew with about the cube of the span count here (2000-span micro-cluster 21 s, 8000
    // killed; 1000-span zig-zag 90 s Square).
    for (const bool zigzag : {false, true}) {
        CAPTURE(zigzag);
        const Contour small = zigzag ? tinyZigZag(4000) : microCluster(4000);
        const Contour large = zigzag ? tinyZigZag(8000) : microCluster(8000);
        const double t4 = worstSeconds(small, 2);
        const double t8 = worstSeconds(large, 2);
        CAPTURE(t4, t8);
#ifdef NDEBUG
        CHECK(t8 < 3.0);
        CHECK(t8 < 3.0 * std::max(t4, 0.02));
#else
        CHECK(t8 < 120.0);  // Debug: only guards against blow-ups
#endif
    }
    // Their offsets keep the distance.
    const Contour in = microCluster(2000);
    for (const JoinType join : kJoins) {
        const auto out = offsetOf(in, 1.0, OffsetSide::Left, join);
        CHECK(minDistanceToInput(out, in) >= 1.0 - allowance(1.0, Vec2{}));
    }
}

TEST_CASE("T-0009 round 3: a near-reversal onto a sub-tolerance end span does not swallow valid offset (0xf0003fad)",
          "[geometry][offset][open][t0009]") {
    // Round-2 verifier seed, shrunk to 37 spans: an arc of radius 6.13 split by a 9.5e-4 mm piece,
    // and at the path's end a 171-degree turn onto a 3.6e-15 mm span. The Sharp and Square corner
    // there reached out 1.5 d along that span and its region swallowed 1.6 mm of the arc's inner
    // offset. Owner decision (c): such a sub-tolerance near-reversal gets no join.
    const Contour in = openContour({
        ArcSpan{{-9.7657935140949483, 3.6832692691671056}, 6.1314298168347152, 1.5727673337095269, 5.0807484666166811, ArcDirection::CCW},
        ArcSpan{{-9.7657935140949483, 3.6832692691671056}, 6.1314298168347152, 5.0807484666166811, 5.0809032214833456, ArcDirection::CCW},
        ArcSpan{{-9.7657935140949483, 3.6832692691671056}, 6.1314298168347152, 5.0809032214833456, 5.9757298090547959, ArcDirection::CCW},
        LineSpan{{-3.9218866236763743, 1.8276874588207921}, {-3.8414065016546917, 1.8310445690061281}},
        LineSpan{{-3.8414065016546917, 1.8310445690061281}, {-3.6610307423528958, 1.8233317773581352}},
        LineSpan{{-3.6610307423528958, 1.8233317773581352}, {-3.5620060331626431, 1.8069878609698211}},
        LineSpan{{-3.5620060331626431, 1.8069878609698211}, {-3.4723562559811869, 1.7778729002709919}},
        LineSpan{{-3.4723562559811869, 1.7778729002709919}, {-3.4035310245848027, 1.7326523723109197}},
        LineSpan{{-3.4035310245848027, 1.7326523723109197}, {-3.3669799527497646, 1.6679917541388767}},
        ArcSpan{{-3.4013608806846363, 1.5485257590971973}, 0.12431481077072599, 1.2905801399630836, -3.0587451747935508, ArcDirection::CW},
        LineSpan{{-3.5252493053432974, 1.5382383681472691}, {-3.5257646604258466, 1.5376487673821662}},
        LineSpan{{-3.5257646604258466, 1.5376487673821662}, {-3.5252493053432974, 1.5382383681472691}},
        LineSpan{{-3.5252493053432974, 1.5382383681472691}, {-4.0940583747297161, 2.5734413416737265}},
        LineSpan{{-4.0940583747297161, 2.5734413416737265}, {-4.9039182805254775, 4.0473443220252676}},
        LineSpan{{-4.9039182805254775, 4.0473443220252676}, {-6.4001859664064646, 8.4202933914154947}},
        LineSpan{{-6.4001859664064646, 8.4202933914154947}, {-6.400185964610742, 8.4202933909548392}},
        LineSpan{{-6.400185964610742, 8.4202933909548392}, {-6.399989987003126, 8.4209013093871476}},
        LineSpan{{-6.399989987003126, 8.4209013093871476}, {-6.4004355298061473, 8.4208971859457904}},
        LineSpan{{-6.4004355298061473, 8.4208971859457904}, {-6.4004089549716632, 8.4199486785755404}},
        LineSpan{{-6.4004089549716632, 8.4199486785755404}, {-6.4001858117894761, 8.4203908350593011}},
        LineSpan{{-6.4001858117894761, 8.4203908350593011}, {-6.4003724985285251, 8.4208863903481816}},
        LineSpan{{-6.4003724985285251, 8.4208863903481816}, {-6.4001859664174887, 8.4202933914477143}},
        LineSpan{{-6.4001859664174887, 8.4202933914477143}, {-4.9227493885352196, 7.8158725890008522}},
        LineSpan{{-4.9227493885352196, 7.8158725890008522}, {-3.292449192123907, 7.1489151251775063}},
        ArcSpan{{-3.1234917151611863, 7.5619120894793275}, 0.4462209335566048, -1.9591237013597871, 1.3156439703307843, ArcDirection::CCW},
        ArcSpan{{-3.0109582368434329, 7.9933434739771254}, 0.00035455977183647484, 1.3156439703303258, 3.6003834139845878, ArcDirection::CCW},
        LineSpan{{-3.0112761311219174, 7.9931864521330676}, {-3.0108687484972774, 7.993686554824313}},
        LineSpan{{-3.0108687484972774, 7.993686554824313}, {-7.0976954952268816, 9.059684174878905}},
        LineSpan{{-7.0976954952268816, 9.059684174878905}, {-7.1902037239605798, 8.486752939519576}},
        LineSpan{{-7.1902037239605798, 8.486752939519576}, {-7.1902037240120489, 8.4867529395271912}},
        LineSpan{{-7.1902037240120489, 8.4867529395271912}, {-7.2076108830038335, 8.3789451772127546}},
        LineSpan{{-7.2076108830038335, 8.3789451772127546}, {-7.3820328583287953, 7.2986975153056504}},
        LineSpan{{-7.3820328583287953, 7.2986975153056504}, {-7.3820339809405038, 7.2986971145173518}},
        LineSpan{{-7.3820339809405038, 7.2986971145173518}, {-7.3821820032373129, 7.2982094346449315}},
        LineSpan{{-7.3821820032373129, 7.2982094346449315}, {-7.3820328583287953, 7.2986975153056504}},
        LineSpan{{-7.3820328583287953, 7.2986975153056504}, {-7.7139098457922044, 5.2432836886924115}},
        LineSpan{{-7.7139098457922044, 5.2432836886924115}, {-7.7139098457922062, 5.243283688692415}},
    });
    const double d = 3.753745644092505;
    for (const JoinType join : kJoins) {
        CAPTURE(static_cast<int>(join));
        const auto out = offsetOf(in, d, OffsetSide::Left, join, 2.0341221181052518);
        requireClean(checkOffset(in, out, d, OffsetSide::Left, join));
        // The arc's inner offset (radius r - d about its centre) is there between 3.9 and 4.8 rad.
        const Vec2 centre{-9.7657935140949483, 3.6832692691671056};
        for (int k = 0; k <= 18; ++k) {
            const double a = 3.9 + 0.9 * k / 18.0;
            CHECK(distanceToOutput(centre + Vec2::fromAngle(a, 6.1314298168347152 - d), out) <= kTol.distance);
        }
    }
}

TEST_CASE("T-0009 round 3: a closed loop with a sub-tolerance near-reversal stays closed on its side (0xe0000119)",
          "[geometry][offset][open][t0009]") {
    // v9 seed: a micro-cluster with a near-reversal in a closed loop. The Round join at the reversal
    // wrapped round to the other side, and the Right offset came back as one open chain with a
    // 0.67 mm gap (owner decision (c): it stays on the requested side and closed).
    Contour in;
    in.closed = true;
    in.spans = {
        LineSpan{{-2688.0930095459198, -2675.2263265138004}, {-2688.0930095459198, -2678.0977115237588}},
        LineSpan{{-2688.0930095459198, -2678.0977115237588}, {-2688.0930095517188, -2678.0977115243759}},
        LineSpan{{-2688.0930095517188, -2678.0977115243759}, {-2688.0936053609507, -2678.0985146503681}},
        LineSpan{{-2688.0936053609507, -2678.0985146503681}, {-2688.0936053599016, -2678.0985146513608}},
        LineSpan{{-2688.0936053599016, -2678.0985146513608}, {-2688.0943167007817, -2678.0979507225161}},
        LineSpan{{-2688.0943167007817, -2678.0979507225161}, {-2688.0943265236151, -2678.0978924870701}},
        LineSpan{{-2688.0943265236151, -2678.0978924870701}, {-2688.0943574179801, -2678.0978541169575}},
        LineSpan{{-2688.0943574179801, -2678.0978541169575}, {-2688.0930095459198, -2678.0995265700581}},
        LineSpan{{-2688.0930095459198, -2678.0995265700581}, {-2687.2990340859446, -2678.5493170777045}},
        LineSpan{{-2687.2990340859446, -2678.5493170777045}, {-2686.5163241374321, -2678.0801969776116}},
        LineSpan{{-2686.5163241374321, -2678.0801969776116}, {-2686.577307465815, -2675.5934121983678}},
        LineSpan{{-2686.577307465815, -2675.5934121983678}, {-2686.5770682204861, -2675.5924412392169}},
        LineSpan{{-2686.5770682204861, -2675.5924412392169}, {-2686.612981011408, -2674.1387124869143}},
        LineSpan{{-2686.612981011408, -2674.1387124869143}, {-2688.0930095459198, -2675.2263265138004}},
    };
    const double d = 0.91252773458987702;
    for (const JoinType join : kJoins) {
        CAPTURE(static_cast<int>(join));
        const auto out = offsetOf(in, d, OffsetSide::Right, join);
        REQUIRE(out.size() == 1);
        CHECK(out.front().closed);
        requireClean(checkOffset(in, out, d, OffsetSide::Right, join));
    }
}
