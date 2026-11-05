// T-0128: open offsets keep the path of narrow loops and needles (a tool centre still runs along
// them) and do not cross themselves at reversal cusps.

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <string>
#include <variant>
#include <vector>

#include "carvetto/geometry/intersect.hpp"
#include "carvetto/geometry/offset.hpp"
#include "offset_chain_checks.hpp"

using namespace carvetto::geometry;
using carvetto::geometry::testing::countBackwardSteps;
using carvetto::geometry::testing::distanceToSpanExact;

namespace {

const Tolerance kTol{};  // distance 0.001
const JoinType kJoins[] = {JoinType::Round, JoinType::Sharp, JoinType::Square};

Contour polyline(const std::vector<Vec2>& pts, bool closed) {
    Contour c;
    c.closed = closed;
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        c.spans.emplace_back(LineSpan{pts[i], pts[i + 1]});
    }
    if (closed) {
        c.spans.emplace_back(LineSpan{pts.back(), pts.front()});
    }
    return c;
}

Contour translated(const Contour& c, Vec2 shift) {
    Contour out = c;
    for (Span& s : out.spans) {
        if (auto* line = std::get_if<LineSpan>(&s)) {
            line->start = line->start + shift;
            line->end = line->end + shift;
        } else if (auto* arc = std::get_if<ArcSpan>(&s)) {
            arc->centre = arc->centre + shift;
        }
    }
    return out;
}

double distanceToChains(Vec2 p, const std::vector<Contour>& chains) {
    double best = 1e300;
    for (const Contour& c : chains) {
        for (const Span& s : c.spans) {
            best = std::min(best, distanceToSpanExact(p, s));
        }
    }
    return best;
}

double distanceToInput(Vec2 p, const Contour& in) {
    double best = 1e300;
    for (const Span& s : in.spans) {
        best = std::min(best, distanceToSpanExact(p, s));
    }
    return best;
}

// Number of points of the segment a-b (sampled 200 times) farther than the tolerance from every
// chain: a stretch the tool centre would have to follow but no chain does.
int uncovered(const std::vector<Contour>& chains, Vec2 a, Vec2 b) {
    int miss = 0;
    for (int k = 0; k <= 200; ++k) {
        if (distanceToChains(a + (b - a) * (k / 200.0), chains) > kTol.distance) {
            ++miss;
        }
    }
    return miss;
}

// No span of a chain crosses or touches another one, except adjacent spans at their junction and
// spans at most three apart touching at a junction between them (the bridge of about g between two
// Round arcs whose circles miss each other by g, ADR-0012 A1; as the T-0017 verifiers' checker).
std::size_t countSelfCrossings(const std::vector<Contour>& chains) {
    const Tolerance exact{1e-9, 1e-9, 0.01};
    std::size_t count = 0;
    for (const Contour& c : chains) {
        const std::size_t n = c.spans.size();
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = i + 1; j < n; ++j) {
                const bool next = j == i + 1;
                const bool wrap = c.closed && i == 0 && j + 1 == n;
                for (const Intersection& x : intersect(c.spans[i], c.spans[j], exact).points) {
                    const double slack = 1e-9 * std::max({1.0, std::fabs(x.point.x), std::fabs(x.point.y)});
                    const bool atJunction =
                        (next && distance(x.point, endPoint(c.spans[i])) <= slack) ||
                        (wrap && distance(x.point, startPoint(c.spans[i])) <= slack) ||
                        (n == 2 && c.closed && distance(x.point, endPoint(c.spans[j])) <= slack);
                    bool nearJunction = false;
                    if (j - i <= 3 || (c.closed && n - j + i <= 3)) {
                        for (const Span& sp : c.spans) {
                            nearJunction = nearJunction || distance(x.point, startPoint(sp)) <= slack;
                        }
                    }
                    if (!atJunction && !nearJunction) {
                        ++count;
                    }
                }
            }
        }
    }
    return count;
}

// The toolpath rules every chain keeps (offset.hpp): exact junctions (bit-identical where a
// line is involved, 1e-12 of the scale between arcs), no backward step, no span no longer than
// the tolerance at an open chain's end, every point at least d from the input less the
// snapping allowance and, for Round, at most d beyond it.
void requireToolpathChains(const std::vector<Contour>& chains, const Contour& in, double d, JoinType join) {
    const double allowance =
        std::max(kTol.distance * kTol.distance / (8.0 * d), std::min(5e-7 * std::max(1.0, d), 0.1e-6 / d));
    for (const Contour& c : chains) {
        REQUIRE_FALSE(c.spans.empty());
        const std::size_t n = c.spans.size();
        for (std::size_t j = c.closed ? 0 : 1; j < n; ++j) {
            const Span& a = c.spans[(j + n - 1) % n];
            const Span& b = c.spans[j];
            const Vec2 e = endPoint(a);
            const Vec2 s = startPoint(b);
            if (std::holds_alternative<LineSpan>(a) || std::holds_alternative<LineSpan>(b)) {
                CHECK(e == s);
            } else {
                CHECK(distance(e, s) <= 1e-12 * std::max({1.0, std::fabs(e.x), std::fabs(e.y), d}));
            }
        }
        if (!c.closed) {
            CHECK((n == 1 || length(c.spans.front()) > kTol.distance));
            CHECK((n == 1 || length(c.spans.back()) > kTol.distance));
            CHECK(length(c.spans.front()) > 0.0);
        }
        std::string why;
        CHECK(countBackwardSteps(c, kTol, &why) == 0);
        CAPTURE(why);
        for (const Span& s : c.spans) {
            for (int k = 0; k <= 64; ++k) {
                const double m = distanceToInput(pointAt(s, k / 64.0), in);
                CHECK(m >= d - allowance - 1e-12 * std::max(1.0, d));
                if (join == JoinType::Round) {
                    CHECK(m <= d + allowance + 1e-12 * std::max(1.0, d));
                }
            }
        }
    }
    CHECK(countSelfCrossings(chains) == 0);
}

}  // namespace

TEST_CASE("T-0128 AC1: a channel one tool width wide plus w keeps its path for every w >= 0",
          "[geometry][offset][open][t0128]") {
    // A straight channel 3.6 mm long and 2d + w wide (a DXF slot exactly one tool width wide plus
    // coordinate noise): the offset on the inside is a needle w wide that T-0017 dropped for w up
    // to the snapping allowance (merged opposite copies) and, with Sharp/Square or as a closed
    // rectangle, up to tol / 8 (a thin closed loop). A tool centre still runs along it.
    const double L = 3.6;
    const double ws[] = {0.0, 1e-12, 1e-10, 3e-9, 1e-8, 1e-7, 3e-7, 4e-7, 1e-6, 3e-6, 1e-5};
    for (const double d : {0.1317, 1.0, 3.175}) {
        for (int shape = 0; shape < 4; ++shape) {
            if (shape == 3 && L <= 2.0 * d + 0.1) {
                continue;  // a closed rectangle shorter than the tool: nothing inside
            }
            for (const double w : ws) {
                const double H = 2.0 * d + w;
                Contour base;
                OffsetSide inside = OffsetSide::Left;
                Vec2 a{0.05, 0.5 * H};
                Vec2 b{L - d - 0.05, 0.5 * H};
                if (shape == 0) {  // U: along the bottom, up, back along the top
                    base = polyline({{0, 0}, {L, 0}, {L, H}, {0, H}}, false);
                } else if (shape == 1) {  // the same U the other way round: the channel is on the right
                    base = polyline({{0, H}, {L, H}, {L, 0}, {0, 0}}, false);
                    inside = OffsetSide::Right;
                } else if (shape == 2) {  // U with a semicircular end
                    const ArcSpan end{{L, 0.5 * H}, 0.5 * H, -0.5 * kPi, 0.5 * kPi, ArcDirection::CCW};
                    base.spans.emplace_back(LineSpan{{0, 0}, end.startPoint()});
                    base.spans.emplace_back(end);
                    base.spans.emplace_back(LineSpan{end.endPoint(), {0, H}});
                    b = Vec2{L - 0.05, 0.5 * H};
                } else {  // closed rectangle
                    base = polyline({{0, 0}, {L, 0}, {L, H}, {0, H}}, true);
                    a = Vec2{d + 0.05, 0.5 * H};
                }
                for (const Vec2 shift : {Vec2{0, 0}, Vec2{12345.678, -5014.09}}) {
                    const Contour in = translated(base, shift);
                    for (const JoinType join : kJoins) {
                        CAPTURE(d, shape, w, shift.x, static_cast<int>(join));
                        const std::vector<Contour> chains = offsetOpenContour(in, d, inside, join, kTol, 2.0);
                        CHECK(uncovered(chains, a + shift, b + shift) == 0);
                        requireToolpathChains(chains, in, d, join);
                    }
                }
            }
        }
    }
}

TEST_CASE("T-0128 AC1: a loop whose whole extent is within the tolerance is still dropped",
          "[geometry][offset][open][t0128]") {
    // A closed rectangle 2d + 5e-4 long and 2d + 1e-6 wide: the inner offset is a loop 5e-4 x 1e-6
    // mm, sub-tolerance in every direction, so there is nothing a tool would follow.
    const double d = 1.0;
    const double L = 2 * d + 5e-4;
    const double H = 2 * d + 1e-6;
    const Contour in = polyline({{0, 0}, {L, 0}, {L, H}, {0, H}}, true);
    for (const JoinType join : kJoins) {
        CAPTURE(static_cast<int>(join));
        CHECK(offsetOpenContour(in, d, OffsetSide::Left, join, kTol, 2.0).empty());
    }
}

TEST_CASE("T-0128 AC1: a 3.6 mm needle 5e-6 mm wide with coincident ends is kept (fam5 0xc5000544)",
          "[geometry][offset][open][t0128]") {
    // T-0017 round-3 verifier, fam5 (tieGap), Sharp, Left: the out-and-back needle at x 6.43..10.08,
    // y ~ -5014.0939 ends where it starts (within the tolerance), so the node walk closed it and it
    // was dropped as a thin loop; main before T-0017 kept it. Stretches of opposite copies merged
    // within the snapping allowance (x 2.29..4.38) were lost too.
    Contour in;
    in.spans.emplace_back(
        LineSpan{{4.3828924136353002, -5013.9621518492413}, {10.222405319813255, -5013.9621518492413}});
    in.spans.emplace_back(
        LineSpan{{10.222405319813255, -5013.9621518492413}, {12.049998673088634, -4984.7963839171007}});
    in.spans.emplace_back(
        LineSpan{{12.049998673088634, -4984.7963839171007}, {7.12578116803809, -4985.6380672649211}});
    in.spans.emplace_back(
        LineSpan{{7.12578116803809, -4985.6380672649211}, {6.1076925834855365, -5013.9621518492413}});
    in.spans.emplace_back(
        LineSpan{{6.1076925834855365, -5013.9621518492413}, {2.1513289549787871, -5013.9621518492413}});
    in.spans.emplace_back(
        LineSpan{{2.1513289549787871, -5013.9621518492413}, {1.3908568498168634, -4981.961618863389}});
    in.spans.emplace_back(
        LineSpan{{1.3908568498168634, -4981.961618863389}, {6.203062603875658, -4981.0665267826944}});
    in.spans.emplace_back(
        LineSpan{{6.203062603875658, -4981.0665267826944}, {6.4342830047972068, -5013.9621518492413}});
    in.spans.emplace_back(
        LineSpan{{6.4342830047972068, -5013.9621518492413}, {3.3070285268827395, -5013.9621518492413}});
    in.spans.emplace_back(
        LineSpan{{3.3070285268827395, -5013.9621518492413}, {4.3317364233392386, -5044.8922041807418}});
    in.spans.emplace_back(
        LineSpan{{4.3317364233392386, -5044.8922041807418}, {4.5340969548439691, -5045.6260810630047}});
    in.spans.emplace_back(
        LineSpan{{4.5340969548439691, -5045.6260810630047}, {4.3828924136353002, -5014.2255576144689}});
    in.spans.emplace_back(
        LineSpan{{4.3828924136353002, -5014.2255576144689}, {10.222405319813255, -5014.2255576144689}});
    in.spans.emplace_back(
        LineSpan{{10.222405319813255, -5014.2255576144689}, {8.7298903247886219, -4985.9087126188087}});
    in.spans.emplace_back(
        LineSpan{{8.7298903247886219, -4985.9087126188087}, {3.8565195223734809, -4985.7634505504748}});
    in.spans.emplace_back(
        LineSpan{{3.8565195223734809, -4985.7634505504748}, {2.1513289549787871, -5014.2255576144689}});
    in.spans.emplace_back(
        LineSpan{{2.1513289549787871, -5014.2255576144689}, {6.1076925834855365, -5014.2255576144689}});
    in.spans.emplace_back(
        LineSpan{{6.1076925834855365, -5014.2255576144689}, {6.594400893497335, -4984.8538549414534}});
    in.spans.emplace_back(
        LineSpan{{6.594400893497335, -4984.8538549414534}, {1.7951463180756022, -4985.5995886737273}});
    in.spans.emplace_back(
        LineSpan{{1.7951463180756022, -4985.5995886737273}, {3.3070285268827395, -5014.2255576144689}});
    in.spans.emplace_back(
        LineSpan{{3.3070285268827395, -5014.2255576144689}, {6.4342830047972068, -5014.2255576144689}});
    in.spans.emplace_back(
        LineSpan{{6.4342830047972068, -5014.2255576144689}, {5.7781805228377578, -5047.1661575679327}});
    in.spans.emplace_back(
        LineSpan{{5.7781805228377578, -5047.1661575679327}, {8.4881840621325004, -5047.7873261168952}});
    in.spans.emplace_back(
        LineSpan{{8.4881840621325004, -5047.7873261168952}, {10.222405319813255, -5013.9621518492413}});
    in.spans.emplace_back(
        LineSpan{{10.222405319813255, -5013.9621518492413}, {4.3828924136353002, -5013.9621518492413}});
    const double d = 0.13170020844382047;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Left, JoinType::Sharp, kTol, 1.6627767487828973);
    const double y = -5014.0938547576856;  // between the two strands, 5.3e-6 mm apart
    CHECK(uncovered(chains, {6.6, y}, {10.0, y}) == 0);
    CHECK(uncovered(chains, {2.4, y}, {3.1, y}) == 0);
    CHECK(uncovered(chains, {3.5, y}, {4.3, y}) == 0);
    requireToolpathChains(chains, in, d, JoinType::Sharp);
}

TEST_CASE("T-0128 AC2: no self-crossing at a cusp between nearly touching Round arcs (fam5 0xc500076c)",
          "[geometry][offset][open][t0128]") {
    // T-0017 round-3 verifier, fam5 (axisCusp), Round, Right, ~1e4 mm from the origin: Round arcs
    // about two vertices 2d - 1.9e-9 mm apart cross twice, 2.1e-4 mm apart; the lens between them
    // passed the pruning (inside d only by rounding), and the closed chain ran down one side of it
    // and back up the other, crossing itself at the far crossing (2.4e-8 mm loop).
    Contour in;
    in.spans.emplace_back(
        LineSpan{{9973.3181749575033, 4287.5970899958429}, {9978.5020255591153, 4295.935463921991}});
    in.spans.emplace_back(
        LineSpan{{9978.5020255591153, 4295.935463921991}, {9973.3181749575033, 4287.8970899958431}});
    in.spans.emplace_back(
        LineSpan{{9973.3181749575033, 4287.8970899958431}, {9973.3181749575033, 4280.3467060387984}});
    in.spans.emplace_back(
        LineSpan{{9973.3181749575033, 4280.3467060387984}, {9990.4961596835365, 4280.3467060387984}});
    in.spans.emplace_back(
        LineSpan{{9990.4961596835365, 4280.3467060387984}, {9990.4961596835365, 4288.0707740670941}});
    in.spans.emplace_back(
        LineSpan{{9990.4961596835365, 4288.0707740670941}, {9985.0524095126821, 4295.935463921991}});
    in.spans.emplace_back(
        LineSpan{{9985.0524095126821, 4295.935463921991}, {9990.4961596835365, 4287.770774067094}});
    const double d = 3.2751919785224555;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Right, JoinType::Round, kTol, 2.3698544420767025);
    CHECK(chains.size() == 2);
    requireToolpathChains(chains, in, d, JoinType::Round);
}

TEST_CASE("T-0128 AC2: Round cusps between arcs about vertices 2d + g apart neither cross nor open",
          "[geometry][offset][open][t0128]") {
    // The T-0017 round-3 verifier's synthetic cusp (which = 1): the apex of a Lambda and of a V,
    // 2d + g apart, joined round the right. For g < 0 the Round arcs about the apexes overlap in a
    // lens 2 sqrt(d |g|) long; far from the origin the chain crossed itself there (21 of 732 in
    // the sweep), and main before T-0017 left junction gaps instead. Neither may come back.
    const double d = 1.0328719931380388;
    const double s = 3.0;
    std::vector<double> gaps{0.0};
    for (int k = 0; k <= 12; ++k) {
        gaps.push_back(1e-6 * std::pow(10.0, -0.5 * k));
        gaps.push_back(-1e-6 * std::pow(10.0, -0.5 * k));
    }
    for (const double g : gaps) {
        const double H = 2.0 * d + g;
        const double r = 2 * s + 2 * d;
        const Contour base =
            polyline({{-s, -s}, {0, 0}, {s, -s}, {r, 0}, {r, H}, {s, H + s}, {0, H}, {-s, H + s}}, false);
        for (const Vec2 shift :
             {Vec2{0, 0}, Vec2{1e3, 1e3}, Vec2{-1e4, 1e4}, Vec2{1e4, 1e4}, Vec2{12345.678, -9876.54321}}) {
            const Contour in = translated(base, shift);
            for (const OffsetSide side : {OffsetSide::Left, OffsetSide::Right}) {
                CAPTURE(g, shift.x, shift.y, static_cast<int>(side));
                const std::vector<Contour> chains =
                    offsetOpenContour(in, d, side, JoinType::Round, kTol, 3.8787999757578069);
                requireToolpathChains(chains, in, d, JoinType::Round);
            }
        }
    }
}
