// T-0128: open offsets keep the path of narrow loops and needles (a tool centre still runs along
// them) and do not cross themselves at reversal cusps. Round 2: the recovery is near-linear in the
// number of stretches put back, an isolated stray shorter than 10 x the tolerance is not returned
// (owner decision 2026-11-05), and the regression seeds of ADR-0012 A3 are tests. T-0172 (end of
// file): long strays stay whole, and the rules the T-0128 verifiers' mutants showed untested.

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <variant>
#include <vector>

#include "carvetto/geometry/intersect.hpp"
#include "carvetto/geometry/offset.hpp"
#include "offset_chain_checks.hpp"
#include "offset_junction.hpp"
// Last: on Windows it includes <windows.h> (macros such as near/far/small).
#include "cpu_scaling.hpp"

using namespace carvetto::geometry;
using carvetto::geometry::testing::countBackwardSteps;
using carvetto::geometry::testing::countOverlaps;
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
// A second crossing of two adjacent spans at a reversal junction (turning back by more than a
// right angle), within the tolerance of the junction along both, counts only when it stands off
// the junction by more than rounding: distance x sine of the crossing angle above 100 rounding
// units of the coordinates and lengths. These are exactly the crossings removeCuspLoops leaves
// (offset.hpp, ADR-0012 A3 (e); T-0172: the filter applied to every adjacent pair before). Below
// the bound the spans are tangent at the junction to rounding (two circles that touch: their
// computed circles overlap or miss by ~1e-16 mm, and where they "cross" moves by ~1e-8 mm with the
// last bit of a radius or an angle, so it differs between platforms' libm, e.g. MSVC against
// glibc at g = 0 below).
std::size_t countSelfCrossings(const std::vector<Contour>& chains) {
    const Tolerance exact{1e-9, 1e-9, 0.01};
    const double eps = std::numeric_limits<double>::epsilon();
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
                    bool noise = false;
                    if (next || wrap) {
                        const Span& si = c.spans[i];
                        const Span& sj = c.spans[j];
                        const double li = length(si);
                        const double lj = length(sj);
                        const double sine = std::fabs(tangentAt(si, x.tA).cross(tangentAt(sj, x.tB)));
                        const double unit =
                            eps * std::max({1.0, std::fabs(x.point.x), std::fabs(x.point.y), li, lj});
                        // A junction from span `from` (ending, crossing at tFrom) to span `to`
                        // (starting, crossing at tTo), as removeCuspLoops sees it.
                        auto cuspNoise = [&](const Span& from, double tFrom, double lFrom, const Span& to,
                                             double tTo, double lTo) {
                            const bool reversal = tangentAt(from, 1.0).dot(tangentAt(to, 0.0)) < 0.0;
                            const bool withinTol =
                                (1.0 - tFrom) * lFrom <= kTol.distance && tTo * lTo <= kTol.distance;
                            return reversal && withinTol &&
                                   distance(x.point, startPoint(to)) * sine <= 100.0 * unit;
                        };
                        noise = (next && cuspNoise(si, x.tA, li, sj, x.tB, lj)) ||
                                (wrap && cuspNoise(sj, x.tB, lj, si, x.tA, li));
                    }
                    if (!atJunction && !nearJunction && !noise) {
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

namespace {

#ifdef NDEBUG
constexpr bool kRelease = true;
#else
constexpr bool kRelease = false;
#endif

// Windows CI Release (MSVC) runs the offset 2.5-3x slower than the Linux machine the absolute
// bound was measured on (as in toolpath_residual_scale_test.cpp); the ratio is the criterion.
#ifdef _WIN32
constexpr double kAbsSlack = 3.0;
#else
constexpr double kAbsSlack = 1.0;
#endif

// Open chains no longer than `limit` that meet no other chain at either end (no other chain within
// tol / 2 of either end point): isolated pieces.
std::size_t isolatedShortChains(const std::vector<Contour>& chains, double limit) {
    std::size_t count = 0;
    for (std::size_t a = 0; a < chains.size(); ++a) {
        const Contour& c = chains[a];
        double len = 0.0;
        for (const Span& s : c.spans) {
            len += length(s);
        }
        if (c.closed || len > limit) {
            continue;
        }
        bool meets = false;
        for (const Vec2 end : {startPoint(c.spans.front()), endPoint(c.spans.back())}) {
            for (std::size_t b = 0; b < chains.size() && !meets; ++b) {
                for (const Span& s : chains[b].spans) {
                    meets = meets || (b != a && distanceToSpanExact(end, s) <= 0.5 * kTol.distance);
                }
            }
        }
        count += meets ? 0 : 1;
    }
    return count;
}

// A boustrophedon hatch of n lines 10 mm long at pitch exactly 2d (a raster one tool width apart):
// the offset on its left runs both ways along the middle between neighbouring lines, so every
// middle is dropped as opposite copies and put back.
Contour hatch(int n, double d) {
    std::vector<Vec2> pts;
    for (int i = 0; i < n; ++i) {
        const double y = 2.0 * d * i;
        if (i % 2 == 0) {
            pts.push_back({0, y});
            pts.push_back({10, y});
        } else {
            pts.push_back({10, y});
            pts.push_back({0, y});
        }
    }
    return polyline(pts, false);
}

struct SeedCase {
    Contour in;
    double d = 0.0;
    double mitre = 2.0;
};

// fam0 0x71280859 (T-0016 verifier generators), 16 spans.
SeedCase fam0_71280859() {
    SeedCase c;
    c.d = 0.056068419344742035;
    c.mitre = 2;
    Contour& in = c.in;
    in.spans.emplace_back(
        LineSpan{{32.920864914418615, -31.439726469852104}, {32.957338998794228, -31.546770270567905}});
    in.spans.emplace_back(
        ArcSpan{{32.647958548548516, -31.547237455891189}, 0.30938080298616516, 0.0015100662236576441, -2.6428727729581682,
                ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{32.376261699596533, -31.695214830938614}, {32.357702689161762, -31.808621611576047}});
    in.spans.emplace_back(
        LineSpan{{32.357702689161762, -31.808621611576047}, {32.254364954933969, -31.777361957114032}});
    in.spans.emplace_back(
        LineSpan{{32.254364954933969, -31.777361957114032}, {32.18211186219807, -31.892133426783694}});
    in.spans.emplace_back(
        LineSpan{{32.18211186219807, -31.892133426783694}, {32.196244240276947, -32.016992766278342}});
    in.spans.emplace_back(
        ArcSpan{{32.175802282247311, -32.019306517742322}, 0.020572483902594942, 0.11270672354192333, -3.0085143017550813,
                ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{32.155411697435049, -32.022036196283175}, {32.294420537666049, -32.20800925994039}});
    in.spans.emplace_back(
        LineSpan{{32.294420537666049, -32.20800925994039}, {32.312736701800567, -32.18184815315017}});
    in.spans.emplace_back(
        LineSpan{{32.312736701800567, -32.18184815315017}, {32.110323751288874, -32.302203007137216}});
    in.spans.emplace_back(
        ArcSpan{{32.180336608708629, -32.309416650846693}, 0.070383498489674415, 3.0389218145456214, 6.4677922886770567,
                ArcDirection::CCW});
    in.spans.emplace_back(
        ArcSpan{{32.247148738270788, -32.298763443513309}, 0.0032831875649587611, 0.76191112558818286, -2.3142492914655821,
                ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{32.244926565979206, -32.3011803156434}, {32.383352799559184, -32.290532978056547}});
    in.spans.emplace_back(
        ArcSpan{{32.396078874108824, -32.455984909470914}, 0.16594063571707401, 1.6475622270025965, -1.6940576449528422,
                ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{32.375676567576726, -32.620666543147401}, {32.46653620971135, -32.590570881342359}});
    in.spans.emplace_back(
        ArcSpan{{32.459821097927097, -32.570297771630941}, 0.021356303604469586, -1.2509377582421279, 0.79491405387958314,
                ArcDirection::CCW});
    return c;
}

// fam4 0x2a0006b4 (T-0016 verifier generators), 29 spans.
SeedCase fam4_2a0006b4() {
    SeedCase c;
    c.d = 1.5836471025950722;
    c.mitre = 2;
    Contour& in = c.in;
    in.spans.emplace_back(
        LineSpan{{-10002.543689571876, 10000.000047461781}, {-10002.340194406126, 10000.000035921443}});
    in.spans.emplace_back(
        LineSpan{{-10002.340194406126, 10000.000035921443}, {-10002.136699240376, 10000.000025342799}});
    in.spans.emplace_back(
        LineSpan{{-10002.136699240376, 10000.000025342799}, {-10001.933204074625, 10000.000015725851}});
    in.spans.emplace_back(
        LineSpan{{-10001.933204074625, 10000.000015725851}, {-10001.729708908875, 10000.000007070597}});
    in.spans.emplace_back(
        LineSpan{{-10001.729708908875, 10000.000007070597}, {-10001.526213743125, 9999.999999377038}});
    in.spans.emplace_back(
        LineSpan{{-10001.526213743125, 9999.999999377038}, {-10001.322718577376, 9999.9999926451746}});
    in.spans.emplace_back(
        LineSpan{{-10001.322718577376, 9999.9999926451746}, {-10001.119223411626, 9999.9999868750056}});
    in.spans.emplace_back(
        LineSpan{{-10001.119223411626, 9999.9999868750056}, {-10000.915728245875, 9999.9999820665307}});
    in.spans.emplace_back(
        LineSpan{{-10000.915728245875, 9999.9999820665307}, {-10000.712233080125, 9999.9999782197519}});
    in.spans.emplace_back(
        LineSpan{{-10000.712233080125, 9999.9999782197519}, {-10000.508737914375, 9999.9999753346674}});
    in.spans.emplace_back(
        LineSpan{{-10000.508737914375, 9999.9999753346674}, {-10000.305242748625, 9999.9999734112771}});
    in.spans.emplace_back(
        LineSpan{{-10000.305242748625, 9999.9999734112771}, {-10000.101747582876, 9999.9999724495829}});
    in.spans.emplace_back(
        LineSpan{{-10000.101747582876, 9999.9999724495829}, {-9999.8982524171242, 9999.9999724495829}});
    in.spans.emplace_back(
        LineSpan{{-9999.8982524171242, 9999.9999724495829}, {-9999.6947572513745, 9999.9999734112771}});
    in.spans.emplace_back(
        LineSpan{{-9999.6947572513745, 9999.9999734112771}, {-9999.4912620856248, 9999.9999753346674}});
    in.spans.emplace_back(
        LineSpan{{-9999.4912620856248, 9999.9999753346674}, {-9999.2877669198751, 9999.9999782197519}});
    in.spans.emplace_back(
        LineSpan{{-9999.2877669198751, 9999.9999782197519}, {-9999.0842717541254, 9999.9999820665307}});
    in.spans.emplace_back(
        LineSpan{{-9999.0842717541254, 9999.9999820665307}, {-9998.8807765883739, 9999.9999868750056}});
    in.spans.emplace_back(
        LineSpan{{-9998.8807765883739, 9999.9999868750056}, {-9998.6772814226242, 9999.9999926451746}});
    in.spans.emplace_back(
        LineSpan{{-9998.6772814226242, 9999.9999926451746}, {-9998.4737862568745, 9999.999999377038}});
    in.spans.emplace_back(
        LineSpan{{-9998.4737862568745, 9999.999999377038}, {-9998.2702910911248, 10000.000007070597}});
    in.spans.emplace_back(
        LineSpan{{-9998.2702910911248, 10000.000007070597}, {-9998.0667959253751, 10000.000015725851}});
    in.spans.emplace_back(
        LineSpan{{-9998.0667959253751, 10000.000015725851}, {-9997.8633007596236, 10000.000025342799}});
    in.spans.emplace_back(
        LineSpan{{-9997.8633007596236, 10000.000025342799}, {-9997.6598055938739, 10000.000035921443}});
    in.spans.emplace_back(
        LineSpan{{-9997.6598055938739, 10000.000035921443}, {-9997.4563104281242, 10000.000047461781}});
    in.spans.emplace_back(
        LineSpan{{-9997.4563104281242, 10000.000047461781}, {-9996.4563104281242, 10016.33458841038}});
    in.spans.emplace_back(
        LineSpan{{-9996.4563104281242, 10016.33458841038}, {-9997.9563104281242, 10017.33458841038}});
    in.spans.emplace_back(
        LineSpan{{-9997.9563104281242, 10017.33458841038}, {-9997.5563104281246, 10003.16729420519}});
    in.spans.emplace_back(
        LineSpan{{-9997.5563104281246, 10003.16729420519}, {-10002.443689571875, 10003.16729420519}});
    return c;
}

// fam5 0x2a500001 (T-0016 verifier generators), 25 spans.
SeedCase fam5_2a500001() {
    SeedCase c;
    c.d = 0.80466025596839674;
    c.mitre = 3.1313215664442859;
    Contour& in = c.in;
    in.spans.emplace_back(
        LineSpan{{100005.65045117219, 18748.624537049727}, {100000.92203795791, 18745.018295249705}});
    in.spans.emplace_back(
        LineSpan{{100000.92203795791, 18745.018295249705}, {100021.36514775429, 18715.401873268023}});
    in.spans.emplace_back(
        LineSpan{{100021.36514775429, 18715.401873268023}, {100021.70793860184, 18716.08624795244}});
    in.spans.emplace_back(
        LineSpan{{100021.70793860184, 18716.08624795244}, {100001.8979803597, 18743.738664208598}});
    in.spans.emplace_back(
        LineSpan{{100001.8979803597, 18743.738664208598}, {100006.62639357398, 18747.34490600862}});
    in.spans.emplace_back(
        LineSpan{{100006.62639357398, 18747.34490600862}, {100026.04195537471, 18723.762806650579}});
    in.spans.emplace_back(
        LineSpan{{100026.04195537471, 18723.762806650579}, {100025.75657854136, 18722.458007479501}});
    in.spans.emplace_back(
        LineSpan{{100025.75657854136, 18722.458007479501}, {100006.62639357398, 18747.34490600862}});
    in.spans.emplace_back(
        LineSpan{{100006.62639357398, 18747.34490600862}, {100001.8979803597, 18743.738664208598}});
    in.spans.emplace_back(
        LineSpan{{100001.8979803597, 18743.738664208598}, {99982.592329844978, 18770.52742434066}});
    in.spans.emplace_back(
        LineSpan{{99982.592329844978, 18770.52742434066}, {99983.881237262234, 18770.564854596567}});
    in.spans.emplace_back(
        LineSpan{{99983.881237262234, 18770.564854596567}, {100003.70075028291, 18747.137549232499}});
    in.spans.emplace_back(
        LineSpan{{100003.70075028291, 18747.137549232499}, {100000.07860937138, 18744.375033485929}});
    in.spans.emplace_back(
        LineSpan{{100000.07860937138, 18744.375033485929}, {100019.42607421454, 18718.406179038444}});
    in.spans.emplace_back(
        LineSpan{{100019.42607421454, 18718.406179038444}, {100023.4237686655, 18720.650926091505}});
    in.spans.emplace_back(
        LineSpan{{100023.4237686655, 18720.650926091505}, {100004.62543921775, 18745.81882846159}});
    in.spans.emplace_back(
        LineSpan{{100004.62543921775, 18745.81882846159}, {100009.10021253186, 18749.231625490876}});
    in.spans.emplace_back(
        LineSpan{{100009.10021253186, 18749.231625490876}, {99990.049811125718, 18775.674568580449}});
    in.spans.emplace_back(
        LineSpan{{99990.049811125718, 18775.674568580449}, {99980.676332535673, 18767.537797999034}});
    in.spans.emplace_back(
        LineSpan{{99980.676332535673, 18767.537797999034}, {100001.05455177318, 18743.095402444826}});
    in.spans.emplace_back(
        LineSpan{{100001.05455177318, 18743.095402444826}, {100004.6766926847, 18745.857918191392}});
    in.spans.emplace_back(
        LineSpan{{100004.6766926847, 18745.857918191392}, {99985.228329665289, 18773.916853618324}});
    in.spans.emplace_back(
        LineSpan{{99985.228329665289, 18773.916853618324}, {99989.334855427602, 18776.025702344406}});
    in.spans.emplace_back(
        LineSpan{{99989.334855427602, 18776.025702344406}, {100008.12427013007, 18750.511256531983}});
    in.spans.emplace_back(
        LineSpan{{100008.12427013007, 18750.511256531983}, {100003.64949681595, 18747.098459502697}});
    return c;
}

// fam5 0x2a5000a6 (T-0016 verifier generators), 37 spans.
SeedCase fam5_2a5000a6() {
    SeedCase c;
    c.d = 0.53679983596294656;
    c.mitre = 2;
    Contour& in = c.in;
    in.spans.emplace_back(
        LineSpan{{-99998.388710434956, -40127.283578282433}, {-99994.216692779082, -40127.283578282433}});
    in.spans.emplace_back(
        LineSpan{{-99994.216692779082, -40127.283578282433}, {-99992.953655521007, -40158.916081590789}});
    in.spans.emplace_back(
        LineSpan{{-99992.953655521007, -40158.916081590789}, {-99996.119334553805, -40158.250255415609}});
    in.spans.emplace_back(
        LineSpan{{-99996.119334553805, -40158.250255415609}, {-99995.873514930936, -40127.283578282433}});
    in.spans.emplace_back(
        LineSpan{{-99995.873514930936, -40127.283578282433}, {-99994.886440330694, -40127.283578282433}});
    in.spans.emplace_back(
        LineSpan{{-99994.886440330694, -40127.283578282433}, {-99994.222216039794, -40093.464662070379}});
    in.spans.emplace_back(
        LineSpan{{-99994.222216039794, -40093.464662070379}, {-99991.297066316518, -40093.871063131432}});
    in.spans.emplace_back(
        LineSpan{{-99991.297066316518, -40093.871063131432}, {-99989.813142938889, -40126.209968821335}});
    in.spans.emplace_back(
        LineSpan{{-99989.813142938889, -40126.209968821335}, {-99994.6946062473, -40126.209968821335}});
    in.spans.emplace_back(
        LineSpan{{-99994.6946062473, -40126.209968821335}, {-99996.409255414619, -40092.517031873409}});
    in.spans.emplace_back(
        LineSpan{{-99996.409255414619, -40092.517031873409}, {-99995.244661793433, -40091.593298674416}});
    in.spans.emplace_back(
        LineSpan{{-99995.244661793433, -40091.593298674416}, {-99994.724591552338, -40127.283578282433}});
    in.spans.emplace_back(
        LineSpan{{-99994.724591552338, -40127.283578282433}, {-99999.180801814335, -40127.283578282433}});
    in.spans.emplace_back(
        LineSpan{{-99999.180801814335, -40127.283578282433}, {-99999.056085305303, -40160.636453913277}});
    in.spans.emplace_back(
        LineSpan{{-99999.056085305303, -40160.636453913277}, {-99992.479233354534, -40160.291024168364}});
    in.spans.emplace_back(
        LineSpan{{-99992.479233354534, -40160.291024168364}, {-99994.216692779082, -40126.209968821335}});
    in.spans.emplace_back(
        LineSpan{{-99994.216692779082, -40126.209968821335}, {-99998.388710434956, -40126.209968821335}});
    in.spans.emplace_back(
        LineSpan{{-99998.388710434956, -40126.209968821335}, {-99996.738853519564, -40096.871864668094}});
    in.spans.emplace_back(
        LineSpan{{-99996.738853519564, -40096.871864668094}, {-99995.248448594793, -40096.158231440342}});
    in.spans.emplace_back(
        LineSpan{{-99995.248448594793, -40096.158231440342}, {-99994.724591552338, -40126.209968821335}});
    in.spans.emplace_back(
        LineSpan{{-99994.724591552338, -40126.209968821335}, {-99999.180801814335, -40126.209968821335}});
    in.spans.emplace_back(
        LineSpan{{-99999.180801814335, -40126.209968821335}, {-99998.728254464557, -40094.34609230511}});
    in.spans.emplace_back(
        LineSpan{{-99998.728254464557, -40094.34609230511}, {-99996.28272640759, -40094.751436047765}});
    in.spans.emplace_back(
        LineSpan{{-99996.28272640759, -40094.751436047765}, {-99994.886440330694, -40127.283578282433}});
    in.spans.emplace_back(
        LineSpan{{-99994.886440330694, -40127.283578282433}, {-99995.873514930936, -40127.283578282433}});
    in.spans.emplace_back(
        LineSpan{{-99995.873514930936, -40127.283578282433}, {-99994.91285086704, -40092.964788967409}});
    in.spans.emplace_back(
        LineSpan{{-99994.91285086704, -40092.964788967409}, {-99995.895885516788, -40093.610112278249}});
    in.spans.emplace_back(
        LineSpan{{-99995.895885516788, -40093.610112278249}, {-99994.886440330694, -40126.209968821335}});
    in.spans.emplace_back(
        LineSpan{{-99994.886440330694, -40126.209968821335}, {-99995.873514930936, -40126.209968821335}});
    in.spans.emplace_back(
        LineSpan{{-99995.873514930936, -40126.209968821335}, {-99996.868377919105, -40095.781879614027}});
    in.spans.emplace_back(
        LineSpan{{-99996.868377919105, -40095.781879614027}, {-99997.711694677346, -40095.192969426484}});
    in.spans.emplace_back(
        LineSpan{{-99997.711694677346, -40095.192969426484}, {-99999.180801814335, -40127.283578282433}});
    in.spans.emplace_back(
        LineSpan{{-99999.180801814335, -40127.283578282433}, {-99994.724591552338, -40127.283578282433}});
    in.spans.emplace_back(
        LineSpan{{-99994.724591552338, -40127.283578282433}, {-99994.276344687532, -40162.158206085922}});
    in.spans.emplace_back(
        LineSpan{{-99994.276344687532, -40162.158206085922}, {-99996.693849833682, -40161.753727497584}});
    in.spans.emplace_back(
        LineSpan{{-99996.693849833682, -40161.753727497584}, {-99994.6946062473, -40127.283578282433}});
    in.spans.emplace_back(
        LineSpan{{-99994.6946062473, -40127.283578282433}, {-99989.813142938889, -40127.283578282433}});
    return c;
}

// fam5 0x285001c5 (T-0016 verifier generators), 9 spans.
SeedCase fam5_285001c5() {
    SeedCase c;
    c.d = 2.0339764316836457;
    c.mitre = 2.6615633061010131;
    Contour& in = c.in;
    in.spans.emplace_back(
        LineSpan{{99998.844115025, 98874.599913734564}, {99999.667181658617, 98869.857221492275}});
    in.spans.emplace_back(
        LineSpan{{99999.667181658617, 98869.857221492275}, {100038.08379462056, 98876.19613738732}});
    in.spans.emplace_back(
        LineSpan{{100038.08379462056, 98876.19613738732}, {100037.50774036873, 98877.895759442676}});
    in.spans.emplace_back(
        LineSpan{{100037.50774036873, 98877.895759442676}, {99999.667181658617, 98869.857221492275}});
    in.spans.emplace_back(
        LineSpan{{99999.667181658617, 98869.857221492275}, {99998.844115025, 98874.599913734564}});
    in.spans.emplace_back(
        LineSpan{{99998.844115025, 98874.599913734564}, {99961.753678907553, 98869.548769557616}});
    in.spans.emplace_back(
        LineSpan{{99961.753678907553, 98869.548769557616}, {99961.757215190417, 98866.920959049923}});
    in.spans.emplace_back(
        LineSpan{{99961.757215190417, 98866.920959049923}, {99994.836054162239, 98873.904338100911}});
    in.spans.emplace_back(
        LineSpan{{99994.836054162239, 98873.904338100911}, {99995.659120795855, 98869.161645858621}});
    return c;
}

// fam5 0x28500192 (T-0016 verifier generators), 5 spans.
SeedCase fam5_28500192() {
    SeedCase c;
    c.d = 1.5813295947368495;
    c.mitre = 2.9791545655960556;
    Contour& in = c.in;
    in.spans.emplace_back(
        ArcSpan{{0, 0}, 4.9669270916395387, 3.2769609324857245, 1.4539197767051775, ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{0.5791965486506967, 4.9330412619087145}, {3.5714767916373145, 30.418417409668443}});
    in.spans.emplace_back(
        LineSpan{{3.5714767916373145, 30.418417409668443}, {20.275592508162411, 24.596533582060779}});
    in.spans.emplace_back(
        LineSpan{{20.275592508162411, 24.596533582060779}, {1.3888528575287036, 1.1517249684365909}});
    in.spans.emplace_back(
        ArcSpan{{0, 0}, 1.8042679021658374, 0.69233158924662819, 3.1800580331161474, ArcDirection::CCW});
    return c;
}

// fam5 0x2850005d (T-0016 verifier generators), 13 spans.
SeedCase fam5_2850005d() {
    SeedCase c;
    c.d = 1.2247891650848122;
    c.mitre = 2;
    Contour& in = c.in;
    in.spans.emplace_back(
        ArcSpan{{0, 9300.119358797534}, 2.683332280484656, -0.95341838805714874, 0.89867313754469214,
                ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{1.6707735918165854, 9302.2190701425479}, {19.711310034670621, 9324.8911540351401}});
    in.spans.emplace_back(
        LineSpan{{19.711310034670621, 9324.8911540351401}, {9.9005837479403684, 9327.534395184306}});
    in.spans.emplace_back(
        LineSpan{{9.9005837479403684, 9327.534395184306}, {1.3117123503174195, 9305.0818362077589}});
    in.spans.emplace_back(
        ArcSpan{{0, 9300.119358797534}, 5.1329106106542799, 1.312380372543589, -0.34180675484378886,
                ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{4.8359747997211517, 9298.3988591394609}, {27.227954685355034, 9290.4324417505541}});
    in.spans.emplace_back(
        LineSpan{{27.227954685355034, 9290.4324417505541}, {-25.275045711709421, 9286.0496678577729}});
    in.spans.emplace_back(
        LineSpan{{-25.275045711709421, 9286.0496678577729}, {-2.1581144653265656, 9298.5247384408231}});
    in.spans.emplace_back(
        ArcSpan{{0, 9300.119358797534}, 2.683332280484656, -2.5052365998826533, -0.11242314439818113,
                ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{2.6663928639172982, 9299.8183252068138}, {29.213328244174185, 9296.8211977436113}});
    in.spans.emplace_back(
        LineSpan{{29.213328244174185, 9296.8211977436113}, {30.48391601642065, 9300.258674214052}});
    in.spans.emplace_back(
        LineSpan{{30.48391601642065, 9300.258674214052}, {5.1239067404318961, 9300.4232519569232}});
    in.spans.emplace_back(
        ArcSpan{{0, 9300.119358797534}, 5.1329106106542799, 0.059239483819164995, -2.5736509008502311,
                ArcDirection::CW});
    return c;
}

// fam5 0x2d5004b2 (T-0016 verifier generators), 21 spans.
SeedCase fam5_2d5004b2() {
    SeedCase c;
    c.d = 1.6392970321946911;
    c.mitre = 2;
    Contour& in = c.in;
    in.spans.emplace_back(
        ArcSpan{{0, -100000}, 5.6916848775194948, -1.680841921297185, -2.8104278989031153, ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{-5.382423031895347, -100001.85062126075}, {-30.095467858211791, -100010.34762825967}});
    in.spans.emplace_back(
        LineSpan{{-30.095467858211791, -100010.34762825967}, {-25.926045645772323, -100017.6697204814}});
    in.spans.emplace_back(
        LineSpan{{-25.926045645772323, -100017.6697204814}, {-2.0971122189341336, -100001.19378706646}});
    in.spans.emplace_back(
        ArcSpan{{0, -100000}, 2.4130908020274613, -2.62408825050061, -0.65532029648813417,
                ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{1.9132257407880484, -100001.47056944195}, {25.008778200879433, -100019.22258530126}});
    in.spans.emplace_back(
        LineSpan{{25.008778200879433, -100019.22258530126}, {1.6378645235036995, -99968.218317423074}});
    in.spans.emplace_back(
        LineSpan{{1.6378645235036995, -99968.218317423074}, {-0.12440974358082611, -99997.590118377484}});
    in.spans.emplace_back(
        ArcSpan{{0, -100000}, 2.4130908020274613, 1.622375373817639, 2.6279625810627971, ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{-2.1017215938135245, -99998.814346778803}, {-26.914514984365649, -99984.816599171827}});
    in.spans.emplace_back(
        LineSpan{{-26.914514984365649, -99984.816599171827}, {-28.049410737558958, -99982.323171132564}});
    in.spans.emplace_back(
        LineSpan{{-28.049410737558958, -99982.323171132564}, {-5.1189942361820959, -99997.511792863341}});
    in.spans.emplace_back(
        ArcSpan{{0, -100000}, 5.6916848775194948, 2.689148235397484, 1.6103461663683516, ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{-0.22504654384851963, -99994.312765980008}, {-1.2341412846297171, -99968.811561469076}});
    in.spans.emplace_back(
        LineSpan{{-1.2341412846297171, -99968.811561469076}, {15.279445652222503, -100029.05377638827}});
    in.spans.emplace_back(
        LineSpan{{15.279445652222503, -100029.05377638827}, {2.8147977810352076, -100004.94693745633}});
    in.spans.emplace_back(
        ArcSpan{{0, -100000}, 5.6916848775194948, -1.0534843677992882, -2.3705021632233949,
                ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{-4.0817981511977477, -100003.96663592958}, {-23.416918698074337, -100022.75624311327}});
    in.spans.emplace_back(
        LineSpan{{-23.416918698074337, -100022.75624311327}, {-32.196151550167585, -100000.69520435485}});
    in.spans.emplace_back(
        LineSpan{{-32.196151550167585, -100000.69520435485}, {-2.3756742988191215, -100000.42329522176}});
    in.spans.emplace_back(
        ArcSpan{{0, -100000}, 2.4130908020274613, -2.9652641449829851, -1.6242730603649584,
                ArcDirection::CCW});
    return c;
}

}  // namespace

TEST_CASE("T-0128: an isolated stray shorter than 10 x tol is not returned (owner decision, fam0 0x71280859)",
          "[geometry][offset][open][t0128]") {
    // Round 1 put back a 0.00513 mm piece of dropped opposite copies here, 0.18 mm from every other
    // chain: noise to the user. Pieces that meet a chain only need to be longer than the tolerance.
    const SeedCase c = fam0_71280859();
    for (const JoinType join : kJoins) {
        for (const OffsetSide side : {OffsetSide::Left, OffsetSide::Right}) {
            CAPTURE(static_cast<int>(join), static_cast<int>(side));
            const std::vector<Contour> chains = offsetOpenContour(c.in, c.d, side, join, kTol, c.mitre);
            CHECK(isolatedShortChains(chains, 10.0 * kTol.distance) == 0);
            requireToolpathChains(chains, c.in, c.d, join);
        }
    }
    CHECK(offsetOpenContour(c.in, c.d, OffsetSide::Left, JoinType::Round, kTol, c.mitre).size() == 3);
}

TEST_CASE("T-0128: a thin loop is returned as one strand only from 10 x tol long",
          "[geometry][offset][open][t0128]") {
    // A closed rectangle (2d + e) x (2d + w): the inner offset is a loop about e long and w wide.
    // Up to an extent of tol it encloses nothing and is dropped; beyond that its strand is an
    // isolated stray, returned only when longer than 10 x tol (owner decision 2026-11-05).
    const double d = 1.0;
    for (const double w : {0.0, 1e-6, 1e-4}) {
        for (const double e : {5e-4, 1.5e-3, 5e-3, 9.9e-3, 1.2e-2, 5e-2}) {
            const Contour base = polyline({{0, 0}, {2 * d + e, 0}, {2 * d + e, 2 * d + w}, {0, 2 * d + w}}, true);
            for (const Vec2 shift : {Vec2{0, 0}, Vec2{1.2e4, -3e4}}) {
                const Contour in = translated(base, shift);
                for (const JoinType join : kJoins) {
                    CAPTURE(w, e, shift.x, static_cast<int>(join));
                    const std::vector<Contour> chains = offsetOpenContour(in, d, OffsetSide::Left, join, kTol, 2.0);
                    if (e < 10.0 * kTol.distance) {
                        CHECK(chains.empty());
                        continue;
                    }
                    REQUIRE(chains.size() == 1);
                    double len = 0.0;
                    for (const Span& s : chains[0].spans) {
                        len += length(s);
                    }
                    CHECK(len >= e - 2.0 * kTol.distance);
                    CHECK(len <= e + w + 1e-9);
                    requireToolpathChains(chains, in, d, join);
                }
            }
        }
    }
}

TEST_CASE("T-0128: only the part of dropped opposite copies exactly d from the input is put back",
          "[geometry][offset][open][t0128]") {
    // A comb whose slots are exactly 2d wide, Sharp/Square: the walls' offsets run both ways along
    // each slot's middle and are dropped; their mitre extensions continue over the slot up to the
    // teeth's top offset (y = 11), up to 1.1 mm beyond d (corner regions). Only the walls' own
    // extent (RawChain::exact) is a path that touches the input; the rest is not put back.
    const double d = 1.0;
    std::vector<Vec2> pts{{0, 10}};
    double x = 0.0;
    for (int i = 0; i < 3; ++i) {
        pts.push_back({x + 1, 10});
        pts.push_back({x + 1, 0});
        pts.push_back({x + 1 + 2 * d, 0});
        pts.push_back({x + 1 + 2 * d, 10});
        x += 1 + 2 * d;
    }
    pts.push_back({x + 1, 10});
    const Contour in = polyline(pts, false);
    const double allowance = std::min(5e-7 * std::max(1.0, d), 0.1e-6 / d);
    for (const JoinType join : kJoins) {
        CAPTURE(static_cast<int>(join));
        const std::vector<Contour> chains = offsetOpenContour(in, d, OffsetSide::Left, join, kTol, 2.0);
        for (int i = 0; i < 3; ++i) {
            const double mid = 2.0 + 3.0 * i;
            CHECK(uncovered(chains, {mid, 1.05}, {mid, 9.95}) == 0);
            // The chain put back along the middle stops where the walls end (y = 10) and stays d
            // from them.
            for (const Contour& c : chains) {
                const bool alongMiddle = std::all_of(c.spans.begin(), c.spans.end(), [&](const Span& s) {
                    return std::fabs(startPoint(s).x - mid) <= 1e-9 && std::fabs(endPoint(s).x - mid) <= 1e-9;
                });
                if (!alongMiddle) {
                    continue;
                }
                for (const Span& s : c.spans) {
                    for (int k = 0; k <= 64; ++k) {
                        const Vec2 p = pointAt(s, k / 64.0);
                        CHECK(p.y <= 10.0 + 1e-9);
                        CHECK(distanceToInput(p, in) <= d + allowance);
                    }
                }
            }
        }
        CHECK(countOverlaps(chains, 1e-6) == 0);
    }
}

TEST_CASE("T-0128: strays are put back only where no chain covers them (fam4 0x2a0006b4)",
          "[geometry][offset][open][t0128]") {
    // A slot nearly 2d wide at ~1e4 mm whose floor is a fine polyline: the offsets of its segments
    // are dropped as opposite copies and put back one by one, each only where the strays put back
    // before it leave it uncovered, so neighbouring strays meet but never run along each other.
    const SeedCase c = fam4_2a0006b4();
    for (const JoinType join : kJoins) {
        CAPTURE(static_cast<int>(join));
        const std::vector<Contour> chains = offsetOpenContour(c.in, c.d, OffsetSide::Left, join, kTol, c.mitre);
        CHECK(countOverlaps(chains, 1e-6) == 0);
        requireToolpathChains(chains, c.in, c.d, join);
        // The middle of the slot where it is at least 2d wide (x -10001.5 .. -9999.0) stays covered.
        const double y = 10000.0 + c.d;
        CHECK(uncovered(chains, {-10001.5, y}, {-9999.0, y}) == 0);
    }
}

TEST_CASE("T-0128: ADR-0012 A3 regression seeds keep the toolpath rules (fam5)",
          "[geometry][offset][open][t0128]") {
    // Found on intermediate versions of T-0128 (ADR-0012 A3): 0x2a500001 and 0x2a5000a6 (a loop's
    // strand 1.03e-6 mm beyond d), 0x285001c5 (mitre copies put back 8.4e-6 mm beyond d),
    // 0x28500192 (cutting a cusp at a rounding-noise crossing of two touching circles made the
    // chain cross itself: the stand-off test), 0x2850005d (arcs at ~1e4 mm 7.7e-7 mm apart that
    // exactJunctions cannot join), 0x2d5004b2 (a strand at 1e5 mm closed over a 1e-9 x scale gap
    // dipped 1.56e-6 mm inside d).
    const SeedCase cases[] = {fam5_2a500001(), fam5_2a5000a6(), fam5_285001c5(),
                              fam5_28500192(), fam5_2850005d(), fam5_2d5004b2()};
    int k = 0;
    for (const SeedCase& c : cases) {
        for (const JoinType join : kJoins) {
            for (const OffsetSide side : {OffsetSide::Left, OffsetSide::Right}) {
                CAPTURE(k, static_cast<int>(join), static_cast<int>(side));
                const std::vector<Contour> chains = offsetOpenContour(c.in, c.d, side, join, kTol, c.mitre);
                requireToolpathChains(chains, c.in, c.d, join);
            }
        }
        ++k;
    }
}

TEST_CASE("T-0128: putting strays back is near-linear in their number (hatch at pitch 2d)",
          "[geometry][offset][open][t0128][perf]") {
    // Round 1 compared every sample of a stray with every stray put back before it: 2.94 s for 3,000
    // lines (main 0.037 s). Release, 1,000 / 4,000 lines: about 0.010 / 0.066 s (main 0.008 /
    // 0.057 s). Linear cost gives a ratio of ~4, quadratic ~16 (round 1 in Debug: 11).
    const double d = 1.0;
    const int nSmall = kRelease ? 1000 : 250;
    const int nLarge = 4 * nSmall;
    const Contour cs = hatch(nSmall, d);
    const Contour cl = hatch(nLarge, d);
    std::vector<Contour> rs;
    std::vector<Contour> rl;
    const carvetto::test::ScalingTimes t = carvetto::test::interleavedCpuSeconds(
        3, [&] { rs = offsetOpenContour(cs, d, OffsetSide::Left, JoinType::Round, kTol); },
        [&] { rl = offsetOpenContour(cl, d, OffsetSide::Left, JoinType::Round, kTol); });
    INFO("CPU seconds: " << nSmall << " lines " << t.smallSeconds << ", " << nLarge << " lines "
                         << t.largeSeconds << ", ratio " << t.ratio());
    // The middle above every line run left to right (the U turns at x = 10 enclose it) is put back
    // and covered.
    CHECK(rl.size() >= static_cast<std::size_t>(nLarge / 2));
    for (const int i : {0, nLarge / 2, nLarge - 2}) {
        const double y = d * (2 * i + 1);
        CHECK(uncovered(rl, {1.0, y}, {9.0, y}) == 0);
    }
    CHECK(t.ratio() < 8.0);
    if constexpr (kRelease) {
        CHECK(t.largeSeconds < 0.25 * kAbsSlack);
    }
}

// --- T-0172: stray recovery follow-ups ----------------------------------------------------------

namespace {

// Deterministic on every platform (no <random> distributions).
struct Lcg {
    std::uint64_t state;
    double uniform(double lo, double hi) {
        state = state * 6364136223846793005ULL + 1442695040888963407ULL;
        return lo + (hi - lo) * static_cast<double>(state >> 11) * (1.0 / 9007199254740992.0);
    }
};

double chainLength(const Contour& c) {
    double len = 0.0;
    for (const Span& s : c.spans) {
        len += length(s);
    }
    return len;
}

}  // namespace

TEST_CASE("T-0172 (a): a stray of more than 512 spans comes back whole, no span dropped",
          "[geometry][offset][open][t0128][t0172]") {
    // A channel exactly 2d wide whose walls are polylines of M unit segments (a finely segmented
    // DXF slot): the walls' offsets run both ways along the middle and are put back as one stray of
    // ~M spans. T-0128 returned a stray of more than 512 spans as one chain per span (Profile cut
    // each separately: one plunge per mm) and dropped its spans no longer than the tolerance.
    const double d = 1.0;
    const int M = 600;
    for (const bool closed : {false, true}) {
        std::vector<Vec2> pts;
        for (int i = 0; i <= M; ++i) {
            pts.push_back({static_cast<double>(i), 0.0});
        }
        if (closed) {
            pts.push_back({M + 2 * d, 0.0});
            pts.push_back({M + 2 * d, 2 * d});
        }
        for (int i = M; i >= 0; --i) {
            pts.push_back({static_cast<double>(i), 2 * d});
        }
        if (closed) {
            pts.push_back({-2 * d, 2 * d});
            pts.push_back({-2 * d, 0.0});
        }
        const Contour in = polyline(pts, closed);
        for (const JoinType join : kJoins) {
            CAPTURE(closed, static_cast<int>(join));
            const std::vector<Contour> chains = offsetOpenContour(in, d, OffsetSide::Left, join, kTol, 2.0);
            std::size_t along = 0;
            double len = 0.0;
            for (const Contour& c : chains) {
                const bool onMiddle = std::all_of(c.spans.begin(), c.spans.end(), [&](const Span& s) {
                    return std::fabs(startPoint(s).y - d) <= 1e-9 && std::fabs(endPoint(s).y - d) <= 1e-9;
                });
                if (!onMiddle) {
                    continue;
                }
                ++along;
                len += chainLength(c);
                for (std::size_t k = 1; k < c.spans.size(); ++k) {
                    CHECK(endPoint(c.spans[k - 1]) == startPoint(c.spans[k]));
                }
            }
            CHECK(along >= 1);
            CHECK(along <= static_cast<std::size_t>((M + 511) / 512));
            CHECK(len >= M - 1.0 - 1e-9);
            CHECK(uncovered(chains, {0.05, d}, {M - 1.05, d}) == 0);
        }
    }
}

namespace {

// The pairwise test selfCrossingCuts replaced, applied greedily: a new part starts at the first
// span that meets (detail::spansMeet: carrierIntersections + inUnit) an earlier span of its part
// other than its neighbour.
std::vector<std::size_t> greedyCuts(const std::vector<Span>& spans) {
    std::vector<std::size_t> cuts;
    std::size_t from = 0;
    for (std::size_t i = 0; i < spans.size(); ++i) {
        bool hit = false;
        for (std::size_t j = from; j + 2 <= i && !hit; ++j) {
            hit = detail::spansMeet(spans[j], spans[i]);
        }
        if (hit) {
            cuts.push_back(i);
            from = i;
        }
    }
    return cuts;
}

// A spiral of n line spans (never meets itself).
std::vector<Span> lineSpiral(int n) {
    std::vector<Span> spans;
    Vec2 prev{1.0, 0.0};
    for (int i = 1; i <= n; ++i) {
        const double a = 0.05 * i;
        const Vec2 q = Vec2::fromAngle(a, 1.0 + 0.02 * a);
        spans.emplace_back(LineSpan{prev, q});
        prev = q;
    }
    return spans;
}

// A spiral of n line spans 0.05 long at pitch 0.2 (never meets itself; each span's box meets only
// its neighbours').
std::vector<Span> evenSpiral(int n) {
    std::vector<Span> spans;
    const double pitch = 0.2 / (2.0 * kPi);
    double angle = 0.0;
    Vec2 prev{1.0, 0.0};
    for (int i = 0; i < n; ++i) {
        angle += 0.05 / (1.0 + pitch * angle);
        const Vec2 q = Vec2::fromAngle(angle, 1.0 + pitch * angle);
        spans.emplace_back(LineSpan{prev, q});
        prev = q;
    }
    return spans;
}

// A strand going back and forth along one line, span lengths log-uniform over 1e-6..10 (the
// T-0172 round-1 verifier's sxbf: collinear nested boxes; parallel lines never meet).
std::vector<Span> backAndForth(int n, std::uint64_t seed) {
    Lcg rng{seed};
    std::vector<Span> spans;
    double x = 0.0;
    for (int i = 0; i < n; ++i) {
        const double len = std::exp(rng.uniform(std::log(1e-6), std::log(10.0)));
        const double next = i % 2 == 0 ? x + len : x - len;
        spans.emplace_back(LineSpan{{x, 0.0}, {next, 0.0}});
        x = next;
    }
    return spans;
}

// The box of `s` grown by detail::meetReach.
Box2 reachBox(const Span& s) {
    return bounds(s).inflated(detail::meetReach(s));
}

}  // namespace

TEST_CASE("T-0172 (a): a strand is cut where it would cross itself, as by the pairwise test",
          "[geometry][offset][open][t0172]") {
    // detail::selfCrossingCuts (the step-7 and loop-strand check) against the pairwise test it
    // replaced, on random strands of lines and arcs.
    Lcg rng{0x7172u};
    for (int trial = 0; trial < 60; ++trial) {
        std::vector<Span> spans;
        const double scale = trial % 3 == 0 ? 1.0 : (trial % 3 == 1 ? 1e-3 : 1e3);
        const Vec2 shift = trial % 4 == 0 ? Vec2{3e5, -2e5} : Vec2{0, 0};
        auto at = [&](double x, double y) { return shift + Vec2{x, y} * scale; };
        Vec2 p = at(rng.uniform(-5, 5), rng.uniform(-5, 5));
        const int n = 3 + trial;
        for (int i = 0; i < n; ++i) {
            const Vec2 q = at(rng.uniform(-5, 5), rng.uniform(-5, 5));
            if (rng.uniform(0, 1) < 0.3) {
                const double sweep = rng.uniform(0.3, 2.0);
                const Vec2 chord = q - p;
                const double r = chord.length() / (2.0 * std::sin(0.5 * sweep));
                const double h = std::sqrt(r * r - 0.25 * chord.lengthSquared());
                const Vec2 centre = (p + q) * 0.5 + chord.perp() / chord.length() * h;
                ArcSpan arc{centre, r, (p - centre).angle(), (q - centre).angle(), ArcDirection::CCW};
                arc.centre = p - Vec2::fromAngle(arc.startAngle, r);  // starts exactly at p
                spans.emplace_back(arc);
                p = arc.endPoint();
            } else {
                spans.emplace_back(LineSpan{p, q});
                p = q;
            }
        }
        CAPTURE(trial);
        CHECK(detail::selfCrossingCuts(spans) == greedyCuts(spans));
    }
    // Crossings far back along the strand: a spiral, then chords back across it.
    for (const int turns : {40, 300}) {
        std::vector<Span> spans = lineSpiral(turns);
        const Vec2 end = endPoint(spans.back());
        spans.emplace_back(LineSpan{end, {-0.3, 0.2}});
        spans.emplace_back(LineSpan{{-0.3, 0.2}, {3.0, 3.5}});
        spans.emplace_back(LineSpan{{3.0, 3.5}, {-4.0, 0.1}});
        const std::vector<std::size_t> expected = greedyCuts(spans);
        CAPTURE(turns);
        REQUIRE_FALSE(expected.empty());
        CHECK(expected.front() >= static_cast<std::size_t>(turns));
        CHECK(detail::selfCrossingCuts(spans) == expected);
    }
    // Strands that cross themselves again and again: cut after cut, each part tested afresh.
    std::vector<Span> zig;
    for (int i = 0; i < 200; ++i) {
        const double x = 0.37 * i;
        zig.emplace_back(LineSpan{{x, i % 2 ? 1.0 : -1.0}, {x + 1.5, i % 2 ? -1.0 : 1.0}});
        zig.emplace_back(LineSpan{{x + 1.5, i % 2 ? -1.0 : 1.0}, {x + 0.37, i % 2 ? -1.0 : 1.0}});
    }
    const std::vector<std::size_t> zigCuts = greedyCuts(zig);
    CHECK(zigCuts.size() > 50);
    CHECK(detail::selfCrossingCuts(zig) == zigCuts);
}

TEST_CASE("T-0172 (a): near-tangent spans meet in the self-crossing cuts exactly as in the pairwise test",
          "[geometry][offset][open][t0172]") {
    // carrierIntersections treats near-tangent carriers as touching: circles up to 1e-9 mm apart
    // (outside or inside), a line up to 1.25e-10 x r beyond a circle, plus the rounding of its
    // discriminant's cancellation (a long line by a small circle). selfCrossingCuts tests only
    // spans whose boxes, grown by detail::meetReach, meet, so every pair spansMeet accepts must
    // have such boxes meeting (round 1 grew them by inUnit's slack only and missed an arc 5e-10
    // mm from another).
    const double pi = 3.14159265358979323846;
    auto probe = [&](const Span& a, const Span& b) {
        const bool meets = detail::spansMeet(a, b);
        // As a strand: the two spans are not neighbours (a far line between them).
        const std::vector<Span> strand{a, LineSpan{{-7e5, 9e5}, {-7e5 + 1, 9e5}}, b};
        CHECK(detail::selfCrossingCuts(strand).empty() == !meets);
        if (meets) {
            CHECK(reachBox(a).intersects(reachBox(b)));
        }
        return meets;
    };
    // Hand-made pairs at gaps up to and beyond the tolerances, at the origin and far from it: arcs
    // of radius r touching (outside, inside) at gaps up to 1e-9 mm whatever r, a line touching a
    // circle of radius R up to 1.25e-10 x R beyond it.
    for (const Vec2 o : {Vec2{0, 0}, Vec2{1.2e4, -3e4}, Vec2{-3e5, 2e5}}) {
        for (const double gap : {0.0, 1e-11, 1e-10, 5e-10, 9e-10, 1.2e-9, 3e-9}) {
            for (const double r : {1.0, 1e-3}) {
                CAPTURE(o.x, gap, r);
                const ArcSpan top{o, r, pi / 2 - 1e-3, pi / 2 + 1e-3, ArcDirection::CCW};
                // Outside, tangent at the top.
                const ArcSpan above{o + Vec2{0, 2 * r + gap}, r, -pi / 2 - 1e-3, -pi / 2 + 1e-3,
                                    ArcDirection::CCW};
                const bool outside = probe(top, above);
                // Inside, tangent at the top.
                const ArcSpan within{o + Vec2{0, 0.5 * r - gap}, 0.5 * r, pi / 2 - 1e-3, pi / 2 + 1e-3,
                                     ArcDirection::CCW};
                const bool inside = probe(top, within);
                if (o.x == 0.0) {
                    CHECK(outside == (gap <= 1e-9));
                    CHECK(inside == (gap <= 1e-9));
                }
            }
            for (const double R : {10.0, 1e3}) {
                CAPTURE(o.x, gap, R);
                const double g = gap * R / 10.0;
                // A line above a circle of radius R.
                const bool line = probe(ArcSpan{o, R, pi / 2 - 1e-3, pi / 2 + 1e-3, ArcDirection::CCW},
                                        LineSpan{o + Vec2{-0.005, R + g}, o + Vec2{0.005, R + g}});
                if (o.x == 0.0) {
                    CHECK(line == (gap <= 1.2e-9));
                }
            }
        }
    }
    // Random near-tangent pairs: arcs outside or inside each other, lines by circles (long lines
    // by small circles too), lines meeting end to end, over scales and positions.
    Lcg rng{0x7A26u};
    int positives = 0;
    int nearMisses = 0;
    int noisy = 0;
    for (int k = 0; k < 48000; ++k) {
        auto logu = [&](double lo, double hi) { return std::exp(rng.uniform(std::log(lo), std::log(hi))); };
        const double far = k % 3 == 0 ? 0.0 : (k % 3 == 1 ? 1e3 : 2e5);
        const Vec2 c1{rng.uniform(-1, 1) * far, rng.uniform(-1, 1) * far};
        const double r1 = logu(1e-3, 1e4);
        const double angle = rng.uniform(0, 2 * pi);
        const Vec2 u = Vec2::fromAngle(angle, 1.0);
        const double g = (rng.uniform(0, 1) < 0.5 ? -1.0 : 1.0) * logu(1e-14, 3e-9);
        const double sweep = logu(1e-6, 1.0);
        auto arcAbout = [&](Vec2 c, double r, double a, ArcDirection dir) {
            const double s0 = rng.uniform(0.0, sweep);
            return dir == ArcDirection::CCW ? ArcSpan{c, r, a - s0, a - s0 + sweep, dir}
                                            : ArcSpan{c, r, a + s0, a + s0 - sweep, dir};
        };
        const ArcDirection d1 = rng.uniform(0, 1) < 0.5 ? ArcDirection::CCW : ArcDirection::CW;
        const ArcDirection d2 = rng.uniform(0, 1) < 0.5 ? ArcDirection::CCW : ArcDirection::CW;
        Span a = arcAbout(c1, r1, angle, d1);
        Span b;
        switch (k % 6) {
            case 0: {  // outside
                const double r2 = logu(1e-3, 1e4);
                b = arcAbout(c1 + u * (r1 + r2 + g), r2, angle + pi, d2);
                break;
            }
            case 1: {  // inside (the second circle within the first)
                const double r2 = r1 * rng.uniform(0.0, 1.0);
                const double dist = std::max(1e-12 * logu(1.0, 1e3), r1 - r2 - g);
                b = arcAbout(c1 + u * dist, r2, angle, d2);
                break;
            }
            case 2:
            case 3: {  // a line by the circle, at r + g or r (1 + g'), up to 1e4 mm long
                const double h = k % 6 == 2 ? r1 + g : r1 * (1.0 + g / 3e-9 * 1.5e-10);
                const Vec2 foot = c1 + u * h;
                const double len = logu(1e-3, 1e4);
                const double t0 = rng.uniform(-1.05, 0.05);
                const Vec2 dir = u.perp();
                b = LineSpan{foot + dir * (t0 * len), foot + dir * ((t0 + 1.0) * len)};
                break;
            }
            case 4: {  // a long line by a small circle, gap up to 1e-3 mm: the discriminant's rounding
                const double r = logu(1e-4, 1.0);
                a = arcAbout(c1, r, angle, d1);
                const Vec2 foot = c1 + u * (r + logu(1e-13, 1e-3));
                const double len = logu(1.0, 1e4);
                const double t0 = rng.uniform(-1.0, 0.0);
                b = LineSpan{foot + u.perp() * (t0 * len), foot + u.perp() * ((t0 + 1.0) * len)};
                break;
            }
            default: {  // lines meeting (or nearly) end to end
                const Vec2 p = c1 + u * r1;
                a = LineSpan{p - Vec2::fromAngle(rng.uniform(0, 2 * pi), logu(1e-3, 1e3)), p};
                const Vec2 q = p + Vec2::fromAngle(rng.uniform(0, 2 * pi), g);
                b = LineSpan{q, p + Vec2::fromAngle(rng.uniform(0, 2 * pi), logu(1e-3, 1e3))};
                break;
            }
        }
        CAPTURE(k);
        if (probe(a, b)) {
            ++positives;
            const Box2 ba = bounds(a);
            const Box2 bb = bounds(b);
            nearMisses += !ba.intersects(bb);  // met only thanks to the growth
            noisy += k % 6 == 4 && !ba.inflated(1e-9).intersects(bb);  // and beyond any tolerance
        }
    }
    INFO("positives " << positives << ", met with plain boxes apart " << nearMisses << ", over 1e-9 apart "
                       << noisy);
    CHECK(positives > 10000);
    CHECK(nearMisses > 100);
}

TEST_CASE("T-0172 (a): self-crossing cuts cost near-linear time on long and collinear strands",
          "[geometry][offset][open][t0172][perf]") {
    // A spiral of spans shorter than its pitch never meets itself, and each span's box meets only
    // its neighbours': one part however long, at a cost near-linear in the span count. 16x the
    // spans: linear ~16 (measured 18-28 with cache effects), quadratic ~256. (Round 1 compared 4x
    // and measured 5.4-5.9 against a bound of 8.)
    const int nSmall = kRelease ? 4000 : 1000;
    const std::vector<Span> shortSpiral = evenSpiral(nSmall);
    const std::vector<Span> longSpiral = evenSpiral(16 * nSmall);
    std::vector<std::size_t> cs;
    std::vector<std::size_t> cl;
    const carvetto::test::ScalingTimes t = carvetto::test::interleavedCpuSeconds(
        3, [&] { cs = detail::selfCrossingCuts(shortSpiral); },
        [&] { cl = detail::selfCrossingCuts(longSpiral); }, carvetto::test::kCpuTimeFloor, 0.02);
    INFO("CPU seconds: " << nSmall << " spans " << t.smallSeconds << ", " << 16 * nSmall << " spans "
                         << t.largeSeconds << ", ratio " << t.ratio());
    CHECK(cs.empty());
    CHECK(cl.empty());
    CHECK(t.ratio() < 64.0);
    // A strand going back and forth along one line, span lengths over seven decades: nested,
    // collinear boxes (parallel lines never meet, so it is one part). Round 1's clustering grid
    // took 9.9 s and 2.8 GB at 400 spans. The pairs of boxes that meet grow faster than the
    // spans here (17 per span at 1000, 36 at 16 000), so only absolute bounds: Release 0.05 s.
    for (const int n : {400, 16000}) {
        const std::vector<Span> strand = backAndForth(n, 0xBAC4u);
        const double t0 = carvetto::test::processCpuSeconds();
        const std::vector<std::size_t> cuts = detail::selfCrossingCuts(strand);
        const double seconds = carvetto::test::processCpuSeconds() - t0;
        INFO("back and forth, " << n << " spans: " << seconds << " s");
        CHECK(cuts.empty());
        CHECK(seconds < (kRelease ? 0.5 * kAbsSlack : 10.0));
    }
}

TEST_CASE("T-0172 (b): spiral slots and spirals are put back without a blow-up in time or memory",
          "[geometry][offset][open][t0172][perf]") {
    // The T-0172 round-1 verifier's perf2 sslot / spiral: a slot exactly 2d wide along a spiral of
    // semicircles (its walls' offsets meet along the middle: one long stray of arcs), and a spiral
    // of semicircles with a U channel 2d wide beyond it and one far away. The arcs' boxes are
    // nested; round 1's clustering grid filed each into ever finer grids (sslot 400: 12.6 s and
    // 2.5 GB, main 0.47 s; spiral 700 with the far channel at 1e3: bad_alloc at 6 GB). Release now:
    // sslot 200 0.05 s, spiral 400 0.03 s, as main.
    const double d = 0.5;
    const double step = 4 * d;
    auto semicircles = [&](int n, double r0) {
        std::vector<ArcSpan> arcs;
        Vec2 centre{0, 0};
        double r = r0;
        double a0 = 0.0;
        for (int i = 0; i < n; ++i) {
            const ArcSpan arc{centre, r, a0, a0 + kPi, ArcDirection::CCW};
            arcs.push_back(arc);
            const double r2 = r + 0.5 * step;
            centre = arc.endPoint() - Vec2::fromAngle(a0 + kPi, r2);
            r = r2;
            a0 += kPi;
        }
        return arcs;
    };
    {
        const int n = kRelease ? 200 : 100;
        const double w = 2 * d;
        const std::vector<ArcSpan> mid = semicircles(n, step + w);
        Contour in;
        for (ArcSpan a : mid) {
            a.radius += 0.5 * w;
            in.spans.emplace_back(a);
        }
        ArcSpan low = mid.back();
        low.radius -= 0.5 * w;
        in.spans.emplace_back(LineSpan{endPoint(in.spans.back()), low.endPoint()});
        for (int i = n - 1; i >= 0; --i) {
            ArcSpan a = mid[static_cast<std::size_t>(i)];
            a.radius -= 0.5 * w;
            std::swap(a.startAngle, a.endAngle);
            a.direction = ArcDirection::CW;
            in.spans.emplace_back(a);
        }
        const double t0 = carvetto::test::processCpuSeconds();
        const std::vector<Contour> chains =
            offsetOpenContour(in, d, OffsetSide::Left, JoinType::Round, kTol, 2.0);
        const double seconds = carvetto::test::processCpuSeconds() - t0;
        INFO("spiral slot of " << n << " semicircles: " << seconds << " s");
        CHECK(seconds < (kRelease ? 0.5 * kAbsSlack : 10.0));
        // The middle of the slot is covered (the stray put back).
        for (std::size_t i = 1; i + 1 < mid.size(); i += 7) {
            CHECK(distanceToChains(mid[i].pointAt(0.5), chains) <= 0.5 * kTol.distance);
        }
    }
    {
        const int n = kRelease ? 700 : 300;
        Contour in;
        for (const ArcSpan& a : semicircles(n, step)) {
            in.spans.emplace_back(a);
        }
        const Vec2 e = endPoint(in.spans.back());
        const double W = 2 * d;
        const std::vector<Vec2> tail{e,
                                     e + Vec2{0, -5},
                                     e + Vec2{10, -5},
                                     e + Vec2{10, -5 - W},
                                     e + Vec2{0, -5 - W},
                                     {1e3, 1e3},
                                     {1e3 + 5, 1e3},
                                     {1e3 + 5, 1e3 + W},
                                     {1e3, 1e3 + W}};
        for (std::size_t q = 1; q < tail.size(); ++q) {
            in.spans.emplace_back(LineSpan{tail[q - 1], tail[q]});
        }
        const double t0 = carvetto::test::processCpuSeconds();
        const std::vector<Contour> chains =
            offsetOpenContour(in, d, OffsetSide::Left, JoinType::Round, kTol, 2.0);
        const double seconds = carvetto::test::processCpuSeconds() - t0;
        INFO("spiral of " << n << " semicircles: " << seconds << " s");
        CHECK(seconds < (kRelease ? 0.5 * kAbsSlack : 10.0));
        // The far channel's middle is covered (Left: its walls' offsets meet there).
        CHECK(uncovered(chains, {1e3 + 0.5, 1e3 + d}, {1e3 + 4.5, 1e3 + d}) == 0);
    }
}

TEST_CASE("T-0172 (c): a stray between tol and 10 x tol that meets a chain is returned and reaches it",
          "[geometry][offset][open][t0128][t0172]") {
    // A slot exactly 2d wide and d + e deep between two plateaus, Round: the walls' offsets along the
    // slot's middle are dropped as opposite copies; the Round arcs about the slot's top corners meet
    // on the middle at the top, so the stray put back there meets that chain. It is shorter than
    // 10 x tol (the minimum for an isolated stray, owner decision 2026-11-05) but longer than tol,
    // so it is returned, widened into the covered top so that it reaches the chain (ADR-0012 A3 (d)).
    for (const double d : {1.0, 0.5}) {
        for (const double e : {0.003, 0.006, 0.009}) {
            const double D = d + e;
            const Contour base =
                polyline({{0, D}, {1, D}, {1, 0}, {1 + 2 * d, 0}, {1 + 2 * d, D}, {2 + 2 * d, D}}, false);
            for (const Vec2 shift : {Vec2{0, 0}, Vec2{1.2e4, -3e4}}) {
                const Contour in = translated(base, shift);
                CAPTURE(d, e, shift.x);
                const std::vector<Contour> chains =
                    offsetOpenContour(in, d, OffsetSide::Left, JoinType::Round, kTol, 2.0);
                const Vec2 mid = Vec2{1 + d, 0.0} + shift;
                CHECK(uncovered(chains, mid + Vec2{0, d + 1e-4}, mid + Vec2{0, D - 1e-4}) == 0);
                std::size_t strays = 0;
                for (std::size_t a = 0; a < chains.size(); ++a) {
                    const Contour& c = chains[a];
                    const bool onMiddle = std::all_of(c.spans.begin(), c.spans.end(), [&](const Span& s) {
                        return std::fabs(startPoint(s).x - mid.x) <= 1e-9 &&
                               std::fabs(endPoint(s).x - mid.x) <= 1e-9;
                    });
                    if (!onMiddle) {
                        continue;
                    }
                    ++strays;
                    const double len = chainLength(c);
                    CHECK(len > kTol.distance);
                    CHECK(len < 10.0 * kTol.distance);
                    // One end meets another chain within tol / 2.
                    std::vector<Contour> others;
                    for (std::size_t b = 0; b < chains.size(); ++b) {
                        if (b != a) {
                            others.push_back(chains[b]);
                        }
                    }
                    const double gap = std::min(distanceToChains(startPoint(c.spans.front()), others),
                                                distanceToChains(endPoint(c.spans.back()), others));
                    CHECK(gap <= 0.5 * kTol.distance);
                }
                CHECK(strays == 1);
                requireToolpathChains(chains, in, d, JoinType::Round);
            }
        }
    }
}

TEST_CASE("T-0172 (c): a thin loop gives back its longer strand",
          "[geometry][offset][open][t0128][t0172]") {
    // A closed annular slot 2d + w wide sweeping 3 rad: the inner offset is a thin loop of two arcs,
    // on radius R + d and R + d + w, joined by pieces about w long at the ends. Cut at its two
    // points farthest apart (the ends of the outer arc), the outer arc alone is the longer strand
    // (its extra length w x 2.7 rad exceeds the two end pieces), so it is the one returned.
    for (const double R : {5.0, 20.0}) {
        for (const double w : {1e-5, 5e-5}) {
            const double d = 1.0;
            const double R2 = R + 2 * d + w;
            const ArcSpan outer{{0, 0}, R2, 0.0, 3.0, ArcDirection::CCW};
            const ArcSpan inner{{0, 0}, R, 3.0, 0.0, ArcDirection::CW};
            Contour in;
            in.closed = true;
            in.spans.emplace_back(outer);
            in.spans.emplace_back(LineSpan{outer.endPoint(), inner.startPoint()});
            in.spans.emplace_back(inner);
            in.spans.emplace_back(LineSpan{inner.endPoint(), outer.startPoint()});
            for (const JoinType join : kJoins) {
                CAPTURE(R, w, static_cast<int>(join));
                const std::vector<Contour> chains =
                    offsetOpenContour(in, d, OffsetSide::Left, join, kTol, 2.0);
                REQUIRE(chains.size() == 1);
                for (const Span& s : chains[0].spans) {
                    for (int k = 0; k <= 8; ++k) {
                        CHECK(pointAt(s, k / 8.0).length() >= R + d + 0.75 * w);
                    }
                }
                requireToolpathChains(chains, in, d, join);
            }
        }
    }
}

TEST_CASE("T-0172 (c): a thin loop's strand of many spans comes back whole",
          "[geometry][offset][open][t0128][t0172]") {
    // The annular slot above with both walls polylines at the same angles (120 or 600 segments
    // over 1.2 rad), 2d + w wide at the vertices: the offsets of facing segments are parallel,
    // (2d + w) cos(0.005) - 2d ~ w / 2 apart (or closer to w), so the inner offset is a thin loop
    // whose strand has about one span per segment. Round: it comes back as one chain along the
    // whole slot, not as the loop's longest span (the fallback for a strand that would cross
    // itself).
    // (Sharp and Square give the same three chains as origin/main: the mitres at the inner wall's
    // vertices make the inner offset no thin loop.)
    const double d = 1.0;
    const double w = 5e-5;
    auto slot = [&](double R, int M) {
        const double R2 = R + 2 * d + w;
        std::vector<Vec2> pts;
        for (int k = 0; k <= M; ++k) {
            pts.push_back(Vec2::fromAngle(1.2 * k / M, R2));
        }
        for (int k = M; k >= 0; --k) {
            pts.push_back(Vec2::fromAngle(1.2 * k / M, R));
        }
        return polyline(pts, true);
    };
    auto check = [&](const Contour& in, double R, std::size_t minSpans) {
        const std::vector<Contour> chains =
            offsetOpenContour(in, d, OffsetSide::Left, JoinType::Round, kTol, 2.0);
        REQUIRE(chains.size() == 1);
        CHECK(chains[0].spans.size() > minSpans);
        // Along the middle, from d short of one end to d short of the other.
        CHECK(chainLength(chains[0]) > 0.99 * (1.2 * (R + d) - 2 * d));
        for (int k = 2; k <= 10; ++k) {
            const Vec2 middle = Vec2::fromAngle(0.1 * k, R + d);
            CHECK(distanceToChains(middle, chains) <= 1e-3);
        }
        return chains;
    };
    for (const double R : {5.0, 20.0}) {
        CAPTURE(R);
        const Contour in = slot(R, 120);
        requireToolpathChains(check(in, R, 50), in, d, JoinType::Round);
    }
    // Over 512 spans (T-0128 fell back to the longest span there too).
    check(slot(20.0, 600), 20.0, 512);
}

TEST_CASE("T-0172 (a): a stray that would cross itself is cut into consecutive runs without tiny ends",
          "[geometry][offset][open][t0172]") {
    // detail::nonCrossingRuns, step 7's fallback (never seen from an offset): runs in order,
    // together the whole strand but for spans no longer than the tolerance at their ends, none
    // crossing itself.
    auto check = [&](const std::vector<Span>& strand, std::size_t minRuns) {
        const std::vector<std::vector<Span>> runs = detail::nonCrossingRuns(strand, kTol);
        CHECK(runs.size() >= minRuns);
        std::size_t at = 0;  // next span of the strand
        for (const std::vector<Span>& run : runs) {
            REQUIRE_FALSE(run.empty());
            CHECK(length(run.front()) > kTol.distance);
            CHECK(length(run.back()) > kTol.distance);
            CHECK(greedyCuts(run).empty());
            // The run is the strand's next spans, after skipping only tiny ones.
            while (at < strand.size() && !(strand[at] == run.front())) {
                CHECK(length(strand[at]) <= kTol.distance);
                ++at;
            }
            for (const Span& s : run) {
                REQUIRE(at < strand.size());
                CHECK(strand[at] == s);
                ++at;
            }
        }
        for (; at < strand.size(); ++at) {
            CHECK(length(strand[at]) <= kTol.distance);
        }
    };
    // A zig-zag with a span shorter than the tolerance after every leg, then a chord back across
    // all of it: cut at the chord, and the first run loses the tiny span at its end.
    std::vector<Span> zig;
    Vec2 p{0, 0};
    for (int i = 0; i < 40; ++i) {
        const Vec2 q{p.x + 1.0, i % 2 ? -1.0 : 1.0};
        zig.emplace_back(LineSpan{p, q});
        const Vec2 r{q.x + 0.5 * kTol.distance, q.y};
        zig.emplace_back(LineSpan{q, r});
        p = r;
    }
    zig.emplace_back(LineSpan{p, {0, 0.3}});
    zig.emplace_back(LineSpan{{0, 0.3}, {0.0002, 0.3}});
    check(zig, 2);
    // A spiral, a tiny span, then chords back across it.
    std::vector<Span> spiral = lineSpiral(600);
    Vec2 e = endPoint(spiral.back());
    spiral.emplace_back(LineSpan{e, e + Vec2{0.4 * kTol.distance, 0}});
    e = e + Vec2{0.4 * kTol.distance, 0};
    spiral.emplace_back(LineSpan{e, {-0.3, 0.2}});
    spiral.emplace_back(LineSpan{{-0.3, 0.2}, {3.0, 3.5}});
    check(spiral, 2);
    // A figure of eight, and a strand that does not cross itself (one run, itself).
    check({LineSpan{{0, 0}, {2, 2}}, LineSpan{{2, 2}, {2, 0}}, LineSpan{{2, 0}, {0, 2}},
           LineSpan{{0, 2}, {0, 0.5}}},
          2);
    const std::vector<Span> plain = lineSpiral(50);
    const std::vector<std::vector<Span>> one = detail::nonCrossingRuns(plain, kTol);
    REQUIRE(one.size() == 1);
    CHECK(one[0] == plain);
}

TEST_CASE("T-0172 (c): strays put back are ordered by where they start along the input",
          "[geometry][offset][open][t0128][t0172]") {
    // A boustrophedon hatch of 6 lines at pitch 2d, Round, Left: the middles between lines 0-1, 2-3
    // and 4-5 are strays; the turns at x = 0 between lines 1-2 and 3-4 form one chain around the
    // left, starting at the turn after line 1. In input order: the first middle, the left chain,
    // then the other two middles (ADR-0012 §1).
    const double d = 1.0;
    const std::vector<Contour> chains =
        offsetOpenContour(hatch(6, d), d, OffsetSide::Left, JoinType::Round, kTol, 2.0);
    REQUIRE(chains.size() == 4);
    for (const int k : {0, 2, 3}) {
        const double y = k == 0 ? d : (k == 2 ? 5 * d : 9 * d);
        CAPTURE(k);
        REQUIRE(chains[static_cast<std::size_t>(k)].spans.size() == 1);
        CHECK(std::fabs(startPoint(chains[static_cast<std::size_t>(k)].spans.front()).y - y) <= 1e-9);
        CHECK(std::fabs(endPoint(chains[static_cast<std::size_t>(k)].spans.back()).y - y) <= 1e-9);
    }
    bool leftOfLines = false;
    for (const Span& s : chains[1].spans) {
        leftOfLines = leftOfLines || pointAt(s, 0.5).x < -0.5 * d;
    }
    CHECK(leftOfLines);
}

TEST_CASE("T-0172 (c): dropped opposite copies hand back the copy exactly d from the input",
          "[geometry][offset][open][t0128][t0172]") {
    // A slot exactly 2d wide whose left wall is 10 high and right wall 12, Sharp/Square: over the
    // slot's middle from y = 10 to the left top's mitre point or chamfer, the left wall's offset is
    // a corner extension (not d from the input) running against the right wall's, which is still
    // its own offset there. Of the copies running both ways, mergeOnCarriers hands back the one
    // that is exactly d from the input (RawChain::exact), whichever comes first; handing back the
    // first left the middle above y = 10 uncovered after the exact-range cut of step 7.
    const double d = 1.0;
    for (const bool mirrored : {false, true}) {
        const double yl = mirrored ? 12.0 : 10.0;
        const double yr = mirrored ? 10.0 : 12.0;
        const Contour in =
            polyline({{0, yl}, {1, yl}, {1, 0}, {1 + 2 * d, 0}, {1 + 2 * d, yr}, {2 + 2 * d, yr}}, false);
        for (const JoinType join : {JoinType::Sharp, JoinType::Square}) {
            CAPTURE(mirrored, static_cast<int>(join));
            const std::vector<Contour> chains = offsetOpenContour(in, d, OffsetSide::Left, join, kTol, 2.0);
            // Up to where the lower plateau's offset (y = 11) crosses the middle; above it the
            // single copy is part of a chain.
            CHECK(uncovered(chains, {1 + d, d + 0.05}, {1 + d, 10.0 + d - 0.05}) == 0);
            requireToolpathChains(chains, in, d, join);
        }
    }
}

TEST_CASE("T-0172 (e): an arc start ahead by more than 1e-12 x the scale is moved onto the junction",
          "[geometry][offset][open][t0128][t0172]") {
    // detail::closeJunction (exactJunction) between two arcs whose residual is 1.5..95 x 1e-12 x the
    // scale ahead of the previous end, with the second arc's radius 1e3..6e4 about a centre far from
    // the junction: no ulp of its centre or radius hits the end point exactly in most cases (the
    // centre's ulp is coarser than the point's), so it takes the nearest candidate that does not
    // start behind. Before PR #61's MSVC fix it kept the arc wherever it started ahead, leaving the
    // whole residual (up to 1e-10 x the scale) open; found only under perturbed sin/cos until now.
    Lcg rng{0xE7u};
    int fallbacks = 0;
    for (int n = 0; n < 300; ++n) {
        const Vec2 ca{rng.uniform(-1e4, 1e4), rng.uniform(-1e4, 1e4)};
        const double a0 = rng.uniform(0.0, 2.0 * kPi);
        const ArcSpan a{ca, rng.uniform(1.0, 11.0), a0, a0 + rng.uniform(0.3, 1.3), ArcDirection::CCW};
        const Vec2 j = a.endPoint();
        const Vec2 ta = tangentAt(Span{a}, 1.0);
        const double rb = rng.uniform(1e3, 6e4);
        const Vec2 cb = j - Vec2::fromAngle(rng.uniform(0.0, 2.0 * kPi), rb);
        const double scale = std::max({1.0, std::fabs(j.x), std::fabs(j.y), rb});
        const Vec2 start = j + ta * (rng.uniform(1.5, 95.0) * 1e-12 * scale);
        const double angle = (start - cb).angle();
        const ArcDirection dir = rng.uniform(0.0, 1.0) < 0.5 ? ArcDirection::CCW : ArcDirection::CW;
        const double sweep = dir == ArcDirection::CCW ? 0.01 : -0.01;
        const ArcSpan b0{cb, distance(start, cb), angle, angle + sweep, dir};
        CAPTURE(n);
        // The residual is well above 1e-12 x the scale and ahead, within the 1e-10 snapping limit.
        REQUIRE(distance(b0.startPoint(), j) > 1e-12 * scale);
        REQUIRE((b0.startPoint() - j).dot(ta) > 0.0);
        Span sa = a;
        Span sb = b0;
        detail::closeJunction(sa, sb);
        CHECK(std::get<ArcSpan>(sa) == a);  // a is never touched
        const ArcSpan& b = std::get<ArcSpan>(sb);
        if (b.startPoint() == j) {
            continue;
        }
        ++fallbacks;
        CHECK(distance(b.startPoint(), j) <= 1e-12 * scale);
        CHECK((b.startPoint() - j).dot(ta) >= 0.0);
    }
    INFO("fallbacks " << fallbacks);
    CHECK(fallbacks > 0);  // ~3 in 4 with glibc's sin/cos
}
