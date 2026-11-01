// T-0006: open-offset quality for toolpaths — cost at a large distance, exact mitres and chamfers
// at arc corners, snapped junctions.

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <tuple>
#include <variant>
#include <vector>

#include "carvetto/geometry/intersect.hpp"
#include "carvetto/geometry/offset.hpp"
#include "cpu_time.hpp"
#include "offset_chain_checks.hpp"
#include "test_rng.hpp"

using namespace carvetto::geometry;
using carvetto::test::SplitMix;

namespace {

const Tolerance kTol{};  // distance 0.001, chordal 0.01
const JoinType kJoins[] = {JoinType::Round, JoinType::Sharp, JoinType::Square};
const OffsetSide kSides[] = {OffsetSide::Left, OffsetSide::Right};
constexpr double kDeg = kPi / 180.0;

// ---------------------------------------------------------------------------------------------
// Independent geometry helpers.
// ---------------------------------------------------------------------------------------------

double distanceToSpan(Vec2 p, const Span& s) {
    if (const auto* line = std::get_if<LineSpan>(&s)) {
        const Vec2 ab = line->end - line->start;
        const double len2 = ab.lengthSquared();
        const double t = len2 > 0.0 ? std::clamp((p - line->start).dot(ab) / len2, 0.0, 1.0) : 0.0;
        return distance(p, line->start + ab * t);
    }
    const auto& arc = std::get<ArcSpan>(s);
    if (arc.containsAngle((p - arc.centre).angle(), 0.0)) {
        return std::fabs(distance(p, arc.centre) - arc.radius);
    }
    return std::min(distance(p, arc.startPoint()), distance(p, arc.endPoint()));
}

double distanceToInput(Vec2 p, const Contour& input) {
    double best = 1e300;
    for (const Span& s : input.spans) {
        best = std::min(best, distanceToSpan(p, s));
    }
    return best;
}

Contour polyline(const std::vector<Vec2>& pts) {
    Contour c;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        c.spans.emplace_back(LineSpan{pts[i - 1], pts[i]});
    }
    return c;
}

// CPU seconds since `since` (a processCpuSeconds() reading): CPU time on Windows too.
double cpuSeconds(double since) {
    return carvetto::test::processCpuSeconds() - since;
}

// ---------------------------------------------------------------------------------------------
// Junction metrics (AC3).
// ---------------------------------------------------------------------------------------------

struct JunctionStats {
    std::size_t junctions{0};
    std::size_t gaps{0};          // end/start further apart than 1e-12
    std::size_t notIdentical{0};  // a junction with a line on either side that is not bit-identical
    std::size_t backward{0};      // next span starts behind the previous end, or retraces it
    double maxGap{0.0};
};

// One junction a -> b: gap and bit-identity. Backward steps are counted per chain by
// countBackwardSteps (offset_chain_checks.hpp): starting behind the previous end, retracing it
// (a near-reversal passes only as a needle tip whose sides diverge), and short spans that run
// against both neighbours.
void checkJunction(const Span& a, const Span& b, JunctionStats& st) {
    ++st.junctions;
    const Vec2 e = endPoint(a);
    const Vec2 s = startPoint(b);
    const double gap = distance(e, s);
    st.maxGap = std::max(st.maxGap, gap);
    if (gap > 1e-12) {
        ++st.gaps;
    }
    const bool lineSide = std::holds_alternative<LineSpan>(a) || std::holds_alternative<LineSpan>(b);
    if (lineSide && !(e == s)) {
        ++st.notIdentical;
    }
}

void checkJunctions(const std::vector<Contour>& chains, JunctionStats& st) {
    for (const Contour& c : chains) {
        const std::size_t n = c.spans.size();
        for (std::size_t k = 1; k < n; ++k) {
            checkJunction(c.spans[k - 1], c.spans[k], st);
        }
        if (c.closed && n > 1) {
            checkJunction(c.spans[n - 1], c.spans[0], st);
        }
        st.backward += carvetto::geometry::testing::countBackwardSteps(c, kTol);
    }
}

// ---------------------------------------------------------------------------------------------
// Fixed-seed generators (SplitMix64, identical on every platform).
// ---------------------------------------------------------------------------------------------

double spanLength(SplitMix& rng, double d) {
    return rng.uniform(0.0, 1.0) < 0.25 ? rng.uniform(0.02, 0.09) * d : rng.uniform(0.3, 4.0) * d;
}

double sign(SplitMix& rng) {
    return rng.uniform(0.0, 1.0) < 0.5 ? -1.0 : 1.0;
}

Contour genWalk(SplitMix& rng, double d) {
    std::vector<Vec2> pts{{0.0, 0.0}};
    const int n = 3 + static_cast<int>(rng.uniform(0.0, 12.0));
    for (int i = 0; i < n; ++i) {
        pts.push_back(pts.back() + Vec2::fromAngle(rng.uniform(-kPi, kPi), spanLength(rng, d)));
    }
    return polyline(pts);
}

Contour genNearReversals(SplitMix& rng, double d) {
    std::vector<Vec2> pts{{0.0, 0.0}};
    double heading = rng.uniform(-kPi, kPi);
    const int n = 3 + static_cast<int>(rng.uniform(0.0, 7.0));
    for (int i = 0; i < n; ++i) {
        const double len =
            rng.uniform(0.0, 1.0) < 0.2 ? rng.uniform(0.02, 0.09) * d : rng.uniform(0.5, 5.0) * d;
        pts.push_back(pts.back() + Vec2::fromAngle(heading, len));
        heading += sign(rng) * rng.uniform(150.0, 179.9) * kDeg;
    }
    return polyline(pts);
}

Contour genMixed(SplitMix& rng, double d) {
    Contour c;
    Vec2 p{0.0, 0.0};
    double heading = rng.uniform(-kPi, kPi);
    const int n = 2 + static_cast<int>(rng.uniform(0.0, 9.0));
    for (int i = 0; i < n; ++i) {
        if (i > 0 && rng.uniform(0.0, 1.0) < 0.5) {
            heading += rng.uniform(-kPi, kPi);
        }
        if (rng.uniform(0.0, 1.0) < 0.5) {
            const Vec2 q = p + Vec2::fromAngle(heading, spanLength(rng, d));
            c.spans.emplace_back(LineSpan{p, q});
            p = q;
            continue;
        }
        const double r =
            rng.uniform(0.0, 1.0) < 0.2 ? rng.uniform(0.03, 0.09) * d : rng.uniform(0.2, 5.0) * d;
        const double turn = sign(rng);
        const double sweep = rng.uniform(10.0, 330.0) * kDeg;
        const Vec2 centre = p + Vec2::fromAngle(heading).perp() * (r * turn);
        const Vec2 q = centre + Vec2::fromAngle((p - centre).angle() + turn * sweep, r);
        c.spans.emplace_back(
            ArcSpan::fromCentre(centre, p, q, turn > 0.0 ? ArcDirection::CCW : ArcDirection::CW));
        heading += turn * sweep;
        p = q;
    }
    return c;
}

Contour genSpiral(SplitMix& rng, double d) {
    const double spacing = rng.uniform(0.8, 3.0) * d;
    const double r0 = rng.uniform(0.3, 2.0) * d;
    const double turns = rng.uniform(1.0, 2.5);
    const double step = rng.uniform(12.0, 40.0) * kDeg;
    const double dir = sign(rng);
    std::vector<Vec2> pts;
    for (double th = 0.0; th <= turns * kTwoPi; th += step) {
        pts.push_back(Vec2::fromAngle(dir * th, r0 + spacing * th / kTwoPi));
    }
    return polyline(pts);
}

using Generator = Contour (*)(SplitMix&, double);
const Generator kGenerators[] = {genWalk, genNearReversals, genMixed, genSpiral};

// ---------------------------------------------------------------------------------------------
// Inputs for the cost criterion (AC1).
// ---------------------------------------------------------------------------------------------

// 2000 steps of 0.5 mm, every heading drawn afresh: a dense tangle many times thicker than d.
Contour randomWalk2000() {
    SplitMix rng{20261025u};
    std::vector<Vec2> pts{{0.0, 0.0}};
    for (int i = 0; i < 2000; ++i) {
        pts.push_back(pts.back() + Vec2::fromAngle(rng.uniform(-kPi, kPi), 0.5));
    }
    return polyline(pts);
}

// 8000-span hypotrochoid (R = 5, r = 3, h = 5, scaled by 4): three self-crossing lobes.
Contour hypotrochoid8000() {
    std::vector<Vec2> pts;
    for (int k = 0; k <= 8000; ++k) {
        const double t = 3.0 * kTwoPi * k / 8000.0;
        pts.push_back(Vec2{2.0 * std::cos(t) + 5.0 * std::cos(2.0 * t / 3.0),
                           2.0 * std::sin(t) - 5.0 * std::sin(2.0 * t / 3.0)} *
                      4.0);
    }
    return polyline(pts);
}

// Text-like: 200 letter-sized loops (2-4 mm) written in one stroke in rows, 40 spans each.
Contour textLoops8000() {
    std::vector<Vec2> pts;
    for (int letter = 0; letter < 200; ++letter) {
        const double cx = (letter % 20) * 6.0;
        const double cy = -(letter / 20) * 10.0;
        const double rx = 2.0 + 0.8 * std::sin(letter * 1.3);
        const double ry = 3.0 + std::cos(letter * 0.7);
        for (int k = 0; k < 40; ++k) {
            const double t = kTwoPi * k / 40.0;
            pts.push_back(Vec2{cx + rx * std::sin(t) + 1.5 * k / 40.0, cy + ry * (1.0 - std::cos(t))});
        }
    }
    return polyline(pts);
}

// ---------------------------------------------------------------------------------------------
// Inputs for exact arc corners (AC2).
// ---------------------------------------------------------------------------------------------

// Keyhole slot: up the right side of a neck of half-width w, counter-clockwise round a head of
// radius R about the origin, down the left side. The inside is on the left.
Contour keyhole(double w, double R, double neck) {
    const double h = std::sqrt(R * R - w * w);
    Contour c;
    c.spans.emplace_back(LineSpan{{w, -h - neck}, {w, -h}});
    c.spans.emplace_back(ArcSpan::fromCentre({0.0, 0.0}, {w, -h}, {-w, -h}, ArcDirection::CCW));
    c.spans.emplace_back(LineSpan{{-w, -h}, {-w, -h - neck}});
    return c;
}

bool hasJunctionAt(const std::vector<Contour>& chains, Vec2 p, double within) {
    for (const Contour& c : chains) {
        for (const Span& s : c.spans) {
            if (distance(startPoint(s), p) <= within || distance(endPoint(s), p) <= within) {
                return true;
            }
        }
    }
    return false;
}

bool hasArcAbout(const std::vector<Contour>& chains, Vec2 centre, double radius) {
    for (const Contour& c : chains) {
        for (const Span& s : c.spans) {
            const auto* arc = std::get_if<ArcSpan>(&s);
            if (arc && distance(arc->centre, centre) < 1e-9 && std::fabs(arc->radius - radius) < 1e-9) {
                return true;
            }
        }
    }
    return false;
}

// Offset carrier of one span on side `sgn` (+1 left), as a line (point + direction) or circle.
struct Carrier {
    bool circle{false};
    Vec2 p;       // line: a point; circle: centre
    Vec2 u;       // line: unit direction of travel
    double r{0};  // circle radius
    ArcSpan arc;  // circle: the offset arc itself
};

Carrier offsetCarrier(const Span& s, double d, double sgn) {
    Carrier c;
    if (const auto* line = std::get_if<LineSpan>(&s)) {
        c.u = (line->end - line->start).normalized();
        c.p = line->end + c.u.perp() * (d * sgn);
        return c;
    }
    const auto& arc = std::get<ArcSpan>(s);
    const double towards = arc.direction == ArcDirection::CCW ? sgn : -sgn;
    c.circle = true;
    c.p = arc.centre;
    c.r = arc.radius - towards * d;
    c.arc = arc;
    c.arc.radius = c.r;
    return c;
}

std::vector<Vec2> meets(const Carrier& a, const Carrier& b) {
    std::vector<Vec2> out;
    if (!a.circle && !b.circle) {
        const double den = a.u.cross(b.u);
        if (std::fabs(den) > 1e-12) {
            out.push_back(a.p + a.u * ((b.p - a.p).cross(b.u) / den));
        }
        return out;
    }
    if (a.circle != b.circle) {
        const Carrier& l = a.circle ? b : a;
        const Carrier& c = a.circle ? a : b;
        const double along = (c.p - l.p).dot(l.u);
        const Vec2 foot = l.p + l.u * along;
        const double h2 = c.r * c.r - (foot - c.p).lengthSquared();
        if (h2 >= 0.0) {
            out.push_back(foot - l.u * std::sqrt(h2));
            out.push_back(foot + l.u * std::sqrt(h2));
        }
        return out;
    }
    const Vec2 delta = b.p - a.p;
    const double dist = delta.length();
    if (dist < 1e-12 || dist > a.r + b.r || dist < std::fabs(a.r - b.r)) {
        return out;
    }
    const Vec2 u = delta / dist;
    const double along = (a.r * a.r - b.r * b.r + dist * dist) / (2.0 * dist);
    const double h = std::sqrt(std::max(0.0, a.r * a.r - along * along));
    out.push_back(a.p + u * along + u.perp() * h);
    out.push_back(a.p + u * along - u.perp() * h);
    return out;
}

// Length by which carrier `c` must be extended beyond its end (`forward`) or before its start to
// reach `p`; an arc may use its gap up to a quarter turn. Nothing when p lies on the span itself,
// is reached only by wrapping onto the arc's other end, or lies beyond the quarter-turn cap.
std::optional<double> extension(const Carrier& c, const Span& offsetSpan, Vec2 p, bool forward) {
    if (!c.circle) {
        const double t =
            forward ? (p - endPoint(offsetSpan)).dot(c.u) : (startPoint(offsetSpan) - p).dot(c.u);
        return t >= -1e-9 ? std::optional<double>(std::max(0.0, t)) : std::nullopt;
    }
    const ArcSpan& arc = std::get<ArcSpan>(offsetSpan);
    const double sweep = std::fabs(arc.sweepAngle());
    const double a = (p - arc.centre).angle();
    const bool ccw = arc.direction == ArcDirection::CCW;
    const double fromEnd = normalizeAngle(ccw ? a - (arc.startAngle + sweep) : (arc.startAngle - sweep) - a);
    const double toStart = normalizeAngle(ccw ? arc.startAngle - a : a - arc.startAngle);
    const double phi = forward ? fromEnd : toStart;
    if (phi > kTwoPi - 1e-9) {
        return 0.0;
    }
    // ADR-0012 A1: an arc is extended over at most a quarter turn (and never past its other end).
    if (phi >= kTwoPi - sweep - 1e-9 || phi > kPi / 2.0 - 1e-9) {
        return std::nullopt;
    }
    return phi * arc.radius;
}

Span offsetOf(const Span& s, const Carrier& c) {
    if (const auto* line = std::get_if<LineSpan>(&s)) {
        const Vec2 n = c.p - line->end;
        return LineSpan{line->start + n, line->end + n};
    }
    return c.arc;
}

}  // namespace

// --- AC1: cost at a large distance relative to the feature size --------------------------------

TEST_CASE("T-0006 AC1: dense input at a large distance offsets quickly (CPU time; bounds in Release)",
          "[geometry][offset][open][t0006][perf]") {
    struct Case {
        const char* name;
        Contour input;
        std::vector<double> distances;
        double bound;  // seconds of CPU time per offset, Release build (AC1)
    };
    const Case cases[] = {{"random walk", randomWalk2000(), {1.0, 2.0}, 0.5},
                          {"hypotrochoid", hypotrochoid8000(), {2.0}, 1.0},
                          {"text loops", textLoops8000(), {2.0}, 1.0}};
    for (const Case& c : cases) {
        for (const double d : c.distances) {
            for (const JoinType join : kJoins) {
                for (const OffsetSide side : kSides) {
                    CAPTURE(c.name, d, static_cast<int>(join), static_cast<int>(side));
                    const double t0 = carvetto::test::processCpuSeconds();
                    const std::vector<Contour> chains = offsetOpenContour(c.input, d, side, join, kTol);
                    const double seconds = cpuSeconds(t0);
                    REQUIRE_FALSE(chains.empty());
#ifdef NDEBUG
                    CHECK(seconds < c.bound);
#else
                    CHECK(seconds < 40.0 * c.bound);  // Debug: only guards against blow-ups
#endif
                    // Spot-check the distance property and the junctions.
                    std::size_t k = 0;
                    for (const Contour& chain : chains) {
                        for (const Span& s : chain.spans) {
                            if (k++ % 97 == 0) {
                                CHECK(distanceToInput(pointAt(s, 0.5), c.input) >= d - 1e-6 * d);
                            }
                        }
                    }
                    JunctionStats st;
                    checkJunctions(chains, st);
                    CHECK(st.gaps == 0);
                    CHECK(st.backward == 0);
                }
            }
        }
    }
}

TEST_CASE("T-0006 AC1: cost grows about linearly with the length of a dense walk (CPU-time ratio)",
          "[geometry][offset][open][t0006][perf]") {
    // Before T-0006 a rejected piece scanned every input span within d, so doubling the walk
    // (same density) roughly quadrupled the time at d = 2. Measured as a ratio, so it holds in
    // Debug too; the repetitions keep the shorter run long enough for the CPU clock.
    SplitMix rng{7u};
    std::vector<Vec2> pts{{0.0, 0.0}};
    for (int i = 0; i < 2000; ++i) {
        pts.push_back(pts.back() + Vec2::fromAngle(rng.uniform(-kPi, kPi), 0.5));
    }
    const Contour longWalk = polyline(pts);
    const Contour shortWalk = polyline(std::vector<Vec2>(pts.begin(), pts.begin() + 1001));
    auto cost = [](const Contour& in, int reps) {
        const double t0 = carvetto::test::processCpuSeconds();
        std::size_t chains = 0;
        for (int r = 0; r < reps; ++r) {
            chains += offsetOpenContour(in, 2.0, OffsetSide::Left, JoinType::Round, kTol).size();
        }
        CHECK(chains > 0);
        return cpuSeconds(t0) / reps;
    };
    const double shortCost = std::max(cost(shortWalk, 4), 1e-4);
    const double longCost = cost(longWalk, 2);
    CAPTURE(shortCost, longCost);
    CHECK(longCost < 3.5 * shortCost);
}

// --- AC2: exact mitres and chamfers at arc corners ----------------------------------------------

TEST_CASE("T-0006 AC2: keyhole d = 0.25, Left, Sharp has true mitres at 0.304 from the vertices",
          "[geometry][offset][open][t0006]") {
    // Neck 0.534 wide into a head of diameter 1. The offset head arc (radius 0.25) is extended
    // back past its start at the first corner, then forward past its end at the second; before
    // T-0006 the second extension had only half of the remaining gap and squared the corner off.
    const double w = 0.267;
    const Contour in = keyhole(w, 0.5, 3.0);
    const double d = 0.25;
    const double h = std::sqrt(0.25 - w * w);
    const Vec2 v1{w, -h};
    const Vec2 v2{-w, -h};
    const double my = -std::sqrt(0.25 * 0.25 - (w - d) * (w - d));
    const Vec2 m1{w - d, my};
    const Vec2 m2{d - w, my};
    CHECK_THAT(distance(m1, v1), Catch::Matchers::WithinAbs(0.304, 0.0005));
    const std::vector<Contour> chains = offsetOpenContour(in, d, OffsetSide::Left, JoinType::Sharp, kTol);
    REQUIRE(chains.size() == 1);
    const Contour& c = chains.front();
    REQUIRE(c.spans.size() == 3);  // line, head arc, line: no chamfer, no round corner
    CHECK(std::holds_alternative<LineSpan>(c.spans[0]));
    CHECK(std::holds_alternative<ArcSpan>(c.spans[1]));
    CHECK(std::holds_alternative<LineSpan>(c.spans[2]));
    CHECK(distance(endPoint(c.spans[0]), m1) < 1e-12);
    CHECK(distance(startPoint(c.spans[2]), m2) < 1e-12);
    CHECK_FALSE(hasArcAbout(chains, v1, d));
    CHECK_FALSE(hasArcAbout(chains, v2, d));
    // The mitre limit still applies: with 1.2 (0.3 < 0.304) both corners are squared off.
    const std::vector<Contour> limited =
        offsetOpenContour(in, d, OffsetSide::Left, JoinType::Sharp, kTol, 1.2);
    REQUIRE(limited.size() == 1);
    CHECK(limited.front().spans.size() == 5);
    CHECK_FALSE(hasJunctionAt(limited, m1, 1e-6));
}

TEST_CASE("T-0006 AC2: keyhole d = 0.25, Left, Square chamfers both corners (no round fallback)",
          "[geometry][offset][open][t0006]") {
    const Contour in = keyhole(0.267, 0.5, 3.0);
    const std::vector<Contour> chains = offsetOpenContour(in, 0.25, OffsetSide::Left, JoinType::Square, kTol);
    REQUIRE(chains.size() == 1);
    const Contour& c = chains.front();
    REQUIRE(c.spans.size() == 5);  // line, chamfer, head arc, chamfer, line
    const double h = std::sqrt(0.25 - 0.267 * 0.267);
    for (const std::size_t k : {std::size_t{1}, std::size_t{3}}) {
        const auto* chamfer = std::get_if<LineSpan>(&c.spans[k]);
        REQUIRE(chamfer != nullptr);
        const Vec2 v{k == 1 ? 0.267 : -0.267, -h};
        // The chamfer line touches the circle of radius d about its vertex.
        const Vec2 u = (chamfer->end - chamfer->start).normalized();
        CHECK_THAT(std::fabs((v - chamfer->start).cross(u)), Catch::Matchers::WithinAbs(0.25, 1e-9));
    }
}

TEST_CASE(
    "T-0006 AC2 generative: Sharp and Square at arc corners meet exactly where the extended "
    "carriers do",
    "[geometry][offset][open][t0006][generative]") {
    // One outside corner between a line or arc and an arc (either order). The expected mitre is
    // computed independently: the offset carriers' meeting point reached first by extending the
    // incoming element past its end and the outgoing one before its start (an arc over its whole
    // gap), kept when it is within mitreLimit * d of the vertex and at least d from the input.
    SplitMix rng{60062u};
    std::size_t sharpChecked = 0;
    std::size_t sharpBeyondHalfGap = 0;
    std::size_t squareChecked = 0;
    for (int trial = 0; trial < 3000; ++trial) {
        const double d = rng.logUniform(0.2, 3.0);
        const double mitreLimit = rng.uniform(1.0, 4.0);
        const Vec2 v{0.0, 0.0};
        const double headingIn = rng.uniform(-kPi, kPi);
        const double kink = sign(rng) * rng.uniform(20.0, 170.0) * kDeg;
        // Incoming span: a line or an arc ending at v with tangent headingIn.
        Span a;
        if (rng.uniform(0.0, 1.0) < 0.4) {
            a = LineSpan{v - Vec2::fromAngle(headingIn, rng.uniform(3.0, 8.0) * d), v};
        } else {
            const double r = rng.uniform(1.3, 6.0) * d;
            const double turn = sign(rng);
            const Vec2 centre = v + Vec2::fromAngle(headingIn).perp() * (r * turn);
            const double sweep = rng.uniform(60.0, 345.0) * kDeg;
            const Vec2 start = centre + Vec2::fromAngle((v - centre).angle() - turn * sweep, r);
            a = ArcSpan::fromCentre(centre, start, v, turn > 0.0 ? ArcDirection::CCW : ArcDirection::CW);
        }
        // Outgoing arc starting at v with tangent headingIn + kink.
        const double headingOut = headingIn + kink;
        const double r = rng.uniform(1.3, 6.0) * d;
        const double turn = sign(rng);
        const Vec2 centre = v + Vec2::fromAngle(headingOut).perp() * (r * turn);
        const double sweep = rng.uniform(60.0, 345.0) * kDeg;
        const Vec2 end = centre + Vec2::fromAngle((v - centre).angle() + turn * sweep, r);
        Span b = ArcSpan::fromCentre(centre, v, end, turn > 0.0 ? ArcDirection::CCW : ArcDirection::CW);
        Contour in;
        if (rng.uniform(0.0, 1.0) < 0.5) {
            in.spans = {a, b};
        } else {  // the arc first, then a line or the arc's mirror image
            in.spans = {Span{std::get<ArcSpan>(b).reversed()}, std::holds_alternative<LineSpan>(a)
                                                                   ? Span{std::get<LineSpan>(a).reversed()}
                                                                   : Span{std::get<ArcSpan>(a).reversed()}};
        }
        // Outside side of the corner: right of a left turn.
        const Vec2 tin = tangentAt(in.spans[0], 1.0);
        const Vec2 tout = tangentAt(in.spans[1], 0.0);
        const double sgn = tin.cross(tout) > 0.0 ? -1.0 : 1.0;
        const OffsetSide side = sgn > 0.0 ? OffsetSide::Left : OffsetSide::Right;
        const Carrier ca = offsetCarrier(in.spans[0], d, sgn);
        const Carrier cb = offsetCarrier(in.spans[1], d, sgn);
        if ((ca.circle && ca.r < 0.2 * d) || (cb.circle && cb.r < 0.2 * d)) {
            continue;
        }
        const Span oa = offsetOf(in.spans[0], ca);
        const Span ob = offsetOf(in.spans[1], cb);
        std::optional<Vec2> mitre;
        double best = 1e300;
        for (const Vec2 p : meets(ca, cb)) {
            const auto ea = extension(ca, oa, p, true);
            const auto eb = extension(cb, ob, p, false);
            if (ea && eb && *ea + *eb < best) {
                best = *ea + *eb;
                mitre = p;
            }
        }
        CAPTURE(trial, d, mitreLimit, sgn);
        if (mitre && distance(*mitre, v) <= mitreLimit * d * (1.0 - 1e-9) &&
            distanceToInput(*mitre, in) >= d * (1.0 + 1e-6)) {
            const std::vector<Contour> chains =
                offsetOpenContour(in, d, side, JoinType::Sharp, kTol, mitreLimit);
            CAPTURE(mitre->x, mitre->y);
            CHECK(hasJunctionAt(chains, *mitre, 1e-9 * std::max(1.0, d)));
            CHECK_FALSE(hasArcAbout(chains, v, d));
            ++sharpChecked;
            // Count corners the old half-gap rule rejected: an arc extension longer than half
            // its gap.
            for (const auto& [c, o, fwd] : {std::tuple{ca, oa, true}, std::tuple{cb, ob, false}}) {
                if (c.circle) {
                    const double gap = kTwoPi - std::fabs(std::get<ArcSpan>(o).sweepAngle());
                    if (*extension(c, o, *mitre, fwd) / c.r > 0.5 * gap) {
                        ++sharpBeyondHalfGap;
                        break;
                    }
                }
            }
        }
        // Square: the chamfer line is tangent to the circle of radius d about the vertex, on the
        // bisector of the two offset end points; it must appear unless an extension would reach
        // farther than 1.5 d (the documented limit).
        const Vec2 e = endPoint(oa);
        const Vec2 s = startPoint(ob);
        const Vec2 bis = ((e - v) + (s - v)).normalized();
        const Carrier chamfer{false, v + bis * d, bis.perp(), 0.0, {}};
        std::optional<Vec2> h1;
        std::optional<Vec2> h2;
        double e1 = 1e300;
        double e2 = 1e300;
        for (const Vec2 p : meets(ca, chamfer)) {
            const auto ext = extension(ca, oa, p, true);
            if (ext && *ext < e1 && distance(p, v) <= 1.5 * d) {
                e1 = *ext;
                h1 = p;
            }
        }
        for (const Vec2 p : meets(cb, chamfer)) {
            const auto ext = extension(cb, ob, p, false);
            if (ext && *ext < e2 && distance(p, v) <= 1.5 * d) {
                e2 = *ext;
                h2 = p;
            }
        }
        if (h1 && h2 && (*h2 - *h1).dot(s - e) > 0.0 && distance(*h1, *h2) > 4.0 * kTol.distance &&
            distanceToInput(*h1, in) >= d * (1.0 + 1e-6) && distanceToInput(*h2, in) >= d * (1.0 + 1e-6)) {
            const std::vector<Contour> chains = offsetOpenContour(in, d, side, JoinType::Square, kTol);
            CAPTURE(h1->x, h1->y, h2->x, h2->y);
            CHECK(hasJunctionAt(chains, *h1, 1e-9 * std::max(1.0, d)));
            CHECK(hasJunctionAt(chains, *h2, 1e-9 * std::max(1.0, d)));
            CHECK_FALSE(hasArcAbout(chains, v, d));
            ++squareChecked;
        }
    }
    CHECK(sharpChecked > 800);
    CHECK(sharpBeyondHalfGap > 50);
    CHECK(squareChecked > 800);
}

TEST_CASE("T-0006 AC2: a tiny offset arc is not dragged round its circle to make a corner",
          "[geometry][offset][open][t0006]") {
    // Right of the clockwise second arc the offset radius is 2.549 - 2.5 = 0.049, about 2% of d.
    // Before the quarter-turn cap (ADR-0012 A1) Square extended that arc back by ~2 rad (sweep
    // 4.15 -> 6.19 rad) to reach the chamfer line; now Square falls back to Round.
    const Vec2 v{7.971660624, 2.014180349};
    const Vec2 c2{7.566507, 4.530770};
    Contour in;
    in.spans.emplace_back(LineSpan{{0.374398076, 3.071823909}, {7.910979919, 1.784433171}});
    in.spans.emplace_back(
        ArcSpan::fromCentre({7.931065, 1.902015}, {7.910979919, 1.784433171}, v, ArcDirection::CCW));
    in.spans.emplace_back(ArcSpan::fromCentre(c2, v, {9.482077052, 6.212425025}, ArcDirection::CW));
    const double inputSweep = std::fabs(std::get<ArcSpan>(in.spans[2]).sweepAngle());
    const double d = 2.5;
    for (const JoinType join : {JoinType::Sharp, JoinType::Square}) {
        CAPTURE(static_cast<int>(join));
        const std::vector<Contour> chains = offsetOpenContour(in, d, OffsetSide::Right, join, kTol);
        REQUIRE_FALSE(chains.empty());
        bool tinyArcSeen = false;
        for (const Contour& c : chains) {
            for (const Span& s : c.spans) {
                const auto* arc = std::get_if<ArcSpan>(&s);
                if (arc && distance(arc->centre, c2) < 1e-6 && arc->radius < 0.1) {
                    tinyArcSeen = true;
                    CHECK(std::fabs(arc->sweepAngle()) <= inputSweep + kPi / 2.0 + 1e-9);
                }
                for (int k = 0; k <= 20; ++k) {
                    CHECK(distanceToInput(pointAt(s, k / 20.0), in) >= d - 1e-6 * d);
                }
            }
            JunctionStats st;
            checkJunctions(chains, st);
            CHECK(st.gaps == 0);
            CHECK(st.backward == 0);
        }
        CHECK(tinyArcSeen);
    }
    // Square cannot reach its chamfer within the cap, so the corner is Round (whose join arc is
    // then pruned here: the result equals the Round offset). Sharp meets within a quarter turn.
    CHECK(offsetOpenContour(in, d, OffsetSide::Right, JoinType::Square, kTol) ==
          offsetOpenContour(in, d, OffsetSide::Right, JoinType::Round, kTol));
}

// --- AC3: snapped junctions ---------------------------------------------------------------------

TEST_CASE("T-0006 AC3: a sub-tolerance step sideways is closed without a gap or a backward move",
          "[geometry][offset][open][t0006]") {
    // A tiny near-reversal between two long legs: the Square cap of the tiny span is shorter than
    // the tolerance, so it is dropped, and its neighbours' carriers meet behind one of them.
    const Contour in = polyline({{-0.278685114, -0.558231668},
                                 {1.043333738, 4.001699722},
                                 {1.036752285, 3.981470170},
                                 {1.515944663, 7.329509130},
                                 {1.050708771, 4.684069708},
                                 {1.527340951, 9.063525877}});
    for (const JoinType join : kJoins) {
        for (const OffsetSide side : kSides) {
            CAPTURE(static_cast<int>(join), static_cast<int>(side));
            const std::vector<Contour> chains = offsetOpenContour(in, 1.0, side, join, kTol);
            REQUIRE_FALSE(chains.empty());
            JunctionStats st;
            checkJunctions(chains, st);
            CHECK(st.gaps == 0);
            CHECK(st.notIdentical == 0);
            CHECK(st.backward == 0);
            for (const Contour& c : chains) {
                CHECK(c.isContinuous(Tolerance{1e-12, 1e-9, 1e-12}));
                for (const Span& s : c.spans) {
                    CHECK_FALSE(isDegenerate(s, kTol));
                    for (int k = 0; k <= 20; ++k) {
                        CHECK(distanceToInput(pointAt(s, k / 20.0), in) >= 1.0 - 1e-6);
                    }
                }
            }
        }
    }
}

TEST_CASE("T-0006 AC3: junctions are shared exactly in typical toolpath shapes",
          "[geometry][offset][open][t0006]") {
    // A rounded rectangle as a closed contour, a slot narrower than 2d and a line/arc S-curve.
    Contour rounded;
    const double r = 2.0;
    rounded.closed = true;
    rounded.spans.emplace_back(LineSpan{{r, 0.0}, {20.0 - r, 0.0}});
    rounded.spans.emplace_back(
        ArcSpan::fromCentre({20.0 - r, r}, {20.0 - r, 0.0}, {20.0, r}, ArcDirection::CCW));
    rounded.spans.emplace_back(LineSpan{{20.0, r}, {20.0, 10.0 - r}});
    rounded.spans.emplace_back(
        ArcSpan::fromCentre({20.0 - r, 10.0 - r}, {20.0, 10.0 - r}, {20.0 - r, 10.0}, ArcDirection::CCW));
    rounded.spans.emplace_back(LineSpan{{20.0 - r, 10.0}, {r, 10.0}});
    rounded.spans.emplace_back(
        ArcSpan::fromCentre({r, 10.0 - r}, {r, 10.0}, {0.0, 10.0 - r}, ArcDirection::CCW));
    rounded.spans.emplace_back(LineSpan{{0.0, 10.0 - r}, {0.0, r}});
    rounded.spans.emplace_back(ArcSpan::fromCentre({r, r}, {0.0, r}, {r, 0.0}, ArcDirection::CCW));
    const Contour slot =
        polyline({{-10.0, 0.0}, {0.0, 0.0}, {0.0, 10.0}, {1.0, 10.0}, {1.0, 0.0}, {11.0, 0.0}});
    Contour sCurve;
    sCurve.spans.emplace_back(LineSpan{{-5.0, 0.0}, {0.0, 0.0}});
    sCurve.spans.emplace_back(ArcSpan::fromCentre({0.0, 3.0}, {0.0, 0.0}, {3.0, 3.0}, ArcDirection::CCW));
    sCurve.spans.emplace_back(ArcSpan::fromCentre({6.0, 3.0}, {3.0, 3.0}, {6.0, 6.0}, ArcDirection::CW));
    sCurve.spans.emplace_back(LineSpan{{6.0, 6.0}, {12.0, 6.0}});
    for (const Contour* in : std::vector<const Contour*>{&rounded, &slot, &sCurve}) {
        for (const double d : {0.5, 1.0, 1.7}) {
            for (const JoinType join : kJoins) {
                for (const OffsetSide side : kSides) {
                    CAPTURE(d, static_cast<int>(join), static_cast<int>(side));
                    JunctionStats st;
                    checkJunctions(offsetOpenContour(*in, d, side, join, kTol), st);
                    CHECK(st.gaps == 0);
                    CHECK(st.notIdentical == 0);
                    CHECK(st.backward == 0);
                }
            }
        }
    }
}

TEST_CASE("T-0006 AC3 generative: no gap above 1e-12 and no backward step over 20k offsets",
          "[geometry][offset][open][t0006][generative]") {
    JunctionStats st;
    std::size_t offsets = 0;
    std::size_t chains = 0;
    std::uint64_t seed = 600600u;
    for (const Generator gen : kGenerators) {
        SplitMix rng{seed++};
        for (int trial = 0; trial < 300; ++trial) {
            for (const double d : {0.3, 1.0, 2.5}) {
                const Contour in = gen(rng, d);
                for (const JoinType join : kJoins) {
                    for (const OffsetSide side : kSides) {
                        const std::vector<Contour> out = offsetOpenContour(in, d, side, join, kTol);
                        const std::size_t gapsBefore = st.gaps;
                        const std::size_t backBefore = st.backward;
                        const std::size_t identicalBefore = st.notIdentical;
                        checkJunctions(out, st);
                        if (st.gaps != gapsBefore || st.backward != backBefore ||
                            st.notIdentical != identicalBefore) {
                            CAPTURE(trial, d, static_cast<int>(join), static_cast<int>(side), st.maxGap);
                            FAIL_CHECK("junction defect");
                        }
                        ++offsets;
                        chains += out.size();
                    }
                }
            }
        }
    }
    CAPTURE(offsets, chains, st.junctions, st.maxGap);
    CHECK(offsets >= 20000);
    CHECK(st.junctions > 50000);
    CHECK(st.gaps == 0);
    CHECK(st.backward == 0);
    CHECK(st.notIdentical == 0);
}

// --- AC3: steep zig-zags (overlapping collinear chamfers) ----------------------------------------

namespace {

// Zig-zag through (origin + (i * pitch, (i % 2) * amplitude)), i = 0..lines; closed contours
// return straight to the first vertex.
Contour zigZag(int lines, double pitch, double amplitude, bool closed, Vec2 origin = {}) {
    std::vector<Vec2> pts;
    for (int i = 0; i <= lines; ++i) {
        pts.push_back(origin + Vec2{i * pitch, (i % 2) * amplitude});
    }
    Contour c = polyline(pts);
    if (closed) {
        c.spans.emplace_back(LineSpan{pts.back(), pts.front()});
        c.closed = true;
    }
    return c;
}

// Junction defects plus the distance criterion (every point at least d from the input).
void checkSteepZigZag(const Contour& in, double d, OffsetSide side, JoinType join, double mitre,
                      JunctionStats& st, std::size_t& closeViolations) {
    const std::vector<Contour> chains = offsetOpenContour(in, d, side, join, kTol, mitre);
    checkJunctions(chains, st);
    for (const Contour& c : chains) {
        for (const Span& s : c.spans) {
            for (int k = 0; k <= 8; ++k) {
                if (distanceToInput(pointAt(s, k / 8.0), in) < d - 1e-6 * std::max(1.0, d)) {
                    ++closeViolations;
                }
            }
        }
    }
}

}  // namespace

TEST_CASE("T-0006 AC3: steep zig-zag, Square: neighbouring chamfers on one line do not step back",
          "[geometry][offset][open][t0006]") {
    // Verifier repro (round 2): the chamfers of neighbouring peaks lie on one line and overlap by
    // 6.12e-4 mm (less than the tolerance). Both copies of the overlap survived and the chain ran
    // forward, stepped back over the overlap and ran forward again.
    const Contour in = zigZag(10, 0.32926969138355855, 0.29643099557445474, false);
    for (const JoinType join : kJoins) {
        for (const OffsetSide side : kSides) {
            CAPTURE(static_cast<int>(join), static_cast<int>(side));
            const std::vector<Contour> chains =
                offsetOpenContour(in, 2.574417216178353, side, join, kTol, 1.7950104685437678);
            REQUIRE(chains.size() == 1);
            JunctionStats st;
            checkJunctions(chains, st);
            CHECK(st.gaps == 0);
            CHECK(st.notIdentical == 0);
            CHECK(st.backward == 0);
            // With Square the peaks' chamfers merge into one straight run: every span is longer
            // than the tolerance and each consecutive pair turns, never continues collinearly.
            for (const Span& s : chains.front().spans) {
                CHECK_FALSE(isDegenerate(s, kTol));
            }
        }
    }
}

TEST_CASE("T-0006 AC3: closed steep zig-zag, Sharp falling back to Square, has no reversals",
          "[geometry][offset][open][t0006]") {
    // Verifier fuzz seed 0x60360355: 18 zig-zag lines closed by a line back to the start,
    // d = 0.0768948, Sharp with mitre limit 2, Left. The peaks exceed the mitre limit, square
    // off, and their chamfers overlapped by 8.6e-4 mm: 16 reversals.
    const Contour in = zigZag(18, 0.064345757583246344, 0.37327572041736617, true);
    for (const JoinType join : kJoins) {
        for (const OffsetSide side : kSides) {
            CAPTURE(static_cast<int>(join), static_cast<int>(side));
            const std::vector<Contour> chains = offsetOpenContour(in, 0.07689480169075523, side, join, kTol, 2.0);
            // The contour runs clockwise: Left is outside. Right is inside the teeth, which are too
            // thin for d, so only Sharp/Square corner regions at the valleys may leave anything.
            if (side == OffsetSide::Left) {
                REQUIRE_FALSE(chains.empty());
            }
            JunctionStats st;
            checkJunctions(chains, st);
            CHECK(st.gaps == 0);
            CHECK(st.notIdentical == 0);
            CHECK(st.backward == 0);
        }
    }
}

TEST_CASE("T-0006 AC3 generative: steep zig-zags at all joins, both sides, open and closed",
          "[geometry][offset][open][t0006][generative]") {
    // Pitch, amplitude and distance chosen so that the chamfers or mitres of neighbouring peaks
    // land on one line with overlaps from far above to well below the tolerance; irrational
    // factors keep the overlaps off round numbers. Some inputs sit far from the origin.
    JunctionStats st;
    std::size_t closeViolations = 0;
    std::size_t offsets = 0;
    for (const double pitch : {0.0643457575832463, 0.1371, 0.32926969138355855}) {
        for (const double steep : {0.9013, 2.2361, 5.8011}) {
            for (const double dOverPitch : {0.6180, 1.1952, 3.1416, 7.8174}) {
                for (const int lines : {7, 12}) {
                    for (const bool closed : {false, true}) {
                        for (const Vec2 origin : {Vec2{}, Vec2{37.31, -112.97}}) {
                            const Contour in = zigZag(lines, pitch, pitch * steep, closed, origin);
                            const double d = pitch * dOverPitch;
                            for (const JoinType join : kJoins) {
                                for (const double mitre : {1.0, 1.4142, 2.0, 4.1}) {
                                    if (join != JoinType::Sharp && mitre != 2.0) {
                                        continue;  // the limit only matters for Sharp (and arcs)
                                    }
                                    for (const OffsetSide side : kSides) {
                                        const std::size_t before = st.backward + st.gaps + st.notIdentical;
                                        const std::size_t closeBefore = closeViolations;
                                        checkSteepZigZag(in, d, side, join, mitre, st, closeViolations);
                                        if (st.backward + st.gaps + st.notIdentical != before ||
                                            closeViolations != closeBefore) {
                                            CAPTURE(pitch, steep, d, lines, closed, origin.x,
                                                    static_cast<int>(join), mitre, static_cast<int>(side));
                                            FAIL_CHECK("steep zig-zag defect");
                                        }
                                        ++offsets;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    CAPTURE(offsets, st.junctions);
    CHECK(offsets == 3 * 3 * 4 * 2 * 2 * 2 * 6 * 2);
    CHECK(st.backward == 0);
    CHECK(st.gaps == 0);
    CHECK(st.notIdentical == 0);
    CHECK(closeViolations == 0);
}

// --- AC3: nearly collinear carriers (verification round 3) ---------------------------------------

namespace {

// Distance the chain may come closer to the input than d: the bridge sagitta and the snapping
// margin documented in offset.hpp, plus rounding of coordinates far from the origin.
double distanceAllowance(double d, Vec2 at) {
    const double t = kTol.distance;
    return std::max(t * t / (8.0 * d), std::min(5e-7 * std::max(1.0, d), 0.1 * t * t / d)) +
           1e-12 * std::max({1.0, d, std::fabs(at.x), std::fabs(at.y)});
}

struct NearCollinearStats {
    std::size_t offsets{0};
    std::size_t backward{0};
    std::size_t overlaps{0};
    std::size_t gaps{0};
    std::size_t tooClose{0};
};

// Every defect AC3 rules out, plus the distance criterion, for one offset.
void checkNearCollinear(const Contour& in, double d, OffsetSide side, JoinType join, double mitre,
                        NearCollinearStats& st, std::string* why = nullptr) {
    const std::vector<Contour> chains = offsetOpenContour(in, d, side, join, kTol, mitre);
    ++st.offsets;
    JunctionStats js;
    checkJunctions(chains, js);  // counts backward steps with countBackwardSteps
    st.backward += js.backward;
    st.gaps += js.gaps + js.notIdentical;
    st.overlaps += carvetto::geometry::testing::countOverlaps(chains, 1e-6, why);
    for (const Contour& c : chains) {
        for (const Span& s : c.spans) {
            for (int k = 0; k <= 16; ++k) {
                const Vec2 p = pointAt(s, k / 16.0);
                if (distanceToInput(p, in) < d - distanceAllowance(d, p)) {
                    ++st.tooClose;
                }
            }
        }
    }
}

const std::vector<Vec2> kR1{{0, 0},
                            {0.34314657975231688, 0.42689234201510018},
                            {0.68629315950463377, 4.1466788538240213e-09},
                            {1.0294397392569508, 0.4268923496590592},
                            {1.3725863190092675, -5.1970845120572706e-09},
                            {1.7157328987615843, 0.42689234496395445},
                            {2.0588794785139015, -3.0832318809454741e-09},
                            {2.4020260582662183, 0.42689234858743808},
                            {2.7451726380185351, 1.3336163515167116e-09},
                            {3.0883192177708518, 0.42689235209290904},
                            {3.4314657975231686, 1.9554402014056211e-09},
                            {3.7746123772754858, 0.42689233652465358},
                            {4.117758957027803, 7.5870395793344307e-09},
                            {4.4609055367801194, 0.4268923516090688},
                            {4.8040521165324366, -7.8081612624261673e-09},
                            {5.1471986962847529, 0.42689234700998913},
                            {5.4903452760370701, 7.6129937363389407e-09},
                            {5.8334918557893873, 0.42689234436136914}};

const std::vector<Vec2> kR2{{0, 0},
                            {0.13501705870620517, 0.27352602793522046},
                            {0.27003411741241035, 1.6459969677485449e-09},
                            {0.40505117611861552, 0.27352603356583249},
                            {0.54006823482482069, 6.0794457483971358e-09},
                            {0.67508529353102587, 0.27352603017211757},
                            {0.81010235223723104, -8.2462010234247105e-09},
                            {0.94511941094343621, 0.27352603547300997},
                            {1.0801364696496414, 1.9453315567158852e-09},
                            {1.2151535283558466, 0.27352601631583845},
                            {1.3501705870620517, 7.4452776617249392e-09},
                            {1.4851876457682569, 0.2735260193650989},
                            {1.6202047044744621, 5.0472290070040781e-09},
                            {1.7552217631806672, 0.2735260169191826},
                            {1.8902388218868724, -1.3246464711254609e-09},
                            {2.0252558805930776, 0.27352602908387069},
                            {2.1602729392992828, 8.9026067543794283e-09},
                            {2.2952899980054879, 0.27352603460625025}};

// Exact input far from the origin: runs of collinear lines at two heights, x ~ -800.
const std::vector<Vec2> kR3{
    {-799.04665111856445, -358.12890674720455}, {-804.49964414289707, -358.12890674720455},
    {-803.10454902039055, -386.09106214164575}, {-799.74334805970784, -385.3456905353446},
    {-800.77279240785708, -358.24083861981262}, {-795.26389224877698, -358.24083861981262},
    {-796.11944403511757, -387.72102577857282}, {-805.68565535706261, -387.33030905810119},
    {-804.83924329636636, -358.12890674720455}, {-804.88596323010142, -358.12890674720455},
    {-805.25050996888797, -388.98479156842029}, {-799.68453512309236, -389.85776028071899},
    {-798.68133173415049, -358.24083861981262}, {-802.75694351264576, -358.24083861981262},
    {-804.56049472068457, -386.85222138708349}, {-803.73023188598006, -386.03983269228053},
    {-802.75694351264576, -358.24083861981262}, {-798.68133173415049, -358.24083861981262},
    {-800.47855209748525, -391.04362303030348}, {-805.27018421811158, -391.63422975439784},
    {-803.80217372728339, -358.12890674720455}, {-803.94846823604598, -358.12890674720455},
    {-805.22674363390411, -387.19918857067739}, {-803.10173476342152, -386.41037224351277},
    {-802.8225499081251, -358.24083861981262},  {-801.6963092051468, -358.24083861981262}};

}  // namespace

TEST_CASE("T-0006 AC3: steep zig-zags with ~1e-8 mm vertex noise neither step back nor overlap (R1, R2)",
          "[geometry][offset][open][t0006]") {
    // Verifier round 3. The chamfers (R1, Square) and mitre fallbacks (R2, Sharp) of the peaks lie
    // on carriers ~1e-8 apart: too far for an exact same-carrier test, too close for their
    // crossings to cut the copies consistently. Before: 5 and 3 chains, retraces and short steps
    // back, the later chains repeating stretches of the first. Now one chain, as without noise.
    struct Repro {
        const std::vector<Vec2>* pts;
        double d;
        JoinType join;
        double mitre;
    };
    for (const Repro& r : {Repro{&kR1, 2.9084271393964491, JoinType::Square, 1.8241758062497766},
                           Repro{&kR2, 1.2018887962144966, JoinType::Sharp, 1.3463749409772734}}) {
        const Contour in = polyline(*r.pts);
        CAPTURE(r.d);
        CHECK(offsetOpenContour(in, r.d, OffsetSide::Left, r.join, kTol, r.mitre).size() == 1);
        for (const JoinType join : kJoins) {
            for (const OffsetSide side : kSides) {
                CAPTURE(static_cast<int>(join), static_cast<int>(side));
                NearCollinearStats st;
                std::string why;
                checkNearCollinear(in, r.d, side, join, r.mitre, st, &why);
                CAPTURE(why);
                CHECK(st.backward == 0);
                CHECK(st.overlaps == 0);
                CHECK(st.gaps == 0);
                CHECK(st.tooClose == 0);
            }
        }
    }
}

TEST_CASE("T-0006 AC3: exact collinear runs far from the origin do not step back (R3)",
          "[geometry][offset][open][t0006]") {
    // Verifier round 3: d = 0.0559659, Sharp, Right, mitre 2 gave a 1.2e-5 mm step back at span 4
    // of a closed chain.
    const Contour in = polyline(kR3);
    for (const JoinType join : kJoins) {
        for (const OffsetSide side : kSides) {
            CAPTURE(static_cast<int>(join), static_cast<int>(side));
            NearCollinearStats st;
            std::string why;
            checkNearCollinear(in, 0.055965936304044207, side, join, 2.0, st, &why);
            CAPTURE(why);
            CHECK(st.backward == 0);
            CHECK(st.overlaps == 0);
            CHECK(st.gaps == 0);
            CHECK(st.tooClose == 0);
        }
    }
}

TEST_CASE("T-0006 AC3 generative: steep zig-zags with vertex noise 1e-12..1e-4 mm",
          "[geometry][offset][open][t0006][generative]") {
    // Each level: random pitch, amplitude (0.5-25 x pitch), line count and distance (0.5-12 x
    // amplitude), y noise of that size on every vertex, all joins, both sides, open and closed,
    // half of them ~640 mm from the origin.
    NearCollinearStats total;
    std::uint64_t seed = 600603u;
    for (const double noise : {1e-12, 1e-11, 1e-10, 1e-9, 1e-8, 1e-7, 1e-6, 1e-5, 1e-4}) {
        SplitMix rng{seed++};
        NearCollinearStats st;
        for (int trial = 0; trial < 12; ++trial) {
            const double pitch = rng.uniform(0.02, 0.5);
            const double amplitude = pitch * rng.uniform(0.5, 25.0);
            const int lines = 5 + static_cast<int>(rng.uniform(0.0, 20.0));
            const double d = amplitude * rng.uniform(0.5, 12.0);
            const double mitre = rng.uniform(1.0, 4.0);
            const Vec2 origin = trial % 2 == 0 ? Vec2{} : Vec2{600.0, -222.0};
            std::vector<Vec2> pts;
            for (int i = 0; i <= lines; ++i) {
                pts.push_back(origin + Vec2{i * pitch, (i % 2) * amplitude + rng.uniform(-noise, noise)});
            }
            for (const bool closed : {false, true}) {
                Contour in = polyline(pts);
                if (closed) {
                    in.spans.emplace_back(LineSpan{pts.back(), pts.front()});
                    in.closed = true;
                }
                for (const JoinType join : kJoins) {
                    for (const OffsetSide side : kSides) {
                        const NearCollinearStats before = st;
                        std::string why;
                        checkNearCollinear(in, d, side, join, mitre, st, &why);
                        if (st.backward != before.backward || st.overlaps != before.overlaps ||
                            st.gaps != before.gaps || st.tooClose != before.tooClose) {
                            CAPTURE(noise, trial, closed, d, pitch, amplitude, lines, mitre,
                                    static_cast<int>(join), static_cast<int>(side), why);
                            FAIL_CHECK("near-collinear defect");
                        }
                    }
                }
            }
        }
        total.offsets += st.offsets;
        total.backward += st.backward;
        total.overlaps += st.overlaps;
        total.gaps += st.gaps;
        total.tooClose += st.tooClose;
    }
    CAPTURE(total.offsets);
    CHECK(total.offsets == 9 * 12 * 2 * 3 * 2);
    CHECK(total.backward == 0);
    CHECK(total.overlaps == 0);
    CHECK(total.gaps == 0);
    CHECK(total.tooClose == 0);
}

// ---------------------------------------------------------------------------------------------
// T-0017: residuals of T-0006 round 3 (verifier repros, inputs inlined).
// ---------------------------------------------------------------------------------------------

namespace {

// Worst shortfall below d and (for Round) excess beyond d over every chain, sampled finely.
struct DistanceExtremes {
    double under{0.0};
    double over{0.0};
};

DistanceExtremes distanceExtremes(const std::vector<Contour>& chains, const Contour& in, double d) {
    DistanceExtremes ex;
    for (const Contour& c : chains) {
        for (const Span& s : c.spans) {
            for (int k = 0; k <= 2000; ++k) {
                const double m = distanceToInput(pointAt(s, k / 2000.0), in);
                ex.under = std::max(ex.under, d - m);
                ex.over = std::max(ex.over, m - d);
            }
        }
    }
    return ex;
}

// Every AC3 defect over all chains: backward steps (shared checker), gaps, overlaps.
// `c` moved by `shift`.
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

void requireCleanChains(const std::vector<Contour>& chains, double overlapLateral) {
    JunctionStats js;
    checkJunctions(chains, js);
    CHECK(js.backward == 0);
    CHECK(js.gaps == 0);
    CHECK(js.notIdentical == 0);
    std::string why;
    const std::size_t overlaps = carvetto::geometry::testing::countOverlaps(chains, overlapLateral, &why);
    CAPTURE(why);
    CHECK(overlaps == 0);
}

}  // namespace

TEST_CASE("T-0017 AC1: a short piece at 11-20 degrees to a carrier is not merged into it (R4)",
          "[geometry][offset][open][t0017]") {
    // Six exact lines turning gently (11-20 degrees apart, 1.5-2.5e-3 mm long). The round-3
    // "nearly the same carrier" test (|u x v| x length <= tol/2) chained all their offsets into
    // one group and projected them onto the first one's axis: a second chain, a 1.57e-3 mm span
    // 1.86e-4 mm inside d (Square, Right).
    const Contour in = polyline({{0.0036197263399135272, -0.29193303409146748},
                                 {0.0049012208081527647, -0.2944254877427343},
                                 {0.006583562492423152, -0.29666697825870963},
                                 {0.0086117770255438111, -0.29860111865782218},
                                 {0.010933978192457157, -0.30017017411134422},
                                 {0.013068744511359252, -0.30198602819125325},
                                 {0.015475029509577408, -0.30342281132840071}});
    const double d = 0.03169564160714932;
    const double mitre = 2.1158214539163573;
    for (const JoinType join : kJoins) {
        for (const OffsetSide side : kSides) {
            CAPTURE(static_cast<int>(join), static_cast<int>(side));
            const std::vector<Contour> chains = offsetOpenContour(in, d, side, join, kTol, mitre);
            CHECK(chains.size() == 1);
            const DistanceExtremes ex = distanceExtremes(chains, in, d);
            CHECK(ex.under <= distanceAllowance(d, Vec2{}));
            if (join == JoinType::Round) {
                CHECK(ex.over <= distanceAllowance(d, Vec2{}));
            }
            requireCleanChains(chains, 1e-6);
        }
    }
}

TEST_CASE("T-0017 AC1: nearly co-circular arcs keep the offset at least d away (R5)",
          "[geometry][offset][open][t0017]") {
    // A line and two arcs whose circles differ by ~1e-5 mm: the outer copy, chosen at the middle
    // of a merged stretch, was the inner one at its ends (2.4e-6 mm inside d, Sharp, Right).
    Contour in;
    in.spans.emplace_back(
        LineSpan{{-0.54490984941556175, 0.47713618213141934}, {-0.65087324196032126, 0.74692846369896915}});
    in.spans.emplace_back(ArcSpan{{-0.66128590342338933, 0.74283879542663755},
                                  0.011186997153936269,
                                  0.37424873933798614,
                                  3.7359281215955029,
                                  ArcDirection::CCW});
    in.spans.emplace_back(ArcSpan{{-0.66130088294241851, 0.74282867147309783},
                                  0.011168917319157794,
                                  -2.5472571855840811,
                                  -1.6946626886190583,
                                  ArcDirection::CCW});
    const double d = 0.1712170592382718;
    const double mitre = 2.3199483387667215;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Right, JoinType::Sharp, kTol, mitre);
    REQUIRE(chains.size() == 1);
    CHECK(distanceExtremes(chains, in, d).under <= 5e-7);
    for (const JoinType join : kJoins) {
        for (const OffsetSide side : kSides) {
            CAPTURE(static_cast<int>(join), static_cast<int>(side));
            const std::vector<Contour> all = offsetOpenContour(in, d, side, join, kTol, mitre);
            CHECK(distanceExtremes(all, in, d).under <= distanceAllowance(d, Vec2{}));
            requireCleanChains(all, 1e-6);
        }
    }
}

TEST_CASE("T-0017 AC1: opposite strands ~1e-4 mm apart keep their needle tip, Round stays at d (R6)",
          "[geometry][offset][open][t0017]") {
    // Eight exact lines near (-5280, -5270) with two strands a little more than 2d apart running
    // opposite ways. Their offsets, 0.8-1.9e-4 mm apart, form a genuine thin needle. Round 3
    // merged them as copies and dropped both, leaving a sliver loop and a sideways bridge 9.5e-5 mm
    // beyond d, and 4 chains where main gives 3. Opposite copies are now only merged on exactly
    // one carrier (ADR-0012 section 5).
    const Contour in = polyline({{-5281.0715789769911, -5268.2784733563021},
                                 {-5284.9414577865855, -5267.8446579537076},
                                 {-5288.7708788228092, -5290.2632742408732},
                                 {-5286.9348942352071, -5291.4753527022858},
                                 {-5284.2200656054292, -5269.286576947201},
                                 {-5280.3502977692388, -5269.7213822953418},
                                 {-5276.2576290492298, -5245.9702232636519},
                                 {-5281.4940880823642, -5268.2313251412743},
                                 {-5285.3638950837394, -5267.7968691687456}});
    const double d = 0.67641900916302644;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Left, JoinType::Round, kTol, 2.0);
    CHECK(chains.size() == 3);
    const Vec2 farAway{-5281.0, -5268.0};
    const DistanceExtremes ex = distanceExtremes(chains, in, d);
    CHECK(ex.under <= distanceAllowance(d, farAway));
    CHECK(ex.over <= distanceAllowance(d, farAway));
    requireCleanChains(chains, 1e-6);
}

TEST_CASE("T-0017 AC2: three-span chains get the back-step safety net; co-circular arcs merge (0x7300029)",
          "[geometry][offset][open][t0017]") {
    // Verifier generator gen3, seed 0x7300029 (co-circular noise): input arcs split in two with
    // ~2e-8 mm of noise, so the offsets' circles are nested and never meet. One chain was
    // [arc, 6.5e-8 mm line running backwards, arc]; the safety net skipped chains of 3 spans. Now
    // the two arcs become one.
    Contour in;
    in.spans.emplace_back(ArcSpan{{4.0771360287070938e-08, -2.6491191809783244e-09},
                                  0.97686775285815508,
                                  -0.084521906830785198,
                                  0.51371505051026611,
                                  ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{0.85077847712394983, 0.4800485978191919}, {22.823916188798368, 12.878310641376288}});
    in.spans.emplace_back(
        LineSpan{{22.823916188798368, 12.878310641376288}, {25.536721000586631, -5.588673086139968}});
    in.spans.emplace_back(
        LineSpan{{25.536721000586631, -5.588673086139968}, {2.0395558685517137, -0.037802706440730069}});
    in.spans.emplace_back(ArcSpan{{-3.5099739204654406e-08, -8.1836547595284496e-09},
                                  2.0399062057151083,
                                  -0.018532647326693574,
                                  -0.39553391165788609,
                                  ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{1.8824070100285553, -0.78597777158817572}, {21.85893111380544, -9.1269493908143104}});
    in.spans.emplace_back(
        LineSpan{{21.85893111380544, -9.1269493908143104}, {21.388027898563045, 15.558140734969852}});
    in.spans.emplace_back(
        LineSpan{{21.388027898563045, 15.558140734969852}, {1.5724223496101351, 1.2995019723108592}});
    in.spans.emplace_back(ArcSpan{{-1.8222062058436283e-08, -1.2713031498273938e-08},
                                  2.0399062507723351,
                                  0.69065219871825922,
                                  -0.91143433170557686,
                                  ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{1.2496724747413674, -1.6123077290843861}, {14.821831581440948, -19.122893040790881}});
    in.spans.emplace_back(
        LineSpan{{14.821831581440948, -19.122893040790881}, {-12.154150677486221, -21.797895592825167}});
    in.spans.emplace_back(
        LineSpan{{-12.154150677486221, -21.797895592825167}, {-0.58415529700295565, -0.78296438693836323}});
    in.spans.emplace_back(ArcSpan{{3.181346313573212e-08, 8.2326847196507777e-09},
                                  0.97686779673105373,
                                  -2.2117849393835041,
                                  -0.92627887375258466,
                                  ArcDirection::CCW});
    const double d = 0.531519219172784;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Right, JoinType::Sharp, kTol, 2.0);
    requireCleanChains(chains, 1e-6);
    // The chain round the split arcs is one arc of radius r + d ~ 1.508 from angle -0.0847 on.
    bool merged = false;
    for (const Contour& c : chains) {
        if (c.spans.size() == 1 && std::holds_alternative<ArcSpan>(c.spans[0])) {
            const auto& arc = std::get<ArcSpan>(c.spans[0]);
            merged = merged || (std::fabs(arc.radius - 1.50838699) < 1e-6 &&
                                std::fabs(arc.startAngle + 0.0847310235) < 1e-6);
        }
    }
    CHECK(merged);
    CHECK(distanceExtremes(chains, in, d).under <= distanceAllowance(d, Vec2{}));
    for (const JoinType join : kJoins) {
        for (const OffsetSide side : kSides) {
            CAPTURE(static_cast<int>(join), static_cast<int>(side));
            requireCleanChains(offsetOpenContour(in, d, side, join, kTol, 2.0), 1e-6);
        }
    }
}

TEST_CASE("T-0017 AC3: a closed lens of two arcs narrower than the tolerance is dropped (0x82004c8)",
          "[geometry][offset][open][t0017]") {
    // Verifier legacy generator, seed 0x82004c8 (closed input, Sharp, Right, d = 3.33): a closed
    // chain of two arcs on nearly one circle running opposite ways, 0.018 mm long and far
    // narrower than the tolerance (main identical). Such a loop encloses no width a tool could
    // follow and only retraces itself; it is dropped like a two-line sliver.
    Contour in;
    in.closed = true;
    in.spans.emplace_back(ArcSpan{{-0.013668801848060654, -0.025068497936788177},
                                  0.02855285857437569,
                                  1.0716010092392056,
                                  0.69025107450886569,
                                  ArcDirection::CW});
    in.spans.emplace_back(ArcSpan{{1.3337557731103167, -0.39067286236501525},
                                  1.3798539078669199,
                                  2.8597413207868971,
                                  7.297019467738151,
                                  ArcDirection::CCW});
    in.spans.emplace_back(ArcSpan{{2.2444598694046363, 0.69309892306564291},
                                  0.20132660162268834,
                                  2.6917715449140163,
                                  1.9525503827318134,
                                  ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{2.1694558799877948, 0.87993254448453795}, {2.7032272642781559, 1.0942140156367008}});
    in.spans.emplace_back(ArcSpan{{3.9202096316477171, -1.9372677050881584},
                                  3.2666385636579651,
                                  1.9525503827318125,
                                  1.2156945406390398,
                                  ArcDirection::CW});
    in.spans.emplace_back(ArcSpan{{3.4717581102005326, -3.1466156930745752},
                                  4.5564568931660734,
                                  1.2156945406390398,
                                  -2.5059257006901596,
                                  ArcDirection::CW});
    in.spans.emplace_back(ArcSpan{{-0.10453024482922757, -7.2890377279896335},
                                  1.4400149696835061,
                                  1.6334635449873582,
                                  2.6474758745571094,
                                  ArcDirection::CCW});
    in.spans.emplace_back(ArcSpan{{-1.9545746400937465, -6.2924413300687396},
                                  0.66138186214016503,
                                  -0.49411677903268325,
                                  -4.9767435460372091,
                                  ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{-1.7817646139728338, -5.654034984204225}, {-2.0552874154611018, -7.962389519448883}});
    in.spans.emplace_back(
        LineSpan{{-2.0552874154611018, -7.962389519448883}, {-0.6044455076602695, -7.3141432031992597}});
    in.spans.emplace_back(ArcSpan{{-0.57224982707146621, -7.2552405283190611},
                                  0.067127393488936982,
                                  4.2121666982044026,
                                  8.2466903499160757,
                                  ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{-0.59793896590920292, -7.1932231508963254}, {-1.7347234759768071e-18, 0}});
    const double d = 3.332298622967294;
    for (const JoinType join : kJoins) {
        for (const OffsetSide side : kSides) {
            CAPTURE(static_cast<int>(join), static_cast<int>(side));
            const std::vector<Contour> chains = offsetOpenContour(in, d, side, join, kTol, 2.0);
            for (const Contour& c : chains) {
                // No closed two-span chain that never opens wider than half the tolerance.
                if (c.closed && c.spans.size() == 2) {
                    double width = 0.0;
                    for (int k = 1; k < 64; ++k) {
                        width = std::max(width, distanceToSpan(pointAt(c.spans[0], k / 64.0), c.spans[1]));
                    }
                    CHECK(width > 0.5 * kTol.distance);
                }
            }
            requireCleanChains(chains, 1e-6);
        }
    }
}

TEST_CASE("T-0017 AC3: a thin needle tip is the same wherever it lies (0x840117a)",
          "[geometry][offset][open][t0017]") {
    // Verifier generator gen3 (crossShort), seed 0x840117a, shrunk from 895 to 116 lines: a long
    // line near (-1e4, 1e4) and, a little less than 2d above it, a return path of short noisy
    // pieces whose offsets dip to within 3.6e-6 mm of the long line's. A closed chain ends in a
    // 2.9e-3 mm needle 3.6e-6 mm wide. Round 1 removed it as "the same stretch traversed twice"
    // by a rounding unit scaled by the coordinates (1e-5 mm at 1e4), so the result depended on
    // where the input lay; the needle is real geometry (far above rounding) and is now kept at
    // any translation (fix round 2, director decision).
    const Contour in = polyline({{-10000, 10000},
                                 {-9995.7168746242442, 10000},
                                 {-9994.7168746242442, 10015.279770683441},
                                 {-9995.2168746242442, 10000.139779498651},
                                 {-9995.2981677593943, 10000.140055151576},
                                 {-9995.3033866750848, 10000.140818557737},
                                 {-9995.3065293031359, 10000.140605677798},
                                 {-9995.3138569388157, 10000.141345094353},
                                 {-9995.3204197424457, 10000.140915982538},
                                 {-9995.3290104336847, 10000.140076305861},
                                 {-9995.3324839386314, 10000.140618611138},
                                 {-9995.3364355558788, 10000.140428648398},
                                 {-9995.3432400118527, 10000.141070722075},
                                 {-9995.3502347612248, 10000.140038497131},
                                 {-9995.3591880187214, 10000.14051190371},
                                 {-9995.3629114170672, 10000.140698680592},
                                 {-9995.3681815764157, 10000.140227745711},
                                 {-9995.3728548542967, 10000.139531212368},
                                 {-9995.3808989788959, 10000.138512024907},
                                 {-9995.3857931012735, 10000.13913799765},
                                 {-9995.39244105957, 10000.138583634018},
                                 {-9995.3988341352142, 10000.139523440894},
                                 {-9995.4063805123951, 10000.138876393892},
                                 {-9995.4138096113657, 10000.139468209305},
                                 {-9995.4210359455519, 10000.138627521268},
                                 {-9995.4257283495481, 10000.139318104029},
                                 {-9995.431594440508, 10000.140230579227},
                                 {-9995.4353500922589, 10000.140699308877},
                                 {-9995.4433813558571, 10000.140115552791},
                                 {-9995.4496175926961, 10000.139492413729},
                                 {-9995.4570188905382, 10000.138551317825},
                                 {-9995.4638726540707, 10000.139097073261},
                                 {-9995.4702445493767, 10000.138133131868},
                                 {-9995.4783734130106, 10000.137093926442},
                                 {-9995.48361007946, 10000.137887346234},
                                 {-9995.4889086990024, 10000.138572006696},
                                 {-9995.4937318824232, 10000.138117447772},
                                 {-9995.5027136169265, 10000.138809662918},
                                 {-9995.5077346639773, 10000.139423542762},
                                 {-9995.5142366976288, 10000.140226317302},
                                 {-9995.5174757201585, 10000.14069966693},
                                 {-9995.5249045895707, 10000.141737749611},
                                 {-9995.5317302297772, 10000.141219326222},
                                 {-9995.5358839069304, 10000.141766893339},
                                 {-9995.5404907869452, 10000.141255534299},
                                 {-9995.5435995842472, 10000.141003260414},
                                 {-9995.5477242207508, 10000.141579978888},
                                 {-9995.5536859653039, 10000.142120283617},
                                 {-9995.5603612425293, 10000.141229886278},
                                 {-9995.5652485020819, 10000.141828094162},
                                 {-9995.5690770148976, 10000.141490213771},
                                 {-9995.5775880271467, 10000.142157112772},
                                 {-9995.5825744441081, 10000.142630238615},
                                 {-9995.5873327073641, 10000.143298758117},
                                 {-9995.5957271629632, 10000.143986454332},
                                 {-9995.5991818559087, 10000.14358752479},
                                 {-9995.6066903900228, 10000.144000876249},
                                 {-9995.613634899848, 10000.144459559708},
                                 {-9995.6166159220611, 10000.144668021057},
                                 {-9995.6206660250755, 10000.144379003241},
                                 {-9995.6237382876698, 10000.144095717791},
                                 {-9995.626882817638, 10000.143685251016},
                                 {-9995.6331255306868, 10000.144680658987},
                                 {-9995.6410527720491, 10000.143664866115},
                                 {-9995.6453567934659, 10000.143034154728},
                                 {-9995.6521382392566, 10000.143384563951},
                                 {-9995.6587776014294, 10000.142676444852},
                                 {-9995.666476798353, 10000.143643476649},
                                 {-9995.6752991481371, 10000.144794095533},
                                 {-9995.6809810802006, 10000.144473746082},
                                 {-9995.6899110531103, 10000.145246085711},
                                 {-9995.6933610515371, 10000.144730018743},
                                 {-9995.69654044854, 10000.144377161982},
                                 {-9995.7030225580602, 10000.143587707258},
                                 {-9995.7081349436776, 10000.142784248501},
                                 {-9995.7156786100932, 10000.141748818685},
                                 {-9995.7242945927628, 10000.142682740307},
                                 {-9995.7329291902915, 10000.141977869151},
                                 {-9995.7400621551969, 10000.14094771636},
                                 {-9995.7474511313776, 10000.140360400714},
                                 {-9995.7543905710081, 10000.14091011066},
                                 {-9995.762948894655, 10000.14146730362},
                                 {-9995.766998501118, 10000.14201875491},
                                 {-9995.7743934145492, 10000.141280419841},
                                 {-9995.7819888367376, 10000.140149951205},
                                 {-9995.7869246409191, 10000.140758397434},
                                 {-9995.7948544071169, 10000.141911595309},
                                 {-9995.7998034732718, 10000.141157979995},
                                 {-9995.8047537934053, 10000.141580536485},
                                 {-9995.8094848126202, 10000.141321558138},
                                 {-9995.8143735008707, 10000.14173606282},
                                 {-9995.817994959345, 10000.141511803717},
                                 {-9995.8238444081562, 10000.141865403422},
                                 {-9995.8286050719926, 10000.142517910799},
                                 {-9995.8350583230967, 10000.141772432647},
                                 {-9995.8424548865678, 10000.141256769159},
                                 {-9995.8480084828443, 10000.14054232015},
                                 {-9995.8526503849062, 10000.141182536609},
                                 {-9995.8553386775475, 10000.140967320323},
                                 {-9995.8588949572768, 10000.141220234766},
                                 {-9995.8668395263521, 10000.141689862379},
                                 {-9995.8751515628228, 10000.140622130597},
                                 {-9995.8840750154395, 10000.140057286602},
                                 {-9995.8929478212322, 10000.139226335401},
                                 {-9995.8969309920158, 10000.139625162377},
                                 {-9995.9034305011864, 10000.139082916538},
                                 {-9995.9080170708985, 10000.139763307603},
                                 {-9995.9122555813501, 10000.140037563342},
                                 {-9995.9196404305585, 10000.139139461013},
                                 {-9995.9251641331139, 10000.138686239987},
                                 {-9995.9320666865988, 10000.139476737018},
                                 {-9995.9389693534722, 10000.139044773467},
                                 {-9995.9457655180613, 10000.139709648292},
                                 {-9995.9500540826775, 10000.140301547968},
                                 {-9995.9558170007531, 10000.140655046023},
                                 {-9995.9631212660443, 10000.139884319824},
                                 {-9995.9688798750885, 10000.140477210467}});
    const double d = 0.069942670860089734;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Left, JoinType::Square, kTol, 2.0);
    requireCleanChains(chains, 1e-6);
    const DistanceExtremes ex = distanceExtremes(chains, in, d);
    CHECK(ex.under <= distanceAllowance(d, Vec2{-1e4, 1e4}));
    // The same input moved to the origin gives the same chains, needle included.
    const Vec2 shift{1e4, -1e4};
    const std::vector<Contour> nearOrigin =
        offsetOpenContour(translated(in, shift), d, OffsetSide::Left, JoinType::Square, kTol, 2.0);
    REQUIRE(nearOrigin.size() == chains.size());
    for (std::size_t i = 0; i < chains.size(); ++i) {
        CAPTURE(i);
        CHECK(nearOrigin[i].spans.size() == chains[i].spans.size());
        CHECK(nearOrigin[i].closed == chains[i].closed);
    }
}

namespace {

// No span of a chain crosses or touches another one, except adjacent spans at their junction
// (within 1e-9 of the coordinates' size).
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
                    const double junctionSlack =
                        1e-9 * std::max({1.0, std::fabs(x.point.x), std::fabs(x.point.y)});
                    const bool atJunction =
                        (next && distance(x.point, endPoint(c.spans[i])) <= junctionSlack) ||
                        (wrap && distance(x.point, startPoint(c.spans[i])) <= junctionSlack) ||
                        (n == 2 && c.closed && distance(x.point, endPoint(c.spans[j])) <= junctionSlack);
                    if (!atJunction) {
                        ++count;
                    }
                }
            }
        }
    }
    return count;
}

}  // namespace

TEST_CASE("T-0017: a Round extension next to a wiggly input stays at d between its samples (gen3 0x9300156)",
          "[geometry][offset][open][t0017]") {
    // Exact short walk (steps ~0.006 mm), d = 0.108, Round, Left. A snapped junction extended an
    // offset line 5.4e-4 mm past its segment; checked only at its far end and middle, it ended
    // 1.32e-6 mm beyond d (allowance 1.16e-6) where the distance to the wiggly input is not
    // monotone. Junction repairs are now checked every tol / 8.
    const Contour in = polyline({{0, 0},
                                 {-0.0054136337315826809, -0.0025513712606099938},
                                 {-0.010435280620443549, -0.0058071335389083389},
                                 {-0.015615556189461859, -0.008804077068277634},
                                 {-0.021206138261324621, -0.010940035304518017},
                                 {-0.027121885865398345, -0.011846046217306842},
                                 {-0.032587608415365107, -0.014283832609914128},
                                 {-0.038151992166310836, -0.016487138980214824},
                                 {-0.044066883875394251, -0.017398720879449756},
                                 {-0.049601350907194808, -0.019676131725807781},
                                 {-0.055226785835634028, -0.02171853407478129},
                                 {-0.061211119511915574, -0.021786914920923005},
                                 {-0.066992106567154888, -0.02023865326462846},
                                 {-0.072438653913284121, -0.017758321130504254},
                                 {-0.078415672595350705, -0.017454720795485646},
                                 {-0.084337993590136301, -0.018316718199146598},
                                 {-0.090218877003883091, -0.017206702089752494},
                                 {-0.096201462748697522, -0.017046723043813074},
                                 {-0.10206400352584247, -0.018249861076421601},
                                 {-0.10804295392483335, -0.017987034868437526},
                                 {-0.11394118638693301, -0.019000829380274191},
                                 {-0.11987313465770358, -0.018207787639822535},
                                 {-0.12572678684708058, -0.019453452737269745},
                                 {-0.1317113948805563, -0.019490764596701137},
                                 {-0.13748613617917541, -0.021062161562212866},
                                 {-0.14302565564023909, -0.023327255297512395},
                                 {-0.14877820870608496, -0.024978030483242892},
                                 {-0.1547617990750168, -0.025094528491906836},
                                 {-0.16060178960575788, -0.026402747162485585},
                                 {-0.16607177003914822, -0.028830964522509852},
                                 {-0.17059708798170764, -0.032747394402891511},
                                 {-0.17592911043898501, -0.035465199032419931},
                                 {-0.18094159860274781, -0.038735044272998502},
                                 {-0.18657790043225719, -0.040747263738132838},
                                 {-0.19249973053678401, -0.04161262707356847},
                                 {-0.19847782710576806, -0.04133104777550535},
                                 {-0.20412152100751962, -0.039339655139050023},
                                 {-0.20946869948564775, -0.036651792082225902},
                                 {-0.21525958399474723, -0.035140970421185712},
                                 {-0.22122466401062144, -0.034656464597263202},
                                 {-0.22720386406497417, -0.034399380565817143},
                                 {-0.23267827882034514, -0.031981176981465124},
                                 {-0.23854542798392867, -0.030800716824806909},
                                 {-0.24390231676424395, -0.028132258827286899},
                                 {-0.24870131216484534, -0.02455642764639996},
                                 {-0.25283447467529546, -0.020228160885743213},
                                 {-0.25512396578487384, -0.014698680263807919},
                                 {-0.25901347802037256, -0.010150205787777252},
                                 {-0.26095847373085673, -0.0044903544491470054},
                                 {-0.26241585170772019, 0.0013142105579616072},
                                 {-0.2657226709359839, 0.0063023837787691802},
                                 {-0.26989537677953185, 0.010592541285483491},
                                 {-0.27350443127072438, 0.015366601524818509},
                                 {-0.27717519119902018, 0.020093381299465762},
                                 {-0.28059381210100065, 0.025005606520049472},
                                 {-0.28451438860189271, 0.029527332450471355},
                                 {-0.28826449314742975, 0.034191412358313723},
                                 {-0.2926928253139085, 0.038217179330273993},
                                 {-0.29747286888876012, 0.041818305391899208},
                                 {-0.30303959150093968, 0.044015695855209161},
                                 {-0.30794599999941386, 0.047442659691507516},
                                 {-0.31357943231598756, 0.049462898742654519},
                                 {-0.31862535196844177, 0.052680913713125103},
                                 {-0.32413198304196777, 0.055024829178596128},
                                 {-0.32986069544143759, 0.056756527689699368},
                                 {-0.33547451231131525, 0.058830650550253577},
                                 {-0.34044236153502844, 0.062167924824215563},
                                 {-0.34604644417591057, 0.064268206529930316},
                                 {-0.35203086247899867, 0.0642076832876117},
                                 {-0.35756396144419078, 0.061926950657383795},
                                 {-0.362501932743235, 0.058545624743040113},
                                 {-0.36637842783930874, 0.053986051159982965},
                                 {-0.37051965875724385, 0.049665503512573601},
                                 {-0.37594594335389059, 0.047141149938806456},
                                 {-0.3818572380660018, 0.046206526851266826},
                                 {-0.38774331899002656, 0.045124410016064515},
                                 {-0.39289388647352014, 0.042076692823283107},
                                 {-0.39831833876602191, 0.039548404308186576},
                                 {-0.40287375725284275, 0.035667027245966577},
                                 {-0.40775436531081832, 0.032203417788533403},
                                 {-0.41104553910027997, 0.027204907952916683},
                                 {-0.41331606392175396, 0.021667612356303941},
                                 {-0.41498221464480395, 0.015919493772169096},
                                 {-0.41573221389067527, 0.0099819499516664717},
                                 {-0.41806748487584056, 0.0044716473361069839},
                                 {-0.41868928088753032, -0.0014806879374678478},
                                 {-0.41919797614525767, -0.0074437538119959974},
                                 {-0.42008636559973073, -0.013362173336360379},
                                 {-0.41906735292613612, -0.019259506518422419},
                                 {-0.41911143018753161, -0.025244068547863197},
                                 {-0.4206963256572942, -0.031015119737995072},
                                 {-0.42181845212806673, -0.036893704403778843},
                                 {-0.42166497414740534, -0.042876460457526022},
                                 {-0.42041846820246498, -0.04872993365029156},
                                 {-0.41875567022179028, -0.05447902300119406},
                                 {-0.416443591197242, -0.059999096920394818},
                                 {-0.41237981988432532, -0.064392579389406346},
                                 {-0.40735165546775665, -0.067638267003633734},
                                 {-0.40168715865995402, -0.069569691711871204},
                                 {-0.39697437891113146, -0.07325840863355064},
                                 {-0.39219340740913528, -0.076858302657865349}});
    const double d = 0.10786838074101836;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Left, JoinType::Round, kTol, 1.1680522392316532);
    const DistanceExtremes ex = distanceExtremes(chains, in, d);
    CHECK(ex.under <= distanceAllowance(d, Vec2{}));
    CHECK(ex.over <= distanceAllowance(d, Vec2{}));
    requireCleanChains(chains, 1e-6);
}

TEST_CASE("T-0017: Round arcs about vertices 2d apart meet where they cross, not past it (gen3 0x9300047)",
          "[geometry][offset][open][t0017]") {
    // Collinear runs ~9000 mm from the origin with ~1e-9 mm of noise, d = 0.454, Round, Left. Two
    // Round arcs about vertices almost exactly 2d apart cross twice 1.6e-5 mm apart at a needle
    // tip; snapping met them at the crossing that needed extensions, so they crossed at the
    // other. Equally near meeting points now prefer the one reached by trimming.
    const Contour in =
        polyline({{8996.5094022876892, -9023.9341099606918}, {8999.2511640390348, -9023.9341099587818},
                  {8997.5246192206669, -8992.5000074484542}, {8996.48212271169, -8992.8555907930477},
                  {8994.8010534908444, -9023.0256787452054}, {8993.9247984476751, -9023.025678744023},
                  {8995.1172128405397, -8989.5814432155785}, {8998.5650113316915, -8989.7458334173298},
                  {9000.0492545201978, -9023.0256787483322}, {8994.9771669496567, -9023.0256787496073},
                  {8994.1409311328771, -9055.0230421255274}, {8993.5300442453972, -9055.682506074776},
                  {8993.9247984443646, -9023.934109956379},  {8994.8010534924397, -9023.9341099604699},
                  {8993.8927706896393, -9057.3145200221243}, {9000.5377438816668, -9057.6143270223802},
                  {8999.2511640359662, -9023.9341099589401}, {8996.5094022831763, -9023.9341099573357},
                  {8995.350273246313, -8992.892519298588},   {8995.4452588795812, -8993.8226974029476},
                  {8995.2315027401946, -9023.9341099615867}, {9000.2950330495114, -9023.9341099596986},
                  {9002.0501470963081, -8993.4333677861396}, {9000.8696419065764, -8994.3047799592132},
                  {9000.2950330493695, -9023.9341099569938}, {8995.2315027418499, -9023.9341099572648},
                  {8996.7095258541285, -9054.1356312614553}, {8993.3627061846601, -9053.2101330708447},
                  {8994.9771669507554, -9023.934109959253},  {9000.0492545222696, -9023.9341099613594},
                  {8998.2486953084026, -9058.7286199307491}, {9001.1408081919781, -9057.7687373789704},
                  {9000.2950330491803, -9023.025678745269},  {8995.2315027422701, -9023.0256787482613},
                  {8995.2569920857932, -8989.5014313946849}, {8998.3831134823431, -8990.0418276246655},
                  {8999.2511640344055, -9023.0256787473154}, {8996.5094022868943, -9023.025678744636},
                  {8994.7248328600272, -9056.898322267878},  {9000.8417351245698, -9057.7457798926607},
                  {9000.0492545213856, -9023.9341099617523}, {8994.9771669528945, -9023.9341099606299},
                  {8996.8447922136711, -9054.9939859294263}, {8993.9389586835623, -9055.7191093262081},
                  {8994.9771669484981, -9023.0256787461949}, {9000.0492545186535, -9023.0256787492835}});
    const double d = 0.45421560625090113;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Left, JoinType::Round, kTol, 2.0);
    CHECK(countSelfCrossings(chains) == 0);
    requireCleanChains(chains, 1e-6);
}

TEST_CASE(
    "T-0017: a lens between opposite near-co-circular arcs is dropped, not bent into a crossing (gen3 "
    "0x9300698)",
    "[geometry][offset][open][t0017]") {
    // Co-circular arcs with ~5e-6 mm of noise, d = 0.246, Sharp, Right: two offset arcs running
    // opposite ways 5e-6 mm apart bound a 1.45 mm lens whose near-tangent ends were closed by
    // 1e-3 mm bends, one crossing an arc. A loop that narrow encloses nothing a tool could follow.
    Contour in;
    in.spans.emplace_back(ArcSpan{{1.0635070514947627e-06, 2.0310758936650285e-06},
                                  2.2353028219769837,
                                  1.2369595170096863,
                                  -0.71174911541833641,
                                  ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{1.692618371792735, -1.4600066516482859}, {18.600960985750532, -16.044718856765844}});
    in.spans.emplace_back(
        LineSpan{{18.600960985750532, -16.044718856765844}, {15.688773039021658, -19.660770343560522}});
    in.spans.emplace_back(
        LineSpan{{15.688773039021658, -19.660770343560522}, {1.319724712158967, -1.13835881791578}});
    in.spans.emplace_back(ArcSpan{{2.5939678405104105e-06, -5.1341604924822524e-08},
                                  1.7428502949706124,
                                  -0.71174911541833641,
                                  1.2369595170096863,
                                  ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{0.57108308015990561, 1.6466311244138478}, {7.6781616166322673, 22.13890996724724}});
    in.spans.emplace_back(
        LineSpan{{7.6781616166322673, 22.13890996724724}, {-3.4832788764084661, -22.917347153429596}});
    in.spans.emplace_back(
        LineSpan{{-3.4832788764084661, -22.917347153429596}, {-0.25229425265220268, -1.7244884834202889}});
    in.spans.emplace_back(ArcSpan{{-3.326028964102671e-08, 2.7250480846609615e-06},
                                  1.7428489037272528,
                                  -1.7160663991944269,
                                  -0.7032058428069663,
                                  ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{1.3293980533757581, -1.127038990449484}, {17.828712809418498, -15.114890691773354}});
    in.spans.emplace_back(
        LineSpan{{17.828712809418498, -15.114890691773354}, {15.259617869706736, 18.483838556097343}});
    in.spans.emplace_back(
        LineSpan{{15.259617869706736, 18.483838556097343}, {1.1361037315557854, 1.9250635846418378}});
    in.spans.emplace_back(ArcSpan{{-1.9956875122604129e-06, 1.7219368796383501e-06},
                                  2.2353074506018418,
                                  1.037639217216928,
                                  -0.078442976715351165,
                                  ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{2.2284317213441112, -0.17516267944858732}, {23.280033388236522, -1.8299078731122673}});
    in.spans.emplace_back(
        LineSpan{{23.280033388236522, -1.8299078731122673}, {22.999238432503073, 1.8497128504756084}});
    in.spans.emplace_back(
        LineSpan{{22.999238432503073, 1.8497128504756084}, {1.7374863840970372, -0.13657355434039661}});
    in.spans.emplace_back(ArcSpan{{-3.0030163236669816e-06, 5.2730324187531646e-07},
                                  1.7428487743084209,
                                  -0.078442976715351165,
                                  1.037639217216928,
                                  ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{0.88580824128371216, 1.5009536543461843}, {11.945648516992138, 20.241173453132586}});
    in.spans.emplace_back(
        LineSpan{{11.945648516992138, 20.241173453132586}, {15.110816324336502, -18.987403547125403}});
    in.spans.emplace_back(
        LineSpan{{15.110816324336502, -18.987403547125403}, {1.7050275381662989, -1.4454968880867101}});
    in.spans.emplace_back(ArcSpan{{-8.7338639162805878e-07, -1.994385595265258e-06},
                                  2.2353025235788304,
                                  -0.7032058428069663,
                                  -1.7160663991944269,
                                  ArcDirection::CW});
    const double d = 0.24622823291257359;
    const double mitre = 2.9164368991153471;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Right, JoinType::Sharp, kTol, mitre);
    CHECK(countSelfCrossings(chains) == 0);
    requireCleanChains(chains, 1e-6);
    for (const Contour& c : chains) {
        if (c.closed) {
            double perimeter = 0.0;
            for (const Span& s : c.spans) {
                perimeter += length(s);
            }
            CHECK(perimeter > 0.1);  // the loops left are the big ones round the input's lobes
        }
    }
}

TEST_CASE("T-0017: next to a gap in the input the safety net still removes a step back (gen3 0x930046a)",
          "[geometry][offset][open][t0017]") {
    // Mixed arcs with ~1e-5 mm of noise that leaves gaps between consecutive input spans, d =
    // 0.181, Round, Right. Next to a 1.4e-5 mm gap the Round join lies 3.2e-6 mm beyond d (the
    // documented input-gap limitation) and missed the next offset by 3.2e-6 mm, leaving a short
    // span running back; every bridge failed the "not beyond d" check. A bridge may now stay as
    // far beyond d as its end points already are.
    Contour in;
    in.spans.emplace_back(ArcSpan{{10042.990176592648, -10145.151172822918},
                                  0.0056692329841511848,
                                  -0.70988936876973974,
                                  1.1577814838729976,
                                  ArcDirection::CCW});
    in.spans.emplace_back(ArcSpan{{10042.996415540973, -10145.143141531787},
                                  0.0048758087929862042,
                                  -2.5192450317754358,
                                  0.11860784482414033,
                                  ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{10043.001257093951, -10145.142564577587}, {10043.000277831445, -10145.134088870665}});
    in.spans.emplace_back(ArcSpan{{10042.979080930432, -10145.136614845522},
                                  0.021346877089786512,
                                  0.11860784482414001,
                                  0.31812762086531415,
                                  ArcDirection::CCW});
    in.spans.emplace_back(ArcSpan{{10042.719236402705, -10145.30706673615},
                                  0.3314244343242761,
                                  0.56389629323009038,
                                  3.4712910678799975,
                                  ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{10042.405662479381, -10145.414367948999}, {10042.363951879093, -10145.451959472068}});
    in.spans.emplace_back(ArcSpan{{10042.06764071936, -10145.123235078358},
                                  0.44256076464333405,
                                  -0.83720997562975841,
                                  -2.2007998320611875,
                                  ArcDirection::CW});
    in.spans.emplace_back(ArcSpan{{10041.816610415408, -10145.480120011447},
                                  0.0097442891481605547,
                                  -3.0702430420692881,
                                  -7.2296568191126553,
                                  ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{10041.822306435628, -10145.488026118242}, {10041.769486524105, -10145.526070902673}});
    in.spans.emplace_back(ArcSpan{{10041.777021015308, -10145.536528817112},
                                  0.012889396111548008,
                                  2.1951211416567191,
                                  6.0938396469038771,
                                  ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{10041.789680047012, -10145.538954811431}, {10041.86855333064, -10145.96000199588}});
    in.spans.emplace_back(ArcSpan{{10041.870011712253, -10145.952799179953},
                                  0.0073489750449806978,
                                  -1.7705694186065517,
                                  -1.0176007312260649,
                                  ArcDirection::CCW});
    in.spans.emplace_back(ArcSpan{{10041.879725305487, -10145.945266630533},
                                  0.014981792285013819,
                                  -1.9726231224360227,
                                  2.4091341498226946,
                                  ArcDirection::CCW});
    in.spans.emplace_back(ArcSpan{{10041.88291248247, -10145.93323573652},
                                  0.014431950033253168,
                                  -2.999101844609831,
                                  -0.10486320297883678,
                                  ArcDirection::CCW});
    in.spans.emplace_back(ArcSpan{{10041.906472570889, -10145.936268691617},
                                  0.0093631665265022663,
                                  2.9781627670283424,
                                  -1.3040645916978875,
                                  ArcDirection::CW});
    in.spans.emplace_back(ArcSpan{{10041.776912172925, -10146.014506946385},
                                  0.14909250829287946,
                                  0.48306542533138924,
                                  -1.6242324809407036,
                                  ArcDirection::CW});
    in.spans.emplace_back(ArcSpan{{10041.769456643042, -10146.153877254184},
                                  0.0095135448898883233,
                                  -1.6242324809406936,
                                  -4.3680582406578932,
                                  ArcDirection::CW});
    const double d = 0.18060785958129802;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Right, JoinType::Round, kTol, 2.0);
    requireCleanChains(chains, 1e-6);
    double inputGap = 0.0;
    for (std::size_t i = 0; i + 1 < in.spans.size(); ++i) {
        inputGap = std::max(inputGap, distance(endPoint(in.spans[i]), startPoint(in.spans[i + 1])));
    }
    const DistanceExtremes ex = distanceExtremes(chains, in, d);
    CHECK(ex.under <= distanceAllowance(d, Vec2{1e4, 1e4}));
    CHECK(ex.over <= inputGap);
}

TEST_CASE(
    "T-0017: a snapped extension of rounding length far from the origin keeps the offset at d (gen3 "
    "0x9400bf7)",
    "[geometry][offset][open][t0017]") {
    // Exact short walk ~1e4 mm from the origin, d = 0.182, Square, Left (shrunk from 239 to 10
    // lines). A junction was closed by extending both spans ~9e-6 mm, below the "rounding" level
    // 1e-9 x |x| at those coordinates, so the extension went unchecked; at a steep angle to
    // another part of the input it ended 9.4e-7 mm inside d (allowance 6.9e-7). Only moves at the
    // level of the coordinates' last bits now go unchecked.
    const Contour in = polyline({{-612.44354139524489, -9540.2395988581757},
                                 {-612.45619885366364, -9540.2418256919191},
                                 {-612.46902217709624, -9540.2409698775209},
                                 {-612.48186941473148, -9540.2413141554425},
                                 {-612.49410278892799, -9540.2452530128794},
                                 {-612.50692112676938, -9540.2461805133516},
                                 {-612.51970810823968, -9540.2474701439278},
                                 {-612.53167917914664, -9540.2521459864693},
                                 {-612.54309403377624, -9540.2580511652086},
                                 {-612.55225443128097, -9540.2670654405374},
                                 {-612.56306569485218, -9540.2740142981511}});
    const double d = 0.18198055231497526;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Left, JoinType::Square, kTol, 2.0);
    CHECK(distanceExtremes(chains, in, d).under <= distanceAllowance(d, Vec2{-612.5, -9540.4}));
    requireCleanChains(chains, 1e-6);
}

namespace {

// Hairpin: an entry (line, or arc turning gently left), a U-turn arc of radius d (1 + eps)
// turning left by pi (1 + 1e-3), and an exit (line, or arc turning gently right), starting at
// `origin` with heading `h`. Offsetting to the left puts the offset inside the U-turn, where it
// nearly vanishes.
Contour hairpin(Vec2 origin, double h, double d, double eps, bool arcIn, bool arcOut) {
    Contour c;
    Vec2 p = origin;
    auto dir = [](double a) { return Vec2{std::cos(a), std::sin(a)}; };
    auto left = [&](double radius, double sweep) {
        const Vec2 centre = p + dir(h + kPi / 2.0) * radius;
        const ArcSpan arc{centre, radius, h - kPi / 2.0, h - kPi / 2.0 + sweep, ArcDirection::CCW};
        c.spans.emplace_back(arc);
        p = arc.endPoint();
        h += sweep;
    };
    auto right = [&](double radius, double sweep) {
        const Vec2 centre = p + dir(h - kPi / 2.0) * radius;
        const ArcSpan arc{centre, radius, h + kPi / 2.0, h + kPi / 2.0 - sweep, ArcDirection::CW};
        c.spans.emplace_back(arc);
        p = arc.endPoint();
        h -= sweep;
    };
    auto line = [&](double len) {
        const Vec2 q = p + dir(h) * len;
        c.spans.emplace_back(LineSpan{p, q});
        p = q;
    };
    if (arcIn) {
        left(12.0, 0.15);
    } else {
        line(2.5);
    }
    left(d * (1.0 + eps), kPi * (1.0 + 1e-3));
    if (arcOut) {
        right(9.0, 0.12);
    } else {
        line(2.0);
    }
    return c;
}

std::size_t junctionGaps(const std::vector<Contour>& chains) {
    JunctionStats js;
    checkJunctions(chains, js);
    return js.gaps;
}

}  // namespace

TEST_CASE("T-0017: hairpins with a U-turn of radius about d keep exact junctions and d (fix round 2)",
          "[geometry][offset][open][t0017]") {
    // The verifier's hairpin family (hp): a U-turn arc of radius d (1 + eps) entered from and
    // left to a line or an arc, at 0, 5e3 and 1e4 mm from the origin. Arc-arc crossings computed
    // as a = (D^2 + r1^2 - r2^2) / 2D lost ~1e-4 of h there and left junction gaps (repro_hp (a));
    // line ends moved unchecked by up to 1e-9 x |x| tilted lines inwards or past d at 5e3..1e4
    // (repro_hp (b), (c)).
    std::size_t offsets = 0;
    for (const double at : {0.0, 5e3, 1e4}) {
        for (const double eps : {-1e-3, -1e-6, 1e-7, 1e-6, 1e-3}) {
            for (int variant = 0; variant < 4; ++variant) {
                for (const double d : {0.07, 0.46, 1.9}) {
                    const Vec2 origin{at, -0.7 * at};
                    const Contour in =
                        hairpin(origin, 0.3 + variant, d, eps, (variant & 1) != 0, (variant & 2) != 0);
                    for (const JoinType join : kJoins) {
                        CAPTURE(at, eps, variant, d, static_cast<int>(join));
                        const std::vector<Contour> chains =
                            offsetOpenContour(in, d, OffsetSide::Left, join, kTol, 2.0);
                        ++offsets;
                        JunctionStats js;
                        checkJunctions(chains, js);
                        CHECK(js.backward == 0);
                        CHECK(js.notIdentical == 0);
                        // Arc-arc junctions: exact, or within 1e-12 x the scale (ADR-0012 A1).
                        CHECK(js.maxGap <= 1e-12 * std::max({1.0, at, 0.7 * at, 12.0}));
                        const DistanceExtremes ex = distanceExtremes(chains, in, d);
                        CHECK(ex.under <= distanceAllowance(d, origin));
                        if (join == JoinType::Round) {
                            CHECK(ex.over <= distanceAllowance(d, origin));
                        }
                    }
                }
            }
        }
    }
    CHECK(offsets == 540);
}

TEST_CASE("T-0017: an arc into a U-turn of radius d (1 + 1e-6) meets it exactly (repro_hp a)",
          "[geometry][offset][open][t0017]") {
    Contour in;
    in.spans.emplace_back(ArcSpan{{-6.1835881590179307, -4.5609137211896442},
                                  7.6836642620876354,
                                  0.63550719201869088,
                                  0.59034783467233742,
                                  ArcDirection::CW});
    in.spans.emplace_back(ArcSpan{{0.5819774350952871, -0.027581994245153507},
                                  0.46028897665152524,
                                  -2.5512448189174557,
                                  0.58554158006787915,
                                  ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{0.96558820755480146, 0.22679708759909528}, {-1.0290918042762391, 3.2348303527579176}});
    const double d = 0.46028851636300894;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Left, JoinType::Round, kTol, 2.0);
    CHECK(junctionGaps(chains) == 0);
    requireCleanChains(chains, 1e-6);
    const DistanceExtremes ex = distanceExtremes(chains, in, d);
    CHECK(ex.under <= distanceAllowance(d, Vec2{}));
    CHECK(ex.over <= distanceAllowance(d, Vec2{}));
}

TEST_CASE("T-0017: a rounding-gap line end at 5e3..1e4 mm is moved only where it stays at d (repro_hp b, c)",
          "[geometry][offset][open][t0017]") {
    {
        Contour in;
        in.spans.emplace_back(
            LineSpan{{5188.4461692588793, -4233.4568454048558}, {5192.3005941971678, -4231.826230991167}});
        in.spans.emplace_back(ArcSpan{{5192.2285411430012, -4231.6559129279085},
                                      0.18493210993996412,
                                      -1.1705785180382842,
                                      1.9688563277542461,
                                      ArcDirection::CCW});
        in.spans.emplace_back(
            LineSpan{{5192.1568557699384, -4231.4854397846402}, {5186.9319562479923, -4233.6825534429363}});
        const double d = 0.18493026061898363;
        const std::vector<Contour> chains =
            offsetOpenContour(in, d, OffsetSide::Left, JoinType::Round, kTol, 2.0);
        const DistanceExtremes ex = distanceExtremes(chains, in, d);
        CHECK(ex.under <= distanceAllowance(d, Vec2{5192, -4231}));
        CHECK(ex.over <= distanceAllowance(d, Vec2{5192, -4231}));
        CHECK(junctionGaps(chains) == 0);
    }
    {
        Contour in;
        in.spans.emplace_back(LineSpan{{10000, -7000}, {9999.7699140805944, -6998.8108121181322}});
        in.spans.emplace_back(ArcSpan{{9999.295782931682, -6998.9025477496334},
                                      0.48292418913950697,
                                      0.19112003894738341,
                                      3.3333454522343238,
                                      ArcDirection::CCW});
        in.spans.emplace_back(ArcSpan{{9979.9701657394107, -7002.654385997299},
                                      19.203512006852304,
                                      0.1917527986445309,
                                      0.10631536567386451,
                                      ArcDirection::CW});
        const double d = 0.48291935994590746;
        // All joins: with Sharp a moved line end checked at only 4 points came 3.8e-6 mm inside d
        // just before its end (verifier hp sweep, variant 2, eps 1e-5).
        for (const JoinType join : kJoins) {
            CAPTURE(static_cast<int>(join));
            const std::vector<Contour> chains = offsetOpenContour(in, d, OffsetSide::Left, join, kTol, 2.0);
            const DistanceExtremes ex = distanceExtremes(chains, in, d);
            CHECK(ex.under <= distanceAllowance(d, Vec2{1e4, -7e3}));
            if (join == JoinType::Round) {
                CHECK(ex.over <= distanceAllowance(d, Vec2{1e4, -7e3}));
            }
            CHECK(junctionGaps(chains) == 0);
        }
    }
}

TEST_CASE("T-0017: stitching matches ends globally, so a closed zig-zag is not split (0xe2000124)",
          "[geometry][offset][open][t0017]") {
    // Verifier fam2 closedZigzag-seam (noisy), Square, Right, d = 88.6. The piece-by-piece stitching
    // took a start that another chain needed and returned 4 chains for main's 2.
    Contour in = polyline({{-2.7580688007887257, -13.331069956806303},
                           {-7.3094735972188536, -2.7940768555638265},
                           {-3.6717530105012752, -13.680329537433709},
                           {-8.2231577846887163, -3.1433364943794411},
                           {-9.1964891937801028, -0.59704283910109668},
                           {-0.97333140669553064, 2.5462936490106416},
                           {0, 0},
                           {3.6377206110157543, -10.886252745435373},
                           {-0.91368419608293883, -0.34925961628329061},
                           {2.7240364034279625, -11.235512331621276},
                           {-1.827368386596339, -0.69851924713683033},
                           {1.8103522160388277, -11.584771970648081},
                           {-2.7410525972899773, -1.047778825197651},
                           {0.89666799704907874, -11.934031527005775},
                           {-3.6547368124913011, -1.3970383914660924},
                           {-0.01701619545865718, -12.283291152642011},
                           {-4.5684210050908778, -1.746298016862069},
                           {-0.93070041323631703, -12.632550712170605},
                           {-5.4821052001583341, -2.095557635801923},
                           {-1.8443845804940686, -12.981810403862326},
                           {-6.3957894031340929, -2.4448172340531831},
                           {-2.7580688007887257, -13.331069956806303}});
    in.closed = true;
    const double d = 88.588646191724834;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Right, JoinType::Square, kTol, 2.0);
    CHECK(chains.size() == 2);
    requireCleanChains(chains, 1e-6);
}

TEST_CASE("T-0017: no spurious chain splits on co-circular ties (0xe20010e4)",
          "[geometry][offset][open][t0017]") {
    // Verifier coCircular-tie, Square, Right: more chains than main (30), from the greedy stitching
    // or the merge keeping a stretch that copies run both ways.
    Contour in;
    in.spans.emplace_back(ArcSpan{{-528.52220731529064, -795.48055650428023},
                                  2.3862428321970466,
                                  -4.5621665668092648,
                                  -2.6741011505546641,
                                  ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{-530.65240998014099, -796.55591293862028}, {-548.94963649378906, -805.79261167470577}});
    in.spans.emplace_back(
        LineSpan{{-548.94963649378906, -805.79261167470577}, {-516.13281398203776, -774.65919515999315}});
    in.spans.emplace_back(
        LineSpan{{-516.13281398203776, -774.65919515999315}, {-527.20304046700051, -793.4921014053192}});
    in.spans.emplace_back(ArcSpan{{-528.52220731529064, -795.48055650428023},
                                  2.3862428321970466,
                                  -5.2981356216840663,
                                  -3.1700591557281022,
                                  ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{-530.90748337668583, -795.41263769137049}, {-551.66878317794499, -794.82147641667063}});
    in.spans.emplace_back(
        LineSpan{{-551.66878317794499, -794.82147641667063}, {-553.25672246990587, -792.42687469612883}});
    in.spans.emplace_back(
        LineSpan{{-553.25672246990587, -792.42687469612883}, {-530.90843047481098, -795.47086696454119}});
    in.spans.emplace_back(ArcSpan{{-528.52220731529064, -795.48055650428023},
                                  2.3862428321970466,
                                  -3.1456532488636326,
                                  -0.70919585426922938,
                                  ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{-526.71132151439201, -797.03453445467574}, {-511.02037592237946, -810.49942741410098}});
    in.spans.emplace_back(
        LineSpan{{-511.02037592237946, -810.49942741410098}, {-550.89713992024394, -804.94076250819387}});
    in.spans.emplace_back(
        LineSpan{{-550.89713992024394, -804.94076250819387}, {-530.96291329301039, -796.00011019586691}});
    in.spans.emplace_back(ArcSpan{{-528.52220731529064, -795.48055650428023},
                                  2.4953920950661193,
                                  -2.9318530340915321,
                                  -4.4399499502028501,
                                  ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{-529.19367068242752, -793.077200797209}, {-535.14309552346174, -771.78254073785104}});
    in.spans.emplace_back(
        LineSpan{{-535.14309552346174, -771.78254073785104}, {-533.34364821798954, -820.07641355322892}});
    in.spans.emplace_back(
        LineSpan{{-533.34364821798954, -820.07641355322892}, {-528.99483158744624, -797.93078261741505}});
    in.spans.emplace_back(ArcSpan{{-528.52220731529064, -795.48055650428023},
                                  2.4953920950661193,
                                  -1.7613461565305366,
                                  -3.3952454119238,
                                  ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{-530.93775248525071, -794.85435905937891}, {-552.10605282837889, -789.36676323520794}});
    in.spans.emplace_back(
        LineSpan{{-552.10605282837889, -789.36676323520794}, {-509.72595117382872, -780.35017544508185}});
    in.spans.emplace_back(
        LineSpan{{-509.72595117382872, -780.35017544508185}, {-526.64332021418943, -793.83837351525801}});
    in.spans.emplace_back(ArcSpan{{-528.52220731529064, -795.48055650428023},
                                  2.4953920950661193,
                                  0.7182741078915007,
                                  0.35533061320593529,
                                  ArcDirection::CW});
    const double d = 0.054574631434536308;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Right, JoinType::Square, kTol, 2);
    CHECK(chains.size() <= 30);
    JunctionStats js;
    checkJunctions(chains, js);
    CHECK(js.backward == 0);
    CHECK(js.gaps == 0);
}

TEST_CASE("T-0017: no spurious chain splits on co-circular ties (0xe20006d8)",
          "[geometry][offset][open][t0017]") {
    // Verifier coCircular-tie, Sharp, Right: more chains than main (21), from the greedy stitching
    // or the merge keeping a stretch that copies run both ways.
    Contour in;
    in.spans.emplace_back(
        ArcSpan{{0, 0}, 1.1919727791070662, 2.4357476808179968, 4.5278080532487923, ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{-0.21876824183394267, -1.1717250370701773}, {-4.39504096909259, -23.539886316515425}});
    in.spans.emplace_back(
        LineSpan{{-4.39504096909259, -23.539886316515425}, {-5.6677075625630859, -22.456921546747697}});
    in.spans.emplace_back(
        LineSpan{{-5.6677075625630859, -22.456921546747697}, {-0.21876824183394267, -1.1717250370701773}});
    in.spans.emplace_back(
        ArcSpan{{0, 0}, 1.1919727791070662, 4.5278080532487923, 2.4357476808179972, ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{-0.90716722429277774, 0.77320549228595103}, {-18.088632626063685, 15.417477307250458}});
    in.spans.emplace_back(
        LineSpan{{-18.088632626063685, 15.417477307250458}, {0.16115639308495969, 23.457553774169011}});
    in.spans.emplace_back(
        LineSpan{{0.16115639308495969, 23.457553774169011}, {0.080254774639365339, 1.3080613106940997}});
    in.spans.emplace_back(
        ArcSpan{{0, 0}, 1.3105209732725309, 1.5095191530933008, 0.4869420860726652, ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{1.1581963220008005, 0.61322630495682384}, {19.917905323155367, 10.545866233367279}});
    in.spans.emplace_back(
        LineSpan{{19.917905323155367, 10.545866233367279}, {23.399749486168027, -0.96406011694593097}});
    in.spans.emplace_back(
        LineSpan{{23.399749486168027, -0.96406011694593097}, {1.1828970666470202, -0.14681156579131652}});
    in.spans.emplace_back(
        ArcSpan{{0, 0}, 1.1919727791070662, -0.1234804317390692, 2.0702834027244332, ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{-0.57092556993018517, 1.0463475042891421}, {-11.591943268876204, 21.244802384193687}});
    in.spans.emplace_back(
        LineSpan{{-11.591943268876204, 21.244802384193687}, {13.480154468575595, -20.30503941079964}});
    in.spans.emplace_back(
        LineSpan{{13.480154468575595, -20.30503941079964}, {0.47285918223161294, -1.0941678572830922}});
    in.spans.emplace_back(
        ArcSpan{{0, 0}, 1.1919727791070662, -1.1628739864816351, 0.89591121102210303, ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{0.74475367723229968, 0.93066700080167797}, {13.498116409923712, 16.867659603078756}});
    in.spans.emplace_back(
        LineSpan{{13.498116409923712, 16.867659603078756}, {-2.0317464307992732, 24.111758353574221}});
    in.spans.emplace_back(
        LineSpan{{-2.0317464307992732, 24.111758353574221}, {-0.20305133823571422, 1.2946950897519722}});
    in.spans.emplace_back(
        ArcSpan{{0, 0}, 1.3105209732725309, 1.7263624429144553, 0.15341426555344007, ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{1.2951290280355914, 0.20026487991349423}, {21.280654653116034, 3.2906124844185856}});
    in.spans.emplace_back(
        LineSpan{{21.280654653116034, 3.2906124844185856}, {20.368338190482071, -9.1039641829667719}});
    in.spans.emplace_back(
        LineSpan{{20.368338190482071, -9.1039641829667719}, {1.1020365401844114, -0.70921131229384238}});
    in.spans.emplace_back(
        ArcSpan{{0, 0}, 1.3105209732725309, -0.5718248168727591, -2.8994887782050154, ArcDirection::CW});
    const double d = 0.059274097082732395;
    const std::vector<Contour> chains = offsetOpenContour(in, d, OffsetSide::Right, JoinType::Sharp, kTol, 2);
    CHECK(chains.size() <= 21);
    JunctionStats js;
    checkJunctions(chains, js);
    CHECK(js.backward == 0);
    CHECK(js.gaps == 0);
}

TEST_CASE("T-0017: no spurious chain splits on co-circular ties (0xe3001ac2)",
          "[geometry][offset][open][t0017]") {
    // Verifier coCircularNoisy, Square, Right: more chains than main (17), from the greedy stitching
    // or the merge keeping a stretch that copies run both ways.
    Contour in;
    in.spans.emplace_back(ArcSpan{{-86.78814498830485, 715.58515024155804},
                                  1.0908054494621033,
                                  0.48575230134896552,
                                  1.4728805836930272,
                                  ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{-86.681508548943796, 716.67073082280319}, {-84.456158542599809, 739.32524524425139}});
    in.spans.emplace_back(
        LineSpan{{-84.456158542599809, 739.32524524425139}, {-68.781084618225648, 728.29482572264112}});
    in.spans.emplace_back(
        LineSpan{{-68.781084618225648, 728.29482572264112}, {-85.968984963650939, 716.30545113882147}});
    in.spans.emplace_back(ArcSpan{{-86.788144988299209, 715.58515024155474},
                                  1.0908054494661392,
                                  0.72126949998118706,
                                  2.9140318809814372,
                                  ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{-87.850829022667, 715.83123797323765}, {-108.77292355259084, 720.6762065795341}});
    in.spans.emplace_back(
        LineSpan{{-108.77292355259084, 720.6762065795341}, {-78.894600869747634, 736.50093129607183}});
    in.spans.emplace_back(
        LineSpan{{-78.894600869747634, 736.50093129607183}, {-86.316656641626636, 716.73643824866053}});
    in.spans.emplace_back(ArcSpan{{-86.7881449882981, 715.58515024155656},
                                  1.2440921735741632,
                                  1.1821005065108441,
                                  0.8207966410408235,
                                  ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{-85.940123827802736, 716.49543890330108}, {-71.845465712074571, 731.62502301229119}});
    in.spans.emplace_back(
        LineSpan{{-71.845465712074571, 731.62502301229119}, {-72.810726151152167, 699.05233169187397}});
    in.spans.emplace_back(
        LineSpan{{-72.810726151152167, 699.05233169187397}, {-86.067854540091147, 714.76598102901153}});
    in.spans.emplace_back(ArcSpan{{-86.788144988302264, 715.5851502415594},
                                  1.0908054494594388,
                                  -0.84953958255408324,
                                  -0.37838253750279938,
                                  ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{-85.774499439794525, 715.18218717804007}, {-65.073000573041512, 706.95254574660839}});
    in.spans.emplace_back(
        LineSpan{{-65.073000573041512, 706.95254574660839}, {-75.968802944125443, 736.08970093582138}});
    in.spans.emplace_back(
        LineSpan{{-75.968802944125443, 736.08970093582138}, {-86.449783729782922, 716.62214984970683}});
    in.spans.emplace_back(ArcSpan{{-86.788144988304609, 715.5851502415544},
                                  1.09080544946236,
                                  1.2553992806336334,
                                  1.7333888049510962,
                                  ArcDirection::CCW});
    const double d = 0.076643362054408748;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Right, JoinType::Square, kTol, 3.310653624707161);
    CHECK(chains.size() <= 17);
    JunctionStats js;
    checkJunctions(chains, js);
    CHECK(js.backward == 0);
    CHECK(js.gaps == 0);
}

TEST_CASE("T-0017: the safety net cuts exactly as far as a back-step needs (0xe40010cb)",
          "[geometry][offset][open][t0017]") {
    // Verifier fam4 chainDrift, Sharp, Left: a sub-tolerance back-step survived the safety net,
    // whose fixed cuts (tol ... tol/8) were all too long or too short to clear it.
    Contour in = polyline({{0.0010496765366513603, 0},
                           {0.0073955920818131581, 1.071854596146545e-07},
                           {0.50739559208181317, 10.454619281539813},
                           {-0.49929214562655427, 11.454619281539813},
                           {0.00070785437344571852, -0.0004675716854893085},
                           {0.0070537657326320757, -0.00046028202401149105},
                           {0.50705376573263206, -10.454619281539813},
                           {-0.49876294728992132, -9.4546192815398129},
                           {0.001237052710078681, 0.00093514337097861701},
                           {0.007582967280140393, 0.00093162381229032216},
                           {0.50758296728014041, 10.454619281539813},
                           {-0.5001602088216196, 11.454619281539813},
                           {-0.00016020882161960155, 0.0014027150564679255},
                           {0.0061857031540754814, 0.0014094466643453415},
                           {0.50618570315407552, -10.454619281539813},
                           {-0.49984270599068781, -9.4546192815398129},
                           {0.00015729400931219689, 0.001870286741957234},
                           {0.0065032088971614865, 0.0018673964153976411}});
    const double d = 0.11365482038495342;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Left, JoinType::Sharp, kTol, 3.3483382136580797);
    JunctionStats js;
    checkJunctions(chains, js);
    CHECK(js.backward == 0);
    CHECK(js.gaps == 0);
    CHECK(distanceExtremes(chains, in, d).under <= distanceAllowance(d, Vec2{}));
}

TEST_CASE("T-0017: a snapped Round extension stays at d between its samples too (0xd4001762)",
          "[geometry][offset][open][t0017]") {
    // Verifier fam4 shortWalkFar, Round, Left: 4.2e-7 mm beyond d between the tol/8 samples
    // (allowance 3.9e-7). The distance between samples is now refined to its local extremes.
    Contour in = polyline({{0, 0},
                           {-0.002617144792886075, 0.0009616838997198524},
                           {-0.0052765476359014889, 0.0017994543387260344},
                           {-0.008045278194458173, 0.0021287169755761261},
                           {-0.010795029459401299, 0.0016670344322037784},
                           {-0.013546444716577715, 0.0012153743028056768},
                           {-0.016327788626580839, 0.0010193925582486062},
                           {-0.019112947400273729, 0.0011504398597568331},
                           {-0.021770301608403587, 0.0019946859153861188},
                           {-0.02417496671896488, 0.0034060216288010091},
                           {-0.026582775287410959, 0.0048119877335235633},
                           {-0.028600957928098303, 0.0067358436581549646},
                           {-0.030442355213943104, 0.0088295338992351695},
                           {-0.031881474876612963, 0.011217674888881311},
                           {-0.033828674501609056, 0.013213344302969268},
                           {-0.035937846844188145, 0.015036987584757267},
                           {-0.037821024356946897, 0.017093179512897936},
                           {-0.040186547984078849, 0.01856918112515709},
                           {-0.042479338094820802, 0.020155811610807404},
                           {-0.04513833765219262, 0.020994861158693322},
                           {-0.04764930443405687, 0.022207019815918265},
                           {-0.050193514522477577, 0.023347755462722289},
                           {-0.052955947726039759, 0.023726233448565958},
                           {-0.055715183046279475, 0.024127360906430303},
                           {-0.058491654482236727, 0.023871452416487977},
                           {-0.061252249814253576, 0.024263110689422208},
                           {-0.064037253210832756, 0.024397419553439041},
                           {-0.066656642779243797, 0.025352972314311256},
                           {-0.069085035074225809, 0.026723079482230273},
                           {-0.071767248208076423, 0.027484666965393111},
                           {-0.074208083513842185, 0.028832482160454542},
                           {-0.076733741964666632, 0.030013723956062093},
                           {-0.078820518552206315, 0.031862952515112397},
                           {-0.081147015754012647, 0.033399732772630836},
                           {-0.083828398939788842, 0.034164237187888039},
                           {-0.086283418849209828, 0.0354860390014205},
                           {-0.088510024534533382, 0.037164285083494238},
                           {-0.090201621684830258, 0.039380764695631149},
                           {-0.091650542402871946, 0.041762971979378866},
                           {-0.093473412670568995, 0.043872812441844482},
                           {-0.094822867131133748, 0.046312741836063293},
                           {-0.095658598613456117, 0.048972786136327162}});
    const double d = 0.32201947116387097;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Left, JoinType::Round, kTol, 2.0);
    const DistanceExtremes ex = distanceExtremes(chains, in, d);
    CHECK(ex.under <= distanceAllowance(d, Vec2{}));
    CHECK(ex.over <= distanceAllowance(d, Vec2{}));
}

TEST_CASE("T-0017: a bend on a short walk stays within the Round allowance (0xe30009ae)",
          "[geometry][offset][open][t0017]") {
    // Verifier gen3 shortWalk, Round, Right, shrunk from 214 to 4 lines: 6.7e-7 mm beyond d
    // (allowance 5.7e-7) between the samples of a checked join.
    Contour in = polyline({{-1.3663708911957717, 0.45694987180560709},
                           {-1.370032676929517, 0.46338718093797288},
                           {-1.3739290056930742, 0.46968529540118953},
                           {-1.3773921176868829, 0.47623162864536828},
                           {-1.3808726239246627, 0.48276873044618773}});
    const double d = 0.22085132033759949;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Right, JoinType::Round, kTol, 2.0);
    const DistanceExtremes ex = distanceExtremes(chains, in, d);
    CHECK(ex.under <= distanceAllowance(d, Vec2{}));
    CHECK(ex.over <= distanceAllowance(d, Vec2{}));
}

TEST_CASE("T-0017: a Round corner at an input gap is an arc about each end of the gap (0xe30005b6)",
          "[geometry][offset][open][t0017]") {
    // Verifier gen3 mixedNoisy, Round, Left, ~6e3 mm from the origin, input gaps of 1.4-5.3e-6 mm.
    // One join arc about the next span's start ran 5.3e-6 mm beyond d and dipped ~1e-8 mm inside d
    // of the previous span's end, so the pruning cut 0.019 mm of it out: 9 chains for main's 7, a
    // valid offset point 0.0093 mm from the output, and a corner left open.
    Contour in;
    in.spans.emplace_back(ArcSpan{{-4867.1435005564144, -5990.1403459968051},
                                  2.5914697416262196,
                                  2.2680277277875818,
                                  4.5488985366090446,
                                  ArcDirection::CCW});
    in.spans.emplace_back(ArcSpan{{-4867.5716266691343, -5992.735627499721},
                                  0.038888278167742055,
                                  1.4073058830192622,
                                  -0.88998075797788312,
                                  ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{-4867.5471493376926, -5992.7658460110069}, {-4870.3532144266082, -5995.0387951216017}});
    in.spans.emplace_back(ArcSpan{{-4870.0108046612486, -5995.4615174017608},
                                  0.54400236539630853,
                                  2.2516118956119011,
                                  6.7456041835472593,
                                  ArcDirection::CCW});
    in.spans.emplace_back(ArcSpan{{-4871.2378237019284, -5996.7187762641806},
                                  2.2775586815494946,
                                  0.71892639065476116,
                                  -3.6734215059202042,
                                  ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{-4873.200808443351, -5995.5638024753689}, {-4873.1044659320241, -5995.4000621348878}});
    in.spans.emplace_back(
        LineSpan{{-4873.1044659320241, -5995.4000621348878}, {-4872.4208286289313, -5994.2381560861986}});
    in.spans.emplace_back(
        LineSpan{{-4872.4208286289313, -5994.2381560861986}, {-4872.183153306416, -5995.379374479643}});
    in.spans.emplace_back(ArcSpan{{-4872.1357752352696, -5995.3695072298642},
                                  0.048394671645930316,
                                  -2.9362616275055844,
                                  -0.40856347193870146,
                                  ArcDirection::CCW});
    in.spans.emplace_back(ArcSpan{{-4872.1164981933262, -5995.3106720228197},
                                  0.08200759952260718,
                                  -1.2593249932060053,
                                  -4.286710894276867,
                                  ArcDirection::CW});
    in.spans.emplace_back(ArcSpan{{-4872.1721666826006, -5995.1878940402348},
                                  0.052803404343170164,
                                  -1.1451182406870728,
                                  2.7573911258417785,
                                  ArcDirection::CCW});
    in.spans.emplace_back(ArcSpan{{-4867.9675977595271, -5997.6983506405086},
                                  4.9492005148181279,
                                  -3.6782235048821015,
                                  -3.9097180104070226,
                                  ArcDirection::CW});
    in.spans.emplace_back(ArcSpan{{-4868.5293542659247, -5992.5733144167962},
                                  3.4395616646332146,
                                  -2.6291691962538533,
                                  -3.6832948435304935,
                                  ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{-4871.4764817364194, -5990.7998928319857}, {-4871.0087413022802, -5993.3502787775042}});
    in.spans.emplace_back(
        LineSpan{{-4871.0087413022802, -5993.3502787775042}, {-4869.9389687583534, -5990.9623466465164}});
    in.spans.emplace_back(
        LineSpan{{-4869.9389687583534, -5990.9623466465164}, {-4869.1773749469203, -5989.2623248076634}});
    in.spans.emplace_back(
        LineSpan{{-4869.1773749469203, -5989.2623248076634}, {-4867.8498940150221, -5986.2991454069124}});
    in.spans.emplace_back(
        LineSpan{{-4867.8498940150221, -5986.2991454069124}, {-4867.7167074323106, -5986.0018497522196}});
    in.spans.emplace_back(ArcSpan{{-4867.6453016329133, -5986.0338389172166},
                                  0.078243816782941927,
                                  -3.5627747492598156,
                                  -5.2939234071540042,
                                  ArcDirection::CW});
    in.spans.emplace_back(ArcSpan{{-4869.2921912588772, -5987.0847785922051},
                                  2.0252969108898462,
                                  0.58378829667518972,
                                  -1.8265216833646691,
                                  ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{-4869.8044845290333, -5989.0442129959583}, {-4869.9973616594034, -5988.9937871034563}});
    const double d = 0.13880492771945446;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Left, JoinType::Round, kTol, 1.4732506836278474);
    CHECK(chains.size() <= 7);
    const DistanceExtremes ex = distanceExtremes(chains, in, d);
    CHECK(ex.under <= distanceAllowance(d, Vec2{-4870, -5995}));
    CHECK(ex.over <= distanceAllowance(d, Vec2{-4870, -5995}));
    // Points of the exact offset that the split chain missed.
    for (const Vec2 p : {Vec2{-4869.6523, -5995.2716}, Vec2{-4867.5169, -5985.8590}}) {
        CAPTURE(p.x, p.y);
        double nearest = std::numeric_limits<double>::infinity();
        for (const Contour& c : chains) {
            for (const Span& s : c.spans) {
                nearest = std::min(nearest, carvetto::geometry::testing::distanceToSpanExact(p, s));
            }
        }
        CHECK(nearest <= 1e-3);
    }
}

TEST_CASE("T-0017: arcs about vertices 2d apart that just miss each other still join exactly (0xe300070e)",
          "[geometry][offset][open][t0017]") {
    // Verifier gen3 outerTie, Round, Left: two Round arcs about vertices 2d + 7.2e-12 mm apart, the
    // chain passing from one to the other at their (near) touching point, kept a 7.2e-12 mm gap: the
    // second arc's centre lies on the x axis, whose ulp steps (1e-28) could not move it there.
    Contour in = polyline(
        {{6.6283223938981575, -1.2779953526139643e-11},  {5.023667459767422, 2.1623209668586711e-11},
         {6.9756096886500769, 34.51051481312367},        {6.3189901091486398, 34.312480314617368},
         {4.6667279880904458, 2.0657439862650513},       {0.69533962297162577, 2.0657439862566656},
         {1.9562893692302896, 33.119825834028326},       {7.04511858784476, 33.809684447840162},
         {6.6283223939411231, 2.0657439862576599},       {5.0236674597854529, 2.0657439862438181},
         {5.4657050614391105, 36.723239966815832},       {4.5703929732694029, 36.057839455949605},
         {4.8076418394176894, 3.1120034057046923e-12},   {8.8588515885397729, -8.7094539924375882e-13},
         {9.7907278097679189, -35.00186354855979},       {1.9705389447012767, -34.43692474497638},
         {0.69533962299185814, -2.6581188530012263e-11}, {4.6667279880936041, -3.0346127572489825e-11},
         {3.1154143986946496, 34.81196163001529},        {4.5941198790408642, 35.289788649021553},
         {5.0236674597518789, 1.9744273286251705e-11},   {6.6283223939582836, -1.3296097279548633e-11},
         {6.980860505868165, 32.133493994828015},        {9.603450489764624, 31.205313249352219},
         {8.858851588581409, 2.0657439862506815},        {4.8076418393909703, 2.0657439862946823}});
    const double d = 1.0328719931380388;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Left, JoinType::Round, kTol, 3.8787999757578069);
    CHECK(junctionGaps(chains) == 0);
    const DistanceExtremes ex = distanceExtremes(chains, in, d);
    CHECK(ex.under <= distanceAllowance(d, Vec2{}));
    CHECK(ex.over <= distanceAllowance(d, Vec2{}));
}

TEST_CASE("T-0017: a reversal between nearly co-circular arcs is kept wherever it lies (0xd4001274)",
          "[geometry][offset][open][t0017]") {
    // Verifier fam4 coCircNear, Square, Right, at (-1e4, 1e4): two offset arcs on circles 3e-6 mm
    // apart run opposite ways and meet where they cross. Round 1 removed the second as a needle of
    // "rounding" width 2e-9 x |x| = 2e-5 mm at 1e4 mm (losing 5.35 mm of path) but kept it near the
    // origin; the bound no longer depends on where the input lies.
    Contour in;
    in.spans.emplace_back(ArcSpan{
        {-10000, 10000}, 1.8130057389650542, 1.6909617265835539, 4.0236245777151183, ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{-10001.152316980988, 9998.6003016807736}, {-10015.178649135607, 9981.5627730617762}});
    in.spans.emplace_back(
        LineSpan{{-10015.178649135607, 9981.5627730617762}, {-10012.596667411821, 9979.7109217343568}});
    in.spans.emplace_back(
        LineSpan{{-10012.596667411821, 9979.7109217343568}, {-10001.609698146518, 9997.9070491012317}});
    in.spans.emplace_back(ArcSpan{{-10000.000000089174, 10000.00000296611},
                                  2.6403756771195135,
                                  4.0567733419701861,
                                  0.76780841659784071,
                                  ArcDirection::CW});
    const double d = 0.41368496906985969;
    const std::vector<Contour> far =
        offsetOpenContour(in, d, OffsetSide::Right, JoinType::Square, kTol, 3.1518687530154437);
    const std::vector<Contour> near = offsetOpenContour(translated(in, Vec2{1e4, -1e4}), d, OffsetSide::Right,
                                                        JoinType::Square, kTol, 3.1518687530154437);
    REQUIRE(far.size() == near.size());
    double farLength = 0.0;
    double nearLength = 0.0;
    for (std::size_t i = 0; i < far.size(); ++i) {
        CAPTURE(i);
        CHECK(far[i].spans.size() == near[i].spans.size());
        for (const Span& s : far[i].spans) {
            farLength += length(s);
        }
        for (const Span& s : near[i].spans) {
            nearLength += length(s);
        }
    }
    CHECK(std::fabs(farLength - nearLength) <= 1e-6);
    CHECK(farLength > 55.0);
}

TEST_CASE("T-0017: a lens slot far from the origin closes exactly where its arcs cross (0xe400137f)",
          "[geometry][offset][open][t0017]") {
    // Verifier fam4 lensSlot, Round, Left: the closed chain's closing junction is the crossing of
    // arcs of radius 39785 and 1.9 mm (checked to 30 digits: span 4 stays inside the large circle
    // and span 0 outside the small one but at the junction). The verifier's own crossing
    // (h^2 = r^2 - a^2, ~4e-4 relative error at r = 39785) read it as a self-crossing.
    Contour in;
    in.spans.emplace_back(ArcSpan{{-6308.8745429246874, 31581.569216038675},
                                  39785.30676902362,
                                  -1.570920458202786,
                                  -1.5706721953870071,
                                  ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{-6303.9359367948146, -8203.7372464668806}, {-6302.9359367948146, -8193.37889961566}});
    in.spans.emplace_back(
        LineSpan{{-6302.9359367948146, -8193.37889961566}, {-6305.2879140935202, -8192.37889961566}});
    in.spans.emplace_back(
        LineSpan{{-6305.2879140935202, -8192.37889961566}, {-6304.1211345246902, -8203.558166433495}});
    in.spans.emplace_back(
        LineSpan{{-6304.1211345246902, -8203.558166433495}, {-6313.6279513246845, -8203.558166433495}});
    const double d = 0.089633408917393007;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Left, JoinType::Round, kTol, 2.0);
    JunctionStats js;
    checkJunctions(chains, js);
    CHECK(js.backward == 0);
    CHECK(js.notIdentical == 0);
    CHECK(js.maxGap <= 1e-12 * 39785.4);
    // No self-crossing check here: intersect()'s circle-circle crossing has the same
    // cancellation for r = 39785 against 0.09 and reports a spurious second crossing 1e-5 mm from
    // the junction (the true one is 0.011 mm away); a fix belongs outside T-0017.
    const DistanceExtremes ex = distanceExtremes(chains, in, d);
    CHECK(ex.under <= distanceAllowance(d, Vec2{-6304, -8204}));
    CHECK(ex.over <= distanceAllowance(d, Vec2{-6304, -8204}));
}

TEST_CASE(
    "T-0017: nearly collinear chamfer copies are joined by moving a line end, not a zero-length span "
    "(0xe2000055)",
    "[geometry][offset][open][t0017]") {
    // Verifier fam2 zigzag-nearcollinear, Square, Left, d = 7.69: two chamfer copies 4.3e-14 mm
    // apart at ~9 mm. With only 16-ulp gaps taken as rounding, the junction was bridged by a
    // 4.3e-14 mm line; a line end now moves across gaps up to 1e-9 x |x| where the tilted line
    // passes the distance checks.
    Contour in = polyline({{0, 0},
                           {0.34246809807709416, 1.3292456870100586},
                           {0.68493619615418833, -9.8720926466949901e-15},
                           {1.0274042942312824, 1.3292456870100819},
                           {1.3698723923083767, 9.6758026023871068e-15}});
    const double d = 7.6917751206911928;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Left, JoinType::Square, kTol, 2.0);
    for (const Contour& c : chains) {
        for (const Span& s : c.spans) {
            CHECK(length(s) > 1e-9);
        }
    }
    requireCleanChains(chains, 1e-6);
    CHECK(distanceExtremes(chains, in, d).under <= distanceAllowance(d, Vec2{}));
}

TEST_CASE("T-0017: a loop of sub-tolerance pieces does not take the starts its chain needs (0xe40003fb)",
          "[geometry][offset][open][t0017]") {
    // Verifier fam4 shortWalkFar, Round, Left, d = 0.0213 (shrunk from 137 to 6 lines). The global
    // stitching linked two sub-tolerance pieces into a loop first (gap 3.2e-4 mm), so the chain
    // through them could not continue (gaps 9.9e-4 and 7.3e-4 mm) and fell apart in two, losing
    // 3.6e-3 mm of offset. A link that would close a loop no longer than 4 x tol is skipped.
    Contour in = polyline({{0.12516456875199689, -0.031632953553321519},
                           {0.1279511103475624, -0.032220275701318545},
                           {0.13064769290544898, -0.03313581045098489},
                           {0.13347013052959394, -0.033514765893161984},
                           {0.13627188406074953, -0.034024607554858859},
                           {0.13911239817775328, -0.033821528948287299},
                           {0.14195821566057526, -0.033716247307868007}});
    const double d = 0.021263305223598185;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Left, JoinType::Round, kTol, 1.2917229761801405);
    CHECK(chains.size() == 1);
    requireCleanChains(chains, 1e-6);
    const DistanceExtremes ex = distanceExtremes(chains, in, d);
    CHECK(ex.under <= distanceAllowance(d, Vec2{}));
    CHECK(ex.over <= distanceAllowance(d, Vec2{}));
}

TEST_CASE("T-0017: a needle tip between nearly co-circular arcs is bridged, not bent (0xe40016aa)",
          "[geometry][offset][open][t0017]") {
    // Verifier fam4 coCircNear (noisy), Sharp, Right: two offset arcs on circles 1.7e-7 mm apart
    // run back along each other with a 1.5e-8 mm gap at the tip. The step repair bent the last
    // tolerance of one arc into a 1e-3 mm line onto the other (a spike); at a reversal the gap is
    // now bridged across the tip.
    Contour in;
    in.spans.emplace_back(
        ArcSpan{{0, 0}, 2.2033079169640071, -2.2267861109534142, -1.3064720620699988, ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{0.57562975164911068, -2.1267854066578167}, {7.422449049853542, -27.423801976296062}});
    in.spans.emplace_back(
        LineSpan{{7.422449049853542, -27.423801976296062}, {10.123179626699358, -26.545788723983144}});
    in.spans.emplace_back(
        LineSpan{{10.123179626699358, -26.545788723983144}, {1.2243009918830434, -4.5234379448674229}});
    in.spans.emplace_back(ArcSpan{{-3.5187571150240017e-08, 1.6838212700014806e-07},
                                  4.6861930571930994,
                                  -1.3064720620699988,
                                  -2.592518075747611,
                                  ArcDirection::CW});
    const double d = 1.2414424801835482;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Right, JoinType::Sharp, kTol, 2.0);
    for (const Contour& c : chains) {
        for (std::size_t i = 1; i + 1 < c.spans.size(); ++i) {
            if (std::holds_alternative<LineSpan>(c.spans[i]) &&
                std::holds_alternative<ArcSpan>(c.spans[i - 1]) &&
                std::holds_alternative<ArcSpan>(c.spans[i + 1])) {
                CHECK(length(c.spans[i]) <= 1e-6);
            }
        }
    }
    JunctionStats js;
    checkJunctions(chains, js);
    CHECK(js.gaps == 0);
    CHECK(js.notIdentical == 0);
    CHECK(distanceExtremes(chains, in, d).under <= distanceAllowance(d, Vec2{}));
}

TEST_CASE(
    "T-0017: Round arcs about vertices 2d + g apart meet exactly at their cusp for every g (round 3 B-1)",
    "[geometry][offset][open][t0017]") {
    // The input of the 0xe300070e test with its upper row raised by g: the Round arcs about
    // vertices 2d + g apart only nearly touch. Snapped to a point on neither circle, the arcs
    // kept a gap of about g where g lay above exactJunction's 1e-10 x scale limit and below the
    // bridging threshold (1.1e-10..8.3e-10 mm for g = 8e-11..8e-10; main the same). Such a snap is
    // now refused and the junction bridged. Verifier fam5 0xf5000172, 0xf51007a0, 0xf5100965,
    // 0xf5100e98 hit the same.
    const double d = 1.0328719931380388;
    const std::vector<Vec2> base{
        {6.6283223938981575, -1.2779953526139643e-11},  {5.023667459767422, 2.1623209668586711e-11},
        {6.9756096886500769, 34.51051481312367},        {6.3189901091486398, 34.312480314617368},
        {4.6667279880904458, 2.0657439862650513},       {0.69533962297162577, 2.0657439862566656},
        {1.9562893692302896, 33.119825834028326},       {7.04511858784476, 33.809684447840162},
        {6.6283223939411231, 2.0657439862576599},       {5.0236674597854529, 2.0657439862438181},
        {5.4657050614391105, 36.723239966815832},       {4.5703929732694029, 36.057839455949605},
        {4.8076418394176894, 3.1120034057046923e-12},   {8.8588515885397729, -8.7094539924375882e-13},
        {9.7907278097679189, -35.00186354855979},       {1.9705389447012767, -34.43692474497638},
        {0.69533962299185814, -2.6581188530012263e-11}, {4.6667279880936041, -3.0346127572489825e-11},
        {3.1154143986946496, 34.81196163001529},        {4.5941198790408642, 35.289788649021553},
        {5.0236674597518789, 1.9744273286251705e-11},   {6.6283223939582836, -1.3296097279548633e-11},
        {6.980860505868165, 32.133493994828015},        {9.603450489764624, 31.205313249352219},
        {8.858851588581409, 2.0657439862506815},        {4.8076418393909703, 2.0657439862946823}};
    for (const double g : {0.0, 1e-11, 5e-11, 8e-11, 1e-10, 2e-10, 3e-10, 5e-10, 8e-10, 1e-9, 3e-9, 1e-8}) {
        CAPTURE(g);
        std::vector<Vec2> pts;
        for (const Vec2 p : base) {
            pts.push_back(std::fabs(p.y - 2.0 * d) < 1e-6 ? Vec2{p.x, 2.0 * d + g} : p);
        }
        const Contour in = polyline(pts);
        const std::vector<Contour> chains =
            offsetOpenContour(in, d, OffsetSide::Left, JoinType::Round, kTol, 3.8787999757578069);
        JunctionStats js;
        checkJunctions(chains, js);
        CHECK(js.gaps == 0);
        CHECK(js.notIdentical == 0);
        CHECK(js.backward == 0);
        const DistanceExtremes ex = distanceExtremes(chains, in, d);
        CHECK(ex.under <= distanceAllowance(d, Vec2{}));
        CHECK(ex.over <= distanceAllowance(d, Vec2{}));
    }
}

TEST_CASE("T-0017: a loop's closing junction is not extended along another chain's piece (round 3 B-2)",
          "[geometry][offset][open][t0017]") {
    // Verifier fam5 tinyLoops 0xf500163e (shrunk from 232 to 95 lines), Sharp, Left, d = 0.0113,
    // ~1e4 mm from the origin and at the origin. The closing junction of a three-span loop was
    // snapped by extending a line 1.49e-3 mm along the first span of the next chain: that stretch
    // was traversed twice and the corner poked 1.08e-3 mm past the offset.
    Contour in = polyline(
        {{0.026512328066050458, -9999.9619524956488},    {0.02899641530262561, -9999.9666107279518},
         {0.027960912837961833, -9999.9717873631744},    {0.032472380100241782, -9999.9745289832681},
         {0.030559571925013341, -9999.9794494489433},    {0.02713266075752619, -9999.9834651801903},
         {0.02440598743829827, -9999.9879856967764},     {0.019133368724036259, -9999.9882489637184},
         {0.018972990071086233, -9999.9829722131781},    {0.019618131674756373, -9999.9777325939249},
         {0.015602646045186217, -9999.9743053949605},    {0.010411278655083719, -9999.9752643121683},
         {0.0052272954736983599, -9999.9742662460558},   {2.6047428970474122e-05, -9999.9751700360575},
         {-0.0039929746585559145, -9999.9785930871876},  {-0.0084078097432390801, -9999.9756984285268},
         {-0.011438896082712265, -9999.9800207349945},   {-0.0071108566577601879, -9999.983043629687},
         {-0.0021152764329345208, -9999.9847506726874},  {0.0030536053300058118, -9999.9836771369828},
         {0.0056095821256054931, -9999.9882963144209},   {0.00099030408451915124, -9999.9908521093985},
         {0.0020871403460188884, -9999.9960160969858},   {-0.00065839329528937646, -10000.000525183659},
         {-0.0058686170814738411, -9999.9996746628258},  {-0.010630474323550104, -10000.001953814684},
         {-0.0099344894512666086, -10000.007186923},     {-0.0058902675020763157, -10000.010580164284},
         {-0.00082995675489461445, -10000.012084515551}, {0.0018238400830776157, -10000.016648196075},
         {-0.0016175510142198814, -10000.020651525303},  {-0.00040149940260952345, -10000.025788745959},
         {-0.00046510737850854534, -10000.031067549953}, {0.0036607713931153421, -10000.034361019545},
         {0.0011017666910006357, -10000.038978520224},   {-0.0025215229715790837, -10000.042817998003},
         {-0.0040643314726566135, -10000.047866716604},  {-0.0085982300385175382, -10000.050571080003},
         {-0.0058334952958253878, -10000.055068419148},  {-0.0011759582636346536, -10000.057553809738},
         {0.0029314683307220162, -10000.06087026332},    {0.0074862864326199234, -10000.063539242459},
         {0.012712238836934894, -10000.064287065337},    {0.017737017817691637, -10000.062667987271},
         {0.019952279909052215, -10000.057876074141},    {0.018860790647690501, -10000.052710953736},
         {0.023862227952011119, -10000.05102114828},     {0.026226018545085297, -10000.046300733931},
         {0.028325441695787525, -10000.041456950285},    {0.027598244133901197, -10000.036228087951},
         {0.0223352155894339, -10000.036640820466},      {0.017058184924060006, -10000.036791701035},
         {0.016822771443554322, -10000.031517765306},    {0.015128099181980378, -10000.026517974959},
         {0.010785749886525253, -10000.023515672478},    {0.0090147385009240136, -10000.018542410344},
         {0.010800609275308778, -10000.013574464776},    {0.015113856489577939, -10000.010530500904},
         {0.019911870202737846, -10000.012732518613},    {0.020947623628423197, -10000.01790910363},
         {0.025715103774642763, -10000.020176470085},    {0.024662332056383355, -10000.025349620895},
         {0.02974724977742892, -10000.026768574509},     {0.028240033956152238, -10000.031828032787},
         {0.033289129234728466, -10000.033369608103},    {0.036744124988044476, -10000.037361202012},
         {0.03439649115342041, -10000.042089672481},     {0.032702207259866771, -10000.047089594447},
         {0.028938301609804134, -10000.050791328074},    {0.024053990653956678, -10000.048787999825},
         {0.020963071631091092, -10000.044508275863},    {0.015706798371564652, -10000.044016941954},
         {0.015824364970797106, -10000.038739064003},    {0.01439623397157129, -10000.033656716174},
         {0.015316142380849529, -10000.028458294673},    {0.018493314670096989, -10000.02424220717},
         {0.016312337567199867, -10000.019434592745},    {0.015539278173608602, -10000.014212313858},
         {0.01865521164039052, -10000.009950767633},     {0.020244597560789588, -10000.004916517939},
         {0.025261553656194723, -10000.003273359662},    {0.030202777101412566, -10000.001414831089},
         {0.033929901489222869, -9999.9976760664013},    {0.037871826327414482, -9999.9941645074668},
         {0.043083151493092608, -9999.9933207612921},    {0.045891312727751259, -9999.9888504091723},
         {0.050324913040292782, -9999.9917162434067},    {0.055466614137498871, -9999.9905192774859},
         {0.056031291709599695, -9999.9852703769502},    {0.052382980611753227, -9999.9814546668294},
         {0.054937119169813378, -9999.9768344726981},    {0.058174606036302576, -9999.972664521094},
         {0.059092150144910136, -9999.9674656817624},    {0.054149107194191942, -9999.9656119978699},
         {0.052095370556358184, -9999.9607486673831},    {0.0538844939328098, -9999.9557818922603}});
    const double d = 0.011310362902320363;
    for (const Vec2 shift : {Vec2{}, Vec2{0.0, 1e4}}) {
        CAPTURE(shift.y);
        const Contour moved = translated(in, shift);
        const std::vector<Contour> chains =
            offsetOpenContour(moved, d, OffsetSide::Left, JoinType::Sharp, kTol, 3.5060801369523507);
        std::string why;
        CHECK(carvetto::geometry::testing::countOverlaps(chains, 1e-6, &why) == 0);
        CAPTURE(why);
        JunctionStats js;
        checkJunctions(chains, js);
        CHECK(js.gaps == 0);
        CHECK(js.backward == 0);
        CHECK(distanceExtremes(chains, moved, d).under <= distanceAllowance(d, Vec2{0.0, 1e4}));
        for (const Contour& c : chains) {
            if (!c.closed && c.spans.size() > 1) {
                CHECK(length(c.spans.front()) > kTol.distance);
                CHECK(length(c.spans.back()) > kTol.distance);
            }
        }
    }
}
