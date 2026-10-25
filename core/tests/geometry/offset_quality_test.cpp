// T-0006: open-offset quality for toolpaths — cost at a large distance, exact mitres and chamfers
// at arc corners, snapped junctions.

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <optional>
#include <string>
#include <tuple>
#include <variant>
#include <vector>

#include "carvetto/geometry/offset.hpp"
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

double cpuSeconds(std::clock_t since) {
    return static_cast<double>(std::clock() - since) / static_cast<double>(CLOCKS_PER_SEC);
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

// One junction a -> b. "Backward": b's start lies behind a's end along a's direction of travel,
// or b sets off straight back along a (a retrace that would be a backward G-code move).
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
    bool back = (s - e).dot(tangentAt(a, 1.0)) < -1e-12;
    if (tangentAt(a, 1.0).dot(tangentAt(b, 0.0)) < -0.9999) {
        const double probe = 0.5 * std::min(length(a), length(b));
        const Vec2 q = pointAt(b, std::min(1.0, probe / length(b)));
        back = back || distanceToSpan(q, a) < 1e-9;
    }
    if (back) {
        ++st.backward;
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
                    const std::clock_t t0 = std::clock();
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
    // Debug too; the repetitions keep the shorter run long enough for std::clock.
    SplitMix rng{7u};
    std::vector<Vec2> pts{{0.0, 0.0}};
    for (int i = 0; i < 2000; ++i) {
        pts.push_back(pts.back() + Vec2::fromAngle(rng.uniform(-kPi, kPi), 0.5));
    }
    const Contour longWalk = polyline(pts);
    const Contour shortWalk = polyline(std::vector<Vec2>(pts.begin(), pts.begin() + 1001));
    auto cost = [](const Contour& in, int reps) {
        const std::clock_t t0 = std::clock();
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
