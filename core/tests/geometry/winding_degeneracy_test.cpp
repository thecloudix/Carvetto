// T-0143: winding number with spans, vertices and bridges lying on the +X test ray.
// The product casts a horizontal ray; the oracle here sums the angle the contour subtends at the
// query point (long double, curves sampled finely), so it never sees the ray's degeneracies.
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <variant>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "test_rng.hpp"

using namespace carvetto::geometry;
using carvetto::test::SplitMix;

namespace {

const Tolerance tol{};

// ------------------------------------------------------------------------------- oracle

void sampleSpan(const Span& s, std::vector<Vec2>& pts) {
    std::size_t n = 1;
    if (const auto* arc = std::get_if<ArcSpan>(&s)) {
        // Sagitta r * dtheta^2 / 8 below 1e-4 mm, well inside the boundary margin of checkPoint.
        const double dTheta = std::sqrt(8e-4 / arc->radius);
        n = static_cast<std::size_t>(std::ceil(std::fabs(arc->sweepAngle()) / dTheta)) + 8;
    } else if (std::holds_alternative<BezierSpan>(s)) {
        n = 1000;  // chords of the 9 mm grid Beziers stay below 0.02 mm
    }
    for (std::size_t i = 0; i <= n; ++i) {
        pts.push_back(pointAt(s, static_cast<double>(i) / static_cast<double>(n)));
    }
}

// The contour sampled finely and closed implicitly from its end back to its start.
std::vector<Vec2> oracleSamples(const Contour& c) {
    std::vector<Vec2> pts;
    for (const Span& s : c.spans) {
        sampleSpan(s, pts);
    }
    if (!pts.empty()) {
        pts.push_back(pts.front());
    }
    return pts;
}

// Winding number by angle summation over the samples.
int oracleWinding(const std::vector<Vec2>& pts, Vec2 p) {
    long double sum = 0.0L;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        const long double ax = static_cast<long double>(pts[i - 1].x) - p.x;
        const long double ay = static_cast<long double>(pts[i - 1].y) - p.y;
        const long double bx = static_cast<long double>(pts[i].x) - p.x;
        const long double by = static_cast<long double>(pts[i].y) - p.y;
        // Each term is in (-pi, pi]; cross and dot are formed in long double, so the angle is
        // exact to rounding even for nearly collinear neighbours.
        sum += std::atan2(static_cast<double>(ax * by - ay * bx), static_cast<double>(ax * bx + ay * by));
    }
    const long double turns = sum / (2.0L * 3.14159265358979323846264338327950288L);
    return static_cast<int>(std::lround(static_cast<double>(turns)));
}

bool hasBezier(const Contour& c) {
    for (const Span& s : c.spans) {
        if (std::holds_alternative<BezierSpan>(s)) {
            return true;
        }
    }
    return false;
}

std::string describe(const Contour& c) {
    std::string out = c.closed ? " closed:" : " open:";
    for (const Span& s : c.spans) {
        const Vec2 a = startPoint(s);
        const Vec2 b = endPoint(s);
        const char* k = std::holds_alternative<LineSpan>(s) ? " L" : std::holds_alternative<ArcSpan>(s) ? " A" : " B";
        out += std::string(k) + "(" + std::to_string(a.x) + "," + std::to_string(a.y) + ")->(" + std::to_string(b.x) +
               "," + std::to_string(b.y) + ")";
        if (const auto* z = std::get_if<BezierSpan>(&s)) {
            out += "[" + std::to_string(z->p1.x) + "," + std::to_string(z->p1.y) + " " + std::to_string(z->p2.x) + "," +
                   std::to_string(z->p2.y) + "]";
        }
        if (const auto* arc = std::get_if<ArcSpan>(&s)) {
            out += arc->direction == ArcDirection::CCW ? "ccw" : "cw";
        }
    }
    return out;
}

struct Stats {
    int checked{0};
    int onRay{0};  // query points whose ray passes exactly through a vertex of the contour
};

// Compares windingNumber and locatePoint with the oracle at p, unless p is too close to the
// boundary for the two to be comparable (Beziers are polygonised at tol.chordal by the product).
void checkPoint(const Contour& c, const std::vector<Vec2>& samples, Vec2 p, Stats& st, const std::string& what) {
    const double margin = hasBezier(c) ? 3.0 * tol.chordal : 10.0 * tol.distance;
    // The implicit closing segment (open contours, gaps) is part of the boundary for both.
    const ClosestPoint gap = closestPoint(LineSpan{c.endPoint(), c.startPoint()}, p);
    if (distanceTo(c, p) <= margin || distance(gap.point, p) <= margin) {
        return;
    }
    const int want = oracleWinding(samples, p);
    const int got = windingNumber(c, p, tol);
    INFO(what << " p=(" << p.x << ", " << p.y << ") spans=" << c.spans.size() << describe(c));
    REQUIRE(got == want);
    const PointLocation nonZero = locatePoint(c, p, FillRule::NonZero, tol);
    const PointLocation evenOdd = locatePoint(c, p, FillRule::EvenOdd, tol);
    REQUIRE(nonZero == (want != 0 ? PointLocation::Inside : PointLocation::Outside));
    REQUIRE(evenOdd == (want % 2 != 0 ? PointLocation::Inside : PointLocation::Outside));
    ++st.checked;
    for (const Span& s : c.spans) {
        if (startPoint(s).y == p.y || endPoint(s).y == p.y) {
            ++st.onRay;
            break;
        }
    }
}

// --------------------------------------------------------------------------- generators

enum class Kind { Arcs, Lines, Beziers };

// A full circle about `c` of radius r starting at angle a, travelled CCW or CW, as one arc,
// a 12-gon or four cubic Beziers.
void appendCircle(Contour& out, Vec2 c, double r, double a, ArcDirection dir, Kind kind) {
    const double sgn = dir == ArcDirection::CCW ? 1.0 : -1.0;
    switch (kind) {
        case Kind::Arcs:
            out.append(ArcSpan{c, r, a, a + 2.0 * kPi, dir});
            break;
        case Kind::Lines: {
            constexpr int n = 12;
            for (int i = 0; i < n; ++i) {
                const Vec2 p0 = c + Vec2::fromAngle(a + sgn * 2.0 * kPi * i / n, r);
                const Vec2 p1 = c + Vec2::fromAngle(a + sgn * 2.0 * kPi * (i + 1) / n, r);
                out.append(LineSpan{p0, p1});
            }
            break;
        }
        case Kind::Beziers: {
            const double k = 0.5522847498307936 * r;
            for (int i = 0; i < 4; ++i) {
                const double a0 = a + sgn * 0.5 * kPi * i;
                const double a1 = a + sgn * 0.5 * kPi * (i + 1);
                const Vec2 p0 = c + Vec2::fromAngle(a0, r);
                const Vec2 p3 = c + Vec2::fromAngle(a1, r);
                const Vec2 p1 = p0 + Vec2::fromAngle(a0 + sgn * 0.5 * kPi, k);
                const Vec2 p2 = p3 + Vec2::fromAngle(a1 - sgn * 0.5 * kPi, k);
                out.append(BezierSpan{p0, p1, p2, p3});
            }
            break;
        }
    }
}

// Bridge from a to b, sometimes split into collinear pieces at random points.
void appendBridge(Contour& out, Vec2 a, Vec2 b, SplitMix& rng) {
    const int pieces = 1 + static_cast<int>(rng.next() % 3);
    Vec2 from = a;
    for (int i = 1; i < pieces; ++i) {
        const Vec2 to = lerp(a, b, (static_cast<double>(i - 1) + rng.uniform(0.2, 0.8)) / pieces);
        out.append(LineSpan{from, to});
        from = to;
    }
    out.append(LineSpan{from, b});
}

Kind randomKind(SplitMix& rng) {
    return static_cast<Kind>(rng.next() % 3);
}

// A ring drawn as one loop: outer circle, bridge in, inner circle reversed, bridge out.
Contour ringLoop(Vec2 c, double rOut, double rIn, double a, SplitMix& rng) {
    Contour ring;
    ring.closed = true;
    const Vec2 po = c + Vec2::fromAngle(a, rOut);
    const Vec2 pi = c + Vec2::fromAngle(a, rIn);
    appendCircle(ring, c, rOut, a, ArcDirection::CCW, randomKind(rng));
    // Bridges start either at the exact point or at the circle's computed end (they differ by
    // rounding for arcs and Beziers, leaving a junction gap that may straddle the ray).
    appendBridge(ring, rng.next() % 2 == 0 ? po : endPoint(ring.spans.back()), pi, rng);
    appendCircle(ring, c, rIn, a, ArcDirection::CW, randomKind(rng));
    appendBridge(ring, rng.next() % 2 == 0 ? pi : endPoint(ring.spans.back()), po, rng);
    if (rng.next() % 2 == 0) {
        ring.reverse();
    }
    return ring;
}

double bridgeAngle(SplitMix& rng) {
    static const double special[] = {0.0,         kPi / 2.0, kPi,          1.5 * kPi, 2.0 * kPi, -kPi / 2.0,
                                     kPi / 4.0,   kPi / 6.0, 1e-12,        -1e-12,    kPi + 1e-12,
                                     kPi - 1e-12, 1e-300,    2.0 * kPi - 1e-15};
    const std::uint64_t pick = rng.next() % 24;
    if (pick < sizeof(special) / sizeof(special[0])) {
        return special[pick];
    }
    if (pick < 20) {
        return static_cast<double>(rng.next() % 24) * kPi / 12.0;  // every 15 degrees
    }
    return rng.uniform(-kPi, 3.0 * kPi);
}

// Query points on and next to the horizontal lines through every vertex of c.
std::vector<Vec2> queryPoints(const Contour& c, Vec2 centre, double rOut, SplitMix& rng) {
    std::vector<double> ys{centre.y};
    for (const Span& s : c.spans) {
        ys.push_back(startPoint(s).y);
        ys.push_back(endPoint(s).y);
    }
    std::vector<Vec2> out{centre};
    for (std::size_t i = 0; i < ys.size(); i += 1 + static_cast<std::size_t>(rng.next() % 3)) {
        const double y = ys[i];
        for (const double dy : {0.0, 0.0, 1e-13, -1e-13, 1e-7, -1e-7}) {
            const double x = centre.x + rng.uniform(-1.3, 1.3) * rOut;
            out.push_back({x, y + dy});
        }
        out.push_back({centre.x, y});
    }
    return out;
}

}  // namespace

TEST_CASE("T-0143: the centre of a ring drawn as one loop with bridges on the ray is outside",
          "[geometry][contour_ops][winding]") {
    // Verifier repro (harness t0150v1/ring.cpp).
    Contour ring;
    ring.closed = true;
    ring.append(ArcSpan{{0.0, 0.0}, 10.0, 0.0, 2.0 * kPi, ArcDirection::CCW});
    ring.append(LineSpan{{10.0, 0.0}, {5.0, 0.0}});
    ring.append(ArcSpan{{0.0, 0.0}, 5.0, 0.0, 2.0 * kPi, ArcDirection::CW});
    ring.append(LineSpan{{5.0, 0.0}, {10.0, 0.0}});
    for (const Vec2 p : {Vec2{0.0, 0.0}, Vec2{-3.0, 0.0}, Vec2{4.0, 0.0}, Vec2{0.1, 0.2}}) {
        CHECK(windingNumber(ring, p, tol) == 0);
        CHECK(locatePoint(ring, p, FillRule::EvenOdd, tol) == PointLocation::Outside);
        CHECK(locatePoint(ring, p, FillRule::NonZero, tol) == PointLocation::Outside);
    }
    for (const Vec2 p : {Vec2{7.0, 0.5}, Vec2{-7.0, 0.0}, Vec2{7.0, 3.0}, Vec2{0.0, -7.5}}) {
        CHECK(windingNumber(ring, p, tol) == 1);
        CHECK(locatePoint(ring, p, FillRule::EvenOdd, tol) == PointLocation::Inside);
    }
    CHECK(windingNumber(ring, {12.0, 0.0}, tol) == 0);
}

TEST_CASE("T-0143: rings drawn as one loop match an angle-sum oracle for bridges at every angle",
          "[geometry][contour_ops][winding][property]") {
    SplitMix rng{0x7143A11CEULL};
    Stats st;
    for (int iter = 0; iter < 120; ++iter) {
        const bool integral = rng.next() % 2 == 0;
        const Vec2 centre = integral ? Vec2{static_cast<double>(rng.next() % 21) - 10.0,
                                            static_cast<double>(rng.next() % 21) - 10.0}
                                     : Vec2{rng.uniform(-50.0, 50.0), rng.uniform(-50.0, 50.0)};
        const double rOut = integral ? static_cast<double>(4 + rng.next() % 12) : rng.uniform(2.0, 20.0);
        const double rIn = rOut * rng.uniform(0.2, 0.8);
        const double a = bridgeAngle(rng);
        const Contour ring = ringLoop(centre, rOut, rIn, a, rng);
        const std::vector<Vec2> samples = oracleSamples(ring);
        for (const Vec2 p : queryPoints(ring, centre, rOut, rng)) {
            checkPoint(ring, samples, p, st, "ring a=" + std::to_string(a));
        }
    }
    CHECK(st.checked > 1000);
    CHECK(st.onRay > 300);
}

TEST_CASE("T-0143: grid contours of lines, arcs and Beziers match an angle-sum oracle on vertex rays",
          "[geometry][contour_ops][winding][property]") {
    // Vertices on an integer grid and query points on the integer and half-integer grid: rays run
    // through vertices, along horizontal lines, along coincident and retraced segments, and through
    // the end points of half-circle arcs (whose computed ends differ from the grid by rounding).
    SplitMix rng{0x0143B0B5ULL};
    Stats st;
    const auto gridPoint = [&rng]() {
        return Vec2{static_cast<double>(rng.next() % 9), static_cast<double>(rng.next() % 9)};
    };
    for (int iter = 0; iter < 120; ++iter) {
        Contour c;
        c.closed = rng.next() % 5 != 0;
        const int n = 3 + static_cast<int>(rng.next() % 8);
        const Vec2 first = gridPoint();
        Vec2 at = first;
        for (int i = 0; i < n; ++i) {
            Vec2 to = i == n - 1 && c.closed ? first : gridPoint();
            const std::uint64_t pick = rng.next() % 10;
            const bool closing = i == n - 1 && c.closed;
            if (!closing && pick < 3 && !c.spans.empty()) {
                to = startPoint(c.spans.back());  // retrace the previous span's chord
            } else if (!closing && pick == 3) {
                to = Vec2{to.x, at.y};  // horizontal edge
            }
            if (to == at) {
                continue;
            }
            const std::uint64_t kind = rng.next() % 4;
            if (kind == 0) {
                const ArcDirection dir = rng.next() % 2 == 0 ? ArcDirection::CCW : ArcDirection::CW;
                c.append(ArcSpan::fromCentre(lerp(at, to, 0.5), at, to, dir));
            } else if (kind == 1) {
                c.append(BezierSpan{at, gridPoint(), gridPoint(), to});
            } else {
                c.append(LineSpan{at, to});
            }
            at = to;
        }
        if (c.spans.empty()) {
            continue;
        }
        const std::vector<Vec2> samples = oracleSamples(c);
        for (int yi = -1; yi <= 18; ++yi) {
            for (int xi = -1; xi <= 18; ++xi) {
                if ((xi + yi + iter) % 3 != 0) {
                    continue;
                }
                checkPoint(c, samples, Vec2{0.5 * xi, 0.5 * yi}, st, "grid iter " + std::to_string(iter));
            }
        }
    }
    CHECK(st.checked > 3000);
    CHECK(st.onRay > 1000);
}
