// T-0178: BezierSpan::length / bounds terminate with documented accuracy on awkward curves, and
// intersect(Span, Span) on long near-coincident Beziers stays within a CPU bound (ADR-0005 A2).
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cfloat>
#include <cmath>
#include <limits>
#include <vector>

#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/intersect.hpp"
#include "carvetto/geometry/span.hpp"
#include "cpu_time.hpp"

using namespace carvetto::geometry;
using carvetto::test::processCpuSeconds;

namespace {

// High-resolution reference: composite Simpson on |B'(t)| with 400k panels, curve evaluated
// relative to p0 so offsets do not matter for the derivative.
double referenceLength(const BezierSpan& b) {
    const int n = 400000;
    double sum = 0.0;
    for (int i = 0; i <= n; ++i) {
        const double t = static_cast<double>(i) / n;
        const double w = (i == 0 || i == n) ? 1.0 : (i % 2 == 1 ? 4.0 : 2.0);
        sum += w * b.derivativeAt(t).length();
    }
    return sum / (3.0 * n);
}

BezierSpan scaledAt(BezierSpan b, double scale, Vec2 offset) {
    for (Vec2* p : {&b.p0, &b.p1, &b.p2, &b.p3}) {
        *p = *p * scale + offset;
    }
    return b;
}

struct Shape {
    const char* name;
    BezierSpan b;
};

std::vector<Shape> shapes() {
    return {
        {"loop, coincident ends", {{0, 0}, {100, 100}, {-100, 100}, {0, 0}}},
        {"closed control polygon", {{0, 0}, {1, 0}, {1, 1}, {0, 0}}},
        {"cusp", {{0, 0}, {1, 1}, {0, 1}, {1, 0}}},
        {"collinear overshoot", {{0, 0}, {10, 0}, {-5, 0}, {3, 0}}},
        {"zigzag", {{0, 0}, {1, 1}, {0, 0}, {1, 1}}},
        {"p1 == p2 return", {{0, 0}, {1, 1}, {1, 1}, {0, 0}}},
        {"plain S", {{0, 0}, {3, 5}, {6, -5}, {9, 0}}},
    };
}

}  // namespace

TEST_CASE("Bezier length terminates and matches a high-resolution reference at every scale", "[bezier][length][geometry]") {
    for (const double scale : {1e-9, 1e-3, 1.0, 1e3, 1e9}) {
        for (const Shape& s : shapes()) {
            CAPTURE(s.name, scale);
            const BezierSpan b = scaledAt(s.b, scale, {0.0, 0.0});
            const double t0 = processCpuSeconds();
            const double len = b.length();
            REQUIRE(processCpuSeconds() - t0 < 1.0);
            const double ref = referenceLength(b);
            REQUIRE(std::fabs(len - ref) <= 1e-6 * ref);
        }
    }
}

TEST_CASE("Bezier length far from the origin: no hang, error within the coordinate rounding", "[bezier][length][geometry]") {
    // Before T-0178 the subdivision never met its halving tolerance below the rounding noise of
    // the coordinates and ran 5-16 s (offset 1e6..3e6, size 1; 2^40 pieces in the worst case).
    for (const double offset : {1e5, 1e6, 3e6, 1e7, 1e8, 1e9}) {
        for (const double scale : {1e-2, 1.0, 1e3}) {
            for (const Shape& s : shapes()) {
                CAPTURE(s.name, offset, scale);
                const BezierSpan b = scaledAt(s.b, scale, {offset, -offset});
                const double t0 = processCpuSeconds();
                const double len = b.length();
                REQUIRE(processCpuSeconds() - t0 < 1.0);
                // Reference on the exact (translation-free) shape: the input is rounded to the
                // grid at `offset`, which moves the length by ~ulp(offset) * a few.
                const double ref = referenceLength(b);
                const double ulp = offset * 2.220446049250313e-16;
                REQUIRE(std::fabs(len - ref) <= 1e-6 * ref + 64.0 * ulp);
            }
        }
    }
}

TEST_CASE("Bezier length of degenerate and extreme curves", "[bezier][length][geometry]") {
    REQUIRE(BezierSpan{{5, 5}, {5, 5}, {5, 5}, {5, 5}}.length() == 0.0);
    REQUIRE(BezierSpan{{1e300, 1e300}, {1e300, 1e300}, {1e300, 1e300}, {1e300, 1e300}}.length() == 0.0);
    // Control values whose squares overflow: scaled internally, the length is still right.
    const BezierSpan huge{{0, 0}, {1e200, 0}, {2e200, 0}, {3e200, 0}};
    REQUIRE(std::fabs(huge.length() - 3e200) <= 1e-9 * 3e200);
    const BezierSpan loop{{0, 0}, {1e300, 1e300}, {-1e300, 1e300}, {0, 0}};
    const double t0 = processCpuSeconds();
    const double len = loop.length();
    REQUIRE(processCpuSeconds() - t0 < 1.0);
    REQUIRE(std::fabs(len - 2.036693895e300) <= 1e-6 * 2.04e300);
    // Beyond the double range the length is +inf, not a hang.
    const BezierSpan over{{0, 0}, {1.7e308, 0}, {-1.7e308, 0}, {1.7e308, 0}};
    REQUIRE(std::isinf(over.length()));
}

TEST_CASE("Bezier length hard work bound: a tiny tolerance still terminates", "[bezier][length][geometry]") {
    const BezierSpan b{{0, 0}, {100, 100}, {-100, 100}, {0, 0}};
    const double t0 = processCpuSeconds();
    const double len = b.length(1e-300);
    REQUIRE(processCpuSeconds() - t0 < 3.0);
    REQUIRE(std::fabs(len - referenceLength(b)) <= 1e-6 * len);
}

TEST_CASE("Bezier bounds survive control points near the double range", "[bezier][bounds][geometry]") {
    for (const double s : {1e150, 1e154, 1e155, 1e160, 1e200, 1e300}) {
        CAPTURE(s);
        const BezierSpan b{{0, 0}, {s, s}, {-s, s}, {10, 10}};
        const Box2 box = b.bounds();
        REQUIRE(std::isfinite(box.min.x));
        REQUIRE(std::isfinite(box.max.y));
        for (int i = 0; i <= 1000; ++i) {
            const Vec2 p = b.pointAt(static_cast<double>(i) / 1000.0);
            REQUIRE(p.x >= box.min.x);
            REQUIRE(p.x <= box.max.x);
            REQUIRE(p.y >= box.min.y);
            REQUIRE(p.y <= box.max.y);
        }
        // The extreme in y is real: the box must reach it (not collapse to the end points).
        REQUIRE(box.max.y >= 0.5 * s);
        REQUIRE(box.max.x >= 0.1 * s);
        REQUIRE(box.min.x <= -0.1 * s);
        // Intersecting with a line that crosses the curve's body finishes quickly.
        const LineSpan line{{-s, 0.5 * s}, {s, 0.5 * s}};
        const double t0 = processCpuSeconds();
        const auto r = intersect(Span{line}, Span{b}, Tolerance::defaults());
        REQUIRE(processCpuSeconds() - t0 < 10.0);
        (void)r;
    }
}

TEST_CASE("Bezier bounds unchanged below the scaling threshold", "[bezier][bounds][geometry]") {
    // x(t) of this curve peaks at t = 1/2 with value 3/4 * 1e90... same curve scaled by 1e90 and
    // by 1e110 (above the 1e100 internal scaling) must give proportional boxes.
    const BezierSpan base{{0, 0}, {4, 6}, {-4, 6}, {1, 0}};
    const Box2 ref = base.bounds();
    for (const double k : {1e90, 1e110, 1e140}) {
        const BezierSpan b{base.p0 * k, base.p1 * k, base.p2 * k, base.p3 * k};
        const Box2 box = b.bounds();
        REQUIRE(std::fabs(box.max.y - ref.max.y * k) <= 1e-12 * ref.max.y * k);
        REQUIRE(std::fabs(box.min.x - ref.min.x * k) <= 1e-12 * std::fabs(ref.min.x) * k);
        REQUIRE(std::fabs(box.max.x - ref.max.x * k) <= 1e-12 * ref.max.x * k);
    }
}

TEST_CASE("Long near-coincident Bezier loops intersect within a CPU bound", "[bezier][intersect][geometry]") {
    // T-0208 repro: two identical (or 1e-4 shifted) loops of extent 1e6 at the default tolerance
    // took 5 s (Release) / > 15 s (Debug): merging ~1e5 raw hits was quadratic and the
    // settling trail was a sorted vector with O(n) insertions.
    const double e = 1e6;
    const BezierSpan a{{0, 0}, {e, e}, {-e, e}, {0, 0}};
    const BezierSpan shifted{{1e-4, 0}, {e, e}, {-e, e}, {0, 1e-4}};
    const double t0 = processCpuSeconds();
    const auto same = intersect(Span{a}, Span{a}, Tolerance::defaults());
    const auto shiftedHits = intersect(Span{a}, Span{shifted}, Tolerance::defaults());
    const double used = processCpuSeconds() - t0;
    REQUIRE(used < 40.0);
    // Counts as before the change (they are results of the unchanged algorithm).
    REQUIRE(same.points.size() == 16385);
    REQUIRE(shiftedHits.points.size() == 4094);
}

TEST_CASE("Near-coincident loops: hit merging keeps first-match semantics", "[bezier][intersect][geometry]") {
    const double e = 1e4;
    const BezierSpan a{{0, 0}, {e, e}, {-e, e}, {0, 0}};
    const auto same = intersect(Span{a}, Span{a}, Tolerance::defaults());
    REQUIRE(same.points.size() == 1943);
    const auto rev = intersect(Span{a}, Span{a.reversed()}, Tolerance::defaults());
    REQUIRE(rev.points.size() == 1936);
}

TEST_CASE("Bezier closest point with control values near the double range", "[bezier][closest][geometry]") {
    for (const double s : {1e99, 1e101, 1e154, 1e200, 1e300}) {
        CAPTURE(s);
        const BezierSpan b{{0, 0}, {s, s}, {-s, s}, {s, 0}};
        const double t0 = processCpuSeconds();
        const Vec2 q = b.pointAt(0.3) + Vec2{0.01 * s, 0.02 * s};
        const ClosestPoint cp = closestPoint(Span{b}, q);
        REQUIRE(processCpuSeconds() - t0 < 1.0);
        REQUIRE(std::isfinite(cp.distance));
        // Compare with a dense sampling: the answer is the best sample up to the sampling step.
        double best = 1e308;
        for (int i = 0; i <= 20000; ++i) {
            best = std::min(best, distance(b.pointAt(static_cast<double>(i) / 20000.0), q));
        }
        REQUIRE(cp.distance <= best * (1.0 + 1e-9));
        REQUIRE(cp.distance >= best * (1.0 - 1e-3));
        REQUIRE(std::fabs(distance(cp.point, q) - cp.distance) <= 1e-9 * cp.distance);
    }
}

TEST_CASE("Bezier polygonisation is bounded when the tolerance cannot be met", "[bezier][polygonise][geometry]") {
    // Extent 1e300 at a 0.01 chordal tolerance needs far more than 2^24 chords: the work bound
    // stops refining (coarser polygon) instead of producing 2^24 points.
    const BezierSpan b{{0, 0}, {1e300, 1e300}, {-1e300, 1e300}, {10, 10}};
    std::vector<Vec2> pts;
    const double t0 = processCpuSeconds();
    polygoniseInto(Span{b}, Tolerance::defaults(), pts);
    REQUIRE(processCpuSeconds() - t0 < 20.0);
    REQUIRE(pts.size() >= 2);
    // Exactly the split budget: a binary tree of 2^20 splits has 2^20 + 1 leaves, one point each.
    REQUIRE(pts.size() == (std::size_t{1} << 20) + 1);
    REQUIRE(pts.back() == b.p3);
    // A modest curve is unaffected by the bound: chordal accuracy holds.
    std::vector<Vec2> fine;
    polygoniseInto(Span{BezierSpan{{0, 0}, {30, 50}, {60, -50}, {90, 0}}}, Tolerance::defaults(), fine);
    REQUIRE(fine.size() > 8);
    REQUIRE(fine.size() < 200);
}

TEST_CASE("Bezier length of non-finite control values returns promptly", "[bezier][length][geometry]") {
    // Before T-0178 a NaN control point made length() recurse to the depth cap (it hung).
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const std::vector<BezierSpan> withNan{
        {{0, 0}, {nan, 0}, {1, 1}, {2, 2}}, {{0, 0}, {1, 1}, {2, 2}, {nan, nan}},
        {{nan, 0}, {1, 1}, {2, 2}, {3, 3}}, {{nan, nan}, {nan, nan}, {nan, nan}, {nan, nan}}};
    const std::vector<BezierSpan> withInf{
        {{0, 0}, {inf, 0}, {1, 1}, {2, 2}}, {{0, 0}, {1, 1}, {-inf, 0}, {2, 2}}, {{0, 0}, {1, 1}, {2, 2}, {3, inf}}};
    const double t0 = processCpuSeconds();
    for (const BezierSpan& b : withNan) {
        REQUIRE(std::isnan(b.length()));
    }
    for (const BezierSpan& b : withInf) {
        REQUIRE(std::isinf(b.length()));
    }
    REQUIRE(processCpuSeconds() - t0 < 1.0);
}

TEST_CASE("Bezier bounds fallback box near the double range", "[bezier][bounds][geometry]") {
    // Evaluating the extrema overflows for these curves; the control-polygon box is returned and
    // it contains the curve (not just the box of the end points).
    const BezierSpan a{{-DBL_MAX, -DBL_MAX * (1 - 1e-16)}, {-DBL_MAX * (1 - 1e-16), -DBL_MAX},
                       {DBL_MAX, DBL_MAX * (1 - 1e-16)}, {DBL_MAX * (1 - 1e-16), DBL_MAX * (1 - 1e-16)}};
    const BezierSpan b{{-DBL_MAX, DBL_MAX * (1 - 1e-16)}, {-DBL_MAX, DBL_MAX}, {-DBL_MAX, -DBL_MAX}, {DBL_MAX * (1 - 1e-16), DBL_MAX}};
    for (const BezierSpan& c : {a, b}) {
        const Box2 box = c.bounds();
        REQUIRE(std::isfinite(box.min.x));
        REQUIRE(std::isfinite(box.min.y));
        REQUIRE(std::isfinite(box.max.x));
        REQUIRE(std::isfinite(box.max.y));
        for (const Vec2 q : {c.p0, c.p1, c.p2, c.p3}) {  // the whole control polygon
            REQUIRE(q.x >= box.min.x);
            REQUIRE(q.x <= box.max.x);
            REQUIRE(q.y >= box.min.y);
            REQUIRE(q.y <= box.max.y);
        }
        for (int i = 0; i <= 200; ++i) {
            const Vec2 p = c.pointAt(static_cast<double>(i) / 200.0);
            if (std::isfinite(p.x) && std::isfinite(p.y)) {
                REQUIRE(p.x >= box.min.x);
                REQUIRE(p.x <= box.max.x);
                REQUIRE(p.y >= box.min.y);
                REQUIRE(p.y <= box.max.y);
            }
        }
    }
}

// The pass-2 piece cap of BezierSpan::length (2^20) is a defensive bound that no finite input
// reaches: pass 2 stops a piece once polygon - chord is below 8 eps of its own magnitude, and
// that needs about 2^17 pieces for a size-1 loop whatever the scale (the criterion is relative), so
// a unit test cannot exercise the cap itself. kLengthSecondPassPieces is covered by review only.
