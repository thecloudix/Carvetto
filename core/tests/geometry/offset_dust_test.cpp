// T-0138: offsetContours drops grid-level dust rings (rings no disk of a few grid units fits
// in), so near collapse no sliver comes back and a result wound against the outermost input is
// always a genuine hole (ADR-0006 §8).
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "carvetto/geometry/fixed_point.hpp"
#include "carvetto/geometry/offset.hpp"
#include "offset_dust.hpp"
#include "test_rng.hpp"

using namespace carvetto::geometry;

namespace {

constexpr double kGrid = 1.0 / kFixedPointScale;  // one Clipper2 grid unit, mm
// Zero-distance offsets of nanometre features need a coincidence tolerance below them (the input
// polygoniser merges points closer than tol.distance).
constexpr Tolerance kFine{1e-9, 1e-9, 0.01};

using Ring = std::vector<Vec2>;

Ring vertices(const Contour& c) {
    Ring r;
    for (const Span& s : c.spans) {
        r.push_back(startPoint(s));
    }
    return r;
}

// Shoelace area about the first vertex (exact enough far from the origin too).
double ringArea(const Ring& r) {
    double a = 0.0;
    for (std::size_t i = 1, n = r.size(); i + 1 < n; ++i) {
        a += (r[i] - r[0]).cross(r[i + 1] - r[0]);
    }
    return 0.5 * a;
}

int winding(const Ring& r, Vec2 p) {
    int w = 0;
    for (std::size_t i = 0, n = r.size(); i < n; ++i) {
        const Vec2 a = r[i];
        const Vec2 b = r[(i + 1) % n];
        const double cross = (b.x - a.x) * (p.y - a.y) - (p.x - a.x) * (b.y - a.y);
        if (a.y <= p.y) {
            if (b.y > p.y && cross > 0.0) {
                ++w;
            }
        } else if (b.y <= p.y && cross < 0.0) {
            --w;
        }
    }
    return w;
}

// Number of result rings whose orientation does not match their nesting depth: at even depth a
// ring must be wound like the outermost input (sign), at odd depth against it.
int orientationViolations(const std::vector<Contour>& out, double sign) {
    std::vector<Ring> rings;
    for (const Contour& c : out) {
        rings.push_back(vertices(c));
    }
    int bad = 0;
    for (std::size_t k = 0; k < rings.size(); ++k) {
        if (rings[k].size() < 3) {
            ++bad;
            continue;
        }
        const Vec2 probe = (rings[k][0] + rings[k][1]) * 0.5;
        int depth = 0;
        for (std::size_t m = 0; m < rings.size(); ++m) {
            if (m != k && winding(rings[m], probe) != 0) {
                ++depth;
            }
        }
        const double want = depth % 2 == 0 ? sign : -sign;
        if (ringArea(rings[k]) * want <= 0.0) {
            ++bad;
        }
    }
    return bad;
}

Contour bezierCircle(Vec2 c, double r) {
    Contour o;
    o.closed = true;
    for (const BezierSpan& b : ArcSpan::fullCircle(c, r).toBeziers()) {
        o.spans.emplace_back(b);
    }
    return o;
}

Contour polygon(const std::vector<Vec2>& p) {
    Contour c;
    c.closed = true;
    for (std::size_t i = 0; i < p.size(); ++i) {
        c.spans.emplace_back(LineSpan{p[i], p[(i + 1) % p.size()]});
    }
    return c;
}

Contour rectangle(Vec2 lo, Vec2 hi) {
    return polygon({lo, {hi.x, lo.y}, hi, {lo.x, hi.y}});
}

}  // namespace

// --- AC1: the reported repros come back clean ------------------------------------------------

TEST_CASE("T-0138 AC1: Bezier annulus inset just past collapse returns no dust", "[geometry][offset][dust]") {
    // T-0018 verifier repro: 8 rings of 3 vertices (|area| 2e-11..9e-9 mm^2, at most 2.5 nm
    // wide), two of them wound against their nesting.
    const Vec2 c{32.939392279875733, -76.249769660635266};
    const double ro = 43.277926783499645 - c.x;
    const double ri = 6.7009639741650435;
    const std::vector<Contour> annulus{bezierCircle(c, ro), Contour::circle(c, ri, ArcDirection::CW)};
    const Tolerance tol{0.001, 1e-9, 0.0002};
    CHECK(offsetContours(annulus, -((ro - ri) / 2.0 + 0.0015), JoinType::Round, tol).empty());
    // Comfortably before collapse the annulus is still there: an outer and a hole.
    const auto kept = offsetContours(annulus, -((ro - ri) / 2.0 - 0.01), JoinType::Round, tol);
    REQUIRE(kept.size() == 2);
    CHECK(orientationViolations(kept, 1.0) == 0);
}

TEST_CASE("T-0138 AC1: circle inset close to its radius keeps one ring and no nested dust",
          "[geometry][offset][dust]") {
    // Found by the generative oracle: a 3-vertex ring (6e-12 mm^2) nested in the genuine one and
    // wound like it.
    const Vec2 c{17.373689449005781, 18.421595748854685};
    const double r = 21.768982155773653;
    const Tolerance tol{0.001, 1e-9, 0.0002};
    for (const JoinType join : {JoinType::Round, JoinType::Sharp, JoinType::Square}) {
        for (const ArcDirection dir : {ArcDirection::CCW, ArcDirection::CW}) {
            const auto out = offsetContours({Contour::circle(c, r, dir)}, -(r - 0.0016), join, tol);
            REQUIRE(out.size() == 1);
            const double sign = dir == ArcDirection::CCW ? 1.0 : -1.0;
            CHECK(ringArea(vertices(out[0])) * sign > 0.0);
            // The genuine ring: radius ~1.6 um less polygonisation error.
            const Box2 b = out[0].bounds();
            CHECK(b.max.x - b.min.x > 0.002);
            CHECK(b.max.x - b.min.x < 0.0034);
        }
    }
}

// --- AC1: orientation versus nesting near collapse, generative --------------------------------

TEST_CASE("T-0138 AC1 generative: near collapse every result ring is wound for its nesting depth",
          "[geometry][offset][dust]") {
    carvetto::test::SplitMix rng{3};  // on main: an annulus inset returns 78 rings, one reversed
    const double chordals[] = {0.0002, 0.001, 0.01};
    // Insets near collapse cost O(n^2) in Clipper2's union (every offset edge crosses the
    // others), so the finest tolerance gets the smallest shapes.
    const double maxRadius[] = {5.0, 12.0, 25.0};
    int offsets = 0;
    int nonEmpty = 0;
    int bad = 0;
    std::string first;
    for (int n = 0; n < 48; ++n) {
        const Vec2 c{rng.uniform(-80.0, 80.0), rng.uniform(-80.0, 80.0)};
        const Tolerance tol{0.001, 1e-9, chordals[n % 3]};
        std::vector<Contour> shape;
        double inradius = 0.0;
        if (n % 2 == 0) {
            const double r = rng.uniform(1.0, maxRadius[n % 3]);
            shape.push_back(n % 4 == 0 ? Contour::circle(c, r) : bezierCircle(c, r));
            inradius = r;
        } else {
            const double ro = rng.uniform(2.0, maxRadius[n % 3]);
            const double ri = ro * rng.uniform(0.2, 0.8);
            shape = {bezierCircle(c, ro), Contour::circle(c, ri, ArcDirection::CW)};
            inradius = (ro - ri) / 2.0;
        }
        const bool reversed = n % 6 >= 3;
        if (reversed) {
            for (Contour& s : shape) {
                s = s.reversed();
            }
        }
        const double sign = reversed ? -1.0 : 1.0;
        // Distances straddling collapse (Bezier circles bulge out by up to 2.7e-4 x r).
        for (const double dd : {-0.0016, -0.0006, -0.0002, 0.0, 0.0002, 0.0006, 0.0015, 0.003}) {
            const double d = inradius + dd + rng.uniform(-1e-5, 1e-5);
            const auto out = offsetContours(shape, -d, JoinType::Round, tol);
            ++offsets;
            nonEmpty += out.empty() ? 0 : 1;
            const int v = orientationViolations(out, sign);
            if (v > 0 && bad == 0) {
                first = "n=" + std::to_string(n) + " d=" + std::to_string(d) +
                        " rings=" + std::to_string(out.size());
            }
            bad += v;
        }
    }
    INFO(first);
    CHECK(bad == 0);
    CHECK(offsets == 384);
    CHECK(nonEmpty >= 150);  // genuine rings close to collapse still come back
}

// --- AC2: genuine thin regions are kept --------------------------------------------------------

TEST_CASE("T-0138 AC2: rings wider than the dust threshold survive, narrower ones are dropped",
          "[geometry][offset][dust]") {
    const Tolerance tol = kFine;
    for (const Vec2 at : {Vec2{0.0, 0.0}, Vec2{12345.6, -9876.5}}) {
        INFO("at " << at.x << "," << at.y);
        // Strips 100 mm long and w wide, offset by zero: Clipper2 returns them unchanged unless
        // they are dust (no disk of radius 4 grid units fits).
        for (const double w : {12.0 * kGrid, 20.0 * kGrid, 1000.0 * kGrid}) {
            INFO("w = " << w);
            const auto out = offsetContours({rectangle(at, at + Vec2{100.0, w})}, 0.0, JoinType::Round, tol);
            REQUIRE(out.size() == 1);
            // Exact but for the grid rounding of the corners.
            CHECK(std::fabs(ringArea(vertices(out[0])) - 100.0 * w) < 100.0 * kGrid);
        }
        for (const double w : {2.0 * kGrid, 5.0 * kGrid}) {
            INFO("w = " << w);
            CHECK(offsetContours({rectangle(at, at + Vec2{100.0, w})}, 0.0, JoinType::Round, tol).empty());
        }
        // A 10 um square with a 100 mm hair 2 nm wide: its mean width is below the threshold
        // but the square holds a large disk, so the ring is kept whole.
        const double s = 0.01;
        const double h = 2.0 * kGrid;
        const Contour hairy =
            polygon({at, at + Vec2{s, 0.0}, at + Vec2{s, s / 2.0}, at + Vec2{s + 100.0, s / 2.0},
                     at + Vec2{s + 100.0, s / 2.0 + h}, at + Vec2{s, s / 2.0 + h}, at + Vec2{s, s},
                     at + Vec2{0.0, s}});
        const auto out = offsetContours({hairy}, 0.0, JoinType::Round, tol);
        REQUIRE(out.size() == 1);
        CHECK(out[0].spans.size() == 8);
        // A pocket whose inset leaves a tiny genuine region (a 1.4 um disk) keeps it.
        const double r = 3.0;
        const auto tiny = offsetContours({Contour::circle(at, r)}, -(r - 0.0016), JoinType::Round,
                                         Tolerance{0.001, 1e-9, 0.0002});
        REQUIRE(tiny.size() == 1);
        CHECK(ringArea(vertices(tiny[0])) > 1e-6);
    }
}

TEST_CASE("T-0138 AC2: dust is judged per ring; a hole inside a dropped outer goes with it",
          "[geometry][offset][dust]") {
    // A strip 7 grid units wide holding a 3-unit slot: both rings are dust.
    const double len = 50.0;
    const Contour strip = rectangle({0.0, 0.0}, {len, 7.0 * kGrid});
    const Contour slot = rectangle({1.0, 2.0 * kGrid}, {len - 1.0, 5.0 * kGrid}).reversed();
    CHECK(offsetContours({strip, slot}, 0.0, JoinType::Round, kFine).empty());
    // A dust-thin hole in a genuine region is dropped (filled), the region kept.
    const Contour block = rectangle({0.0, 0.0}, {len, 1.0});
    const auto filled = offsetContours({block, slot}, 0.0, JoinType::Round, kFine);
    REQUIRE(filled.size() == 1);
    CHECK(std::fabs(ringArea(vertices(filled[0])) - len) < 1e-9);
    // The test is per ring (its own interior): a frame narrow only between an outer and its
    // hole is two genuine rings, correctly wound, and is kept.
    const double s = 0.1;
    const double t = 6.0 * kGrid;
    const std::vector<Contour> frameIn{rectangle({0.0, 0.0}, {s, s}),
                                       rectangle({t, t}, {s - t, s - t}).reversed()};
    const auto frame = offsetContours(frameIn, 0.0, JoinType::Round, kFine);
    REQUIRE(frame.size() == 2);
    CHECK(orientationViolations(frame, 1.0) == 0);
}

// --- The nesting pass on its own, against a brute-force closure ---------------------------------

TEST_CASE("T-0138: the dust nesting pass matches a brute-force closure", "[geometry][offset][dust]") {
    using detail::GridBox;
    carvetto::test::SplitMix rng{138};
    const auto within = [](const GridBox& in, const GridBox& out) {
        return in.left >= out.left && in.right <= out.right && in.top >= out.top && in.bottom <= out.bottom;
    };
    int marked = 0;
    for (int trial = 0; trial < 300; ++trial) {
        const int layout = trial % 3;  // 0: small grid (shared edges), 1: one column, 2: wide spread
        const std::size_t n = 5 + static_cast<std::size_t>(rng.uniform(0.0, 60.0));
        const auto pick = [&rng](std::int64_t lo, std::int64_t hi) {
            return lo + static_cast<std::int64_t>(rng.uniform(0.0, static_cast<double>(hi - lo + 1)));
        };
        std::vector<GridBox> boxes(n);
        std::vector<bool> usable(n);
        std::vector<bool> dust(n);
        for (std::size_t i = 0; i < n; ++i) {
            GridBox b;
            if (layout == 1) {
                b.left = 0;
                b.right = pick(0, 3) * 10;  // equal left edges, often equal right edges too
            } else {
                const std::int64_t span = layout == 0 ? 8 : 1000;
                b.left = pick(-span, span);
                b.right = b.left + pick(0, span);  // zero-width boxes included
            }
            b.top = pick(-8, 8);
            b.bottom = b.top + pick(0, 8);
            boxes[i] = b;
            usable[i] = rng.uniform(0.0, 1.0) < 0.9;
            dust[i] = rng.uniform(0.0, 1.0) < 0.2;
        }
        // Nesting is decided by more than the boxes: some box-contained pairs are not nested.
        const auto nestedRule = [](std::size_t outer, std::size_t inner) {
            return outer != inner && (outer * 7 + inner * 3) % 5 != 0;
        };
        std::vector<bool> expect = dust;
        for (bool changed = true; changed;) {
            changed = false;
            for (std::size_t o = 0; o < n; ++o) {
                for (std::size_t i = 0; i < n; ++i) {
                    if (expect[o] && !expect[i] && usable[o] && usable[i] && within(boxes[i], boxes[o]) &&
                        nestedRule(o, i)) {
                        expect[i] = true;
                        changed = true;
                    }
                }
            }
        }
        bool badCall = false;
        std::vector<bool> got = dust;
        detail::markNestedInDust(boxes, usable, got, [&](std::size_t outer, std::size_t inner) {
            // Only asked for usable pairs, inner not dust yet, inner's box within outer's.
            if (!usable[outer] || !usable[inner] || got[inner] || !within(boxes[inner], boxes[outer])) {
                badCall = true;
            }
            return nestedRule(outer, inner);
        });
        INFO("trial " << trial);
        CHECK_FALSE(badCall);
        CHECK(got == expect);
        for (std::size_t i = 0; i < n; ++i) {
            marked += (got[i] && !dust[i]) ? 1 : 0;
        }
    }
    CHECK(marked > 300);  // the closure did real work, chains included
}

// The erosion that judges dust is Clipper2's and not exact on acute rings (ADR-0006 section 8):
// this thin triangle has an exact inradius of 5.54 grid units, above the nominal 4, and is
// still removed. Pinned so the documented band stays true.
TEST_CASE("T-0138: an acute triangle of inradius 5.5 grid units is judged dust (documented band)",
          "[geometry][offset][dust]") {
    const Contour t = polygon({{0.019917, 0.098387}, {0.024209, 0.096417}, {0.024216, 0.096426}});
    CHECK(offsetContours({t}, 0.0, JoinType::Round, kFine).empty());
    // Scaled up 3x about its first vertex (inradius ~16.6 units) it is kept.
    const Vec2 a{0.019917, 0.098387};
    const Contour big =
        polygon({a, a + (Vec2{0.024209, 0.096417} - a) * 3.0, a + (Vec2{0.024216, 0.096426} - a) * 3.0});
    CHECK(offsetContours({big}, 0.0, JoinType::Round, kFine).size() == 1);
}
