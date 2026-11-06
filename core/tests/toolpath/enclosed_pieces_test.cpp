// T-0140 round 2: the pieces a union of rings encloses, found leaf by leaf
// (core/src/toolpath/enclosed_pieces.hpp, ADR-0016 §14), must be those of one union over
// everything (its holes, less the outers nested directly in them, thick when eroded by the
// clearance they leave something), however small the leaves (so pieces are split across many
// leaf sides and joined again), and the cost must stay near-linear on the layouts that made one
// union quadratic: a row of shapes whose bands overlap, and nested rings.
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/offset.hpp"
#include "enclosed_pieces.hpp"
#include "../geometry/test_rng.hpp"
// Last: on Windows it includes <windows.h> (macros such as near/far/small, and a GDI `Polyline`).
#include "../geometry/cpu_scaling.hpp"

using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::Tolerance;
using carvetto::geometry::Vec2;
using carvetto::test::SplitMix;
using carvetto::toolpath::detail::enclosedPieces;
using carvetto::toolpath::detail::EnclosedOptions;
using carvetto::toolpath::detail::EnclosedPiece;
using carvetto::toolpath::detail::PointRing;

namespace geo = carvetto::geometry;

namespace {

#ifdef NDEBUG
constexpr bool kRelease = true;
#else
constexpr bool kRelease = false;
#endif

constexpr double kPi = 3.14159265358979323846;

// A regular n-gon about c (counter-clockwise, or clockwise when `cw`), first vertex at angle a0.
PointRing ngon(Vec2 c, double r, int n, bool cw, double a0 = 0.0) {
    PointRing p;
    for (int k = 0; k < n; ++k) {
        const double a = a0 + (cw ? -1.0 : 1.0) * 2.0 * kPi * k / n;
        p.push_back(c + Vec2{r * std::cos(a), r * std::sin(a)});
    }
    return p;
}

PointRing rect(Box2 b, bool cw) {
    PointRing p{b.min, {b.max.x, b.min.y}, b.max, {b.min.x, b.max.y}};
    if (cw) std::reverse(p.begin(), p.end());
    return p;
}

Contour contourOf(const PointRing& r) {
    Contour c;
    c.closed = true;
    for (std::size_t k = 0; k < r.size(); ++k) c.spans.push_back(geo::LineSpan{r[k], r[(k + 1) % r.size()]});
    return c;
}

Tolerance tolerance() {
    Tolerance t;
    t.distance = 0.001;
    t.chordal = 0.005;
    return t;
}

EnclosedOptions options(std::size_t leafVertices) {
    EnclosedOptions o;
    o.minArea = 0.04;
    o.clearance = 0.025;
    o.tol = tolerance();
    o.leafVertices = leafVertices;
    return o;
}

// The reference: one union of every ring; each hole less the outers nested directly in it.
struct OraclePiece {
    double area{0.0};
    bool thick{false};
    bool thickLess{false};  // with 2 % less clearance
    bool thickMore{false};  // with 2 % more clearance
    std::vector<Contour> region;
};

std::vector<OraclePiece> oracle(const std::vector<PointRing>& rings, const EnclosedOptions& opt) {
    std::vector<Contour> cs;
    for (const PointRing& r : rings) cs.push_back(contourOf(r));
    const std::vector<Contour> u = geo::unionContours(cs, opt.tol, geo::FillRule::NonZero);
    std::vector<double> sa;
    for (const Contour& c : u) sa.push_back(geo::signedArea(c));
    // The contour that most closely encloses contour k (judged at the first of its vertices that
    // is not on the candidate: nudged rings can touch).
    auto enclosing = [&](std::size_t k) {
        std::size_t best = u.size();
        double bestArea = std::numeric_limits<double>::infinity();
        for (std::size_t m = 0; m < u.size(); ++m) {
            if (m == k || std::fabs(sa[m]) >= bestArea || !u[m].bounds().contains(u[k].bounds())) continue;
            for (const geo::Span& sp : u[k].spans) {
                const geo::PointLocation at =
                    geo::locatePoint(u[m], geo::startPoint(sp), geo::FillRule::NonZero, Tolerance{});
                if (at == geo::PointLocation::OnBoundary) continue;
                if (at == geo::PointLocation::Inside) {
                    best = m;
                    bestArea = std::fabs(sa[m]);
                }
                break;
            }
        }
        return best;
    };
    std::vector<OraclePiece> out;
    for (std::size_t h = 0; h < u.size(); ++h) {
        if (sa[h] >= 0.0) continue;
        OraclePiece p;
        p.area = -sa[h];
        p.region.push_back(u[h]);
        geo::setOrientation(p.region.back(), geo::Orientation::CCW, opt.tol);
        for (std::size_t k = 0; k < u.size(); ++k) {
            if (sa[k] <= 0.0 || !u[h].bounds().contains(u[k].bounds())) continue;
            if (enclosing(k) != h) continue;
            p.area -= sa[k];
            p.region.push_back(u[k]);
            geo::setOrientation(p.region.back(), geo::Orientation::CW, opt.tol);
        }
        if (p.area < opt.minArea) continue;
        auto thickBy = [&](double c) {
            return !geo::offsetContours(p.region, -c, geo::JoinType::Round, opt.tol, opt.mitreLimit).empty();
        };
        p.thick = thickBy(opt.clearance);
        p.thickLess = thickBy(0.98 * opt.clearance);
        p.thickMore = thickBy(1.02 * opt.clearance);
        out.push_back(std::move(p));
    }
    return out;
}

// Checks enclosedPieces against the oracle: the same pieces (by area, within grid rounding plus
// `areaSlack`: slivers a few micrometres wide between nudged edges may go either way), the same
// thickness
// (unless the oracle itself is undecided within 2 % of the clearance), and each inner point
// inside its own piece, at least the clearance from the union (less rounding).
void checkAgainstOracle(const std::vector<PointRing>& rings, std::size_t leafVertices, double areaSlack = 1e-4) {
    const EnclosedOptions opt = options(leafVertices);
    const std::vector<EnclosedPiece> got = enclosedPieces(rings, opt);
    const std::vector<OraclePiece> want = oracle(rings, opt);
    INFO("leaf vertices " << leafVertices << ", pieces " << got.size() << " (oracle " << want.size() << ")");
    REQUIRE(got.size() == want.size());
    std::vector<bool> used(want.size(), false);
    for (const EnclosedPiece& g : got) {
        INFO("piece of " << g.area << " mm2 at " << g.bounds.min.x << ", " << g.bounds.min.y);
        std::size_t match = want.size();
        for (std::size_t k = 0; k < want.size(); ++k) {
            if (used[k] || std::fabs(want[k].area - g.area) > 1e-5 * want[k].area + areaSlack) continue;
            if (g.inner && geo::locatePoint(want[k].region, *g.inner, geo::FillRule::NonZero, Tolerance{}) !=
                               geo::PointLocation::Inside) {
                continue;
            }
            match = k;
            break;
        }
        if (match == want.size()) {
            double nearest = std::numeric_limits<double>::infinity();
            bool innerIn = false;
            for (std::size_t k = 0; k < want.size(); ++k) {
                if (std::fabs(want[k].area - g.area) < std::fabs(nearest - g.area)) {
                    nearest = want[k].area;
                    innerIn = g.inner && geo::locatePoint(want[k].region, *g.inner, geo::FillRule::NonZero,
                                                          Tolerance{}) == geo::PointLocation::Inside;
                }
            }
            INFO("nearest oracle piece " << nearest << " mm2, inner point inside it " << innerIn);
            REQUIRE(match < want.size());
        }
        used[match] = true;
        const OraclePiece& w = want[match];
        if (w.thickLess == w.thickMore) CHECK(g.inner.has_value() == w.thick);
        if (g.inner) {
            double d = std::numeric_limits<double>::infinity();
            for (const Contour& c : w.region) d = std::min(d, geo::distanceTo(c, *g.inner));
            CHECK(d >= opt.clearance - 2.0 * opt.tol.chordal);
        }
    }
}

// Random rings in a 40 mm square: annuli (the band of a closed path, outer and hole), bands of
// open polylines (Clipper2's outline), rectangles with rectangular holes, bars on a grid nudged
// by micrometres, and rings wound twice.
std::vector<PointRing> randomScene(SplitMix& rng, int count, bool& nudged) {
    std::vector<PointRing> rings;
    nudged = false;
    for (int i = 0; i < count; ++i) {
        const Vec2 c{rng.uniform(0.0, 40.0), rng.uniform(0.0, 40.0)};
        const int kind = static_cast<int>(rng.uniform(0.0, 5.0));
        if (kind == 0) {
            const double r = rng.uniform(1.0, 9.0);
            const double w = rng.uniform(0.2, 1.5);
            const int n = 8 + static_cast<int>(rng.uniform(0.0, 40.0));
            const double a0 = rng.uniform(0.0, 6.0);
            rings.push_back(ngon(c, r + w, n, false, a0));
            if (r - w > 0.0) rings.push_back(ngon(c, r - w, n, true, a0));
        } else if (kind == 1) {
            Contour path;
            path.closed = false;
            Vec2 p = c;
            const int legs = 2 + static_cast<int>(rng.uniform(0.0, 5.0));
            for (int k = 0; k < legs; ++k) {
                const Vec2 q = p + Vec2{rng.uniform(-12.0, 12.0), rng.uniform(-12.0, 12.0)};
                path.spans.push_back(geo::LineSpan{p, q});
                p = q;
            }
            for (const Contour& b : geo::offsetContours({path}, rng.uniform(0.2, 1.2), geo::JoinType::Round,
                                                        tolerance(), 2.0)) {
                rings.push_back(b.polygonise(tolerance()).points);
            }
        } else if (kind == 2) {
            const double w = rng.uniform(2.0, 14.0);
            const double h = rng.uniform(2.0, 14.0);
            const double t = rng.uniform(0.3, 2.0);
            const Box2 outer{c, c + Vec2{w, h}};
            rings.push_back(rect(outer, false));
            if (w > 2.0 * t && h > 2.0 * t) rings.push_back(rect(outer.inflated(-t), true));
        } else if (kind == 4) {
            // Bars on a 2 mm grid nudged by up to 2 um: edges within micrometres of each other and
            // of the leaves' sides, and slivers that thin.
            const double x = 2.0 * std::floor(rng.uniform(0.0, 20.0)) + rng.uniform(-2e-3, 2e-3);
            const double y = 2.0 * std::floor(rng.uniform(0.0, 20.0)) + rng.uniform(-2e-3, 2e-3);
            const bool across = rng.uniform(0.0, 1.0) < 0.5;
            const double len = 2.0 * std::floor(rng.uniform(2.0, 10.0));
            const double t = 2.0 + rng.uniform(-2e-3, 2e-3);
            nudged = true;
            rings.push_back(rect(across ? Box2{{x, y}, {x + len, y + t}} : Box2{{x, y}, {x + t, y + len}}, false));
        } else {
            // A pentagram (winding 2 in its middle) or a circle traced twice.
            const double r = rng.uniform(1.0, 6.0);
            PointRing p;
            if (rng.uniform(0.0, 1.0) < 0.5) {
                for (int k = 0; k < 5; ++k) {
                    const double a = 2.0 * kPi * (2 * k) / 5.0;
                    p.push_back(c + Vec2{r * std::cos(a), r * std::sin(a)});
                }
            } else {
                for (int k = 0; k < 48; ++k) {
                    const double a = 2.0 * kPi * (2 * k) / 48.0;
                    p.push_back(c + Vec2{r * std::cos(a), r * std::sin(a)});
                }
            }
            rings.push_back(p);
        }
    }
    return rings;
}

}  // namespace

TEST_CASE("Enclosed pieces (T-0140): leaf by leaf equals one union, at any leaf size",
          "[toolpath][profile][tabs][property]") {
    SplitMix rng{0x71400002};
    const int scenes = kRelease ? 60 : 24;
    int withPieces = 0;
    for (int s = 0; s < scenes; ++s) {
        bool nudged = false;
        const std::vector<PointRing> rings = randomScene(rng, 4 + static_cast<int>(rng.uniform(0.0, 24.0)), nudged);
        INFO("scene " << s);
        for (const std::size_t leaf : {std::size_t{4}, std::size_t{6}, std::size_t{24}, std::size_t{100},
                                       carvetto::toolpath::detail::kEnclosedLeafVertices}) {
            checkAgainstOracle(rings, leaf, nudged ? 5e-3 : 1e-4);
        }
        if (!enclosedPieces(rings, options(4)).empty()) ++withPieces;
    }
    CHECK(withPieces >= scenes / 3);
}

TEST_CASE("Enclosed pieces (T-0140): crafted layouts", "[toolpath][profile][tabs]") {
    SECTION("a ring cut into many leaves, its piece joined again") {
        // A 30 mm annulus band around a disc piece, with small rings scattered inside it so the
        // subdivision cuts the disc into many leaves.
        std::vector<PointRing> rings{ngon({0, 0}, 31.0, 400, false), ngon({0, 0}, 29.0, 400, true)};
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) rings.push_back(ngon({-15.0 + 6.0 * i, -15.0 + 6.0 * j}, 1.0, 40, false));
        }
        for (const std::size_t leaf : {std::size_t{8}, std::size_t{64}, std::size_t{100000}}) {
            const std::vector<EnclosedPiece> got = enclosedPieces(rings, options(leaf));
            REQUIRE(got.size() == 1);
            const double disc = 0.5 * 400.0 * 29.0 * 29.0 * std::sin(2.0 * kPi / 400.0);
            const double holes = 36.0 * 0.5 * 40.0 * std::sin(2.0 * kPi / 40.0);
            CHECK(std::fabs(got.front().area - (disc - holes)) <= 2e-3);  // grid rounding of 1,840 vertices
            CHECK(got.front().inner.has_value());
            checkAgainstOracle(rings, leaf);
        }
    }
    SECTION("nested annuli: every ring between two bands is its own piece") {
        std::vector<PointRing> rings;
        for (int i = 0; i < 12; ++i) {
            const double r = 3.0 + 2.0 * i;
            rings.push_back(ngon({0, 0}, r + 0.5, 64, false));
            rings.push_back(ngon({0, 0}, r - 0.5, 64, true));
        }
        for (const std::size_t leaf : {std::size_t{6}, std::size_t{50}, std::size_t{100000}}) {
            const std::vector<EnclosedPiece> got = enclosedPieces(rings, options(leaf));
            CHECK(got.size() == 12);  // the centre disc and 11 rings
            checkAgainstOracle(rings, leaf);
        }
    }
    SECTION("a thin piece across leaf sides is thin; a thick one is thick") {
        // Two bars 0.04 mm and 0.2 mm apart, closed at the ends: the strip between them is a
        // piece, 0.04 mm wide (not thick) or 0.2 mm (thick), 30 mm long, cut by many leaves.
        for (const double gap : {0.04, 0.2}) {
            std::vector<PointRing> rings{rect(Box2{{0, 0}, {30, 2}}, false), rect(Box2{{0, 2 + gap}, {30, 4 + gap}}, false),
                                         rect(Box2{{-2, 0}, {0, 4 + gap}}, false), rect(Box2{{30, 0}, {32, 4 + gap}}, false)};
            for (int i = 0; i < 30; ++i) rings.push_back(ngon({0.5 + i, -3.0}, 0.3, 12, false));  // leaf fodder
            for (const std::size_t leaf : {std::size_t{4}, std::size_t{30}, std::size_t{100000}}) {
                INFO("gap " << gap << " leaf " << leaf);
                const std::vector<EnclosedPiece> got = enclosedPieces(rings, options(leaf));
                REQUIRE(got.size() == 1);
                CHECK(std::fabs(got.front().area - 30.0 * gap) <= 1e-4);
                CHECK(got.front().inner.has_value() == (gap > 0.1));
            }
        }
    }
    SECTION("pieces touching only at a corner of two bands stay apart; nothing on the frame") {
        // Two squares of material, each enclosed by its own band, the bands sharing an edge.
        std::vector<PointRing> rings{rect(Box2{{0, 0}, {10, 10}}, false), rect(Box2{{1, 1}, {9, 9}}, true),
                                     rect(Box2{{9, 0}, {19, 10}}, false), rect(Box2{{10, 1}, {18, 9}}, true)};
        for (const std::size_t leaf : {std::size_t{4}, std::size_t{100000}}) {
            const std::vector<EnclosedPiece> got = enclosedPieces(rings, options(leaf));
            REQUIRE(got.size() == 2);
            CHECK(std::fabs(got[0].area - 64.0) <= 1e-6);
            CHECK(std::fabs(got[1].area - 64.0) <= 1e-6);
        }
        CHECK(enclosedPieces({rect(Box2{{0, 0}, {10, 10}}, false)}, options(4)).empty());
        CHECK(enclosedPieces({}, options(4)).empty());
    }
}

namespace {

// The bands of a row of n squares cut Outside with overlapping kerfs (as Profile builds them for
// Inside/Outside with every tab left out): each an outer ring and a hole, 5 mm squares 3.8 mm
// apart, bands 3.2 mm wide.
std::vector<PointRing> rowBands(int n) {
    std::vector<PointRing> rings;
    for (int i = 0; i < n; ++i) {
        const Vec2 c{i * 8.8 + 2.5, 2.5};
        PointRing outer;
        for (int k = 0; k < 4; ++k) {  // the square grown by 3.2 mm, round corners
            const Vec2 corner = c + Vec2{(k == 0 || k == 3) ? 2.5 : -2.5, k < 2 ? 2.5 : -2.5};
            const double a0 = kPi / 2.0 * k;
            for (int j = 0; j <= 8; ++j) {
                const double a = a0 + kPi / 2.0 * j / 8.0;
                outer.push_back(corner + Vec2{3.2 * std::cos(a), 3.2 * std::sin(a)});
            }
        }
        rings.push_back(outer);
        rings.push_back(rect(Box2::fromCentre(c, 5.0, 5.0), true));
    }
    return rings;
}

// n nested annular bands 4.8 mm apart, 3.2 mm wide, polygonised with edges of about 0.5 mm (as
// Profile's bands: polygons of their arcs): every ring between them is a piece. `vertices` gets
// their number.
std::vector<PointRing> nestedBands(int n, std::size_t& vertices) {
    std::vector<PointRing> rings;
    vertices = 0;
    for (int i = 0; i < n; ++i) {
        const double r = 5.0 + 4.8 * i;
        for (const double rr : {r + 1.6, r - 1.6}) {
            const int m = std::max(16, static_cast<int>(std::ceil(2.0 * kPi * rr / 0.5)));
            rings.push_back(ngon({0, 0}, rr, m, rr < r));
            vertices += static_cast<std::size_t>(m);
        }
    }
    return rings;
}

}  // namespace

TEST_CASE("Enclosed pieces (T-0140): near-linear on a row of overlapping bands and on nested rings",
          "[toolpath][profile][tabs][perf]") {
    // One Clipper2 union of the whole scene (T-0140 round 1) was a single sweep over every band
    // side by side: a row of 4,000 tabless squares spent 2.9 s in it (x3.8-4 per doubling), and
    // nested free rings went cubic in the island search (0.05 / 0.63 / 7.2 s at 50 / 100 / 200).
    // Release, T-0140 machine: row 4,000 -> 8,000 squares 0.037 -> 0.071 s (ratio 1.9; one
    // union: 3.8-4x per doubling). Time for 2n over time for n at most 2.5.
    const int rowSmall = kRelease ? 4000 : 500;
    const std::vector<PointRing> rs = rowBands(rowSmall);
    const std::vector<PointRing> rl = rowBands(2 * rowSmall);
    std::size_t ns = 0;
    std::size_t nl = 0;
    const carvetto::test::ScalingTimes row = carvetto::test::interleavedCpuSeconds(
        kRelease ? 3 : 2, [&] { ns = enclosedPieces(rs, options(carvetto::toolpath::detail::kEnclosedLeafVertices)).size(); },
        [&] { nl = enclosedPieces(rl, options(carvetto::toolpath::detail::kEnclosedLeafVertices)).size(); });
    INFO("row CPU seconds: " << row.smallSeconds << " -> " << row.largeSeconds << ", ratio " << row.ratio());
    CHECK(ns == static_cast<std::size_t>(rowSmall));
    CHECK(nl == static_cast<std::size_t>(2 * rowSmall));
    CHECK(row.ratio() <= 2.5);

    // The rings' vertices grow as n^2 (their radii grow with n): time for 2n over time for n at
    // most 1.25 times the vertices' ratio (about 4). Release: 100 -> 200 rings 0.38 -> 1.72 s
    // (4.0x the vertices, 4.3x the time); the island search of T-0140 round 1 was cubic in the
    // rings.
    const int ringSmall = kRelease ? 100 : 25;
    std::size_t vs = 0;
    std::size_t vl = 0;
    const std::vector<PointRing> ns1 = nestedBands(ringSmall, vs);
    const std::vector<PointRing> nl1 = nestedBands(2 * ringSmall, vl);
    std::size_t ks = 0;
    std::size_t kl = 0;
    const carvetto::test::ScalingTimes nested = carvetto::test::interleavedCpuSeconds(
        kRelease ? 3 : 2, [&] { ks = enclosedPieces(ns1, options(carvetto::toolpath::detail::kEnclosedLeafVertices)).size(); },
        [&] { kl = enclosedPieces(nl1, options(carvetto::toolpath::detail::kEnclosedLeafVertices)).size(); });
    INFO("nested CPU seconds: " << nested.smallSeconds << " -> " << nested.largeSeconds << ", ratio " << nested.ratio());
    CHECK(ks == static_cast<std::size_t>(ringSmall));
    CHECK(kl == static_cast<std::size_t>(2 * ringSmall));
    CHECK(nested.ratio() <= 1.25 * static_cast<double>(vl) / static_cast<double>(vs));
}
