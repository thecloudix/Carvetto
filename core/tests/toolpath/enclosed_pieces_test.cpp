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
    // (CPU time on a loaded machine is noisy: a measurement that fails the bound is repeated, up to
    // three times, and the best taken; a real super-linear cost fails all three.)
    carvetto::test::ScalingTimes row{};
    for (int attempt = 0; attempt < 3; ++attempt) {
        const carvetto::test::ScalingTimes t = carvetto::test::interleavedCpuSeconds(
            kRelease ? 3 : 2, [&] { ns = enclosedPieces(rs, options(carvetto::toolpath::detail::kEnclosedLeafVertices)).size(); },
            [&] { nl = enclosedPieces(rl, options(carvetto::toolpath::detail::kEnclosedLeafVertices)).size(); });
        if (attempt == 0 || t.ratio() < row.ratio()) row = t;
        if (row.ratio() <= 2.5) break;
    }
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
    const double nestedBound = 1.25 * static_cast<double>(vl) / static_cast<double>(vs);
    carvetto::test::ScalingTimes nested{};
    for (int attempt = 0; attempt < 3; ++attempt) {
        const carvetto::test::ScalingTimes t = carvetto::test::interleavedCpuSeconds(
            kRelease ? 3 : 2, [&] { ks = enclosedPieces(ns1, options(carvetto::toolpath::detail::kEnclosedLeafVertices)).size(); },
            [&] { kl = enclosedPieces(nl1, options(carvetto::toolpath::detail::kEnclosedLeafVertices)).size(); });
        if (attempt == 0 || t.ratio() < nested.ratio()) nested = t;
        if (nested.ratio() <= nestedBound) break;
    }
    INFO("nested CPU seconds: " << nested.smallSeconds << " -> " << nested.largeSeconds << ", ratio " << nested.ratio());
    CHECK(ks == static_cast<std::size_t>(ringSmall));
    CHECK(kl == static_cast<std::size_t>(2 * ringSmall));
    CHECK(nested.ratio() <= nestedBound);
}

namespace {

// T-0140 round 3 (verifier round 2, ep.cpp): where a band crosses a leaf side Clipper2 truncates
// the crossing point, so a part's edge along the side could end a grid unit off it and the join
// across that side was lost: a free piece split in two, and held regions reported as pieces.
// Family 7 seed 502 scene 143: nested annuli centred near x = 0 (91.16 mm2 split into 73.19 and
// 17.97 at 512 vertices per leaf).
std::vector<PointRing> nestedAnnuliNearOrigin() {
    struct Annulus {
        double cx, cy, r, w;
        int n;
        double a0;
    };
    static const Annulus an[] = {
        {-0x1.1f7d6daa0b76p-3, 0x1.31e0a3626781cp+2, 0x1.5e758b6c0d21ap+0, 0x1.8c0f4338a02b4p-2, 163, 0x1.ba4e566f33886p+1},
        {-0x1.1f7d6daa0b76p-3, 0x1.31e0a3626781cp+2, 0x1.264e0f1ceda4fp+1, 0x1.0cc0c7fb8c9c3p-1, 72, 0x1.13bdd719ff21fp+2},
        {-0x1.1f7d6daa0b76p-3, 0x1.31e0a3626781cp+2, 0x1.b553a1642e73cp+1, 0x1.5ae57dc4b8d21p-3, 100, 0x1.06491116d9a2ep+2},
        {-0x1.1f7d6daa0b76p-3, 0x1.31e0a3626781cp+2, 0x1.07d6dc0048f6fp+2, 0x1.19221fb9602dp-1, 198, 0x1.548649c606a12p+1},
        {-0x1.1f7d6daa0b76p-3, 0x1.31e0a3626781cp+2, 0x1.7dd2eec857dd4p+2, 0x1.155c67f719f24p-1, 120, 0x1.0e226dcd8c0d8p+2},
        {-0x1.1f7d6daa0b76p-3, 0x1.31e0a3626781cp+2, 0x1.e4d13254bc292p+2, 0x1.2b33a72680b9dp-1, 178, 0x1.326867887cd2fp+2},
        {-0x1.1f7d6daa0b76p-3, 0x1.31e0a3626781cp+2, 0x1.16a59b37b10f6p+3, 0x1.d06edea99fbddp-4, 287, 0x1.1f82499bb481ep+1},
        {-0x1.1f7d6daa0b76p-3, 0x1.31e0a3626781cp+2, 0x1.454332f8ed4d1p+3, 0x1.5cb8c49e7be2ep-4, 25, 0x1.605a80d07c073p-2},
        {-0x1.1f7d6daa0b76p-3, 0x1.31e0a3626781cp+2, 0x1.74a44cb44bba6p+3, 0x1.85e13a9ecff72p-3, 108, 0x1.187a5edd0921ap+2},
        {-0x1.1f7d6daa0b76p-3, 0x1.31e0a3626781cp+2, 0x1.a0ede4127ce3bp+3, 0x1.b9e9bb80d5c7bp-2, 51, 0x1.c693136dcf3e7p+1},
        {-0x1.1f7d6daa0b76p-3, 0x1.31e0a3626781cp+2, 0x1.bfe337dd874d8p+3, 0x1.c5808eb1e4d5ep-4, 236, 0x1.671169c357dc2p+1},
        {-0x1.1f7d6daa0b76p-3, 0x1.31e0a3626781cp+2, 0x1.e822dd368fe11p+3, 0x1.3a0066dae18bdp-3, 12, 0x1.695113e0a58a4p+1},
        {-0x1.1f7d6daa0b76p-3, 0x1.31e0a3626781cp+2, 0x1.0904a217cb6fdp+4, 0x1.10ca0f8a7b661p-2, 20, 0x1.acd181dcf098ep-2},
        {-0x1.1f7d6daa0b76p-3, 0x1.31e0a3626781cp+2, 0x1.109c3eb51323dp+4, 0x1.02476c009c55ap-3, 95, 0x1.5e3309e516924p+2},
        {-0x1.1f7d6daa0b76p-3, 0x1.31e0a3626781cp+2, 0x1.26b35b0ecd414p+4, 0x1.937069c9c31c4p-2, 48, 0x1.124434bbb2201p-1},
        {-0x1.1f7d6daa0b76p-3, 0x1.31e0a3626781cp+2, 0x1.3f4d2e2321905p+4, 0x1.2c41ad17aec44p-1, 76, 0x1.eeb2e19148278p+0},
        {-0x1.1f7d6daa0b76p-3, 0x1.31e0a3626781cp+2, 0x1.4b4547ebb9c66p+4, 0x1.7b8db0bac4651p-2, 254, 0x1.3aa2f06fdf9b9p+2},
        {-0x1.1f7d6daa0b76p-3, 0x1.31e0a3626781cp+2, 0x1.635ef928b1e25p+4, 0x1.684d6b34437cap-2, 222, 0x1.01b6d83d90ff9p+2},
        {-0x1.1f7d6daa0b76p-3, 0x1.31e0a3626781cp+2, 0x1.6d8186dc56c7ep+4, 0x1.2108eef8b2ccdp-1, 85, 0x1.6ff90be70c614p+1},
        {-0x1.1f7d6daa0b76p-3, 0x1.31e0a3626781cp+2, 0x1.89261b604cc3dp+4, 0x1.f4ff5882e323ap-2, 123, 0x1.ce701fede97bp+1},
        {-0x1.1f7d6daa0b76p-3, 0x1.31e0a3626781cp+2, 0x1.96c8f92f66a9ap+4, 0x1.91fc3be14902ep-2, 108, 0x1.7cfdfeaaff67p+0},
        {-0x1.1f7d6daa0b76p-3, 0x1.31e0a3626781cp+2, 0x1.a09a6daa74187p+4, 0x1.72c9054fdf0c4p-3, 65, 0x1.6c940667967fcp+2},
        {-0x1.1f7d6daa0b76p-3, 0x1.31e0a3626781cp+2, 0x1.b54664afd6423p+4, 0x1.a14523069e523p-2, 217, 0x1.23924e219256ap+2},
        {-0x1.136bee0645d7fp+4, 0x1.45173554fade1p+3, 0x1.ea1779f5550e2p-1, 0x1.54dedad6425dp-3, 36, 0x0p+0},
        {0x1.124f0bded67fdp+2, 0x1.2fee42cdd6685p+3, 0x1.b3aacaadb4ca2p-1, 0x1.aada0b23686bep-3, 18, 0x0p+0},
        {0x1.20ee7c5ea8558p+4, 0x1.6cb576c063bacp+3, 0x1.822c469be17cap-3, 0x1.c5a264c8a817fp-5, 21, 0x0p+0},
        {0x1.9c918aedfce85p+2, -0x1.088dc95bfa2f1p+4, 0x1.eb2a31f9bc16cp-1, 0x1.946c9b9b87d5ep-3, 40, 0x0p+0},
    };
    std::vector<PointRing> rings;
    for (const Annulus& a : an) {
        rings.push_back(ngon({a.cx, a.cy}, a.r + a.w, a.n, false, a.a0));
        if (a.r - a.w > 0.0) rings.push_back(ngon({a.cx, a.cy}, a.r - a.w, a.n, true, a.a0));
    }
    return rings;
}

// Family 7 seed 802 scene 142: there a crossing truncated on a leaf's upper side lost the join
// (6.36 mm2 split into 2.77 and 3.59 at 4 to 512 vertices per leaf).
std::vector<PointRing> nestedAnnuliUpperSide() {
    struct Annulus {
        double cx, cy, r, w;
        int n;
        double a0;
    };
    static const Annulus an[] = {
        {0x1.6d7fe2b9daf52p+1, 0x1.2200e282c5758p+2, 0x1.974f2720e72cap+0, 0x1.af7cd2735a291p-2, 67, 0x1.0c07d03c4c917p+2},
        {0x1.6d7fe2b9daf52p+1, 0x1.2200e282c5758p+2, 0x1.abf699410834fp+1, 0x1.76074a054c95ep-3, 192, 0x1.ab9ea66295c2bp-1},
        {0x1.6d7fe2b9daf52p+1, 0x1.2200e282c5758p+2, 0x1.09cefc740d6bcp+2, 0x1.30dd57e0c36p-2, 119, 0x1.3f1d37f56a144p+2},
        {0x1.6d7fe2b9daf52p+1, 0x1.2200e282c5758p+2, 0x1.4ba912f97d7a3p+2, 0x1.f618d3dafe093p-2, 34, 0x1.3e7894b9947c9p+0},
        {0x1.6d7fe2b9daf52p+1, 0x1.2200e282c5758p+2, 0x1.9f2f0d1d79ca7p+2, 0x1.75fb1b0958d1cp-2, 62, 0x1.0f39ac6c30799p-8},
        {0x1.6d7fe2b9daf52p+1, 0x1.2200e282c5758p+2, 0x1.c61dd4f0ae71ep+2, 0x1.99b1bfe9be4p-4, 189, 0x1.23a97313707f4p+2},
        {0x1.6d7fe2b9daf52p+1, 0x1.2200e282c5758p+2, 0x1.082a429fdb65cp+3, 0x1.709370c1c1fbap-2, 29, 0x1.12a27a065ccaep+1},
        {0x1.6d7fe2b9daf52p+1, 0x1.2200e282c5758p+2, 0x1.2a8efd916a748p+3, 0x1.66c74aafb3c25p-2, 144, 0x1.ef4c7a7b24666p+1},
        {0x1.6d7fe2b9daf52p+1, 0x1.2200e282c5758p+2, 0x1.4ba73de27c81ep+3, 0x1.032b66eee2509p-2, 213, 0x1.3c2d74b64d2b6p+2},
        {0x1.6d7fe2b9daf52p+1, 0x1.2200e282c5758p+2, 0x1.5e35a8116f5cep+3, 0x1.2dcd488cf96a2p-3, 43, 0x1.44cc0240a6f54p+2},
        {0x1.6d7fe2b9daf52p+1, 0x1.2200e282c5758p+2, 0x1.6d40789c626c1p+3, 0x1.fcb3704353cdep-3, 156, 0x1.948dc4d71c0eep+1},
        {0x1.6d7fe2b9daf52p+1, 0x1.2200e282c5758p+2, 0x1.7b1ee17b8ae13p+3, 0x1.b77bd59bbcbc6p-3, 88, 0x1.409d63642bccp-3},
        {0x1.6d7fe2b9daf52p+1, 0x1.2200e282c5758p+2, 0x1.98fee415bced2p+3, 0x1.2e2ce4869d924p-2, 47, 0x1.7ba78a4290ab8p+2},
        {0x1.6d7fe2b9daf52p+1, 0x1.2200e282c5758p+2, 0x1.c140ad25d2cb6p+3, 0x1.2d070a560f04p-1, 287, 0x1.02685c9e32c3fp+2},
        {0x1.6d7fe2b9daf52p+1, 0x1.2200e282c5758p+2, 0x1.dc8707863a745p+3, 0x1.03ad4ead1cff2p-2, 262, 0x1.61485e7bdc40ep+2},
        {0x1.6d7fe2b9daf52p+1, 0x1.2200e282c5758p+2, 0x1.023cf223970eep+4, 0x1.9fe0f15a69168p-2, 153, 0x1.0cf981df04eeap+1},
        {0x1.6d7fe2b9daf52p+1, 0x1.2200e282c5758p+2, 0x1.1416a721f1f8p+4, 0x1.1aabc301ac186p-3, 143, 0x1.c37dc957c3a7cp+1},
        {0x1.6d7fe2b9daf52p+1, 0x1.2200e282c5758p+2, 0x1.1a46f0d61fedfp+4, 0x1.112197c5a45acp-3, 182, 0x1.05310eb90aa41p+2},
        {0x1.6d7fe2b9daf52p+1, 0x1.2200e282c5758p+2, 0x1.2ec8c3c87cd7ep+4, 0x1.13d8b012a8d93p-1, 190, 0x1.2350d1e38bdd2p+2},
        {0x1.6d7fe2b9daf52p+1, 0x1.2200e282c5758p+2, 0x1.3e5ca15a8de9fp+4, 0x1.f24423b7fb70cp-2, 189, 0x1.4313d2636ab7dp+0},
        {0x1.1ea2a74fc05e6p+3, 0x1.ea265746791e8p+3, 0x1.1de517cfcd6a3p-2, 0x1.124eb91119b91p-2, 18, 0x0p+0},
        {0x1.3336703c15eb8p+4, 0x1.cebbcce247299p+1, 0x1.60e57ce3c29e7p-1, 0x1.2736972a3aa1cp-3, 40, 0x0p+0},
        {-0x1.5c1897c8258ccp+0, 0x1.ab0454799015cp+2, 0x1.de2eb31635c75p-1, 0x1.217cf73e6c31p-2, 56, 0x0p+0},
        {-0x1.036c0daf2a05ep+3, 0x1.308b2aae1ee12p+3, 0x1.2dbdec95b44a3p-2, 0x1.1dfddc94494b4p-3, 54, 0x0p+0},
        {0x1.f26bbe3a299e7p+2, 0x1.22d22dc5b50d4p+3, 0x1.885cfbbf61117p-3, 0x1.730d8e5584856p-3, 23, 0x0p+0},
    };
    std::vector<PointRing> rings;
    for (const Annulus& a : an) {
        rings.push_back(ngon({a.cx, a.cy}, a.r + a.w, a.n, false, a.a0));
        if (a.r - a.w > 0.0) rings.push_back(ngon({a.cx, a.cy}, a.r - a.w, a.n, true, a.a0));
    }
    return rings;
}

// The band of an open polyline as Clipper2 outlines it (round caps and joins).
void openBand(std::vector<PointRing>& rings, const std::vector<Vec2>& pts, double w) {
    Contour path;
    path.closed = false;
    for (std::size_t k = 0; k + 1 < pts.size(); ++k) path.spans.push_back(geo::LineSpan{pts[k], pts[k + 1]});
    for (const Contour& b : geo::offsetContours({path}, w, geo::JoinType::Round, tolerance(), 2.0)) {
        rings.push_back(b.polygonise(tolerance()).points);
    }
}

// Family 11 seed 401 scene 17 (4x vertices): nested C-shaped bands of long polylines with their
// gaps in line and a radial wall opposite, every region joined to the outside through the gaps.
// One union finds nothing; leaf by leaf it reported 6.04 and 2.32 mm2.
std::vector<PointRing> nestedCBands() {
    struct Band {
        double r, w;
        int m;
        double a0, a1;
    };
    static const Band bs[] = {
        {0x1.be0e03b4ef712p+0, 0x1.0ace1ca6ce0d8p-4, 972, 0x1.400f36e615ab7p+2, 0x1.62d3fabe7e475p+3},
        {0x1.6fff348c875a5p+1, 0x1.48a5bd5128413p-4, 268, 0x1.3ded93d1287cfp+2, 0x1.63e4cc48f4de8p+3},
        {0x1.bf401ec3990a4p+1, 0x1.9a70455c67088p-3, 736, 0x1.3f659dce430c3p+2, 0x1.6328c74a6796ep+3},
        {0x1.28a9c597293b6p+2, 0x1.3a62b7d344829p-2, 1064, 0x1.3f7d68bccaea2p+2, 0x1.631ce1d323a7fp+3},
    };
    const Vec2 c{0x1.0288604192ab8p+1, -0x1.16d31d3f16dabp+1};
    std::vector<PointRing> rings;
    for (const Band& b : bs) {
        std::vector<Vec2> p;
        for (int k = 0; k <= b.m; ++k) {
            const double a = b.a0 + (b.a1 - b.a0) * k / b.m;
            p.push_back(c + Vec2{b.r * std::cos(a), b.r * std::sin(a)});
        }
        openBand(rings, p, b.w);
    }
    const double len = 0x1.5c4ff1145d839p+2;
    const double tw = 0x1.016dcb18c48e8p+3;
    openBand(rings, {c, c + Vec2{len * std::cos(tw), len * std::sin(tw)}}, 0x1.7d3856eb7a06cp-3);
    return rings;
}

// The pieces at `leaf` vertices per leaf match those of one leaf: count, areas (within 1e-3 mm2)
// and thickness.
void checkSameAsOneLeaf(const std::vector<PointRing>& rings, std::size_t leaf) {
    const std::vector<EnclosedPiece> one = enclosedPieces(rings, options(std::size_t{1} << 30));
    const std::vector<EnclosedPiece> got = enclosedPieces(rings, options(leaf));
    INFO("leaf vertices " << leaf << ": " << got.size() << " pieces, one leaf " << one.size());
    REQUIRE(got.size() == one.size());
    std::vector<bool> used(one.size(), false);
    for (const EnclosedPiece& g : got) {
        INFO("piece of " << g.area << " mm2 at " << g.bounds.min.x << ", " << g.bounds.min.y);
        std::size_t match = one.size();
        for (std::size_t k = 0; k < one.size() && match == one.size(); ++k) {
            const Box2& b = one[k].bounds;
            const bool sameBox = std::fabs(b.min.x - g.bounds.min.x) <= 1e-3 && std::fabs(b.min.y - g.bounds.min.y) <= 1e-3 &&
                                 std::fabs(b.max.x - g.bounds.max.x) <= 1e-3 && std::fabs(b.max.y - g.bounds.max.y) <= 1e-3;
            if (!used[k] && sameBox && std::fabs(one[k].area - g.area) <= 1e-3) match = k;
        }
        REQUIRE(match < one.size());
        used[match] = true;
        CHECK(g.inner.has_value() == one[match].inner.has_value());
        // The anchor (attribution) does not depend on the leaves (to the vertices the booleans drop
        // within the distance tolerance of another).
        CHECK(std::fabs(g.anchor.x - one[match].anchor.x) <= 1e-5);
        CHECK(std::fabs(g.anchor.y - one[match].anchor.y) <= 1e-5);
    }
}

}  // namespace

TEST_CASE("Enclosed pieces (T-0140): joins across leaf sides survive Clipper2's truncated crossings",
          "[toolpath][profile][tabs]") {
    SECTION("nested annuli near x = 0: a 91.16 mm2 piece stays whole") {
        const std::vector<PointRing> rings = nestedAnnuliNearOrigin();
        const std::vector<EnclosedPiece> one = enclosedPieces(rings, options(std::size_t{1} << 30));
        CHECK(one.size() == 22);
        CHECK(std::any_of(one.begin(), one.end(), [](const EnclosedPiece& p) { return std::fabs(p.area - 91.158) < 0.01; }));
        for (const std::size_t leaf : {std::size_t{4}, std::size_t{16}, std::size_t{64}, std::size_t{512}}) {
            checkSameAsOneLeaf(rings, leaf);
        }
    }
    SECTION("nested annuli: a 6.36 mm2 piece stays whole") {
        const std::vector<PointRing> rings = nestedAnnuliUpperSide();
        const std::vector<EnclosedPiece> one = enclosedPieces(rings, options(std::size_t{1} << 30));
        CHECK(std::any_of(one.begin(), one.end(), [](const EnclosedPiece& p) { return std::fabs(p.area - 6.3575) < 0.001; }));
        for (const std::size_t leaf : {std::size_t{4}, std::size_t{8}, std::size_t{16}, std::size_t{64}, std::size_t{512}}) {
            checkSameAsOneLeaf(rings, leaf);
        }
    }
    SECTION("nested C-bands joined to the outside: nothing enclosed") {
        const std::vector<PointRing> rings = nestedCBands();
        for (const std::size_t leaf : {std::size_t{4}, std::size_t{16}, std::size_t{64}, std::size_t{512}}) {
            INFO("leaf vertices " << leaf);
            CHECK(enclosedPieces(rings, options(leaf)).empty());
        }
    }
    SECTION("random scenes about the origin and far from it") {
        // As the generative test above, centred on the origin (cuts near x = 0, where most
        // truncations went the wrong way) and shifted to 1e5 and 1e7 mm.
        SplitMix rng{0x71400003};
        const int scenes = kRelease ? 30 : 8;
        for (int s = 0; s < scenes; ++s) {
            bool nudged = false;
            std::vector<PointRing> rings = randomScene(rng, 4 + static_cast<int>(rng.uniform(0.0, 20.0)), nudged);
            const double shift = s % 3 == 0 ? -20.0 : (s % 3 == 1 ? 1e5 : 1e7);
            for (PointRing& r : rings) {
                for (Vec2& p : r) p = p + Vec2{shift, shift};
            }
            INFO("scene " << s << " shift " << shift);
            for (const std::size_t leaf : {std::size_t{4}, std::size_t{8}, std::size_t{64}}) checkSameAsOneLeaf(rings, leaf);
        }
    }
}

TEST_CASE("Enclosed pieces (T-0140): a piece thick only as a whole stays thick where its parts meet at leaf sides",
          "[toolpath][profile][tabs]") {
    // Verifier family 8 seed 104 scene 39: a 10 x 10 grid of square loops outlined by Clipper2
    // (bands of overlapping kerfs). A 0.0553 mm2 piece (about 0.08 x 2.7 mm) is thick (a 0.05 mm
    // disc fits) but no part of it alone is at 4 to 8 vertices per leaf. Its parts, converted for
    // Clipper2 to be unioned, lost vertices closer than the distance tolerance and a crack opened
    // along a leaf side: it was judged thin. The parts are grown by twice that tolerance first.
    struct Cell {
        double s, w;  // the loop's side and the band's half width
    };
    static const Cell cells[100] = {
        {0x1.1cbda6ebb4e2p+1, 0x1.141a29f2f0304p-1}, {0x1.e160e5c197192p+0, 0x1.e504c1aa6e91p-1},
        {0x1.b574315c54b68p+0, 0x1.f6c08f050744ap-1}, {0x1.19915b84246bbp+1, 0x1.9cc598cb59aeep-2},
        {0x1.52c030c8d9836p+0, 0x1.20d1a043d9a48p+0}, {0x1.60e5587a702f3p+1, 0x1.a2b9be504fa2p-2},
        {0x1.6b907c0fb2a7cp+0, 0x1.9f5394f69993cp-1}, {0x1.5cd252c25ab91p+0, 0x1.61bf98bede3f2p-1},
        {0x1.6f5141ec34fb7p+1, 0x1.3a4bd2d66fa5ap-4}, {0x1.fd4d1877b29b1p+0, 0x1.ebe1adf5a176ep-2},
        {0x1.8d8cd78fea43ep+0, 0x1.96ecd0438e34p-1}, {0x1.6ae470f223482p+1, 0x1.f985fa5db510ep-3},
        {0x1.3de665e72c58ap+1, 0x1.4810b7fc3e8afp-2}, {0x1.0f1d496d1a2cap+1, 0x1.48e49241201b2p-2},
        {0x1.2c71ce2585658p+1, 0x1.1aaf3a96276cbp-1}, {0x1.1d8bff59676ap+1, 0x1.a021ef5e3308ap-1},
        {0x1.6259b8b846b13p+1, 0x1.82002c461f8dcp-2}, {0x1.56b565a4acacdp+0, 0x1.b81ec97cf9a2p-1},
        {0x1.8d5d22e2d71c3p+0, 0x1.60923f40215bap-1}, {0x1.2bb22a786b4b4p+1, 0x1.1b28015f2b44p-1},
        {0x1.08f9dd6dd12p+1, 0x1.0b6014dd53eccp-1}, {0x1.4f2f78ab5e54p+1, 0x1.9c12612e2b35ap-2},
        {0x1.3294c1df611a5p+1, 0x1.93c3608732535p-2}, {0x1.17b9044eee025p+1, 0x1.8d2a84ee15f05p-1},
        {0x1.5db664e612924p+1, 0x1.1421ff3b776a9p-1}, {0x1.4d479685ccde2p+1, 0x1.f81bf4547373bp-2},
        {0x1.536379c6f4275p+0, 0x1.1b3df9ba65772p+0}, {0x1.09d49a1d3251dp+1, 0x1.a2778107c85a6p-2},
        {0x1.6fde202266cp+1, 0x1.2a13f86acc0c9p-2}, {0x1.5e7ace95bb7edp+1, 0x1.8bfbc3086003ep-2},
        {0x1.6fee2413f143p+1, 0x1.9b73477c2319ep-2}, {0x1.ea7ddbe44c88ep+0, 0x1.88f758177fe6dp-1},
        {0x1.392a44320d154p+1, 0x1.a95c4a14152dp-2}, {0x1.42797f2dee2fcp+1, 0x1.a1d579e0754fcp-2},
        {0x1.4800a9cab4a3p+1, 0x1.084d16fcf980bp-1}, {0x1.c0498fccfbcb4p+0, 0x1.4d0ce56b19226p-2},
        {0x1.9d8b59d2e6094p+0, 0x1.333faae8c9869p+0}, {0x1.1f8a74b67cfefp+1, 0x1.d4ee198256e01p-2},
        {0x1.ea73ba0d433f2p+0, 0x1.652ea5689f32p-1}, {0x1.a4e9a5e955d44p+0, 0x1.3cc892a26985p-1},
        {0x1.6b1524d1f4102p+0, 0x1.00028ac46bf71p+0}, {0x1.afc294e1cbc66p+0, 0x1.8dcc02ef4577ep-1},
        {0x1.6c169ce716d5dp+1, 0x1.3e3794447933p-2}, {0x1.69278c1520888p+0, 0x1.fe574d8aaa357p-1},
        {0x1.4defc664d6eadp+1, 0x1.9cadef3254ebfp-2}, {0x1.1e1c203d7027p+1, 0x1.8f7734a0c9d85p-1},
        {0x1.9aed13b7ed703p+0, 0x1.dbe315fc78d5fp-1}, {0x1.542421a922d2ep+0, 0x1.e310fb52560e1p-1},
        {0x1.e5672325298e4p+0, 0x1.1890dd72b9386p-1}, {0x1.934ffed15d5b1p+0, 0x1.5ccaeed72e0c4p-1},
        {0x1.4cfeb8df14984p+1, 0x1.62abf7708ea3ep-3}, {0x1.3ce88bb1ff1cdp+1, 0x1.6fc2ed99bf5e9p-3},
        {0x1.caeb3ac0cc503p+0, 0x1.398684a3506adp-1}, {0x1.170d0c2f06006p+1, 0x1.b35626f840c02p-1},
        {0x1.051e2c8ed3267p+1, 0x1.1551969376cc8p-1}, {0x1.5821e39fdc9a8p+0, 0x1.8993279e31afp-1},
        {0x1.56ace91835f53p+1, 0x1.e630cde9c2dd2p-3}, {0x1.2844fb29d6e52p+1, 0x1.481436c51f06p-1},
        {0x1.c12bd8638e9f7p+0, 0x1.b7f6786a9bf54p-1}, {0x1.638a80e7d0bcdp+1, 0x1.38d11ed5fd027p-2},
        {0x1.e3083bf1f529dp+0, 0x1.1cabd3b5a4b81p-2}, {0x1.9caa0bfbce80fp+0, 0x1.3fc342acbdac2p-1},
        {0x1.22341c7ac96bp+1, 0x1.47b061b0bea66p-1}, {0x1.5dfe648881d15p+1, 0x1.2afc26f277074p-2},
        {0x1.58832b1c494dep+1, 0x1.dc9a9ab476119p-2}, {0x1.6f2e2703c2bc4p+1, 0x1.91e7a0da41caap-2},
        {0x1.b4b5fd99afa26p+0, 0x1.846964b0300a4p-1}, {0x1.bfb1c95dcf70ap+0, 0x1.c7e54ab2e3ef4p-1},
        {0x1.e903e3d590caep+0, 0x1.1629fc055b40fp-1}, {0x1.350ed8acacbefp+1, 0x1.45beb0cc40adep-1},
        {0x1.749829392a306p+0, 0x1.bbc75a2d90655p-1}, {0x1.09797ccf2f733p+1, 0x1.5cb886a7420fcp-1},
        {0x1.b37458f0b728fp+0, 0x1.2b95c124e6282p-1}, {0x1.3ae45bdc0b3adp+1, 0x1.af75857b90ee6p-2},
        {0x1.4d61fcd6aa679p+1, 0x1.189ce6ff6ea1cp-1}, {0x1.c14613b28bb3bp+0, 0x1.dfafb11103d28p-1},
        {0x1.5116cf8f11c6dp+1, 0x1.03cbfb6e117dep-2}, {0x1.a9bc00f2c3b45p+0, 0x1.ea575b3875863p-2},
        {0x1.267933870096fp+1, 0x1.cf616e19e1553p-3}, {0x1.961d0b1d212b2p+0, 0x1.168470fb3f219p+0},
        {0x1.69f754cc9c46dp+1, 0x1.335ccfbe31a7dp-3}, {0x1.bfcd4e7face4ep+0, 0x1.ad8c881000d97p-1},
        {0x1.3e1ef0c5a666ap+1, 0x1.cccdb7c877674p-3}, {0x1.68b62d6411dd9p+1, 0x1.9bda645e447e4p-2},
        {0x1.ca19c0779c8dfp+0, 0x1.26e0ce526127ep-1}, {0x1.d9a8db8b1a29dp+0, 0x1.11267b17e8784p-1},
        {0x1.32660b6511e79p+1, 0x1.bec700290a096p-2}, {0x1.0c4a3fb86271dp+1, 0x1.09cace4c7a56cp-1},
        {0x1.6a141a809e5bp+1, 0x1.42c693d4f516cp-2}, {0x1.6c9a4d8fbb519p+1, 0x1.1d04ed7da4a3dp-3},
        {0x1.7fa6c2e2ddd35p+0, 0x1.18ba4a1996619p+0}, {0x1.6ebaf3d2ce6afp+1, 0x1.67fda5ad86226p-2},
        {0x1.75e8e19ef8603p+0, 0x1.98bf5c74abbap-1}, {0x1.28cf02d3e7e59p+1, 0x1.1a5a6b6fa2a54p-1},
        {0x1.2bd44162e743bp+1, 0x1.69b6dd7cf7f18p-2}, {0x1.b208ec9a03238p+0, 0x1.d57f16b07bcb8p-1},
        {0x1.2894389ac8279p+1, 0x1.0b845bcf8c91p-1}, {0x1.8d555b95da928p+0, 0x1.0436eccbad6d5p+0},
        {0x1.2d9403bb510bep+1, 0x1.896a0071a6377p-1}, {0x1.fc288ee570ed9p+0, 0x1.aa0b9057e3bb3p-2}
    };
    const double sp = 0x1.9bbc74914e18ap+1;
    std::vector<PointRing> rings;
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 10; ++j) {
            const Cell& c = cells[10 * i + j];
            const Vec2 o{i * sp, j * sp};
            openBand(rings, {o, o + Vec2{c.s, 0.0}, o + Vec2{c.s, c.s}, o + Vec2{0.0, c.s}, o}, c.w);
        }
    }
    for (const std::size_t leaf : {std::size_t{4}, std::size_t{6}, std::size_t{8}, std::size_t{1} << 30}) {
        INFO("leaf vertices " << leaf);
        const std::vector<EnclosedPiece> got = enclosedPieces(rings, options(leaf));
        const auto it = std::find_if(got.begin(), got.end(), [](const EnclosedPiece& p) { return std::fabs(p.area - 0.0553) < 5e-4; });
        REQUIRE(it != got.end());
        CHECK(it->inner.has_value());
    }
}

TEST_CASE("Enclosed pieces (T-0140): parts meeting over less than 10 nm of a leaf side stay apart",
          "[toolpath][profile][tabs]") {
    // Two 10 x 8 chambers inside a frame band, joined only by a channel 5 nm high and 2 mm long
    // through the wall between them. Small rings above the frame put the leaves' cuts across
    // the channel: there the two sides meet over 5 nm only, under the 10 nm a join needs (the
    // two leaves compute their crossings a few grid units apart), so the chambers are two pieces.
    // One leaf keeps the channel (one piece of both chambers).
    std::vector<PointRing> rings{rect(Box2{{-12, -5}, {12, 5}}, false), rect(Box2{{-11, -4}, {11, 4}}, true),
                                 rect(Box2{{-1, -4.5}, {1, 0}}, false), rect(Box2{{-1, 5e-6}, {1, 4.5}}, false)};
    for (int i = 0; i < 40; ++i) {
        const Vec2 c{-0.5 + 0.025 * i + 0.003 * (i % 7), 7.0 + 0.02 * i};
        rings.push_back({c, c + Vec2{0.01, 0.0}, c + Vec2{0.0, 0.01}});
    }
    rings.push_back(ngon({-30.0, 7.0}, 0.5, 3, false));
    rings.push_back(ngon({30.0, 7.0}, 0.5, 3, false));
    const std::vector<EnclosedPiece> one = enclosedPieces(rings, options(std::size_t{1} << 30));
    REQUIRE(one.size() == 1);
    CHECK(std::fabs(one.front().area - 160.0) <= 1e-3);
    for (const std::size_t leaf : {std::size_t{4}, std::size_t{8}, std::size_t{16}}) {
        INFO("leaf vertices " << leaf);
        const std::vector<EnclosedPiece> got = enclosedPieces(rings, options(leaf));
        REQUIRE(got.size() == 2);
        CHECK(std::fabs(got[0].area - 80.0) <= 1e-3);
        CHECK(std::fabs(got[1].area - 80.0) <= 1e-3);
    }
}

TEST_CASE("Enclosed pieces (T-0140): a piece cut into thin parts is as thick as all of them",
          "[toolpath][profile][tabs]") {
    // A strip of material (3 mm long) between four bands, and small rings far above it whose
    // x coordinates (within 2 um of x = 0) put the first leaf cut through the strip near x = 0.
    // Width 0.06 mm about x = 0: each part about 0.03 mm wide (thin), together thick (a 0.05 mm
    // disc fits). Width 0.08 mm from x = -0.02: the first part is thin, the second thick.
    for (const double xl : {-0.03, -0.02}) {
        const double xr = xl + (xl < -0.025 ? 0.06 : 0.08);
        std::vector<PointRing> rings{rect(Box2{{-1, -1}, {xl, 4}}, false), rect(Box2{{xr, -1}, {1, 4}}, false),
                                     rect(Box2{{-1, -1}, {1, 0}}, false), rect(Box2{{-1, 3}, {1, 4}}, false)};
        for (int i = 0; i < 40; ++i) {
            const Vec2 c{-0.002 + 1e-4 * i, 10.0 + 0.02 * i};
            rings.push_back({c, c + Vec2{1e-4, 0.0}, c + Vec2{0.0, 0.01}});
        }
        rings.push_back(ngon({-30.0, 10.0}, 0.5, 3, false));
        rings.push_back(ngon({30.0, 10.0}, 0.5, 3, false));
        for (const std::size_t leaf : {std::size_t{8}, std::size_t{16}, std::size_t{1} << 30}) {
            INFO("strip from " << xl << " to " << xr << ", leaf vertices " << leaf);
            const std::vector<EnclosedPiece> got = enclosedPieces(rings, options(leaf));
            REQUIRE(got.size() == 1);
            CHECK(std::fabs(got.front().area - 3.0 * (xr - xl)) <= 1e-4);
            REQUIRE(got.front().inner.has_value());
            CHECK(got.front().inner->x > xl + 0.02);
            CHECK(got.front().inner->x < xr - 0.02);
        }
    }
}

TEST_CASE("Enclosed pieces (T-0140): leaves stay at least 10 um wide", "[toolpath][profile][tabs]") {
    // A chamber with a ring 4 um wide inside it whose 400 vertices all lie at x = 0 or 4 um:
    // cuts at the median vertex would halve the leaves next to it down to fractions of a
    // micrometre, where the booleans' removal of vertices closer than the distance tolerance
    // collapses the leaf's rectangle (and the chamber's material across it). Leaves stop at
    // 10 um: the chamber is one piece of its full area at any leaf size.
    std::vector<PointRing> rings{rect(Box2{{-6, -4}, {6, 4}}, false), rect(Box2{{-5, -3}, {5, 3}}, true)};
    PointRing comb;
    for (int k = 0; k < 200; ++k) comb.push_back({4e-6, -2.0 + 0.02 * k});
    for (int k = 199; k >= 0; --k) comb.push_back({0.0, -2.0 + 0.02 * k + 0.01});
    rings.push_back(comb);
    rings.push_back(ngon({-30.0, 0.0}, 0.5, 3, false));
    rings.push_back(ngon({30.0, 0.0}, 0.5, 3, false));
    const double combArea = std::fabs(geo::signedArea(contourOf(comb)));
    for (const std::size_t leaf : {std::size_t{4}, std::size_t{16}, std::size_t{64}, std::size_t{1} << 30}) {
        INFO("leaf vertices " << leaf);
        const std::vector<EnclosedPiece> got = enclosedPieces(rings, options(leaf));
        REQUIRE(got.size() == 1);
        CHECK(std::fabs(got.front().area - (60.0 - combArea)) <= 1e-4);
        CHECK(got.front().inner.has_value());
    }
}

namespace {

using carvetto::toolpath::detail::bandOutline;
using carvetto::toolpath::detail::pathsToSearch;

// A path of lines and arcs drawn as a turtle: lines, arcs turning left (positive sweep) or
// right, and sharp corners.
struct Turtle {
    Contour path;
    Vec2 p;
    double heading{0.0};
    void line(double length) {
        const Vec2 q = p + Vec2{std::cos(heading), std::sin(heading)} * length;
        path.spans.push_back(geo::LineSpan{p, q});
        p = q;
    }
    void arc(double radius, double sweep) {
        const Vec2 n = Vec2{-std::sin(heading), std::cos(heading)} * (sweep > 0.0 ? 1.0 : -1.0);
        const Vec2 c = p + n * radius;
        const double a0 = std::atan2(p.y - c.y, p.x - c.x);
        const geo::ArcSpan a{c, radius, a0, a0 + sweep, sweep > 0.0 ? geo::ArcDirection::CCW : geo::ArcDirection::CW};
        path.spans.push_back(a);
        p = a.endPoint();
        heading += sweep;
    }
};

Contour randomPath(SplitMix& rng, Vec2 start, double w, int pieces) {
    Turtle t;
    t.p = start;
    t.heading = rng.uniform(0.0, 2.0 * kPi);
    t.path.closed = false;
    for (int i = 0; i < pieces; ++i) {
        const int kind = static_cast<int>(rng.uniform(0.0, 6.0));
        const double sign = rng.uniform(0.0, 1.0) < 0.5 ? 1.0 : -1.0;
        if (kind <= 1) {
            t.line(rng.uniform(0.05, 4.0) * w);
        } else if (kind <= 3) {
            t.arc(rng.uniform(0.1, 3.0) * w, sign * rng.uniform(0.1, 3.0));
        } else if (kind == 4) {
            t.heading += sign * rng.uniform(0.2, 3.0);  // a sharp corner, up to 172 degrees
            t.line(rng.uniform(0.1, 3.0) * w);
        } else {
            t.arc(rng.uniform(0.5, 2.0) * w, sign * rng.uniform(2.5, 6.0));
        }
    }
    return t.path;
}

int windingOf(const std::vector<PointRing>& rings, Vec2 q) {
    int w = 0;
    for (const PointRing& r : rings) {
        for (std::size_t k = 0; k < r.size(); ++k) {
            const Vec2 u = r[k];
            const Vec2 v = r[(k + 1) % r.size()];
            if (u.y <= q.y) {
                if (v.y > q.y && (v - u).cross(q - u) > 0.0) ++w;
            } else if (v.y <= q.y && (v - u).cross(q - u) < 0.0) {
                --w;
            }
        }
    }
    return w;
}

}  // namespace

TEST_CASE("Band outline (T-0140): positive exactly on the band, at most the chordal tolerance beyond it",
          "[toolpath][profile][tabs][property]") {
    // Random paths of lines, arcs (some tighter than the band) and sharp corners, open and closed
    // on themselves: every point within w of the path has a positive winding number, every point
    // more than w + chordal from it none, and no point a negative one. This needs the wedges at
    // outer corners, the vertex at inner corners (left out only where the two lines' rectangles
    // cover its triangle), the closing corner of a closed path, and the polygons' tangents
    // outside the convex arcs (chords there would leave the band's edge uncovered).
    SplitMix rng{0x71400004};
    const int paths = kRelease ? 300 : 80;
    for (int s = 0; s < paths; ++s) {
        const double w = rng.uniform(0.05, 3.0);
        const double chordal = rng.uniform(0.0, 1.0) < 0.5 ? 0.005 : rng.uniform(1e-4, 0.05);
        Contour path = randomPath(rng, {0, 0}, w, 1 + static_cast<int>(rng.uniform(0.0, 8.0)));
        const bool closed = rng.uniform(0.0, 1.0) < 0.3;
        if (closed) path.spans.push_back(geo::LineSpan{geo::endPoint(path.spans.back()), geo::startPoint(path.spans.front())});
        const std::vector<PointRing> rings = bandOutline(path, w, chordal, closed);
        REQUIRE(!rings.empty());
        Contour asPath = path;
        asPath.closed = false;
        const Box2 box = asPath.bounds().inflated(w + 0.2 * w + 0.01);
        INFO("path " << s << " w " << w << " chordal " << chordal << " closed " << closed);
        for (int k = 0; k < 400; ++k) {
            const Vec2 q{rng.uniform(box.min.x, box.max.x), rng.uniform(box.min.y, box.max.y)};
            const double d = geo::distanceTo(asPath, q);
            const int wn = windingOf(rings, q);
            INFO("point " << q.x << ", " << q.y << " at " << d << " winding " << wn);
            CHECK(wn >= 0);
            if (d <= w * (1.0 - 1e-9)) CHECK(wn > 0);
            if (d > w + chordal * (1.0 + 1e-6) + 1e-9) CHECK(wn == 0);
        }
    }
}

TEST_CASE("Paths to search (T-0140): stretches that provably enclose nothing are left out",
          "[toolpath][profile][tabs]") {
    const double reach = 1.6;
    // A rounded rectangle (an Outside cut path) less three tabs, and a circle less one: nothing
    // is enclosed, nothing near; none is searched.
    auto roundedRect = [](Vec2 o, double w, double h, double r) {
        Contour c;
        c.closed = false;
        const Vec2 corners[4] = {o + Vec2{w, 0}, o + Vec2{w, h}, o + Vec2{0, h}, o};
        Vec2 p = o + Vec2{0, -r};
        for (int k = 0; k < 4; ++k) {
            const Vec2 dir = k == 0 ? Vec2{1, 0} : (k == 1 ? Vec2{0, 1} : (k == 2 ? Vec2{-1, 0} : Vec2{0, -1}));
            const Vec2 q = corners[k] + Vec2{dir.y, -dir.x} * r;
            c.spans.push_back(geo::LineSpan{p, q});
            const double a0 = std::atan2(-dir.x, dir.y);
            const geo::ArcSpan a{corners[k], r, a0, a0 + 0.5 * kPi, geo::ArcDirection::CCW};
            c.spans.push_back(a);
            p = a.endPoint();
        }
        return c;
    };
    // Stretches of a closed path between gaps (tabs) at the given arc lengths.
    auto stretches = [](const Contour& loop, std::vector<std::pair<double, double>> gaps) {
        std::vector<Contour> out;
        double total = 0.0;
        for (const geo::Span& sp : loop.spans) total += geo::length(sp);
        for (std::size_t i = 0; i < gaps.size(); ++i) {
            const double u0 = gaps[i].second;
            const double u1 = i + 1 < gaps.size() ? gaps[i + 1].first : gaps[0].first + total;
            Contour o;
            o.closed = false;
            double s = 0.0;
            for (int rep = 0; rep < 2; ++rep) {
                for (const geo::Span& sp : loop.spans) {
                    const double len = geo::length(sp);
                    const double a = std::max(u0, s);
                    const double b = std::min(u1, s + len);
                    if (b > a + 1e-12) {
                        const double t0 = (a - s) / len;
                        const double t1 = (b - s) / len;
                        if (const auto* l = std::get_if<geo::LineSpan>(&sp)) {
                            o.spans.push_back(geo::LineSpan{l->start + (l->end - l->start) * t0, l->start + (l->end - l->start) * t1});
                        } else if (const auto* ar = std::get_if<geo::ArcSpan>(&sp)) {
                            geo::ArcSpan x = *ar;
                            x.startAngle = ar->startAngle + ar->sweepAngle() * t0;
                            x.endAngle = ar->startAngle + ar->sweepAngle() * t1;
                            o.spans.push_back(x);
                        }
                    }
                    s += len;
                }
            }
            out.push_back(o);
        }
        return out;
    };
    const Contour rr = roundedRect({0, 0}, 8.0, 6.0, 1.59);
    Contour circle;
    circle.closed = false;
    circle.spans.push_back(geo::ArcSpan{{40, 0}, 5.0, 0.0, 2.0 * kPi, geo::ArcDirection::CCW});
    std::vector<Contour> paths = stretches(rr, {{1.0, 7.0}, {14.0, 20.0}, {26.0, 32.0}});
    for (const Contour& c : stretches(circle, {{0.0, 6.0}})) paths.push_back(c);
    const std::vector<char> none(paths.size(), 0);
    CHECK(pathsToSearch(paths, none, reach, 0.008) == std::vector<char>(paths.size(), 0));

    SECTION("a gap narrower than two reaches closes the ring: searched") {
        std::vector<Contour> p2 = stretches(circle, {{0.0, 3.1}});
        CHECK(pathsToSearch(p2, {0}, reach, 0.008) == std::vector<char>{1});
        p2 = stretches(circle, {{0.0, 3.3}});
        CHECK(pathsToSearch(p2, {0}, reach, 0.008) == std::vector<char>{0});
    }
    SECTION("closed whole paths are searched; so is everything in the box of what is searched") {
        // The whole rounded rectangle (no tab), a short line inside it, the circle's stretch.
        Turtle t;
        t.p = {3.0, 3.0};
        t.path.closed = false;
        t.line(2.0);
        const std::vector<Contour> p2{rr, t.path, paths[3]};
        const std::vector<char> got = pathsToSearch(p2, {1, 0, 0}, reach, 0.008);
        CHECK(got == std::vector<char>{1, 1, 0});
    }
    SECTION("stretches of different paths whose bands meet are searched") {
        std::vector<Contour> p2 = paths;
        Turtle t;
        t.p = {4.0, 9.2};  // 9.2 - (6 + 1.59) = 1.61 from the top edge's path: the bands overlap
        t.path.closed = false;
        t.line(3.0);
        p2.push_back(t.path);
        const std::vector<char> got = pathsToSearch(p2, std::vector<char>(p2.size(), 0), reach, 0.008);
        CHECK(got[4] == 1);
        CHECK(std::count(got.begin(), got.begin() + 3, char{1}) >= 1);
        CHECK(got[3] == 0);
    }
}

TEST_CASE("Paths to search (T-0140): the bands of the searched paths enclose every thick piece the bands of all enclose",
          "[toolpath][profile][tabs][property]") {
    // Random open paths (lines, arcs, sharp corners, near-closing loops) spread out or crowded,
    // bands as Profile builds them (bandOutline, 2 um past the tool, polygons within 0.005 mm):
    // the thick pieces enclosedPieces finds from the searched paths' bands alone are those it
    // finds from every band.
    SplitMix rng{0x71400005};
    const int scenes = kRelease ? 160 : 40;
    int cleared = 0;
    int withPieces = 0;
    for (int s = 0; s < scenes; ++s) {
        const double w = std::exp(rng.uniform(std::log(0.05), std::log(3.0)));
        const bool spread = rng.uniform(0.0, 1.0) < 0.6;
        const int n = 1 + static_cast<int>(rng.uniform(0.0, 8.0));
        std::vector<Contour> paths;
        for (int i = 0; i < n; ++i) {
            const Vec2 at = spread ? Vec2{(i % 3) * 60.0 * w, (i / 3) * 60.0 * w} : Vec2{rng.uniform(0.0, 12.0) * w, rng.uniform(0.0, 12.0) * w};
            if (rng.uniform(0.0, 1.0) < 0.4) {
                // A loop that nearly closes: a gap about two reaches wide.
                Turtle t;
                t.p = at;
                t.path.closed = false;
                const double r = rng.uniform(1.0, 6.0) * w;
                t.arc(r, 2.0 * kPi - 2.0 * w * rng.uniform(0.9, 1.1) / r);
                paths.push_back(t.path);
            } else {
                paths.push_back(randomPath(rng, at, w, 2 + static_cast<int>(rng.uniform(0.0, 20.0))));
            }
        }
        const double reach = w + 2e-3;
        const double sag = 0.005;
        const std::vector<char> search = pathsToSearch(paths, std::vector<char>(paths.size(), 0), reach + sag, 0.012);
        std::vector<PointRing> all;
        std::vector<PointRing> some;
        for (std::size_t i = 0; i < paths.size(); ++i) {
            for (PointRing& r : bandOutline(paths[i], reach, sag, false)) {
                if (search[i]) some.push_back(r);
                all.push_back(std::move(r));
            }
            if (!search[i]) ++cleared;
        }
        EnclosedOptions opt = options(carvetto::toolpath::detail::kEnclosedLeafVertices);
        opt.tol = Tolerance{};
        std::vector<double> a;
        std::vector<double> b;
        for (const EnclosedPiece& p : enclosedPieces(all, opt)) {
            if (p.inner) a.push_back(p.area);
        }
        if (!some.empty()) {
            for (const EnclosedPiece& p : enclosedPieces(some, opt)) {
                if (p.inner) b.push_back(p.area);
            }
        }
        std::sort(a.begin(), a.end());
        std::sort(b.begin(), b.end());
        INFO("scene " << s << " w " << w << " paths " << n);
        REQUIRE(a.size() == b.size());
        for (std::size_t k = 0; k < a.size(); ++k) CHECK(std::fabs(a[k] - b[k]) <= 1e-4 * a[k] + 1e-3);
        if (!a.empty()) ++withPieces;
    }
    CHECK(cleared >= scenes / 2);   // the proof does clear paths
    CHECK(withPieces >= scenes / 4);  // and there are pieces to miss
}

TEST_CASE("Paths to search (T-0140): arcs that bulge towards each other close a waist", "[toolpath][profile][tabs]") {
    // A peanut-shaped loop open on its left lobe: two radius-10 arcs bulging towards each other
    // form its waist, `gap` wide at the middle, and a radius-4 arc closes the right lobe. Each
    // waist arc is one span whose proof stretches (5, sagitta 0.00796 mm) put one stretch's middle
    // at the waist: its chord lies 0.008 mm behind the arc. With the waist just under two reaches
    // the bands close it and enclose the right lobe, though the stretches' chords lie more than
    // two reaches apart: only the arcs' own hulls show that they meet. Just over two reaches the
    // waist stays open and nothing is searched.
    const double reach = 1.6;
    for (const double gap : {2.0 * reach - 0.004, 2.0 * reach + 0.008}) {
        const double h = 0.5 * gap;
        const double half = 0.5 * (5.0 * 2.0 * std::acos(1.0 - 0.008 / 10.0) - 0.001);  // 5 stretches
        Contour path;
        path.closed = false;
        const Vec2 c1{0.0, -10.0 - h};
        const Vec2 c2{0.0, 10.0 + h};
        const geo::ArcSpan side1{c1, 10.0, 0.5 * kPi + half, 0.5 * kPi - half, geo::ArcDirection::CW};
        const geo::ArcSpan side2{c2, 10.0, -0.5 * kPi + half, -0.5 * kPi - half, geo::ArcDirection::CW};
        const Vec2 l1 = side1.startPoint();
        const Vec2 r1 = side1.endPoint();
        const Vec2 r2 = side2.startPoint();
        const Vec2 l2 = side2.endPoint();
        const double d = std::sqrt(16.0 - r2.y * r2.y);
        const Vec2 cr{r1.x + d, 0.0};
        const Vec2 cl{l1.x - d, 0.0};
        // The left lobe from just past its leftmost point (a 4.8 mm gap there, a tab) round to l1.
        const double gapAngle = 0.6;
        path.spans.push_back(geo::ArcSpan::fromCentre(cl, cl + Vec2::fromAngle(kPi + gapAngle, 4.0), l1, geo::ArcDirection::CCW));
        path.spans.push_back(side1);
        path.spans.push_back(geo::ArcSpan::fromCentre(cr, r1, r2, geo::ArcDirection::CCW));
        path.spans.push_back(side2);
        path.spans.push_back(geo::ArcSpan::fromCentre(cl, l2, cl + Vec2::fromAngle(kPi - gapAngle, 4.0), geo::ArcDirection::CCW));
        const bool closes = gap < 2.0 * reach;
        INFO("waist " << gap);
        CHECK(pathsToSearch({path}, {0}, reach, 0.008) == std::vector<char>{closes ? char{1} : char{0}});
        // The bands (within the reach) enclose the right lobe exactly when the waist closes.
        const std::vector<PointRing> rings = bandOutline(path, reach - 0.0011, 0.001, false);
        EnclosedOptions opt = options(carvetto::toolpath::detail::kEnclosedLeafVertices);
        opt.tol = Tolerance{};
        const std::vector<EnclosedPiece> got = enclosedPieces(rings, opt);
        CHECK(got.size() == (closes ? 1u : 0u));
    }
}
