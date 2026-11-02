// T-0126: the strict inset guard of the pocket's tool-centre region (ADR-0017 §4) on hand-made
// raw insets, through the internal hook `detail::strictInset` (core/src/toolpath/pocket_guard.hpp).
// Clipper2 has not been seen to produce these shapes; they are the T-0125 round-2 verifier's
// raw-injection cases:
//  - N1: a vertex of the round join around a wall tooth pushed towards the tooth tip. The guard
//    removes it and the new chord sags towards the tooth by about four times the arc step
//    (0.015 / 0.148 / 0.708 mm at chordal 0.01 / 0.1 / 0.5), closer than radius + allowance.
//    Every changed edge must now be checked and the region repaired (and reported).
//  - C2 / C4: a 900 mm^2 square region with a small bulge towards a wall that holds most of the
//    vertices (C4) or has reflex junctions too close (C2). The old vertex-count phantom test
//    rejected the whole region silently; it is genuine and must be kept.
// The checks use the independent oracle (pocket_oracle.hpp): exact point and segment distances
// to the wall polygon.

#include "pocket_guard.hpp"

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "pocket_oracle.hpp"

using carvetto::geometry::Contour;
using carvetto::geometry::Polyline;
using carvetto::geometry::Tolerance;
using carvetto::geometry::Vec2;
using carvetto::toolpath::detail::InsetReport;
using carvetto::toolpath::detail::strictInset;
namespace orc = carvetto::test::pocket;

namespace {

struct Outcome {
    std::vector<orc::Poly> rings;
    InsetReport report;
};

/// Feeds `raw` to the guard as the inset of `wall` by d, with polygon tolerance chordal / 2 (as
/// the pocket does for a user chordal tolerance `chordal`).
Outcome guard(const orc::Poly& wall, const std::vector<orc::Poly>& raw, double d, double chordal) {
    Tolerance tol = Tolerance::defaults();
    tol.chordal = 0.5 * chordal;
    std::vector<Contour> rawContours;
    for (const orc::Poly& p : raw) {
        rawContours.push_back(Contour::fromPolyline(Polyline{p, true}));
    }
    Outcome o;
    const std::vector<Contour> out =
        strictInset({Contour::fromPolyline(Polyline{wall, true})}, rawContours, d, tol, &o.report);
    for (const Contour& c : out) {
        o.rings.push_back(c.polygonise(Tolerance::defaults()).points);
    }
    return o;
}

double area(const std::vector<orc::Poly>& rings) {
    double a = 0.0;
    for (const orc::Poly& r : rings) {
        for (std::size_t i = 0, n = r.size(); i < n; ++i) {
            a += r[i].cross(r[(i + 1) % n]);
        }
    }
    return 0.5 * a;
}

/// Every vertex and every edge of the result at least `minDist` from the wall, every vertex
/// inside it.
void checkClear(const orc::Poly& wall, const std::vector<orc::Poly>& rings, double minDist) {
    for (const orc::Poly& r : rings) {
        for (std::size_t i = 0, n = r.size(); i < n; ++i) {
            INFO("edge (" << r[i].x << ", " << r[i].y << ") - (" << r[(i + 1) % n].x << ", "
                          << r[(i + 1) % n].y << ")");
            CHECK(orc::insideEvenOdd({wall}, r[i]));
            CHECK(orc::segBoundaryDist({wall}, r[i], r[(i + 1) % n]) >= minDist);
        }
    }
}

}  // namespace

TEST_CASE("T-0126 N1: a chord left by a removed join vertex is checked and repaired",
          "[toolpath][pocket][guard]") {
    // A 40 x 40 pocket with a tooth poking up from the bottom wall to (20, 12).
    const orc::Poly wall{{0, 0}, {18, 0}, {20, 12}, {22, 0}, {40, 0}, {40, 40}, {0, 40}};
    for (double chordal : {0.01, 0.1, 0.5}) {
        Tolerance tol = Tolerance::defaults();
        tol.chordal = 0.5 * chordal;
        for (double d : {3.0, 5.0}) {
            const std::vector<Contour> raw = carvetto::geometry::offsetContours(
                {Contour::fromPolyline(Polyline{wall, true})}, -d, carvetto::geometry::JoinType::Round, tol);
            REQUIRE(raw.size() == 1);
            const orc::Poly rp = raw[0].polygonise(Tolerance::defaults()).points;
            const double need = d - 0.5 * chordal - 1e-6;
            // The unchanged Clipper inset passes as it is.
            {
                INFO("chordal " << chordal << " d " << d << " unchanged");
                const Outcome o = guard(wall, {rp}, d, chordal);
                CHECK(o.report.dropped == 0);
                CHECK(o.report.trimmed == 0);
                CHECK(std::abs(area(o.rings) - area({rp})) < 1e-9);
            }
            // The join vertex above the tooth tip, pushed towards the tip.
            std::size_t best = 0;
            double bestDist = 1e300;
            for (std::size_t i = 0; i < rp.size(); ++i) {
                const double q = std::hypot(rp[i].x - 20, rp[i].y - 12);
                if (rp[i].y > 12 && q < bestDist) {
                    bestDist = q;
                    best = i;
                }
            }
            for (double delta : {0.05, 0.5, 2.0}) {
                if (delta < chordal) {
                    continue;  // the pushed vertex still keeps the distance at this tolerance
                }
                INFO("chordal " << chordal << " d " << d << " delta " << delta);
                orc::Poly p = rp;
                const Vec2 v = p[best];
                const double len = std::hypot(v.x - 20, v.y - 12);
                p[best] = {20 + (v.x - 20) * (len - delta) / len, 12 + (v.y - 12) * (len - delta) / len};
                const Outcome o = guard(wall, {p}, d, chordal);
                REQUIRE(o.rings.size() == 1);
                CHECK(o.report.dropped == 0);
                // Repaired by a further inset larger than the join tolerance: reported.
                CHECK(o.report.trimmed == 1);
                checkClear(wall, o.rings, need - 1e-7);
                // Only shrinks, and keeps most of the pocket (the true inset is ~780-1075 mm^2;
                // the inset repair costs most at the coarsest tolerance, ~20% at chordal 0.5).
                CHECK(area(o.rings) <= area({p}) + 1e-9);
                CHECK(area(o.rings) > 0.75 * area({rp}));
            }
        }
    }
}

TEST_CASE("T-0126 N2: a genuine region whose bulge holds most vertices is kept (C2, C4)",
          "[toolpath][pocket][guard]") {
    // A 40 x 40 square pocket; the true inset by d = 5 is [5, 35]^2 (900 mm^2). Each raw inset
    // has a bulge towards the bottom wall where no tool centre fits.
    const orc::Poly sq{{0, 0}, {40, 0}, {40, 40}, {0, 40}};
    const double d = 5.0, lo = d, hi = 40 - d;
    const orc::Poly c2{{lo, lo},       {18, lo - 0.1}, {19, 2},  {20, 1.5}, {21, 2},
                       {22, lo - 0.1}, {hi, lo},       {hi, hi}, {lo, hi}};
    orc::Poly c4{{lo, lo}};
    for (int i = 0; i <= 8; ++i) {
        const double t = carvetto::geometry::kPi * i / 8;
        c4.push_back({20 - 2 * std::cos(t), lo - 0.2 - 3 * std::sin(t)});
    }
    c4.insert(c4.end(), {{hi, lo}, {hi, hi}, {lo, hi}});
    REQUIRE(c4.size() == 13);
    for (double chordal : {0.01, 0.1, 0.5}) {
        for (const auto& [name, raw] : {std::pair<std::string, orc::Poly>{"C2", c2}, {"C4", c4}}) {
            INFO(name << " chordal " << chordal);
            const Outcome o = guard(sq, {raw}, d, chordal);
            REQUIRE(o.rings.size() == 1);
            CHECK(o.report.dropped == 0);
            CHECK(orc::insideEvenOdd(o.rings, {20, 20}));
            CHECK(orc::insideEvenOdd(o.rings, {lo + 0.5 * chordal, hi - 0.5 * chordal}));
            // The whole true inset is kept, and of the bulge only a sliver along its base.
            CHECK(area(o.rings) > 900.0 - 1e-6);
            CHECK(area(o.rings) < 900.0 + 40.0 * chordal);
            checkClear(sq, o.rings, d - 0.5 * chordal - 1e-6 - 1e-7);
        }
    }
}

TEST_CASE("T-0126: phantoms are still rejected without a report", "[toolpath][pocket][guard]") {
    // Clipper2 1.4.0 turns a 17 mm circle inset by 19.2 inside out (a 2.2 mm circle comes back).
    orc::Poly circle;
    for (int i = 0; i < 720; ++i) {
        const double t = 2 * carvetto::geometry::kPi * i / 720;
        circle.push_back({17 * std::cos(t), 17 * std::sin(t)});
    }
    for (double chordal : {0.01, 0.1, 0.5}) {
        INFO("chordal " << chordal);
        orc::Poly phantom;
        for (int i = 0; i < 64; ++i) {
            const double t = 2 * carvetto::geometry::kPi * i / 64;
            phantom.push_back({2.2 * std::cos(t), 2.2 * std::sin(t)});
        }
        const Outcome inside = guard(circle, {phantom}, 19.2, chordal);
        CHECK(inside.rings.empty());
        CHECK(inside.report.dropped == 0);
        CHECK(inside.report.trimmed == 0);
        // A region far outside the source, whose vertices all keep the distance.
        const Outcome outside = guard(circle, {orc::rectPoly(60, 60, 70, 70)}, 19.2, chordal);
        CHECK(outside.rings.empty());
        CHECK(outside.report.dropped == 0);
    }
}
