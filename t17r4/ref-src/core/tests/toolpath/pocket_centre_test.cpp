// T-0125: the tool-centre region must never lose a genuine part. The strict inset guard used to
// drop a whole lobe when one of its vertices landed ~1e-6 mm past the slack (Clipper2's grid
// plus round-join sag at the budget limit), leaving a reachable part of the pocket silently
// uncut (verifier sweep seed 3150371, "chain" scene; fixture fixtures/pocket/gap_poly.txt).
//
// The oracle is independent of the offset code: the exact signed distance to the input polygon
// (pocket_oracle.hpp) is sampled on a grid, its local maxima are the centres of the pocket's
// lobes, and every lobe centre deeper than radius + margin (+ 0.02) must lie inside the centre
// region. Every vertex of the centre region must be inside the pocket, and every edge at least
// radius + margin − (chordal/2 + 2e-6) from the walls (the join budget of ADR-0017 §3; exact
// segment distance, T-0126).

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include "../geometry/test_rng.hpp"
#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "pocket_oracle.hpp"

using namespace carvetto::toolpath;
using carvetto::geometry::Contour;
using carvetto::geometry::kPi;
using carvetto::geometry::Polyline;
using carvetto::geometry::Vec2;
namespace orc = carvetto::test::pocket;

namespace {

orc::Poly fixturePoly(const char* name) {
    orc::Poly pts;
    const std::string path = std::string(CARVETTO_POCKET_FIXTURE_DIR) + "/" + name;
    std::ifstream in(path);
    if (!in) {
        FAIL("cannot open " << path);
    }
    double x = 0.0, y = 0.0;
    while (in >> x >> y) {
        pts.push_back({x, y});
    }
    return pts;
}

orc::Poly gapPoly() {
    return fixturePoly("gap_poly.txt");
}

/// A chain of overlapping discs (cusps where they meet), polygonised finely: the shape family
/// where Clipper2 leaves zero-width out-and-back spikes in the inset (T-0125 round 2).
orc::Poly discChain(std::uint64_t seed) {
    carvetto::test::SplitMix rng{seed};
    const int m = 3 + static_cast<int>(rng.next() % 4);
    std::vector<Contour> discs;
    Vec2 c{0, 0};
    double prevR = rng.uniform(3.0, 7.0);
    discs.push_back(Contour::circle(c, prevR));
    for (int i = 1; i < m; ++i) {
        const double r = rng.uniform(3.0, 7.0);
        const double a = rng.uniform(-0.6, 0.6);
        c = c + Vec2{std::cos(a), std::sin(a)} * ((prevR + r) * rng.uniform(0.55, 0.85));
        discs.push_back(Contour::circle(c, r));
        prevR = r;
    }
    const std::vector<Contour> u = carvetto::geometry::unionContours(discs);
    carvetto::geometry::Tolerance fine;
    fine.chordal = 0.01;
    return u.empty() ? orc::Poly{} : u[0].polygonise(fine).points;
}

double signedDist(const orc::Poly& poly, Vec2 p) {
    const double d = orc::boundaryDist({poly}, p);
    return orc::insideEvenOdd({poly}, p) ? d : -d;
}

/// Lobe centres: grid points (spacing h) whose signed distance is a local maximum.
std::vector<std::pair<Vec2, double>> lobeCentres(const orc::Poly& poly, double h) {
    double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
    for (const Vec2 q : poly) {
        x0 = std::min(x0, q.x);
        y0 = std::min(y0, q.y);
        x1 = std::max(x1, q.x);
        y1 = std::max(y1, q.y);
    }
    const int nx = static_cast<int>((x1 - x0) / h) + 1, ny = static_cast<int>((y1 - y0) / h) + 1;
    std::vector<double> sd(static_cast<std::size_t>(nx * ny));
    auto at = [&](int i, int j) -> double& { return sd[static_cast<std::size_t>(j * nx + i)]; };
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            at(i, j) = signedDist(poly, {x0 + i * h, y0 + j * h});
        }
    }
    std::vector<std::pair<Vec2, double>> out;
    for (int j = 1; j + 1 < ny; ++j) {
        for (int i = 1; i + 1 < nx; ++i) {
            const double v = at(i, j);
            bool peak = v > 0.2;
            for (int dj = -1; dj <= 1 && peak; ++dj) {
                for (int di = -1; di <= 1 && peak; ++di) {
                    peak = (di == 0 && dj == 0) || at(i + di, j + dj) <= v;
                }
            }
            if (peak) {
                out.push_back({{x0 + i * h, y0 + j * h}, v});
            }
        }
    }
    return out;
}

std::vector<orc::Poly> polysOf(const std::vector<Contour>& cs) {
    std::vector<orc::Poly> out;
    for (const Contour& c : cs) {
        out.push_back(c.polygonise(carvetto::geometry::Tolerance::defaults()).points);
    }
    return out;
}

/// The wall edges of one polygon in a uniform grid (cell size `cell`), for "is every edge of a
/// region at least `bound` from the wall" over thousands of diameters. Distances are the oracle's
/// exact segment-to-segment distance.
class WallIndex {
   public:
    WallIndex(const orc::Poly& wall, double cell) : wall_(wall), cell_(cell) {
        x0_ = y0_ = 1e300;
        double x1 = -1e300, y1 = -1e300;
        for (const Vec2 q : wall) {
            x0_ = std::min(x0_, q.x);
            y0_ = std::min(y0_, q.y);
            x1 = std::max(x1, q.x);
            y1 = std::max(y1, q.y);
        }
        nx_ = static_cast<int>((x1 - x0_) / cell_) + 1;
        ny_ = static_cast<int>((y1 - y0_) / cell_) + 1;
        cells_.resize(static_cast<std::size_t>(nx_ * ny_));
        for (std::size_t e = 0, n = wall.size(); e < n; ++e) {
            const Vec2 a = wall[e], b = wall[(e + 1) % n];
            for (int j = row(std::min(a.y, b.y)); j <= row(std::max(a.y, b.y)); ++j) {
                for (int i = col(std::min(a.x, b.x)); i <= col(std::max(a.x, b.x)); ++i) {
                    cells_[static_cast<std::size_t>(j * nx_ + i)].push_back(e);
                }
            }
        }
    }

    /// Smallest distance from segment ab to the wall, if below `bound`; `bound` otherwise.
    double segDistBelow(Vec2 a, Vec2 b, double bound) const {
        double best = bound;
        for (int j = row(std::min(a.y, b.y) - bound); j <= row(std::max(a.y, b.y) + bound); ++j) {
            for (int i = col(std::min(a.x, b.x) - bound); i <= col(std::max(a.x, b.x) + bound); ++i) {
                for (const std::size_t e : cells_[static_cast<std::size_t>(j * nx_ + i)]) {
                    best = std::min(best, orc::segSegDist(a, b, wall_[e], wall_[(e + 1) % wall_.size()]));
                }
            }
        }
        return best;
    }

   private:
    int col(double x) const {
        return std::clamp(static_cast<int>(std::floor((x - x0_) / cell_)), 0, nx_ - 1);
    }
    int row(double y) const {
        return std::clamp(static_cast<int>(std::floor((y - y0_) / cell_)), 0, ny_ - 1);
    }

    const orc::Poly& wall_;
    double cell_;
    double x0_, y0_;
    int nx_{1}, ny_{1};
    std::vector<std::vector<std::size_t>> cells_;
};

struct Shape {
    std::string name;
    orc::Poly poly;
    std::vector<double> diameters;
    double chordal{0.01};
};

std::vector<double> scan(double lo, double hi, int n) {
    std::vector<double> out;
    for (int k = 0; k < n; ++k) {
        out.push_back(lo + (hi - lo) * k / (n - 1));
    }
    return out;
}

}  // namespace

TEST_CASE("T-0125 regression: the lobe of gap_poly is kept and cut at d = 2.029 and 1.746",
          "[toolpath][pocket][centre]") {
    const orc::Poly poly = gapPoly();
    REQUIRE(poly.size() == 1200);
    const Contour input = Contour::fromPolyline(Polyline{poly, true});
    for (double d : {2.0289999999998996, 1.7462, 1.745, 1.75}) {
        INFO("d " << d);
        PocketParams p;
        p.cutDepth = 1.0;
        p.strategy = PocketStrategy::Raster;
        p.profilePass = ProfilePass::First;
        PocketTool t;
        t.tool.name = "T";
        t.tool.diameter = d;
        t.stepover = d / 2;
        t.passDepth = 1.0;
        const PocketResult res = pocketToolpath({input}, {t}, p);
        CHECK(res.messages.empty());
        CHECK(res.plans[0].centreRegion.size() == 3);
        // Coverage of the lobe around (-8.0, -2.65): every cell the tool can reach there (a valid
        // centre within r - band) is swept.
        const double r = d / 2, m = p.tolerance.chordal, band = 0.02, h = 0.05;
        orc::Grid swept(-10.0, -4.5, -6.0, -0.8, h);
        orc::sweepFloor(swept, res.toolpaths[0], r, -1.0);
        // Valid centres (exact distance >= r + margin + band) on the same grid, computed once.
        orc::Grid valid(-10.0, -4.5, -6.0, -0.8, h);
        for (int j = 0; j < valid.ny; ++j) {
            for (int i = 0; i < valid.nx; ++i) {
                valid.at(i, j) =
                    static_cast<std::uint8_t>(signedDist(poly, valid.centre(i, j)) >= r + m + band);
            }
        }
        const int win = static_cast<int>(std::ceil(r / h));
        std::size_t judged = 0, missed = 0;
        for (int j = 0; j < swept.ny; ++j) {
            for (int i = 0; i < swept.nx; ++i) {
                const Vec2 q = swept.centre(i, j);
                if (signedDist(poly, q) <= 0.0) {
                    continue;
                }
                bool reachable = false;
                for (int dj = -win; dj <= win && !reachable; ++dj) {
                    for (int di = -win; di <= win && !reachable; ++di) {
                        const int ii = i + di, jj = j + dj;
                        if (ii < 0 || jj < 0 || ii >= valid.nx || jj >= valid.ny || !valid.at(ii, jj)) {
                            continue;
                        }
                        const Vec2 c = valid.centre(ii, jj);
                        reachable = std::hypot(c.x - q.x, c.y - q.y) <= r - band;
                    }
                }
                if (reachable) {
                    ++judged;
                    missed += swept.at(i, j) ? 0u : 1u;
                }
            }
        }
        CHECK(judged > 500);
        CHECK(missed == 0);
    }
}

TEST_CASE("T-0125 regression: Clipper spike vertices no longer cost a whole region",
          "[toolpath][pocket][centre]") {
    // Verifier round-2 repro (spike_poly.txt, a disc union): two zero-width out-and-back spike
    // vertices overshot the slack by ~5.4e-3 (more than one join tolerance), and the guard
    // reported and discarded a 76 mm^2 region around (17.31, 1.50), 5.49 mm from every wall.
    const orc::Poly poly = fixturePoly("spike_poly.txt");
    REQUIRE(poly.size() >= 238);
    const Contour input = Contour::fromPolyline(Polyline{poly, true});
    const Vec2 deep{17.3083, 1.5044};
    REQUIRE(signedDist(poly, deep) > 5.4);
    for (double d : {5.0, 5.04, 5.0673, 5.09, 5.10}) {
        INFO("d " << d);
        PocketParams p;
        std::vector<std::string> messages;
        const std::vector<Contour> centre = pocketCentreRegion({input}, d, p, &messages);
        CHECK(messages.empty());
        const std::vector<orc::Poly> cp = polysOf(centre);
        CHECK(orc::insideEvenOdd(cp, deep));
        for (const orc::Poly& ring : cp) {
            for (const Vec2 v : ring) {
                CHECK(signedDist(poly, v) >=
                      d / 2 + p.tolerance.chordal - (0.5 * p.tolerance.chordal + 2e-6));
            }
        }
    }
}

TEST_CASE("T-0126 regression: Clipper slivers of a collapsing knob are not reported as lost",
          "[toolpath][pocket][centre]") {
    // T-0126 verifier-style scan (knobs N=2, seed 7852, fixture knob_poly.txt): where a knob
    // collapses, Clipper2 leaves dozens of zero-area sliver regions. With the depth test (one
    // vertex at full depth) a sliver that could not be made valid went to the inset repair,
    // vanished and was reported as "could not be kept" although it holds nothing.
    const orc::Poly poly = fixturePoly("knob_poly.txt");
    REQUIRE(poly.size() == 1102);
    const Contour input = Contour::fromPolyline(Polyline{poly, true});
    PocketParams p;
    const WallIndex walls(poly, 0.5);
    for (double d : {6.696687, 6.69, 6.70, 6.71}) {
        INFO("d " << d);
        std::vector<std::string> messages;
        const std::vector<Contour> centre = pocketCentreRegion({input}, d, p, &messages);
        CHECK(messages.empty());
        CHECK(!centre.empty());
        const double bound = d / 2 + p.tolerance.chordal - (0.5 * p.tolerance.chordal + 2e-6);
        for (const orc::Poly& ring : polysOf(centre)) {
            for (std::size_t i = 0, n = ring.size(); i < n; ++i) {
                CHECK(walls.segDistBelow(ring[i], ring[(i + 1) % n], bound) >= bound);
            }
        }
    }
}

TEST_CASE("T-0125 property: no reachable part of the pocket is dropped across 4,000+ tool diameters",
          "[toolpath][pocket][centre]") {
    std::vector<Shape> shapes;
    {
        Shape s{"gap_poly (verifier chain scene 3150371)", gapPoly(), scan(1.0, 3.0, 601)};
        for (double d : {2.0289999999998996, 1.7462, 1.745, 1.75}) {
            s.diameters.push_back(d);
        }
        shapes.push_back(s);
    }
    {
        orc::Poly flower;
        for (int i = 0; i < 360; ++i) {
            const double t = 2 * kPi * i / 360, rr = 12.0 * (1 + 0.55 * std::cos(5 * t + 0.3));
            flower.push_back({rr * std::cos(t), rr * std::sin(t)});
        }
        shapes.push_back({"flower", flower, scan(0.5, 14.0, 1000)});
    }
    {
        // Chain of discs (union polygonised once; the oracle uses the same polygon as input).
        std::vector<Contour> discs;
        const double cx[] = {0, 7, 13.5, 20, 26}, cy[] = {0, 2, -1, 2, 0}, cr[] = {5, 3.2, 4.4, 2.9, 4};
        for (int i = 0; i < 5; ++i) {
            discs.push_back(Contour::circle({cx[i], cy[i]}, cr[i]));
        }
        const std::vector<Contour> u = carvetto::geometry::unionContours(discs);
        REQUIRE(u.size() == 1);
        shapes.push_back({"disc chain", u[0].polygonise(carvetto::geometry::Tolerance::defaults()).points,
                          scan(0.5, 9.5, 1000)});
    }
    {
        orc::Poly star;
        for (int i = 0; i < 24; ++i) {
            const double t = kPi * i / 12, rr = i % 2 == 0 ? 16.0 : 6.5;
            star.push_back({rr * std::cos(t), rr * std::sin(t)});
        }
        shapes.push_back({"star", star, scan(0.5, 12.5, 1000)});
    }

    {
        // Verifier round-2 spike repro (disc union, 238 vertices).
        Shape s{"spike_poly (zero-width Clipper spikes)", fixturePoly("spike_poly.txt"), scan(4.0, 7.0, 301)};
        for (double d : {5.04, 5.0673}) {
            s.diameters.push_back(d);
        }
        shapes.push_back(s);
    }
    {
        // T-0126: knob collapse with Clipper sliver regions (fixture knob_poly.txt).
        Shape s{"knob_poly (collapsing knobs)", fixturePoly("knob_poly.txt"), scan(6.0, 7.5, 151)};
        s.diameters.push_back(6.696687);
        shapes.push_back(s);
    }
    for (std::uint64_t seed : {301ULL, 700ULL, 606ULL}) {
        for (double chordal : {0.01, 0.1, 0.5}) {
            const orc::Poly chain = discChain(0x7125'0000ULL + seed);
            REQUIRE(chain.size() > 100);
            shapes.push_back({"disc chain " + std::to_string(seed) + " chordal " + std::to_string(chordal),
                              chain, scan(1.0, 9.0, 150), chordal});
        }
    }

    std::size_t diameters = 0, tooClose = 0;
    for (const Shape& s : shapes) {
        const Contour input = Contour::fromPolyline(Polyline{s.poly, true});
        const auto centres = lobeCentres(s.poly, 0.2);
        REQUIRE(centres.size() >= 2);
        std::size_t lobeChecks = 0;
        const WallIndex walls(s.poly, 0.5);
        PocketParams p;
        p.tolerance.chordal = s.chordal;
        const double m = p.tolerance.chordal;
        for (std::size_t k = 0; k < s.diameters.size(); ++k) {
            const double d = s.diameters[k], r = d / 2;
            INFO(s.name << " d " << d);
            std::vector<std::string> messages;
            const std::vector<Contour> centre = pocketCentreRegion({input}, d, p, &messages);
            CHECK(messages.empty());
            const std::vector<orc::Poly> cp = polysOf(centre);
            // Every lobe that clearly holds the tool is part of the centre region.
            for (const auto& [q, depth] : centres) {
                if (depth >= r + m + 0.02) {
                    ++lobeChecks;
                    CHECK(orc::insideEvenOdd(cp, q));
                }
            }
            // Every diameter: no edge too close to a wall (T-0126; exact segment distance, so
            // a chord sagging between two good vertices is caught too).
            const double bound = r + m - (0.5 * m + 2e-6);
            for (const orc::Poly& ring : cp) {
                for (std::size_t i = 0, n = ring.size(); i < n; ++i) {
                    const double dist = walls.segDistBelow(ring[i], ring[(i + 1) % n], bound);
                    if (dist < bound) {
                        ++tooClose;
                        FAIL_CHECK("edge " << i << " (" << ring[i].x << ", " << ring[i].y << ") at " << dist
                                           << " < " << bound);
                    }
                }
            }
            // Every 25th diameter: no vertex outside the pocket (no phantom kept).
            if (k % 25 == 0) {
                for (const orc::Poly& ring : cp) {
                    for (const Vec2 v : ring) {
                        CHECK(signedDist(s.poly, v) > 0.0);
                    }
                }
            }
            // Every 200th: the full toolpath plans exactly this region and cuts it.
            if (k % 200 == 0) {
                PocketTool t;
                t.tool.name = "T";
                t.tool.diameter = d;
                t.stepover = d / 2;
                t.passDepth = 1.0;
                PocketParams pp = p;
                pp.cutDepth = 1.0;
                pp.strategy = PocketStrategy::Raster;
                const PocketResult res = pocketToolpath({input}, {t}, pp);
                CHECK(res.plans[0].centreRegion == centre);
                CHECK(res.toolpaths[0].isEmpty() == centre.empty());
            }
            ++diameters;
        }
        INFO(s.name);
        CHECK(2 * lobeChecks > s.diameters.size());  // most diameters fit at least one lobe
    }
    CHECK(diameters >= 5000);
    CHECK(tooClose == 0);
}
