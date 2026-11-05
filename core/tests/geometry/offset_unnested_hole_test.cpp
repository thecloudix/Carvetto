// T-0142 b: an outset of rings wound against the outermost one that are not nested inside a ring
// wound like it (a "hole" with nothing around it) used to leave Clipper2's corner loops behind as
// tiny leftover rings, and an over-shrunk one a whole phantom region (ADR-0006 §10). Such rings
// describe no region, so the outset of the region must not show them.
//
// The generative oracle is independent of the offset and boolean code: polygons are vertex lists,
// the input region is decided by winding numbers from a nesting tree the generator knows, and
// distances are point/segment formulas.
#include <algorithm>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/offset.hpp"
#include "test_rng.hpp"
// Last: on Windows it includes <windows.h>.
#include "cpu_scaling.hpp"

using namespace carvetto::geometry;
using Catch::Approx;

namespace {

using Ring = std::vector<Vec2>;

Contour polygon(const Ring& pts) {
    Contour c;
    c.closed = true;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        c.spans.emplace_back(LineSpan{pts[i], pts[(i + 1) % pts.size()]});
    }
    return c;
}

Ring ringOf(const Contour& c) {
    Ring r;
    for (const Span& s : c.spans) {
        r.push_back(startPoint(s));
    }
    return r;
}

double signedArea(const Ring& r) {
    double a = 0.0;
    for (std::size_t i = 0, n = r.size(); i < n; ++i) {
        a += r[i].cross(r[(i + 1) % n]);
    }
    return 0.5 * a;
}

// Signed winding number of p about the ring.
int winding(const Ring& r, Vec2 p) {
    int w = 0;
    for (std::size_t i = 0, n = r.size(); i < n; ++i) {
        const Vec2 a = r[i];
        const Vec2 b = r[(i + 1) % n];
        const double c = (b - a).cross(p - a);
        if (a.y <= p.y) {
            if (b.y > p.y && c > 0.0) {
                ++w;
            }
        } else if (b.y <= p.y && c < 0.0) {
            --w;
        }
    }
    return w;
}

double segDist(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 d = b - a;
    const double l2 = d.dot(d);
    const double t = l2 > 0.0 ? std::clamp((p - a).dot(d) / l2, 0.0, 1.0) : 0.0;
    return distance(p, a + d * t);
}

double ringDist(const Ring& r, Vec2 p) {
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0, n = r.size(); i < n; ++i) {
        best = std::min(best, segDist(p, r[i], r[(i + 1) % n]));
    }
    return best;
}

Ring regularPolygon(Vec2 c, double radius, int n, double phase, bool ccw) {
    Ring r;
    for (int k = 0; k < n; ++k) {
        const double a = phase + 6.283185307179586 * k / n;
        r.push_back(c + Vec2{radius * std::cos(a), radius * std::sin(a)});
    }
    if (!ccw) {
        std::reverse(r.begin(), r.end());
    }
    return r;
}

// Input rings that never cross, with the sum of the orientations (+1 CCW, -1 CW) of the rings
// around each one.
struct Scene {
    std::vector<Ring> rings;
    std::vector<int> outside;  // winding just outside ring i
    bool reversed{false};      // the lowest ring (largest y) is clockwise
};

int orientationOf(const Ring& r) {
    return signedArea(r) > 0.0 ? 1 : -1;
}

Scene makeScene(carvetto::test::SplitMix& rng) {
    Scene sc;
    auto add = [&sc](Ring r, int outside) {
        sc.rings.push_back(std::move(r));
        sc.outside.push_back(outside);
    };
    const int nx = 1 + static_cast<int>(rng.uniform(0.0, 4.0));
    const int ny = 1 + static_cast<int>(rng.uniform(0.0, 3.0));
    const double pitch = 10.0;
    for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < ny; ++j) {
            if (rng.uniform(0.0, 1.0) < 0.15) {
                continue;
            }
            const Vec2 c{pitch * i, pitch * j};
            const int n = 3 + static_cast<int>(rng.uniform(0.0, 46.0));
            const double r = rng.uniform(0.3, 3.0);
            const Ring root = regularPolygon(c, r, n, rng.uniform(0.0, 6.3), rng.uniform(0.0, 1.0) < 0.5);
            const int sRoot = orientationOf(root);
            add(root, 0);
            const double in = r * std::cos(3.141592653589793 / n);  // inradius
            const int kids = static_cast<int>(rng.uniform(0.0, 3.0));
            if (kids == 1) {
                const int m = 3 + static_cast<int>(rng.uniform(0.0, 30.0));
                const double rc = in * rng.uniform(0.2, 0.8);
                const Ring child = regularPolygon(c, rc, m, rng.uniform(0.0, 6.3), rng.uniform(0.0, 1.0) < 0.5);
                const int sChild = orientationOf(child);
                add(child, sRoot);
                if (rng.uniform(0.0, 1.0) < 0.5) {
                    const double ri = rc * std::cos(3.141592653589793 / m) * rng.uniform(0.2, 0.8);
                    add(regularPolygon(c, ri, 3 + static_cast<int>(rng.uniform(0.0, 20.0)), rng.uniform(0.0, 6.3),
                                       rng.uniform(0.0, 1.0) < 0.5),
                        sRoot + sChild);
                }
            } else if (kids == 2) {
                for (const double side : {-0.5, 0.5}) {
                    add(regularPolygon(c + Vec2{side * in, 0.0}, in * rng.uniform(0.1, 0.4),
                                       3 + static_cast<int>(rng.uniform(0.0, 20.0)), rng.uniform(0.0, 6.3),
                                       rng.uniform(0.0, 1.0) < 0.5),
                        sRoot);
                }
            }
        }
    }
    // A lid above everything decides the orientation (Clipper2's lowest path has the largest y).
    const double lidY = pitch * ny + 5.0;
    Ring lid{{-2.0, lidY}, {pitch * nx, lidY}, {pitch * nx, lidY + 2.0}, {-2.0, lidY + 2.0}};
    sc.reversed = rng.uniform(0.0, 1.0) < 0.3;
    if (sc.reversed) {
        std::reverse(lid.begin(), lid.end());
    }
    add(lid, 0);
    return sc;
}

}  // namespace

TEST_CASE("T-0142 b: a lone clockwise ring under a counter-clockwise lowest contour leaves nothing on an outset",
          "[geometry][offset][T-0142]") {
    for (const JoinType join : {JoinType::Round, JoinType::Square, JoinType::Sharp}) {
        for (const double chordal : {0.01, 0.001, 0.0001}) {
            for (const double d : {0.3, 1.0, 1.5}) {  // 1.5 over-shrinks the 1 mm circle
                // The card's repro: a 1 mm circle wound clockwise, a CCW rectangle above it.
                const std::vector<Contour> cs{Contour::rectangle(Box2{{0, 10}, {10, 20}}),
                                              Contour::circle({5, 5}, 1.0, ArcDirection::CW)};
                const std::vector<Contour> out = offsetContours(cs, d, join, Tolerance{0.001, 1e-9, chordal}, 2.0);
                INFO("join " << static_cast<int>(join) << " chordal " << chordal << " d " << d);
                REQUIRE(out.size() == 1);
                CHECK(out[0].bounds().min.y == Approx(10.0 - d).margin(1e-5));
                // Mirrored: a CW lowest contour and a lone CCW ring.
                const std::vector<Contour> mirrored{cs[0].reversed(), cs[1].reversed()};
                const std::vector<Contour> outM = offsetContours(mirrored, d, join, Tolerance{0.001, 1e-9, chordal}, 2.0);
                REQUIRE(outM.size() == 1);
                CHECK(std::abs(area(outM[0])) == Approx(std::abs(area(out[0]))));
            }
        }
    }
    // A hole inside its outer is untouched by the fix (one call, as before): it shrinks.
    const std::vector<Contour> holed{Contour::rectangle(Box2{{0, 0}, {10, 10}}),
                                     Contour::circle({5, 5}, 2.0, ArcDirection::CW)};
    const std::vector<Contour> out = offsetContours(holed, 0.5, JoinType::Round, Tolerance::defaults());
    REQUIRE(out.size() == 2);
    CHECK(std::min(std::abs(area(out[0])), std::abs(area(out[1]))) == Approx(3.14159265 * 1.5 * 1.5).epsilon(0.01));
}

TEST_CASE("T-0142 b: outsets of rings with lone and nested holes keep exactly the region's neighbourhood",
          "[geometry][offset][T-0142]") {
    carvetto::test::SplitMix rng{1422};
    const int scenes = 120;
    int loneHoles = 0;
    for (int s = 0; s < scenes; ++s) {
        const Scene sc = makeScene(rng);
        const double d = rng.uniform(0.05, 1.8);
        const double chordal = rng.uniform(0.0, 1.0) < 0.5 ? 0.01 : 0.002;
        const JoinType join = s % 3 == 0 ? JoinType::Round : (s % 3 == 1 ? JoinType::Square : JoinType::Sharp);
        const double mitre = 2.0;
        const double reach = join == JoinType::Round ? 1.0 : (join == JoinType::Square ? std::sqrt(2.0) : mitre);
        const double slack = 2.0 * chordal + 1e-5;
        const int rel = sc.reversed ? -1 : 1;
        auto inRegion = [&](int w) { return rel * w > 0; };
        // Rings whose two sides differ in membership bound the input region.
        std::vector<const Ring*> boundary;
        for (std::size_t i = 0; i < sc.rings.size(); ++i) {
            const int w0 = sc.outside[i];
            const int w1 = w0 + orientationOf(sc.rings[i]);
            if (inRegion(w0) != inRegion(w1)) {
                boundary.push_back(&sc.rings[i]);
            }
            if (rel * orientationOf(sc.rings[i]) < 0 && !inRegion(w0)) {
                ++loneHoles;  // a hole with nothing around it
            }
        }
        auto inputWinding = [&](Vec2 p) {
            int w = 0;
            for (const Ring& r : sc.rings) {
                w += winding(r, p);
            }
            return w;
        };
        auto distToRegion = [&](Vec2 p) {
            if (inRegion(inputWinding(p))) {
                return 0.0;
            }
            double best = std::numeric_limits<double>::infinity();
            for (const Ring* r : boundary) {
                best = std::min(best, ringDist(*r, p));
            }
            return best;
        };
        std::vector<Contour> input;
        for (const Ring& r : sc.rings) {
            input.push_back(polygon(r));
        }
        const std::vector<Contour> out = offsetContours(input, d, join, Tolerance{0.001, 1e-9, chordal}, mitre);
        std::vector<Ring> res;
        for (const Contour& c : out) {
            res.push_back(ringOf(c));
        }
        // The result is a set of disjoint outers and holes, wound either way (Clipper2's output
        // orientation follows its fill rule): a point is in it when its winding is not zero.
        auto inResult = [&](Vec2 p) {
            int w = 0;
            for (const Ring& r : res) {
                w += winding(r, p);
            }
            return w != 0;
        };
        INFO("scene " << s << " d " << d << " join " << static_cast<int>(join) << " reversed " << sc.reversed);
        // 1. Every outer result ring holds points within reach x d of the region (no leftovers).
        for (const Ring& r : res) {
            // A point just inside the ring: off the middle of its longest edge, towards its inside.
            std::size_t longest = 0;
            for (std::size_t i = 0; i < r.size(); ++i) {
                if (distance(r[i], r[(i + 1) % r.size()]) > distance(r[longest], r[(longest + 1) % r.size()])) {
                    longest = i;
                }
            }
            const Vec2 a = r[longest];
            const Vec2 b = r[(longest + 1) % r.size()];
            const double len = distance(a, b);
            const Vec2 inward = (b - a).perp() * (1.0 / len) * (signedArea(r) > 0.0 ? 1.0 : -1.0);
            const Vec2 q = (a + b) * 0.5 + inward * std::min(1e-3, 0.01 * len);
            if (inResult(q)) {  // an outer (just inside a hole ring the result is empty)
                CHECK(distToRegion(q) <= reach * d + slack);
            }
        }
        // 2. Points near the region are in the result, points far from it are not.
        for (int k = 0; k < 150; ++k) {
            const Vec2 p{rng.uniform(-6.0, 10.0 * 4 + 6.0), rng.uniform(-6.0, 10.0 * 3 + 12.0)};
            const double dist = distToRegion(p);
            if (dist <= d - slack) {
                CHECK(inResult(p));
            } else if (dist > reach * d + slack) {
                CHECK_FALSE(inResult(p));
            }
        }
    }
    CHECK(loneHoles > scenes / 4);
}

TEST_CASE("T-0142 b: outsets of many shapes with nested holes stay near-linear", "[geometry][offset][T-0142][perf]") {
    // Round 1 tested each hole against every ring wound like the lowest (row of 40,000 -> 80,000
    // holed rectangles 2.0 -> 9.9 s, main 0.39 -> 0.78 s); the scan now runs per batch of boxes.
#ifdef NDEBUG
    const int nSmall = 20000;
#else
    const int nSmall = 3000;
#endif
    auto scene = [](int n, bool grid) {
        std::vector<Contour> cs;
        const int k = static_cast<int>(std::sqrt(static_cast<double>(n)));
        for (int i = 0; i < n; ++i) {
            const double x = 2.0 * (grid ? i % k : i);
            const double y = grid ? 2.0 * (i / k) : 0.0;
            cs.push_back(Contour::rectangle(Box2{{x, y}, {x + 1.0, y + 1.0}}));
            cs.push_back(Contour::rectangle(Box2{{x + 0.3, y + 0.3}, {x + 0.7, y + 0.7}}).reversed());
        }
        return cs;
    };
    const Tolerance tol{0.001, 1e-9, 0.001};
    for (const bool grid : {false, true}) {
        const std::vector<Contour> a = scene(nSmall, grid);
        const std::vector<Contour> b = scene(2 * nSmall, grid);
        std::size_t ra = 0;
        std::size_t rb = 0;
        const carvetto::test::ScalingTimes t = carvetto::test::interleavedCpuSeconds(
            3, [&] { ra = offsetContours(a, 0.05, JoinType::Round, tol).size(); },
            [&] { rb = offsetContours(b, 0.05, JoinType::Round, tol).size(); });
        INFO((grid ? "grid" : "row") << " CPU seconds: " << nSmall << " shapes " << t.smallSeconds << ", "
                                     << 2 * nSmall << " shapes " << t.largeSeconds << ", ratio " << t.ratio());
        CHECK(ra == a.size());  // every shape and its (shrunk) hole
        CHECK(rb == b.size());
        CHECK(t.ratio() <= 2.5);
    }
}
