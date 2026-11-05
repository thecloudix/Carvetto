// T-0158: a closed cut asked to start nearest a point that is the centre of one of its arcs (a
// convex tip's round join on an Outside cut, a reflex vertex's join on an Inside cut, a circle
// round its bounding-box centre) has every point of that arc equally near. The start used to follow
// the direction of a rounding-sized vector (target minus arc centre), so GCC and MSVC chose
// different starts and tab layouts (harness m1_402002_27, m2_800016_13). Such a tie now starts at
// the beginning of the arc (ADR-0016 §11), whatever the rounding. These tests perturb scenes by a
// few ulps and by up to 1e-9 relative and check that the start does not move.

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <variant>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/move.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "../geometry/test_rng.hpp"
#include "profile_test_util.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using carvetto::geometry::ArcDirection;
using carvetto::geometry::ArcSpan;
using carvetto::geometry::Contour;
using carvetto::geometry::LineSpan;
using carvetto::geometry::Vec2;

namespace {

// Scale factors: a few ulps either side of 1 and relative perturbations up to 1e-9.
std::vector<double> perturbations() {
    std::vector<double> out;
    double up = 1.0;
    double down = 1.0;
    for (int k = 0; k < 4; ++k) {
        up = std::nextafter(up, 2.0);
        down = std::nextafter(down, 0.0);
        out.push_back(up);
        out.push_back(down);
    }
    for (double rel : {1e-12, 1e-10, 1e-9}) {
        out.push_back(1.0 + rel);
        out.push_back(1.0 - rel);
    }
    return out;
}

Contour scaled(Contour c, double s) {
    for (geometry::Span& sp : c.spans) {
        if (auto* l = std::get_if<LineSpan>(&sp)) {
            l->start = l->start * s;
            l->end = l->end * s;
        } else if (auto* a = std::get_if<ArcSpan>(&sp)) {
            a->centre = a->centre * s;
            a->radius *= s;
        }
    }
    return c;
}

// Edge from a to b: a line, or an arc through a and b whose sagitta is `sag` times the chord
// (positive bulges to the right of travel, i.e. outward on a counter-clockwise contour). An arc's
// ends are recomputed from its angles, so neighbouring spans meet only to rounding: the vertex is
// then a few ulps off the centre of the offset's round join there.
geometry::Span edge(Vec2 a, Vec2 b, double sag) {
    if (sag == 0.0) {
        return LineSpan{a, b};
    }
    const Vec2 d = b - a;
    const double half = 0.5 * d.length();
    const Vec2 left = Vec2{-d.y, d.x} * (1.0 / d.length());
    const double s = sag * 2.0 * half;
    const double t = (half * half - s * s) / (2.0 * s);
    const Vec2 c = (a + b) * 0.5 + left * t;
    return ArcSpan::fromCentre(c, a, b, s > 0.0 ? ArcDirection::CCW : ArcDirection::CW);
}

struct Star {
    Contour contour;
    Vec2 centre;
    Vec2 start;  // the vector's start point (a tip, or a reflex vertex for Inside)
};

// A counter-clockwise star of `n` points, edges lines or gently curved arcs, starting at a tip
// (`atTip`) or at the reflex vertex after it. The last line edge may end a few ulps off the start
// (it still closes within the tolerance).
Star curvedStar(test::SplitMix& rng, bool atTip) {
    const int n = 4 + static_cast<int>(rng.next() % 4);
    const double rOut = rng.uniform(15.0, 40.0);
    const double rIn = rOut * rng.uniform(0.5, 0.7);
    const double rot = rng.uniform(0.0, 2.0 * geometry::kPi);
    const Vec2 centre{rng.uniform(-60.0, 60.0), rng.uniform(-60.0, 60.0)};
    std::vector<Vec2> v;
    for (int j = 0; j < 2 * n; ++j) {
        const double ang = rot + j * geometry::kPi / n;
        v.push_back(centre + Vec2::fromAngle(ang, j % 2 == 0 ? rOut : rIn));
    }
    const std::size_t first = atTip ? 0 : 1;
    Star s;
    s.centre = centre;
    s.contour.closed = true;
    for (std::size_t k = 0; k < v.size(); ++k) {
        const Vec2 a = v[(first + k) % v.size()];
        Vec2 b = v[(first + k + 1) % v.size()];
        const bool curved = rng.next() % 2 == 0;
        const double sag = curved ? rng.uniform(-0.04, 0.04) : 0.0;
        if (k + 1 == v.size() && sag == 0.0) {
            const int ulps = static_cast<int>(rng.next() % 7) - 3;
            for (int u = 0; u < std::abs(ulps); ++u) {
                b.x = std::nextafter(b.x, ulps > 0 ? 1e9 : -1e9);
                b.y = std::nextafter(b.y, ulps > 0 ? -1e9 : 1e9);
            }
        }
        s.contour.spans.push_back(edge(a, b, sag));
    }
    s.start = geometry::startPoint(s.contour.spans.front());
    return s;
}

ProfileParams params(test::SplitMix& rng, ProfileSide side) {
    ProfileParams p;
    p.side = side;
    p.direction = rng.next() % 2 == 0 ? CutDirection::Climb : CutDirection::Conventional;
    p.passes.cutDepth = 6.0;
    p.passes.passDepth = 3.0;
    p.safeZ = 5.0;
    p.startAt = StartAt::VectorStart;
    return p;
}

// The cut's start (the first rapid's XY) and the first move that leaves it in XY.
struct Start {
    Vec2 at;
    Move first;
    bool found{false};
};

Start startOf(const Toolpath& tp) {
    Start s;
    if (tp.moves.empty()) {
        return s;
    }
    s.at = tp.moves.front().to.xy();
    for (const Move& m : tp.moves) {
        if ((m.kind == MoveKind::Feed || isArc(m.kind)) && geometry::distance(m.to.xy(), s.at) > 1e-9) {
            s.first = m;
            s.found = true;
            break;
        }
    }
    return s;
}

std::size_t tabsLeftOut(const ProfileResult& r) {
    std::size_t n = 0;
    for (const ProfileDiagnostic& d : r.diagnostics) {
        if (d.issue == ProfileIssue::TabsNotPlaced &&
            d.message.find(" tab(s) left out") != std::string::npos) {
            n += static_cast<std::size_t>(std::stoul(d.message.substr(d.message.find(": ") + 2)));
        }
    }
    return n;
}

}  // namespace

TEST_CASE("T-0158 AC1 generative: a cut asked to start at a tip starts at the beginning of the tip's "
          "corner arc, unmoved by ulp and 1e-9 perturbations",
          "[toolpath][profile][start]") {
    test::SplitMix rng{0x7158'0001ULL};
    const std::vector<double> scales = perturbations();
    std::size_t checked = 0;
    for (int scene = 0; scene < 48; ++scene) {
        const bool inside = scene % 3 == 2;
        const Star star = curvedStar(rng, !inside);
        ProfileParams p = params(rng, inside ? ProfileSide::Inside : ProfileSide::Outside);
        const double d = inside ? rng.uniform(1.0, 3.0) : rng.uniform(2.0, 10.0);
        p.allowance = rng.uniform(-0.2, 1.0) * 0.5 * d;
        p.separateLastPass = rng.next() % 3 == 0;
        p.lastPassAllowance = rng.uniform(0.1, 0.5);
        const double dist = 0.5 * d + p.allowance + (p.separateLastPass ? p.lastPassAllowance : 0.0);
        INFO("scene " << scene << (inside ? " Inside" : " Outside") << " d " << d << " allowance "
                      << p.allowance << " separate " << p.separateLastPass);

        const ProfileResult base = profileToolpath({star.contour}, test::endMill(d), p);
        REQUIRE(base.ok());
        const Start s0 = startOf(base.toolpath);
        REQUIRE(s0.found);
        // The (first) cut starts on its round join about the vertex and runs along it first.
        CHECK(std::fabs(geometry::distance(s0.at, star.start) - dist) < 1e-6);
        REQUIRE(isArc(s0.first.kind));
        CHECK(geometry::distance(s0.first.centre, star.start) < 1e-6);
        CHECK(std::fabs(geometry::distance(s0.first.to.xy(), star.start) - dist) < 1e-6);

        for (double k : scales) {
            INFO("scale 1 " << (k >= 1.0 ? "+ " : "- ") << std::fabs(k - 1.0));
            const ProfileResult r = profileToolpath({scaled(star.contour, k)}, test::endMill(d), p);
            REQUIRE(r.ok());
            const Start s = startOf(r.toolpath);
            REQUIRE(s.found);
            // Still the start of the round join about the (scaled) vertex, at the same end of it:
            // the join itself comes from the offset and can move by a few µm (near-tangent
            // intersections), far less than any other point of the arc would.
            CHECK(std::fabs(geometry::distance(s.at, star.start * k) - dist * k) < 1e-6);
            CHECK(geometry::distance(s.at, s0.at * k) < 1e-5);
            // The arc's far end comes from the offset (its join with the next span), which moves
            // by up to a few µm under these perturbations, well inside the distance tolerance.
            CHECK(isArc(s.first.kind));
            CHECK(geometry::distance(s.first.centre, s0.first.centre * k) < 1e-6);
            CHECK(geometry::distance(s.first.to.xy(), s0.first.to.xy() * k) < 1e-4);
            CHECK(r.toolpath.moves.size() == base.toolpath.moves.size());
            ++checked;
        }
    }
    CHECK(checked == 48 * scales.size());
}

TEST_CASE("T-0158: a circle started from its centre starts at its arc's start, whatever the rounding of "
          "the centre",
          "[toolpath][profile][start]") {
    // Bounding-box centre anchor and Optimise from the circle's centre: the anchor is the centre
    // only to rounding, and the start point nudged by ulps in every direction; the start is always
    // the arc's start (angle 0 of the full circle, ArcSpan::fullCircle).
    test::SplitMix rng{0x7158'0002ULL};
    for (int scene = 0; scene < 24; ++scene) {
        const Vec2 c{rng.uniform(-80.0, 80.0), rng.uniform(-80.0, 80.0)};
        const double radius = rng.uniform(4.0, 25.0);
        const bool inside = scene % 2 == 1;
        ProfileParams p = params(rng, inside ? ProfileSide::Inside : ProfileSide::Outside);
        const double d = rng.uniform(1.0, 6.0);
        const double dist = inside ? radius - 0.5 * d : radius + 0.5 * d;
        const Vec2 expected = c + Vec2{dist, 0.0};
        INFO("scene " << scene << " centre " << c.x << "," << c.y << " radius " << radius);
        const Contour circle = Contour::circle(c, radius);

        p.startAt = StartAt::ClosestOnBoundingBox;
        p.startAnchor = BoxAnchor::Centre;
        const ProfileResult anchored = profileToolpath({circle}, test::endMill(d), p);
        REQUIRE(anchored.ok());
        CHECK(geometry::distance(startOf(anchored.toolpath).at, expected) < 1e-6);

        p.startAt = StartAt::Optimise;
        for (int dir = 0; dir < 9; ++dir) {
            Vec2 from = c;
            for (int u = 0; u < 3; ++u) {
                const double inf = std::numeric_limits<double>::infinity();
                if (dir % 3 != 0) from.x = std::nextafter(from.x, dir % 3 == 1 ? inf : -inf);
                if (dir / 3 != 0) from.y = std::nextafter(from.y, dir / 3 == 1 ? inf : -inf);
            }
            INFO("from " << dir);
            p.startPosition = from;
            const ProfileResult r = profileToolpath({circle}, test::endMill(d), p);
            REQUIRE(r.ok());
            CHECK(geometry::distance(startOf(r.toolpath).at, expected) < 1e-6);
        }
    }
}

TEST_CASE("T-0158: tab layouts at tip starts are unmoved by ulp perturbations and the parts still hold "
          "(flood)",
          "[toolpath][profile][start][tabs]") {
    test::SplitMix rng{0x7158'0003ULL};
    const std::vector<double> ulps = [] {
        std::vector<double> out;
        double up = 1.0;
        double down = 1.0;
        for (int k = 0; k < 2; ++k) {
            up = std::nextafter(up, 2.0);
            down = std::nextafter(down, 0.0);
            out.push_back(up);
            out.push_back(down);
        }
        out.push_back(1.0 + 1e-9);
        out.push_back(1.0 - 1e-9);
        return out;
    }();
    for (int scene = 0; scene < 8; ++scene) {
        const Star star = curvedStar(rng, true);
        ProfileParams p = params(rng, ProfileSide::Outside);
        const double d = rng.uniform(3.0, 8.0);
        p.allowance = rng.uniform(-0.3, 0.5) * 0.5 * d;
        p.separateLastPass = rng.next() % 2 == 0;
        p.lastPassAllowance = rng.uniform(0.1, 0.4);
        p.tabs.enabled = true;
        p.tabs.count = 3 + static_cast<int>(rng.next() % 6);
        p.tabs.length = rng.uniform(2.0, 6.0);
        p.tabs.thickness = rng.uniform(1.0, 3.0);
        INFO("scene " << scene << " d " << d << " allowance " << p.allowance << " separate "
                      << p.separateLastPass << " tabs " << p.tabs.count);
        const ProfileResult base = profileToolpath({star.contour}, test::endMill(d), p);
        REQUIRE(base.ok());
        for (const ProfileDiagnostic& g : base.diagnostics) {
            INFO(g.message);
            CHECK(g.message.find("comes free") == std::string::npos);
        }
        // Independent raster flood at the tab top: the star's centre stays joined to the waste.
        const double zTab = p.passes.bottomZ() + std::min(p.tabs.thickness, p.passes.cutDepth);
        const geometry::Box2 window = star.contour.bounds().inflated(d + 2.0);
        CHECK(test::floodHeld(base.toolpath, window, zTab, 0.5 * d, star.centre,
                              window.min + Vec2{0.5, 0.5}, 0.1));
        for (double k : ulps) {
            INFO("scale 1 " << (k >= 1.0 ? "+ " : "- ") << std::fabs(k - 1.0));
            const ProfileResult r = profileToolpath({scaled(star.contour, k)}, test::endMill(d), p);
            REQUIRE(r.ok());
            CHECK(tabsLeftOut(r) == tabsLeftOut(base));
            CHECK(r.toolpath.moves.size() == base.toolpath.moves.size());
            CHECK(geometry::distance(startOf(r.toolpath).at, startOf(base.toolpath).at * k) < 1e-5);
        }
    }
}
