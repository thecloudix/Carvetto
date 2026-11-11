// T-0158: a closed cut asked to start nearest a point that is the centre of one of its arcs (a
// convex tip's round join on an Outside cut, a reflex vertex's join on an Inside cut, a circle
// round its bounding-box centre) has every point of that arc equally near. The start used to follow
// the direction of a rounding-sized vector (target minus arc centre), so GCC and MSVC chose
// different starts and tab layouts (harness m1_402002_27, m2_800016_13). Such a tie now starts at
// the beginning of the arc (ADR-0016 §11), whatever the rounding; a sharp (mitred) convex tip of a
// V-bit cut with sharp external corners, whose two offset lines are equally near, starts at the
// mitre's corner point (or, squared off beyond the mitre limit, at the chamfer's start). These
// tests perturb scenes by a few ulps and by up to 1e-9 relative and check that the start does not
// move.

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
        } else if (auto* b = std::get_if<geometry::BezierSpan>(&sp)) {
            b->p0 = b->p0 * s;
            b->p1 = b->p1 * s;
            b->p2 = b->p2 * s;
            b->p3 = b->p3 * s;
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
// (it still closes within the tolerance). `curves` false: line edges only.
Star curvedStar(test::SplitMix& rng, bool atTip, bool curves = true) {
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
        const bool curved = rng.next() % 2 == 0 && curves;
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

TEST_CASE("T-0158 AC1 generative: a V-bit cut with sharp external corners starts at the tip's mitre corner, "
          "unmoved by ulp and 1e-9 perturbations",
          "[toolpath][profile][start][vbit]") {
    // The mitre adds no span: the offset lines meet at the corner, and the feet of the tip on both
    // lines are equally near it, so the plain projection followed rounding (harness t0158v1,
    // mitre_001). Beyond the mitre limit the corner is squared off by a chamfer at the cut distance
    // from the tip; the start is then the chamfer's first corner.
    test::SplitMix rng{0x7158'0004ULL};
    const std::vector<double> scales = perturbations();
    std::size_t mitres = 0;
    std::size_t chamfers = 0;
    for (int scene = 0; scene < 40; ++scene) {
        const Star star = curvedStar(rng, true, false);
        ProfileParams p = params(rng, ProfileSide::Outside);
        tooling::Tool v = test::endMill(rng.uniform(12.0, 25.0));
        v.type = tooling::ToolType::VBit;
        v.includedAngle = rng.uniform(60.0, 120.0) * geometry::kPi / 180.0;
        p.passes.cutDepth = rng.uniform(1.0, 4.0);
        p.passes.passDepth = p.passes.cutDepth;
        p.corners.sharpExternal = true;
        p.corners.mitreLimit = rng.uniform(1.2, 4.0);
        p.allowance = rng.uniform(-0.3, 1.0);
        const double dist = profileToolRadius(v, p.passes) + p.allowance;
        // Tip geometry: unit directions from the tip along both edges, and the outward bisector.
        const Vec2 tip = star.start;
        const Vec2 along = geometry::endPoint(star.contour.spans.front()) - tip;
        const Vec2 back = geometry::startPoint(star.contour.spans.back()) - tip;
        const Vec2 u1 = along * (1.0 / along.length());
        const Vec2 u2 = back * (1.0 / back.length());
        const Vec2 out = (u1 + u2) * (-1.0 / (u1 + u2).length());
        const double sinHalf = std::sqrt(0.5 * (1.0 - u1.dot(u2)));
        const double mitreLength = dist / sinHalf;
        const bool mitred = mitreLength <= p.corners.mitreLimit * dist;
        INFO("scene " << scene << " dist " << dist << " mitre " << mitreLength / dist << " limit "
                      << p.corners.mitreLimit);

        const ProfileResult base = profileToolpath({star.contour}, v, p);
        REQUIRE(base.ok());
        const Start s0 = startOf(base.toolpath);
        REQUIRE(s0.found);
        CHECK_FALSE(isArc(s0.first.kind));
        // Distance from a point to the offset carrier of each edge at the tip.
        auto offLine = [&](Vec2 q, Vec2 u) { return std::fabs(std::fabs((q - tip).cross(u)) - dist); };
        if (mitred) {
            ++mitres;
            CHECK(geometry::distance(s0.at, tip + out * mitreLength) < 1e-6);
        } else {
            ++chamfers;
            // On the chamfer line (at `dist` from the tip across the bisector) and on one of the
            // offset lines: a corner of the chamfer, not its middle.
            CHECK(std::fabs((s0.at - tip).dot(out) - dist) < 1e-6);
            CHECK(std::min(offLine(s0.at, u1), offLine(s0.at, u2)) < 1e-6);
            CHECK(geometry::distance(s0.at, tip) > dist + 1e-3);
            // The first corner in travel: the cut runs along the chamfer to its other corner first.
            const Vec2 to = s0.first.to.xy();
            CHECK(s0.first.kind == MoveKind::Feed);
            CHECK(std::fabs((to - tip).dot(out) - dist) < 1e-6);
            const bool startOn1 = offLine(s0.at, u1) < offLine(s0.at, u2);
            CHECK((startOn1 ? offLine(to, u2) : offLine(to, u1)) < 1e-6);
        }
        for (double k : scales) {
            INFO("scale 1 " << (k >= 1.0 ? "+ " : "- ") << std::fabs(k - 1.0));
            const ProfileResult r = profileToolpath({scaled(star.contour, k)}, v, p);
            REQUIRE(r.ok());
            const Start s = startOf(r.toolpath);
            REQUIRE(s.found);
            CHECK(geometry::distance(s.at, s0.at * k) < 1e-5);
            CHECK(r.toolpath.moves.size() == base.toolpath.moves.size());
        }
    }
    // Both kinds of sharp corner are exercised.
    CHECK(mitres >= 8);
    CHECK(chamfers >= 8);
}

TEST_CASE("T-0158: a mitred loop of two spans (an arc closed by a line) starts at the start vertex's mitre",
          "[toolpath][profile][start][vbit]") {
    // Every span of the cut is equally near the start vertex (harness t0124v2/s90): the loop is
    // broken where the feet lie farthest apart, so the start is the mitre at the start vertex,
    // on both the offset line and the offset circle.
    test::SplitMix rng{0x7158'0005ULL};
    const std::vector<double> scales = perturbations();
    for (int scene = 0; scene < 16; ++scene) {
        const Vec2 c{rng.uniform(-40.0, 40.0), rng.uniform(-40.0, 40.0)};
        const double radius = rng.uniform(15.0, 40.0);
        const double a0 = rng.uniform(0.0, 2.0 * geometry::kPi);
        const double sweep = rng.uniform(0.9, 1.6) * geometry::kPi;
        ArcSpan arc{c, radius, a0, a0 + sweep, ArcDirection::CCW};
        Contour d;
        d.closed = true;
        d.spans.push_back(arc);
        d.spans.push_back(LineSpan{arc.endPoint(), arc.startPoint()});
        ProfileParams p = params(rng, ProfileSide::Outside);
        tooling::Tool v = test::endMill(25.0);
        v.type = tooling::ToolType::VBit;
        v.includedAngle = 2.0 * geometry::kPi / 3.0;
        p.passes.cutDepth = rng.uniform(1.0, 3.0);
        p.passes.passDepth = p.passes.cutDepth;
        p.corners.sharpExternal = true;
        p.corners.mitreLimit = 50.0;
        const double dist = profileToolRadius(v, p.passes);
        INFO("scene " << scene << " radius " << radius << " sweep " << sweep << " dist " << dist);
        const Vec2 s0 = startOf(profileToolpath({d}, v, p).toolpath).at;
        // On the offset circle and on the offset of the closing line (outside: away from the arc).
        CHECK(std::fabs(geometry::distance(s0, c) - (radius + dist)) < 1e-6);
        const Vec2 ls = arc.endPoint();
        const Vec2 le = arc.startPoint();
        const Vec2 u = (le - ls) * (1.0 / (le - ls).length());
        CHECK(std::fabs(std::fabs((s0 - ls).cross(u)) - dist) < 1e-6);
        CHECK(geometry::distance(s0, le) < geometry::distance(s0, ls));  // at the start vertex
        for (double k : scales) {
            INFO("scale 1 " << (k >= 1.0 ? "+ " : "- ") << std::fabs(k - 1.0));
            const Vec2 s = startOf(profileToolpath({scaled(d, k)}, v, p).toolpath).at;
            CHECK(geometry::distance(s, s0 * k) < 1e-5);
        }
    }
}

namespace {

// Corner of the outward offsets (by `dist`) of the edges into and out of `tip` of a
// counter-clockwise contour: the mitre at the tip.
Vec2 mitreAt(Vec2 tip, Vec2 uIn, Vec2 uOut, double dist) {
    const Vec2 nIn{uIn.y, -uIn.x};
    const Vec2 nOut{uOut.y, -uOut.x};
    const Vec2 p = tip + nIn * dist;
    const Vec2 q = tip + nOut * dist;
    const double s = (q - p).cross(uOut) / uIn.cross(uOut);
    return p + uIn * s;
}

Contour polygon(const std::vector<Vec2>& v) {
    Contour c;
    c.closed = true;
    for (std::size_t i = 0; i < v.size(); ++i) {
        c.spans.push_back(LineSpan{v[i], v[(i + 1) % v.size()]});
    }
    return c;
}

tooling::Tool vBit90() {
    tooling::Tool v = test::endMill(20.0);
    v.type = tooling::ToolType::VBit;
    v.includedAngle = geometry::kPi / 2.0;
    return v;
}

}  // namespace

TEST_CASE("T-0158: a mitred tip with a short, slightly turning neighbour edge starts at the tip's own mitre",
          "[toolpath][profile][start][vbit]") {
    // T-0158 round-2 verifier (harness t0158v2 edge.cpp, shortpost_L0.01_turn2): an edge of length L
    // after (or before) the tip that turns by a small angle t keeps its offset within the tolerance
    // of the tip when L sin t < tol, so it joined the near run and, in one cut direction, its
    // shallow mitre took the start. The start is the run's sharpest junction, the tip's mitre,
    // in both directions and on both sides of L sin t = tol.
    test::SplitMix rng{0x7158'0006ULL};
    const std::vector<double> scales = perturbations();
    for (int scene = 0; scene < 24; ++scene) {
        const Vec2 tip{rng.uniform(-30.0, 30.0), rng.uniform(-30.0, 30.0)};
        const double half = rng.uniform(25.0, 50.0) * geometry::kPi / 180.0;
        const double turn = rng.uniform(1.0, 5.0) * geometry::kPi / 180.0;
        const double length = rng.logUniform(0.003, 0.3);
        const bool after = scene % 2 == 0;
        const Vec2 a = tip + Vec2::fromAngle(geometry::kPi - half, 30.0);
        const Vec2 b = tip + Vec2::fromAngle(geometry::kPi + half, 30.0);
        const Vec2 c = tip + Vec2{-45.0, 0.0};
        std::vector<Vec2> v;
        Vec2 uIn;
        Vec2 uOut;
        if (after) {
            const Vec2 q = tip + Vec2::fromAngle(geometry::kPi - half - turn, length);
            v = {tip, q, a, c, b};
            uIn = (tip - b) * (1.0 / (tip - b).length());
            uOut = (q - tip) * (1.0 / (q - tip).length());
        } else {
            const Vec2 q = tip + Vec2::fromAngle(geometry::kPi + half + turn, length);
            v = {tip, a, c, b, q};
            uIn = (tip - q) * (1.0 / (tip - q).length());
            uOut = (a - tip) * (1.0 / (a - tip).length());
        }
        const Contour poly = polygon(v);
        for (CutDirection dir : {CutDirection::Climb, CutDirection::Conventional}) {
            ProfileParams p;
            p.side = ProfileSide::Outside;
            p.direction = dir;
            p.startAt = StartAt::VectorStart;
            p.passes.cutDepth = 3.0;
            p.passes.passDepth = 3.0;
            p.corners.sharpExternal = true;
            p.corners.mitreLimit = 50.0;
            const double dist = profileToolRadius(vBit90(), p.passes);
            INFO("scene " << scene << (after ? " after" : " before") << " L " << length << " L sin t "
                          << length * std::sin(turn) << " climb " << (dir == CutDirection::Climb));
            const Vec2 s0 = startOf(profileToolpath({poly}, vBit90(), p).toolpath).at;
            CHECK(geometry::distance(s0, mitreAt(tip, uIn, uOut, dist)) < 1e-6);
            for (double k : scales) {
                INFO("scale 1 " << (k >= 1.0 ? "+ " : "- ") << std::fabs(k - 1.0));
                const Vec2 s = startOf(profileToolpath({scaled(poly, k)}, vBit90(), p).toolpath).at;
                CHECK(geometry::distance(s, s0 * k) < 1e-5);
            }
        }
    }
}

TEST_CASE("T-0158: mitred tips of merged-region cuts start at the region's mitre, unmoved by perturbations",
          "[toolpath][profile][start][vbit]") {
    // T-0158 round-2 verifier (harness t0158v2 mmrg_*, gapmitre_1e-4): a vector merged with a
    // neighbour, or with a closing gap within the tolerance, is offset as a region by the cut
    // distance plus the chordal tolerance; the sharp-corner rule compares against that distance.
    // With an end mill the region's round join at the tip is a polygon whose chords are all
    // equally near (inside the circle by up to the chordal tolerance): it starts at a vertex of
    // that polygon, again unmoved (harness t0158v2 gen2 rmrg).
    test::SplitMix rng{0x7158'0007ULL};
    const std::vector<double> scales = perturbations();
    for (int scene = 0; scene < 24; ++scene) {
        const bool round = scene >= 16;
        const Star star = curvedStar(rng, true, false);
        std::vector<Contour> cs{star.contour};
        const bool gap = scene % 2 == 1;
        if (gap) {
            // Close the star a little off its start: the closing edge ends 1e-4 inside the tip.
            auto& last = std::get<LineSpan>(cs[0].spans.back());
            last.end = star.start + (star.centre - star.start) * (1e-4 / (star.centre - star.start).length());
        } else {
            // A copy of the star shifted away from its start tip, overlapping its body: the two
            // cuts merge, and the start tip stays a tip of the region.
            const Vec2 shift = (star.centre - star.start) * 0.7;
            Contour other = star.contour;
            for (geometry::Span& sp : other.spans) {
                auto& l = std::get<LineSpan>(sp);
                l.start = l.start + shift;
                l.end = l.end + shift;
            }
            cs.push_back(other);
        }
        ProfileParams p = params(rng, ProfileSide::Outside);
        p.passes.cutDepth = 3.0;
        p.passes.passDepth = 3.0;
        p.corners.sharpExternal = !round;
        p.corners.mitreLimit = 50.0;
        const tooling::Tool tool = round ? test::endMill(6.0) : vBit90();
        INFO("scene " << scene << (gap ? " gap" : " neighbour") << (round ? " end mill" : " V-bit"));
        const ProfileResult base = profileToolpath(cs, tool, p);
        REQUIRE(base.ok());
        REQUIRE(base.has(ProfileIssue::MergedWithNeighbour));
        // The cut of the star's own start: the cut start nearest the star's tip.
        auto tipStart = [&](const Toolpath& tp, double k) {
            Vec2 best;
            double bestD = std::numeric_limits<double>::infinity();
            for (const Move& m : tp.moves) {
                if (m.kind == MoveKind::Rapid && m.to.z == p.safeZ &&
                    geometry::distance(m.to.xy(), star.start * k) < bestD) {
                    bestD = geometry::distance(m.to.xy(), star.start * k);
                    best = m.to.xy();
                }
            }
            return best;
        };
        const Vec2 s0 = tipStart(base.toolpath, 1.0);
        // At the region's mitre: on the tip's bisector, (cut distance + chordal) / sin(half) out.
        const Vec2 along = geometry::endPoint(cs[0].spans.front()) - star.start;
        const Vec2 back = geometry::startPoint(cs[0].spans.back()) - star.start;
        const Vec2 u1 = along * (1.0 / along.length());
        const Vec2 u2 = back * (1.0 / back.length());
        const double sinHalf = std::sqrt(0.5 * (1.0 - u1.dot(u2)));
        const double offset = profileToolRadius(tool, p.passes) + p.tolerance.chordal;
        const Vec2 out = (u1 + u2) * (-1.0 / (u1 + u2).length());
        if (round) {
            CHECK(std::fabs(geometry::distance(s0, star.start) - offset) < 1e-6);  // a polygon vertex
        } else {
            CHECK(geometry::distance(s0, star.start + out * (offset / sinHalf)) < 1e-4);
        }
        for (double k : scales) {
            INFO("scale 1 " << (k >= 1.0 ? "+ " : "- ") << std::fabs(k - 1.0));
            std::vector<Contour> scs;
            for (const Contour& c : cs) scs.push_back(scaled(c, k));
            const ProfileResult r = profileToolpath(scs, tool, p);
            REQUIRE(r.ok());
            CHECK(geometry::distance(tipStart(r.toolpath, k), s0 * k) < 1e-5);
        }
    }
}

namespace {

// Outward normal (right of travel on a counter-clockwise contour) of a unit direction.
Vec2 rightOf(Vec2 u) { return Vec2{u.y, -u.x}; }

ProfileParams vbitParams(CutDirection dir, double limit) {
    ProfileParams p;
    p.side = ProfileSide::Outside;
    p.direction = dir;
    p.startAt = StartAt::VectorStart;
    p.passes.cutDepth = 3.0;
    p.passes.passDepth = 3.0;
    p.corners.sharpExternal = true;
    p.corners.mitreLimit = limit;
    return p;
}

// The perturbations the knife-edge sweeps check: one and four ulps either way and 1e-9 relative.
std::vector<double> fewPerturbations() {
    const double inf = std::numeric_limits<double>::infinity();
    double up4 = 1.0;
    double down4 = 1.0;
    for (int k = 0; k < 4; ++k) {
        up4 = std::nextafter(up4, inf);
        down4 = std::nextafter(down4, 0.0);
    }
    return {std::nextafter(1.0, inf), std::nextafter(1.0, 0.0), up4, down4, 1.0 + 1e-9, 1.0 - 1e-9};
}

}  // namespace

TEST_CASE("T-0158: a squared-off tip between curved edges starts at the chamfer's first corner in travel",
          "[toolpath][profile][start][vbit]") {
    // T-0158 round-3 verifier (harness t0158v3 repro3/dchamfer_last_corner_climb, t0127v1 m2_273007_12,
    // gen2 marc): where an arc meets the chamfer the two chamfer corners turn by different amounts,
    // and the round-3 rule (the sharpest junction) took the chamfer's LAST corner in travel in two of
    // the four direction/order combinations. Owner 2026-11-05: the first corner in travel. A D shape
    // (an arc closed by a line) starting at either end of the arc, both cut directions, and curved
    // stars; the cut must start on the chamfer and run along it first.
    test::SplitMix rng{0x7158'0008ULL};
    const std::vector<double> scales = fewPerturbations();
    std::size_t chamfers = 0;
    for (int scene = 0; scene < 48; ++scene) {
        Contour shape;
        shape.closed = true;
        if (scene < 32) {
            const Vec2 c{rng.uniform(-40.0, 40.0), rng.uniform(-40.0, 40.0)};
            const double radius = rng.uniform(12.0, 40.0);
            const double a0 = rng.uniform(0.0, 2.0 * geometry::kPi);
            const double sweep = rng.uniform(1.0, 1.4) * geometry::kPi;
            const ArcSpan arc{c, radius, a0, a0 + sweep, ArcDirection::CCW};
            const LineSpan line{arc.endPoint(), arc.startPoint()};
            if (scene % 2 == 0) {
                shape.spans = {arc, line};  // starts at the arc's start
            } else {
                shape.spans = {line, arc};  // starts at the arc's end
            }
        } else {
            test::SplitMix starRng{rng.next()};
            shape = curvedStar(starRng, true).contour;
        }
        const CutDirection dir = (scene / 2) % 2 == 0 ? CutDirection::Climb : CutDirection::Conventional;
        const ProfileParams p = vbitParams(dir, scene < 32 ? 1.1 : 1.02);
        const double dist = profileToolRadius(vBit90(), p.passes);
        const Vec2 tip = geometry::startPoint(shape.spans.front());
        // squareJoin's chamfer: perpendicular to the sum of the offset normals at the tip, `dist` out.
        const Vec2 nIn = rightOf(geometry::tangentAt(shape.spans.back(), 1.0));
        const Vec2 nOut = rightOf(geometry::tangentAt(shape.spans.front(), 0.0));
        const Vec2 bis = (nIn + nOut) * (1.0 / (nIn + nOut).length());
        const ProfileResult base = profileToolpath({shape}, vBit90(), p);
        REQUIRE(base.ok());
        const Start s0 = startOf(base.toolpath);
        REQUIRE(s0.found);
        INFO("scene " << scene << (dir == CutDirection::Climb ? " climb" : " conventional") << " start "
                      << s0.at.x << "," << s0.at.y);
        if (std::fabs((s0.at - tip).dot(bis) - dist) > 1e-6) {
            continue;  // not squared off here (a mitre within the limit, or a round fallback)
        }
        ++chamfers;
        // On the chamfer line, off its middle (a corner), and the first move runs along it.
        CHECK(geometry::distance(s0.at, tip) > dist + 1e-3);
        CHECK(s0.first.kind == MoveKind::Feed);
        CHECK(std::fabs((s0.first.to.xy() - tip).dot(bis) - dist) < 1e-6);
        CHECK(geometry::distance(s0.first.to.xy(), s0.at) > 1e-3);
        for (double k : scales) {
            INFO("scale 1 " << (k >= 1.0 ? "+ " : "- ") << std::fabs(k - 1.0));
            const Vec2 s = startOf(profileToolpath({scaled(shape, k)}, vBit90(), p).toolpath).at;
            CHECK(geometry::distance(s, s0.at * k) < 1e-5);
        }
    }
    INFO("chamfers " << chamfers);
    CHECK(chamfers >= 30);
}

TEST_CASE("T-0158: a mitred tip next to a short edge that turns as much or more starts at its own mitre, "
          "on both sides of length x sin(turn) = tol",
          "[toolpath][profile][start][vbit]") {
    // T-0158 round-3 verifier (harness t0158v3 repro3/sharpnb_*, equalnb_*, tietA0.3_d0, probe3 tie):
    // an edge of length L next to the start vertex A, ending at a vertex B that turns by tB, keeps
    // B's offset edges within the tolerance of A when L sin(tB) < tol. When B turned at least as
    // much as A (less 1e-3 rad), the round-3 rule let B's mitre take the start, so it jumped between
    // the two mitres as L sin(tB) crossed the tolerance, and as tB crossed tA - 1e-3. The start is
    // A's own mitre throughout: neighbour before and after A, both cut directions, sharper, equal
    // and just-less-sharp neighbours.
    const std::vector<double> scales = fewPerturbations();
    const double tol = ProfileParams{}.tolerance.distance;
    std::size_t runs = 0;
    for (double tA : {0.1, 0.3}) {
        for (double tB : {tA - 1e-3, std::nextafter(tA - 1e-3, 0.0), tA - 1e-3 + 1e-12, tA, 2.0 * tA, 1.0}) {
            for (double f : {0.5, 1.0 - 1e-9, 1.0, 1.0 + 1e-9, 2.0}) {
                const double length = f * tol / std::sin(tB);
                if (length < 1.1 * tol) {
                    // An edge shorter than the distance tolerance is dropped by the offset, which
                    // then makes one corner of its two vertices.
                    continue;
                }
                const Vec2 a{11.25, 3.5};
                const Vec2 ein{1.0, 0.0};
                const Vec2 eout = Vec2::fromAngle(tA, 1.0);
                const Vec2 eP = Vec2::fromAngle(-tB, 1.0);
                const Vec2 b = a - ein * length;
                const Vec2 pp = b - eP * 30.0;
                const Vec2 q = a + eout * 30.0;
                const Vec2 top = (pp + q) * 0.5 + Vec2{0.0, 25.0};
                for (int order = 0; order < 2; ++order) {
                    std::vector<Vec2> v;
                    if (order == 0) {
                        v = {a, q, top, pp, b};  // B before A
                    } else {
                        // Mirrored about x = A.x (reversed, still counter-clockwise): B after A.
                        auto m = [&](Vec2 w) { return Vec2{2.0 * a.x - w.x, w.y}; };
                        v = {a, m(b), m(pp), m(top), m(q)};
                    }
                    const Contour poly = polygon(v);
                    const Vec2 uIn = (a - v.back()) * (1.0 / (a - v.back()).length());
                    const Vec2 uOut = (v[1] - a) * (1.0 / (v[1] - a).length());
                    for (CutDirection dir : {CutDirection::Climb, CutDirection::Conventional}) {
                        const ProfileParams p = vbitParams(dir, 50.0);
                        const double dist = profileToolRadius(vBit90(), p.passes);
                        const Vec2 mitre = mitreAt(a, uIn, uOut, dist);
                        INFO("tA " << tA << " tB " << tB << " L sin(tB) / tol " << f << " order " << order
                                   << (dir == CutDirection::Climb ? " climb" : " conventional"));
                        const Vec2 s0 = startOf(profileToolpath({poly}, vBit90(), p).toolpath).at;
                        CHECK(geometry::distance(s0, mitre) < 1e-9);
                        for (double k : scales) {
                            INFO("scale 1 " << (k >= 1.0 ? "+ " : "- ") << std::fabs(k - 1.0));
                            const Vec2 s = startOf(profileToolpath({scaled(poly, k)}, vBit90(), p).toolpath).at;
                            CHECK(geometry::distance(s, mitre * k) < 1e-8);
                        }
                        ++runs;
                    }
                }
            }
        }
    }
    CHECK(runs >= 2 * 6 * 4 * 2 * 2);
}

TEST_CASE("T-0158: a finely segmented convex outline starts at the start vertex's own mitre",
          "[toolpath][profile][start][vbit]") {
    // Owner decision 2026-11-05 (after round 3; harness t0158v3 probe3 ngon): where the segments are
    // shorter than about sqrt(2 d tol), many offset edges are within the tolerance of the start
    // vertex; the cut starts at that vertex's exact mitre, not one segment off, either way round.
    const std::vector<double> scales = fewPerturbations();
    for (int segments : {300, 2000, 5000}) {
        for (double radius : {20.0, 60.0}) {
            const Vec2 c{7.3, -4.1};
            const double rot = 0.3;
            std::vector<Vec2> v;
            for (int j = 0; j < segments; ++j) {
                v.push_back(c + Vec2::fromAngle(rot + 2.0 * geometry::kPi * j / segments, radius));
            }
            const Contour poly = polygon(v);
            const Vec2 uIn = (v[0] - v.back()) * (1.0 / (v[0] - v.back()).length());
            const Vec2 uOut = (v[1] - v[0]) * (1.0 / (v[1] - v[0]).length());
            for (CutDirection dir : {CutDirection::Climb, CutDirection::Conventional}) {
                const ProfileParams p = vbitParams(dir, 50.0);
                const Vec2 mitre = mitreAt(v[0], uIn, uOut, profileToolRadius(vBit90(), p.passes));
                INFO("segments " << segments << " radius " << radius
                                 << (dir == CutDirection::Climb ? " climb" : " conventional"));
                const Vec2 s0 = startOf(profileToolpath({poly}, vBit90(), p).toolpath).at;
                CHECK(geometry::distance(s0, mitre) < 1e-9);
                for (double k : {scales.front(), scales.back()}) {
                    INFO("scale 1 " << (k >= 1.0 ? "+ " : "- ") << std::fabs(k - 1.0));
                    const Vec2 s = startOf(profileToolpath({scaled(poly, k)}, vBit90(), p).toolpath).at;
                    CHECK(geometry::distance(s, mitre * k) < 1e-8);
                }
            }
        }
    }
}

TEST_CASE("T-0158: a merged region's polygonal round join starts at its first chord point in travel",
          "[toolpath][profile][start]") {
    // Owner decision 2026-11-05 (after round 3; harness t0158v3 probe3 join): an end-mill cut of a
    // merged region rounds a convex tip with a polygon of chords on the circle about the tip. Like a
    // true round join, which starts at its arc's beginning, the cut starts at the polygon's first
    // point in travel: the foot of the tip on the incoming offset edge, so the cut arrives at its
    // start along that edge, square to the radius. Round 3 started one chord in. Both directions,
    // neighbour-merged and gap-merged stars.
    test::SplitMix rng{0x7158'0009ULL};
    const std::vector<double> scales = fewPerturbations();
    for (int scene = 0; scene < 16; ++scene) {
        const Star star = curvedStar(rng, true, false);
        std::vector<Contour> cs{star.contour};
        if (scene % 2 == 1) {
            auto& last = std::get<LineSpan>(cs[0].spans.back());
            last.end = star.start + (star.centre - star.start) * (1e-4 / (star.centre - star.start).length());
        } else {
            const Vec2 shift = (star.centre - star.start) * 0.7;
            Contour other = star.contour;
            for (geometry::Span& sp : other.spans) {
                auto& l = std::get<LineSpan>(sp);
                l.start = l.start + shift;
                l.end = l.end + shift;
            }
            cs.push_back(other);
        }
        ProfileParams p;
        p.side = ProfileSide::Outside;
        p.direction = (scene / 2) % 2 == 0 ? CutDirection::Climb : CutDirection::Conventional;
        p.startAt = StartAt::VectorStart;
        p.passes.cutDepth = 3.0;
        p.passes.passDepth = 3.0;
        const double d = rng.uniform(3.0, 8.0);
        const tooling::Tool tool = test::endMill(d);
        const double offset = 0.5 * d + p.tolerance.chordal;
        INFO("scene " << scene << (scene % 2 == 1 ? " gap" : " neighbour") << " d " << d
                      << (p.direction == CutDirection::Climb ? " climb" : " conventional"));
        // The cut start nearest the tip, and the point the cut comes from when it closes there.
        auto tipStart = [&](const Toolpath& tp, double k, Vec2& from) {
            std::size_t best = tp.moves.size();
            for (std::size_t i = 0; i < tp.moves.size(); ++i) {
                const Move& m = tp.moves[i];
                if (m.kind == MoveKind::Rapid && m.to.z == p.safeZ &&
                    (best == tp.moves.size() ||
                     geometry::distance(m.to.xy(), star.start * k) <
                         geometry::distance(tp.moves[best].to.xy(), star.start * k))) {
                    best = i;
                }
            }
            REQUIRE(best < tp.moves.size());
            const Vec2 at = tp.moves[best].to.xy();
            for (std::size_t i = best + 2; i < tp.moves.size() && tp.moves[i].kind != MoveKind::Rapid; ++i) {
                if (tp.moves[i].kind == MoveKind::Feed && geometry::distance(tp.moves[i].to.xy(), at) < 1e-9 &&
                    geometry::distance(tp.moves[i - 1].to.xy(), at) > 1e-6) {
                    from = tp.moves[i - 1].to.xy();
                }
            }
            return at;
        };
        const ProfileResult base = profileToolpath(cs, tool, p);
        REQUIRE(base.ok());
        REQUIRE(base.has(ProfileIssue::MergedWithNeighbour));
        Vec2 from;
        const Vec2 s0 = tipStart(base.toolpath, 1.0, from);
        CHECK(std::fabs(geometry::distance(s0, star.start) - offset) < 1e-5);  // on the join's circle
        const Vec2 in = (s0 - from) * (1.0 / (s0 - from).length());
        CHECK(std::fabs((s0 - star.start).dot(in)) < 1e-4);  // the foot on the incoming edge
        for (double k : scales) {
            INFO("scale 1 " << (k >= 1.0 ? "+ " : "- ") << std::fabs(k - 1.0));
            std::vector<Contour> scs;
            for (const Contour& c : cs) scs.push_back(scaled(c, k));
            const ProfileResult r = profileToolpath(scs, tool, p);
            Vec2 f;
            CHECK(geometry::distance(tipStart(r.toolpath, k, f), s0 * k) < 1e-5);
        }
    }
}

TEST_CASE("T-0158: a tip whose round join is shorter than the tolerance starts at the join's first point in "
          "travel, unmoved by scaling and translation",
          "[toolpath][profile][start]") {
    // Found in round 4 (harness t0127c rc_g1_J_87741_47): a Bezier blob is polygonised finely, and
    // at its start vertex, which turns so little that the round join of the 0.115 mm offset would
    // be shorter than the tolerance, the exact offset bridges the two offset edges with a straight
    // chord. Both ends of the chord are the vertex's feet on the edges, equally near, so the start
    // followed rounding (12.7 µm apart under a translation). Like a round join, the cut starts at
    // the chord's first point in travel and arrives there along the incoming offset edge, square to
    // the radius.
    Contour blob;
    blob.closed = true;
    const std::vector<std::vector<double>> beziers{
        {0x1.d15a4e0425dbap+3, -0x1.5b7c6ccdf8dd9p-2, 0x1.d08c86f70d162p+3, 0x1.d93133277b26fp+1,
         0x1.8cd8ca214992ep+3, 0x1.3daeaf4c3eb9p+3, 0x1.2e97c99c03a4fp+3, 0x1.91bf884a428a8p+3},
        {0x1.2e97c99c03a4fp+3, 0x1.91bf884a428a8p+3, 0x1.a0ad922d7b6ep+2, 0x1.e5d06148465cp+3,
         0x1.1e75ed5ab5552p+0, 0x1.04fe5be621ca9p+4, -0x1.90aed46df6e03p+1, 0x1.ed89328da7221p+3},
        {-0x1.90aed46df6e03p+1, 0x1.ed89328da7221p+3, -0x1.d84c4fc4a4358p+2, 0x1.d115ad4f0aafp+3,
         -0x1.c8de30ebafd66p+3, 0x1.5de15ac9a6bb8p+3, -0x1.00a373867b55ap+4, 0x1.ce14d1a52fb02p+2},
        {-0x1.00a373867b55ap+4, 0x1.ce14d1a52fb02p+2, -0x1.1cd7ce971ec01p+4, 0x1.c0cddb6e23d27p+1,
         -0x1.fe922929c03bcp+3, -0x1.dccc83263e7fp+1, -0x1.b69ff9e326b53p+3, -0x1.b700f27964654p+2},
        {-0x1.b69ff9e326b53p+3, -0x1.b700f27964654p+2, -0x1.6eadca9c8d2eap+3, -0x1.3fcdd1afd4c58p+3,
         -0x1.9be1bb64a55b4p+2, -0x1.5928249bbc7d5p+3, -0x1.46672d95760f2p+1, -0x1.72c5a9e037992p+3},
        {-0x1.46672d95760f2p+1, -0x1.72c5a9e037992p+3, 0x1.55ea373cbd306p+0, -0x1.8c632f24b2b4fp+3,
         0x1.b0838a075a166p+2, -0x1.b12de4968b7cap+3, 0x1.336a73ea9845cp+3, -0x1.753198d794d99p+3},
        {0x1.336a73ea9845cp+3, -0x1.753198d794d99p+3, 0x1.8e9322d183805p+3, -0x1.39354d189e368p+3,
         0x1.d22815113ea12p+3, -0x1.1808272d7caf3p+2, 0x1.d15a4e0425dbap+3, -0x1.5b7c6ccdf8dd9p-2}};
    for (const auto& b : beziers) {
        blob.spans.push_back(geometry::BezierSpan{{b[0], b[1]}, {b[2], b[3]}, {b[4], b[5]}, {b[6], b[7]}});
    }
    auto moved = [&](double k, Vec2 o) {
        Contour c = blob;
        for (geometry::Span& sp : c.spans) {
            auto& z = std::get<geometry::BezierSpan>(sp);
            for (Vec2* q : {&z.p0, &z.p1, &z.p2, &z.p3}) {
                *q = *q * k + o;
            }
        }
        return c;
    };
    const Vec2 tip = geometry::startPoint(blob.spans.front());
    // The cut's start, and the point it comes from when it closes there.
    auto startAndFrom = [&](const Toolpath& tp, Vec2& from) {
        const Vec2 at = tp.moves.front().to.xy();
        for (std::size_t i = 2; i < tp.moves.size() && tp.moves[i].kind != MoveKind::Rapid; ++i) {
            if (tp.moves[i].kind == MoveKind::Feed && geometry::distance(tp.moves[i].to.xy(), at) < 1e-9 &&
                geometry::distance(tp.moves[i - 1].to.xy(), at) > 1e-6) {
                from = tp.moves[i - 1].to.xy();
            }
        }
        return at;
    };
    for (CutDirection dir : {CutDirection::Climb, CutDirection::Conventional}) {
        ProfileParams p;
        p.side = ProfileSide::On;
        p.allowance = 0x1.d5ab6304087fp-4;
        p.direction = dir;
        p.startAt = StartAt::VectorStart;
        p.passes.cutDepth = 3.0;
        p.passes.passDepth = 3.0;
        INFO((dir == CutDirection::Climb ? "climb" : "conventional"));
        Vec2 from;
        const Vec2 s0 = startAndFrom(profileToolpath({blob}, test::endMill(6.0), p).toolpath, from);
        CHECK(std::fabs(geometry::distance(s0, tip) - p.allowance) < 1e-6);
        const Vec2 in = (s0 - from) * (1.0 / (s0 - from).length());
        CHECK(std::fabs((s0 - tip).dot(in)) < 1e-6);
        for (double k : fewPerturbations()) {
            INFO("scale 1 " << (k >= 1.0 ? "+ " : "- ") << std::fabs(k - 1.0));
            Vec2 f;
            const Vec2 s = startAndFrom(profileToolpath({moved(k, Vec2{})}, test::endMill(6.0), p).toolpath, f);
            CHECK(geometry::distance(s, s0 * k) < 1e-7);
        }
        for (Vec2 o : {Vec2{3.0, -7.0}, Vec2{0.1, 0.3}, Vec2{1000.25, -999.75}}) {
            INFO("shift " << o.x << "," << o.y);
            Vec2 f;
            const Vec2 s = startAndFrom(profileToolpath({moved(1.0, o)}, test::endMill(6.0), p).toolpath, f);
            CHECK(geometry::distance(s - o, s0) < 1e-7);
        }
    }
}

namespace {

// A contour of line spans scaled by `k` and then moved by `o`.
Contour movedPoly(const Contour& c, double k, Vec2 o) {
    Contour out = c;
    for (geometry::Span& sp : out.spans) {
        auto& l = std::get<LineSpan>(sp);
        l.start = l.start * k + o;
        l.end = l.end * k + o;
    }
    return out;
}

// The rapid of `tp` nearest `p`: the start of the cut asked to start there.
Vec2 rapidNear(const Toolpath& tp, Vec2 p) {
    Vec2 best{std::numeric_limits<double>::infinity(), 0.0};
    for (const Move& m : tp.moves) {
        if (m.kind == MoveKind::Rapid && geometry::distance(m.to.xy(), p) < geometry::distance(best, p)) {
            best = m.to.xy();
        }
    }
    return best;
}

// The probe-shaped polygon of the round-4 verifier (harness t0158v4 probe4): start vertex A turning
// by tA, and a neighbour vertex B at distance `length` before A (order 0) or after it (order 1,
// mirrored), turning by tB (0: exactly collinear, a redundant vertex; negative: the other way).
std::vector<Vec2> tipWithNeighbour(Vec2 a, double tA, double tB, double length, int order) {
    const Vec2 ein{1.0, 0.0};
    const Vec2 b = a - ein * length;
    const Vec2 pp = b - Vec2::fromAngle(-tB, 1.0) * 30.0;
    const Vec2 q = a + Vec2::fromAngle(tA, 1.0) * 30.0;
    const Vec2 top = (pp + q) * 0.5 + Vec2{0.0, 25.0};
    if (order == 0) {
        return {a, q, top, pp, b};
    }
    auto m = [&](Vec2 w) { return Vec2{2.0 * a.x - w.x, w.y}; };
    return {a, m(b), m(pp), m(top), m(q)};
}

}  // namespace

TEST_CASE("T-0158: a mitred tip next to a redundant vertex starts at the tip's mitre",
          "[toolpath][profile][start][vbit]") {
    // T-0158 round-4 verifier (harness t0158v4 repro4/rect_collinear_*, rect4): a 20 x 10 rectangle
    // with an extra, exactly collinear vertex L before its start corner. The offset line of that
    // edge is split there; within sqrt(2 d tol) the split counted as an equally near junction and,
    // cut conventionally, the start went back to it, 3.05 mm from the mitre. A vertex with no corner
    // of its own never takes the start: the cut starts at the corner's mitre (-3, -3), the
    // redundant vertex before or after the corner, both directions, also at 3 m coordinates.
    const std::vector<double> scales = fewPerturbations();
    for (double length : {0.002, 0.01, 0.03, 0.05, 0.07, 0.5}) {
        for (int order = 0; order < 2; ++order) {
            std::vector<Vec2> v{{0.0, 0.0}, {20.0, 0.0}, {20.0, 10.0}, {0.0, 10.0}};
            if (order == 0) {
                v.push_back({0.0, length});
            } else {
                v.insert(v.begin() + 1, Vec2{length, 0.0});
            }
            const Contour rect = polygon(v);
            for (Vec2 o : {Vec2{0.0, 0.0}, Vec2{3000.0, 2100.0}}) {
                for (CutDirection dir : {CutDirection::Climb, CutDirection::Conventional}) {
                    const ProfileParams p = vbitParams(dir, 50.0);
                    INFO("L " << length << (order == 0 ? " before" : " after") << " offset " << o.x
                              << (dir == CutDirection::Climb ? " climb" : " conventional"));
                    const Vec2 mitre = Vec2{-3.0, -3.0} + o;
                    const Vec2 s0 = startOf(profileToolpath({movedPoly(rect, 1.0, o)}, vBit90(), p).toolpath).at;
                    CHECK(geometry::distance(s0, mitre) < 1e-9);
                    for (double k : scales) {
                        INFO("scale 1 " << (k >= 1.0 ? "+ " : "- ") << std::fabs(k - 1.0));
                        const Vec2 s = startOf(profileToolpath({movedPoly(rect, k, o * k)}, vBit90(), p).toolpath).at;
                        CHECK(geometry::distance(s, mitre * k) < 1e-8);
                    }
                }
            }
        }
    }
}

TEST_CASE("T-0158: a mitred tip next to a nearly collinear neighbour starts at the tip's mitre, "
          "across the round-4 equidistance threshold and at large coordinates",
          "[toolpath][profile][start][vbit]") {
    // T-0158 round-4 verifier (harness t0158v4 thr, repro4/nearcollinear_thr_*, bigc_*): a neighbour
    // vertex B within sqrt(2 d tol) of the tip A whose edge length L x sin(turn) was within the
    // rounding of the equidistance test joined A's corner and, first in travel, took the start
    // (2.06 mm off). Its turn at that threshold (3.2e-6, 8e-7, 3.2e-7 rad for L = 5, 20, 50 um)
    // flipped the start under one ulp, 1e-9 scaling, translation and FMA; at 3 m coordinates a
    // 2 um edge turning 1e-3 rad did. Only the offsets of A's own two edges bound A's corner, so the
    // start is A's mitre for every turn of B, either way, before or after A, both directions.
    const std::vector<double> scales{std::nextafter(1.0, 2.0), std::nextafter(1.0, 0.0), 1.0 + 1e-9, 1.0 - 1e-9};
    std::size_t runs = 0;
    for (double tA : {0.3, 1.2}) {
        for (double length : {0.002, 0.005, 0.02, 0.05}) {
            for (double tB : {0.0, 1e-9, 3.16e-7, 7.95e-7, 3.19e-6, -3.19e-6, 1e-5, 1e-3, -1e-3}) {
                for (int order = 0; order < 2; ++order) {
                    const std::vector<Vec2> v = tipWithNeighbour(Vec2{11.25, 3.5}, tA, tB, length, order);
                    const Contour poly = polygon(v);
                    const Vec2 uIn = (v[0] - v.back()) * (1.0 / (v[0] - v.back()).length());
                    const Vec2 uOut = (v[1] - v[0]) * (1.0 / (v[1] - v[0]).length());
                    for (Vec2 o : {Vec2{0.0, 0.0}, Vec2{3000.0, 2100.0}}) {
                        for (CutDirection dir : {CutDirection::Climb, CutDirection::Conventional}) {
                            const ProfileParams p = vbitParams(dir, 50.0);
                            const Vec2 mitre = mitreAt(v[0], uIn, uOut, profileToolRadius(vBit90(), p.passes)) + o;
                            INFO("tA " << tA << " L " << length << " tB " << tB << (order == 0 ? " before" : " after")
                                       << " offset " << o.x << (dir == CutDirection::Climb ? " climb" : " conventional"));
                            const Vec2 s0 =
                                startOf(profileToolpath({movedPoly(poly, 1.0, o)}, vBit90(), p).toolpath).at;
                            CHECK(geometry::distance(s0, mitre) < 1e-9);
                            if (o.x == 0.0) {
                                for (double k : scales) {
                                    INFO("scale 1 " << (k >= 1.0 ? "+ " : "- ") << std::fabs(k - 1.0));
                                    const Vec2 s =
                                        startOf(profileToolpath({movedPoly(poly, k, o)}, vBit90(), p).toolpath).at;
                                    CHECK(geometry::distance(s, mitre * k) < 1e-8);
                                }
                            }
                            ++runs;
                        }
                    }
                }
            }
        }
    }
    CHECK(runs == 2 * 4 * 9 * 2 * 2 * 2);
}

TEST_CASE("T-0158: a cut asked to start at a redundant vertex starts at that vertex's own foot",
          "[toolpath][profile][start][vbit]") {
    // A start vertex in the middle of a straight edge has no corner; its foot on the edge's offset is
    // the one nearest point. Another redundant vertex within sqrt(2 d tol) before it splits the offset
    // line there too, and that junction is as equidistant as the start's own: of the two the cut
    // starts at the one whose feet lie most evenly about it, the start's foot, in both directions
    // (not at the first in travel).
    for (double length : {0.01, 0.05}) {
        const Vec2 t{10.0, 0.0};
        const Contour rect = polygon({t, {20.0, 0.0}, {20.0, 10.0}, {0.0, 10.0}, {0.0, 0.0}, {10.0 - length, 0.0}});
        for (CutDirection dir : {CutDirection::Climb, CutDirection::Conventional}) {
            const ProfileParams p = vbitParams(dir, 50.0);
            INFO("L " << length << (dir == CutDirection::Climb ? " climb" : " conventional"));
            const Vec2 s0 = startOf(profileToolpath({rect}, vBit90(), p).toolpath).at;
            CHECK(geometry::distance(s0, Vec2{10.0, -3.0}) < 1e-9);
            for (double k : fewPerturbations()) {
                INFO("scale 1 " << (k >= 1.0 ? "+ " : "- ") << std::fabs(k - 1.0));
                const Vec2 s = startOf(profileToolpath({movedPoly(rect, k, Vec2{})}, vBit90(), p).toolpath).at;
                CHECK(geometry::distance(s, Vec2{10.0, -3.0} * k) < 1e-8);
            }
        }
    }
}

TEST_CASE("T-0158: a merged region's tip next to a turning region vertex a few um away starts at the "
          "tip's first join point, unmoved by perturbations",
          "[toolpath][profile][start]") {
    // T-0158 round-4 verifier (harness t0158v4 repro4/mrg_twovertex_*): two overlapping polygons
    // merge into one region; the start vertex A (turning 0.9 rad) has a region vertex B 1.2 to 4.6 um
    // before or after it turning 0.3 or 0.8 rad. The polygonal joins of A and B are separate places at
    // the nearest distance, so the start fell back to the plain projection and moved 0.49 mm under 1e-9
    // scaling. The join about A lies between the offsets of A's own two edges: the cut starts at its
    // first point in travel, the foot of A on the edge's offset met first.
    const std::vector<double> scales{std::nextafter(1.0, 2.0), 1.0 + 1e-9};
    for (double tB : {0.3, 0.8}) {
        for (double x : {0.0012, 0.003, 0.0046}) {
            if (tB == 0.8 && x != 0.003) {
                continue;
            }
            for (int order = 0; order < 2; ++order) {
                const std::vector<Vec2> v = tipWithNeighbour(Vec2{11.25, 3.5}, 0.9, tB, x, order);
                const Vec2 a = v[0];
                Vec2 c{0.0, 0.0};
                for (Vec2 q : v) c = c + q * (1.0 / static_cast<double>(v.size()));
                std::vector<Vec2> w;
                for (Vec2 q : v) w.push_back(q + (c - a) * 0.5);
                const std::vector<Contour> cs{polygon(v), polygon(w)};
                const Vec2 uIn = (a - v.back()) * (1.0 / (a - v.back()).length());
                const Vec2 uOut = (v[1] - a) * (1.0 / (v[1] - a).length());
                for (CutDirection dir : {CutDirection::Climb, CutDirection::Conventional}) {
                    ProfileParams p;
                    p.side = ProfileSide::Outside;
                    p.direction = dir;
                    p.startAt = StartAt::VectorStart;
                    p.passes.cutDepth = 3.0;
                    p.passes.passDepth = 3.0;
                    p.corners.mitreLimit = 50.0;
                    const double r = 3.0 + p.tolerance.chordal;
                    const Vec2 fin = a + rightOf(uIn) * r;
                    const Vec2 fout = a + rightOf(uOut) * r;
                    INFO("tB " << tB << " x " << x << (order == 0 ? " before" : " after")
                               << (dir == CutDirection::Climb ? " climb" : " conventional"));
                    const ProfileResult base = profileToolpath(cs, test::endMill(6.0), p);
                    REQUIRE(base.has(ProfileIssue::MergedWithNeighbour));
                    const Vec2 s0 = rapidNear(base.toolpath, a);
                    CHECK(std::min(geometry::distance(s0, fin), geometry::distance(s0, fout)) < 1e-5);
                    for (double k : scales) {
                        INFO("scale 1 " << (k >= 1.0 ? "+ " : "- ") << std::fabs(k - 1.0));
                        const std::vector<Contour> scs{movedPoly(cs[0], k, Vec2{}), movedPoly(cs[1], k, Vec2{})};
                        CHECK(geometry::distance(rapidNear(profileToolpath(scs, test::endMill(6.0), p).toolpath, a * k),
                                                 s0 * k) < 1e-5);
                    }
                    for (Vec2 o : {Vec2{1000.25, -999.75}}) {
                        INFO("shift " << o.x << "," << o.y);
                        const std::vector<Contour> scs{movedPoly(cs[0], 1.0, o), movedPoly(cs[1], 1.0, o)};
                        CHECK(geometry::distance(rapidNear(profileToolpath(scs, test::endMill(6.0), p).toolpath, a + o),
                                                 s0 + o) < 1e-5);
                    }
                }
            }
        }
    }
}

TEST_CASE("T-0158: a polygon tip whose round join is shorter than the tolerance starts at the join's first "
          "point in travel, also translated far",
          "[toolpath][profile][start]") {
    // T-0158 round-4 verifier (harness t0158v4 probe4 chord): a start vertex turning by f x tol / d,
    // whose round join the exact offset replaces by a chord when it is shorter than the tolerance.
    // Translated by 1000 mm such a chord lies within the rounding of both edges' offsets; the cut
    // still starts at its first point in travel (on the circle about the vertex), not its last.
    const double d = 3.0;
    const double tol = ProfileParams{}.tolerance.distance;
    for (double f : {0.5, 0.9, 1.0, 2.0}) {
        const std::vector<Vec2> v = tipWithNeighbour(Vec2{11.25, 3.5}, f * tol / d, 0.4, 5.0, 0);
        const Contour poly = polygon(v);
        for (CutDirection dir : {CutDirection::Climb, CutDirection::Conventional}) {
            ProfileParams p;
            p.side = ProfileSide::Outside;
            p.direction = dir;
            p.startAt = StartAt::VectorStart;
            p.passes.cutDepth = 3.0;
            p.passes.passDepth = 3.0;
            INFO("f " << f << (dir == CutDirection::Climb ? " climb" : " conventional"));
            const Vec2 s0 = startOf(profileToolpath({poly}, test::endMill(2.0 * d), p).toolpath).at;
            CHECK(std::fabs(geometry::distance(s0, v[0]) - d) < 1e-7);
            for (double k : fewPerturbations()) {
                INFO("scale 1 " << (k >= 1.0 ? "+ " : "- ") << std::fabs(k - 1.0));
                const Vec2 s = startOf(profileToolpath({movedPoly(poly, k, Vec2{})}, test::endMill(2.0 * d), p).toolpath).at;
                CHECK(geometry::distance(s, s0 * k) < 1e-7);
            }
            for (Vec2 o : {Vec2{3.0, -7.0}, Vec2{0.1, 0.3}, Vec2{1000.25, -999.75}}) {
                INFO("shift " << o.x << "," << o.y);
                const Vec2 s = startOf(profileToolpath({movedPoly(poly, 1.0, o)}, test::endMill(2.0 * d), p).toolpath).at;
                CHECK(geometry::distance(s - o, s0) < 1e-7);
            }
        }
    }
}

TEST_CASE("T-0174 (a): a squared-off tip whose start vertex has a degenerate edge next to it starts at the "
          "chamfer's first corner in travel, both directions, unmoved by perturbations",
          "[toolpath][profile][start][vbit]") {
    // T-0158 round-5 harness (repro5/sq_tiny_*): the tip's own edge on one side is a duplicate
    // vertex, a 1e-9 mm edge or a collinear 1e-6 mm edge. Such an edge is dropped by the offset and
    // labels no span, so the corner used to be missed, the fallback's "most even feet" tied exactly
    // between the chamfer's two corners and rounding picked one (a jump of 4.3 mm). The vertex's
    // own edges are the nearest non-degenerate ones, and a tie goes to the first in travel.
    const Vec2 p0{11.25, 3.5};
    const std::vector<Vec2> ring{p0,
                                 {-0x1.99190e11a658ap+3, 0x1.574441cf2ce06p+4},
                                 {-0x1.1154404f58dc6p+4, 0x1.5a8bfee8fc9ap+5},
                                 {-0x1.561bf995de8c8p+4, 0x1.e5d773a98b0f4p+3},
                                 {6.25, 3.5}};
    const Vec2 u1 = (ring[1] - p0) * (1.0 / (ring[1] - p0).length());   // along the out edge
    const Vec2 u2 = (ring[4] - p0) * (1.0 / (ring[4] - p0).length());   // back along the in edge
    const Vec2 out = (u1 + u2) * (-1.0 / (u1 + u2).length());
    const double dist = 3.0;
    // Squared off: the mitre would be longer than twice the offset.
    const double sinHalf = std::sqrt(0.5 * (1.0 - u1.dot(u2)));
    REQUIRE(dist / sinHalf > 2.0 * dist);

    enum class Kind { Duplicate, Collinear1e6, Closing1e9, CollinearBefore };
    auto build = [&](Kind kind) {
        Contour c;
        c.closed = true;
        std::vector<Vec2> v = ring;
        const Vec2 along = u1 * 1e-6;
        switch (kind) {
            case Kind::Duplicate:
                c.spans.push_back(LineSpan{p0, p0});
                break;
            case Kind::Collinear1e6:
                v.insert(v.begin() + 1, p0 + along);
                break;
            case Kind::Closing1e9:
                v.push_back(p0 + u2 * 1e-9);
                break;
            case Kind::CollinearBefore:
                v.push_back(p0 + u2 * 1e-6);
                break;
        }
        for (std::size_t i = 0; i < v.size(); ++i) {
            c.spans.push_back(LineSpan{v[i], v[(i + 1) % v.size()]});
        }
        return c;
    };
    auto offLine = [&](Vec2 q, Vec2 u) { return std::fabs(std::fabs((q - p0).cross(u)) - dist); };
    for (Kind kind : {Kind::Duplicate, Kind::Collinear1e6, Kind::Closing1e9, Kind::CollinearBefore}) {
        Vec2 corner[2];
        int k = 0;
        for (CutDirection dir : {CutDirection::Climb, CutDirection::Conventional}) {
            ProfileParams p;
            p.side = ProfileSide::Outside;
            p.direction = dir;
            p.startAt = StartAt::VectorStart;
            p.passes.cutDepth = 3.0;
            p.passes.passDepth = 3.0;
            p.safeZ = 5.0;
            p.corners.sharpExternal = true;
            p.corners.mitreLimit = 2.0;
            INFO("kind " << static_cast<int>(kind) << (dir == CutDirection::Climb ? " climb" : " conventional"));
            const Contour c = build(kind);
            const ProfileResult base = profileToolpath({c}, vBit90(), p);
            REQUIRE(base.ok());
            const Start s0 = startOf(base.toolpath);
            REQUIRE(s0.found);
            CHECK(s0.first.kind == MoveKind::Feed);
            if (kind == Kind::Collinear1e6) {
                // An edge shorter than the tolerance makes the cut a merged region (offset by the
                // chordal tolerance more, 3.01 mm), whose squared-off corner has three points:
                // the start is the first of them in travel, beyond the offset distance from the tip.
                CHECK(geometry::distance(s0.at, p0) > dist + 1e-3);
            } else {
                // A corner of the chamfer, which lies at the offset distance from the tip across
                // the bisector, and on one of the offset lines.
                CHECK(std::fabs((s0.at - p0).dot(out) - dist) < 1e-5);
                CHECK(std::min(offLine(s0.at, u1), offLine(s0.at, u2)) < 1e-5);
                CHECK(geometry::distance(s0.at, p0) > dist + 1e-3);
                // The first corner in travel: the cut runs along the chamfer to the other one.
                CHECK(std::fabs((s0.first.to.xy() - p0).dot(out) - dist) < 1e-5);
            }
            corner[k++] = s0.at;
            for (double s : perturbations()) {
                INFO("scale 1 " << (s >= 1.0 ? "+ " : "- ") << std::fabs(s - 1.0));
                const ProfileResult r = profileToolpath({scaled(c, s)}, vBit90(), p);
                REQUIRE(r.ok());
                const Start st = startOf(r.toolpath);
                CHECK(geometry::distance(st.at, s0.at * s) < 1e-5);
                CHECK(r.toolpath.moves.size() == base.toolpath.moves.size());
            }
            for (Vec2 o : {Vec2{3.0, -7.0}, Vec2{0.1, 0.3}, Vec2{-11.3, 4.7}}) {
                INFO("shift " << o.x << "," << o.y);
                Contour moved = c;
                for (geometry::Span& sp : moved.spans) {
                    auto& l = std::get<LineSpan>(sp);
                    l.start = l.start + o;
                    l.end = l.end + o;
                }
                const Start st = startOf(profileToolpath({moved}, vBit90(), p).toolpath);
                CHECK(geometry::distance(st.at - o, s0.at) < 1e-5);
            }
        }
        // Running the other way round the vector starts at the chamfer's other corner.
        CHECK(geometry::distance(corner[0], corner[1]) > 1.0);
    }
}

TEST_CASE("T-0174 (b): a merged region's mitre at a tip next to an arc edge does not move by a chord as the "
          "arc's radius crosses the labelling band",
          "[toolpath][profile][start][vbit]") {
    // T-0158 round-5 harness (repro5/mrgarc_*): the last chord of a region's polygonised arc is
    // stretched by the mitre, and how far it lies off the offset circle grows continuously with
    // the radius; it used to be labelled as the arc's offset or not by whether that deviation
    // exceeded the chordal band, so the start jumped to the chord's start (1.67 mm) at razor's
    // edge. The chord is on its polygon chord's offset to rounding, whatever its sag.
    const Vec2 tip{11.25, 3.5};
    const Vec2 left{-18.75, 3.5};
    auto scene = [&](double R) {
        Contour a;
        a.closed = true;
        a.spans.push_back(LineSpan{tip, Vec2{0x1.b75858eb9b623p+4, 0x1.cbe7f46187c1fp+4}});
        a.spans.push_back(LineSpan{Vec2{0x1.b75858eb9b623p+4, 0x1.cbe7f46187c1fp+4},
                                   Vec2{0x1.16b0b1d736c46p+2, 0x1.48f9fd1861f08p+5}});
        a.spans.push_back(LineSpan{Vec2{0x1.16b0b1d736c46p+2, 0x1.48f9fd1861f08p+5}, left});
        const Vec2 c{-3.75, 3.5 + std::sqrt(R * R - 225.0)};
        a.spans.push_back(ArcSpan::fromCentre(c, left, tip, ArcDirection::CCW));
        const std::vector<Vec2> q{{0x1.154121585a44ep+3, 0x1.6b76fba492e8cp+3},
                                  {0x1.8df8e997c884ap+4, 0x1.24d1b919e89b2p+5},
                                  {0x1.c4cbd21fad384p+0, 0x1.87d7bc0186aabp+5},
                                  {-0x1.555f6f53d2dd9p+4, 0x1.6b76fba492e8cp+3}};
        return std::vector<Contour>{a, polygon(q)};
    };
    // A V-bit with a sharp (mitred) corner, and a 6 mm end mill with the region's round join (whose
    // first chords lie on the arc's offset to within the band, so one chord more or less is labelled
    // as the arc's).
    struct Case {
        bool vbit;
        double r0;
        double r1;
    };
    for (const Case cs : {Case{true, 20.77190, 20.77215}, Case{false, 18.96680, 18.96695}}) {
        ProfileParams p;
        p.side = ProfileSide::Outside;
        p.direction = CutDirection::Conventional;
        p.startAt = StartAt::VectorStart;
        p.passes.cutDepth = 3.0;
        p.passes.passDepth = 3.0;
        p.safeZ = 5.0;
        p.corners.sharpExternal = cs.vbit;
        p.corners.mitreLimit = 50.0;
        tooling::Tool tool = cs.vbit ? vBit90() : test::endMill(6.0);
        // The plunge of the cut that starts nearest the tip (the other region start is far away).
        auto startNearTip = [&](double R) {
            const ProfileResult r = profileToolpath(scene(R), tool, p);
            REQUIRE(r.ok());
            Vec2 best{1e9, 1e9};
            for (const Move& m : r.toolpath.moves) {
                if (m.kind == MoveKind::Plunge && geometry::distance(m.to.xy(), tip) < geometry::distance(best, tip)) {
                    best = m.to.xy();
                }
            }
            return best;
        };
        Vec2 prev = startNearTip(cs.r0);
        for (int k = 1; k <= 25; ++k) {
            const double R = cs.r0 + (cs.r1 - cs.r0) * k / 25.0;
            INFO("vbit " << cs.vbit << " R " << R);
            const Vec2 s = startNearTip(R);
            // The start moves by micrometres with R, never by a chord (0.4 to 1.7 mm).
            CHECK(geometry::distance(s, prev) < 1e-3);
            prev = s;
        }
    }
}

TEST_CASE("T-0174 (c): a merged region's round join next to an arc edge starts at the tip's foot on the "
          "offset of the arc's last polygon chord",
          "[toolpath][profile][start][vbit]") {
    // Generated scene (harness fuzz, mutant N11: no chordal band on region arcs). An arc edge ends
    // at the tip (a V-bit, Outside, round joins, merged with a quadrilateral). The corner is the
    // region's polygonal round join; its first point in travel is the tip's foot on the offset of
    // the arc's last polygon chord (a chord of the arc as the region polygonises it), 3.29 mm from
    // the tip (the join's other end running the other way), not a later chord vertex of the join
    // (0.2 to 0.4 mm on; the join starts tangent to the arc's offset, so its first chords lie
    // within the chordal band of it).
    const Vec2 tip{0x1.68p+3, 0x1.cp+1};
    const Vec2 e1{0x1.c3f6ea6a8ef48p+4, 0x1.cecf0144eb5d8p+4};
    const Vec2 e2{-0x1.4924c5e104ddep+2, 0x1.1dc714ad7937p+5};
    const Vec2 left{-0x1.2cp+4, 0x1.cp+1};
    const Vec2 centre{-0x1.ep+1, 0x1.ee1f12e6d7c7bp+3};
    Contour a;
    a.closed = true;
    a.spans.push_back(LineSpan{tip, e1});
    a.spans.push_back(LineSpan{e1, e2});
    a.spans.push_back(LineSpan{e2, left});
    const ArcSpan arc = ArcSpan::fromCentre(centre, left, tip, ArcDirection::CCW);
    a.spans.push_back(arc);
    const Contour b = polygon({{0x1.78aab5447b24cp+3, 0x1.17a434dbb845p+3},
                               {0x1.0123a662f656ap+5, 0x1.28509a27ba24cp+4},
                               {0x1.5ad5ee16bd04ep+1, 0x1.c09df5fd79ceap+4},
                               {-0x1.222b703685a62p+4, 0x1.2736d63e30297p+3}});
    tooling::Tool v = test::endMill(20.0);
    v.type = tooling::ToolType::VBit;
    v.includedAngle = 1.4136692725854356;
    ProfileParams p;
    p.side = ProfileSide::Outside;
    p.startAt = StartAt::VectorStart;
    p.passes.cutDepth = 3.8409622087136626;
    p.passes.passDepth = p.passes.cutDepth;
    p.safeZ = 5.0;
    p.corners.sharpExternal = false;
    const double d = profileToolRadius(v, p.passes) + p.tolerance.chordal;
    // The region polygonises the arc as a closed vector does; its last chord ends at the tip.
    std::vector<Vec2> pts{arc.startPoint()};
    geometry::polygoniseRegionInto(arc, p.tolerance, pts);
    const Vec2 chord = pts.back() - pts[pts.size() - 2];
    const Vec2 outward = Vec2{chord.y, -chord.x} * (1.0 / chord.length());  // right of travel: outside
    REQUIRE(outward.dot(tip - centre) > 0.0);
    const Vec2 footIn = tip + outward * d;
    const Vec2 ob = (e1 - tip) * (1.0 / (e1 - tip).length());
    const Vec2 footOut = tip + Vec2{ob.y, -ob.x} * d;  // right of the out edge's direction
    for (CutDirection dir : {CutDirection::Climb, CutDirection::Conventional}) {
        p.direction = dir;
        const ProfileResult r = profileToolpath({a, b}, v, p);
        REQUIRE(r.ok());
        Vec2 best{1e9, 1e9};
        for (const Move& m : r.toolpath.moves) {
            if (m.kind == MoveKind::Plunge && geometry::distance(m.to.xy(), tip) < geometry::distance(best, tip)) {
                best = m.to.xy();
            }
        }
        INFO("dir " << (dir == CutDirection::Climb ? "climb" : "conventional") << " start " << best.x << "," << best.y);
        const double dIn = geometry::distance(best, footIn);
        const double dOut = geometry::distance(best, footOut);
        // The join's first point in travel: Conventional runs the vector's way (the join from the
        // foot on the arc's offset to the foot on the out edge's), Climb against it.
        CHECK((dir == CutDirection::Conventional ? dIn : dOut) < 1e-4);
    }
}

TEST_CASE("T-0174 (d): a mitred tip between two Bezier edges starts at the mitre of their chords at the vertex",
          "[toolpath][profile][start][vbit]") {
    // Generated scene (harness fuzz, mutant N8: Bezier edges not replaced by their chord). A
    // Bezier is offset as its polygon, so the corner is built between the offsets of the chords at
    // the vertex; against the Bezier itself no span of the cut is on its offset, no corner is found
    // and the fallback starts the cut elsewhere (31 mm from the mitre here).
    const Vec2 a{0x1.8f6e105f53b84p+1, 0x1.95887f4e08e68p+1};
    const Vec2 q1{0x1.8a1f80c4f467cp+4, 0x1.244ce2e5f0cfp+3};
    const Vec2 q2{0x1.7c7c551635b81p+4, 0x1.c6c7893296af6p+4};
    const Vec2 q3{0x1.a3702a60842acp+4, 0x1.95887f4e08e68p+1};
    const geometry::BezierSpan outEdge{a,
                                       {0x1.24deea2a11442p+3, 0x1.a5a2504cf2ae7p+1},
                                       {0x1.14e111f332c22p+4, 0x1.f2666ab2f2bf2p+2},
                                       q1};
    const geometry::BezierSpan inEdge{q3,
                                      {0x1.519422abf9164p+4, 0x1.8bd4aefd0d3f8p+1},
                                      {0x1.883395723744ep+3, 0x1.3dc5002c3d352p+2},
                                      a};
    Contour c;
    c.closed = true;
    c.spans.push_back(outEdge);
    c.spans.push_back(LineSpan{q1, q2});
    c.spans.push_back(LineSpan{q2, q3});
    c.spans.push_back(inEdge);
    tooling::Tool v = test::endMill(20.0);
    v.type = tooling::ToolType::VBit;
    v.includedAngle = 1.6507562939419393;
    ProfileParams p;
    p.side = ProfileSide::Outside;
    p.startAt = StartAt::VectorStart;
    p.passes.cutDepth = 2.3639215142316061;
    p.passes.passDepth = p.passes.cutDepth;
    p.safeZ = 5.0;
    p.corners.sharpExternal = true;
    p.corners.mitreLimit = 50.0;
    // The chords at the vertex, the curves polygonised as the offset does (at half the distance
    // tolerance).
    geometry::Tolerance fine = p.tolerance;
    fine.chordal = 0.5 * fine.distance;
    std::vector<Vec2> po{a};
    geometry::polygoniseInto(outEdge, fine, po);
    std::vector<Vec2> pi{q3};
    geometry::polygoniseInto(inEdge, fine, pi);
    const Vec2 uOut = (po[1] - po[0]) * (1.0 / (po[1] - po[0]).length());
    const Vec2 uIn = (pi.back() - pi[pi.size() - 2]) * (1.0 / (pi.back() - pi[pi.size() - 2]).length());
    const double dist = profileToolRadius(v, p.passes);
    const Vec2 mitre = mitreAt(a, uIn, uOut, dist);
    for (CutDirection dir : {CutDirection::Climb, CutDirection::Conventional}) {
        p.direction = dir;
        const Vec2 s0 = startOf(profileToolpath({c}, v, p).toolpath).at;
        INFO("dir " << (dir == CutDirection::Climb ? "climb" : "conventional") << " mitre " << mitre.x << "," << mitre.y
                    << " start " << s0.x << "," << s0.y);
        // The mitre is far (30 mm) and moves with the chords' direction (5 um here at another
        // polygonisation): the check is against the 31 mm miss.
        CHECK(geometry::distance(s0, mitre) < 0.05);
        for (double k : perturbations()) {
            INFO("scale 1 " << (k >= 1.0 ? "+ " : "- ") << std::fabs(k - 1.0));
            const Vec2 s = startOf(profileToolpath({scaled(c, k)}, v, p).toolpath).at;
            CHECK(geometry::distance(s, s0 * k) < 1e-5);
        }
    }
}
