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
