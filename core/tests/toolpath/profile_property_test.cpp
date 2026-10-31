// T-0120 AC6: randomized gouge test. For random scenes of closed and open vectors (lines and arcs,
// placed so that some lie close to or inside each other) and random profile options, no cutting
// move (feed, arc, plunge, retract, leads, ramps, tabs) may come closer than tool radius - tol to
// any input vector. Distances are measured by the brute-force oracle in profile_test_util.hpp,
// which shares no code with the offset or clearance code under test.

#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "../geometry/test_rng.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "profile_test_util.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using carvetto::geometry::ArcDirection;
using carvetto::geometry::ArcSpan;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::LineSpan;
using carvetto::geometry::Vec2;

namespace {

Contour roundedRect(Vec2 c, double w, double h, double r) {
    const double x0 = c.x - w / 2;
    const double y0 = c.y - h / 2;
    const double x1 = c.x + w / 2;
    const double y1 = c.y + h / 2;
    const double hp = geometry::kPi / 2;
    Contour k;
    k.closed = true;
    k.append(LineSpan{{x0 + r, y0}, {x1 - r, y0}});
    k.append(ArcSpan{{x1 - r, y0 + r}, r, -hp, 0, ArcDirection::CCW});
    k.append(LineSpan{{x1, y0 + r}, {x1, y1 - r}});
    k.append(ArcSpan{{x1 - r, y1 - r}, r, 0, hp, ArcDirection::CCW});
    k.append(LineSpan{{x1 - r, y1}, {x0 + r, y1}});
    k.append(ArcSpan{{x0 + r, y1 - r}, r, hp, 2 * hp, ArcDirection::CCW});
    k.append(LineSpan{{x0, y1 - r}, {x0, y0 + r}});
    k.append(ArcSpan{{x0 + r, y0 + r}, r, 2 * hp, 3 * hp, ArcDirection::CCW});
    return k;
}

// Star-shaped polygon with `n` vertices at random radii (edges ≥ ~1 mm).
Contour starPolygon(test::SplitMix& rng, Vec2 c, double rMin, double rMax, int n) {
    std::vector<Vec2> pts;
    for (int i = 0; i < n; ++i) {
        const double a = geometry::kTwoPi * i / n;
        pts.push_back(c + Vec2::fromAngle(a, rng.uniform(rMin, rMax)));
    }
    Contour k;
    k.closed = true;
    for (int i = 0; i < n; ++i) {
        k.append(LineSpan{pts[static_cast<std::size_t>(i)], pts[static_cast<std::size_t>((i + 1) % n)]});
    }
    return k;
}

Contour openPath(test::SplitMix& rng, Vec2 c) {
    Contour k;
    Vec2 p = c;
    double heading = rng.uniform(0, geometry::kTwoPi);
    const int n = 2 + static_cast<int>(rng.next() % 3);
    for (int i = 0; i < n; ++i) {
        if (rng.uniform(0, 1) < 0.5) {
            const Vec2 q = p + Vec2::fromAngle(heading, rng.uniform(5, 20));
            k.append(LineSpan{p, q});
            p = q;
        } else {
            // Tangent-continuous arc turning left or right.
            const double r = rng.uniform(4, 15);
            const bool left = rng.uniform(0, 1) < 0.5;
            const Vec2 n2 = left ? Vec2::fromAngle(heading + geometry::kPi / 2) : Vec2::fromAngle(heading - geometry::kPi / 2);
            const Vec2 centre = p + n2 * r;
            const double a0 = (p - centre).angle();
            const double sweep = rng.uniform(0.3, 1.5);
            const ArcSpan arc{centre, r, a0, left ? a0 + sweep : a0 - sweep, left ? ArcDirection::CCW : ArcDirection::CW};
            k.append(arc);
            p = arc.endPoint();
            heading += left ? sweep : -sweep;
        }
        heading += rng.uniform(-1.2, 1.2);  // a corner (either way)
    }
    return k;
}

std::vector<Contour> randomScene(test::SplitMix& rng, bool& hasOpen) {
    std::vector<Contour> cs;
    hasOpen = false;
    const int n = 1 + static_cast<int>(rng.next() % 4);
    for (int i = 0; i < n; ++i) {
        const Vec2 c{rng.uniform(0, 80), rng.uniform(0, 60)};
        switch (rng.next() % 5) {
            case 0:
                cs.push_back(Contour::rectangle(Box2::fromCentre(c, rng.uniform(4, 40), rng.uniform(4, 30))));
                break;
            case 1:
                cs.push_back(Contour::circle(c, rng.uniform(2, 18), rng.uniform(0, 1) < 0.5 ? ArcDirection::CCW : ArcDirection::CW));
                break;
            case 2: {
                const double w = rng.uniform(8, 40);
                const double h = rng.uniform(8, 30);
                cs.push_back(roundedRect(c, w, h, rng.uniform(1, std::min(w, h) / 2 - 0.5)));
                break;
            }
            case 3:
                cs.push_back(starPolygon(rng, c, 5, 18, 5 + static_cast<int>(rng.next() % 5)));
                break;
            default:
                cs.push_back(openPath(rng, c));
                hasOpen = true;
                break;
        }
    }
    // Sometimes a vector nested inside the first one.
    if (rng.uniform(0, 1) < 0.3 && cs.front().closed) {
        const Box2 b = cs.front().bounds();
        cs.push_back(Contour::circle(b.centre(), std::min(b.width(), b.height()) * rng.uniform(0.1, 0.3)));
    }
    return cs;
}

ProfileParams randomParams(test::SplitMix& rng, bool hasOpen) {
    ProfileParams p;
    p.side = rng.uniform(0, 1) < 0.5 ? ProfileSide::Outside : ProfileSide::Inside;
    p.direction = rng.uniform(0, 1) < 0.5 ? CutDirection::Climb : CutDirection::Conventional;
    p.allowance = rng.uniform(0, 1) < 0.5 ? 0.0 : rng.uniform(0, 1);
    p.passes.cutDepth = rng.uniform(2, 12);
    p.passes.passDepth = rng.uniform(1.5, 6);
    p.separateLastPass = rng.uniform(0, 1) < 0.3;
    p.lastPassAllowance = rng.uniform(0.05, 0.5);
    p.lastPassReverse = rng.uniform(0, 1) < 0.5;
    p.tabs.enabled = rng.uniform(0, 1) < 0.5;
    p.tabs.count = 1 + static_cast<int>(rng.next() % 4);
    p.tabs.length = rng.uniform(2, 8);
    p.tabs.thickness = rng.uniform(0.5, 3);
    p.tabs.threeD = rng.uniform(0, 1) < 0.3;
    p.tabs.avoidCorners = rng.uniform(0, 1) < 0.3;
    const RampType ramps[] = {RampType::None, RampType::Smooth, RampType::ZigZag, RampType::Spiral};
    p.ramp.type = ramps[rng.next() % 4];
    p.ramp.distance = rng.uniform(3, 30);
    p.ramp.onLeadIn = rng.uniform(0, 1) < 0.3;
    const LeadType leads[] = {LeadType::None, LeadType::Straight, LeadType::Circular};
    p.leads.type = leads[rng.next() % 3];
    p.leads.length = rng.uniform(2, 15);
    p.leads.angle = rng.uniform(0.2, geometry::kPi / 2);
    p.leads.radius = rng.uniform(2, 10);
    p.leads.sweep = rng.uniform(0.3, geometry::kPi / 2);
    p.leads.leadOut = rng.uniform(0, 1) < 0.5;
    p.leads.overcut = rng.uniform(0, 1) < 0.5 ? 0.0 : rng.uniform(0, 5);
    const StartAt starts[] = {StartAt::VectorStart, StartAt::Optimise, StartAt::ClosestOnBoundingBox};
    p.startAt = starts[rng.next() % 3];
    p.startAnchor = static_cast<BoxAnchor>(rng.next() % 9);
    p.order.grid = rng.uniform(0, 1) < 0.3;
    (void)hasOpen;
    return p;
}

}  // namespace

TEST_CASE("AC6: random scenes never bring a cutting move within tool radius of a vector",
          "[toolpath][profile][property]") {
    test::SplitMix rng{0x7120A6ULL};
    const double tol = geometry::Tolerance::defaults().distance;
    int produced = 0;
    int merged = 0;
    int trimmed = 0;
    const int kScenes = 120;
    for (int s = 0; s < kScenes; ++s) {
        bool hasOpen = false;
        const std::vector<Contour> cs = randomScene(rng, hasOpen);
        const ProfileParams p = randomParams(rng, hasOpen);
        const double d = rng.uniform(3, 10);
        const ProfileResult r = profileToolpath(cs, test::endMill(d), p);
        if (!r.ok()) {
            CHECK_FALSE(r.diagnostics.empty());
            continue;
        }
        ++produced;
        merged += r.has(ProfileIssue::MergedWithNeighbour) ? 1 : 0;
        trimmed += r.has(ProfileIssue::TrimmedNearVector) ? 1 : 0;
        // Sample every 0.02 mm: between samples the true distance can be lower by at most 0.01.
        const double minD = test::minCutDistance(r.toolpath, cs, 0.02);
        INFO("scene " << s << " tool " << d << " side " << static_cast<int>(p.side));
        CHECK(minD >= d / 2 - tol);
        // Cutting never starts or travels below the bottom of the cut.
        CHECK(*r.toolpath.minZ() >= -(p.passes.cutDepth) - 1e-9);
    }
    // The generator exercises the interesting paths.
    CHECK(produced > kScenes / 2);
    CHECK(merged > 0);
    CHECK(trimmed > 0);
}

TEST_CASE("AC6: Bezier vectors stay gouge-free too", "[toolpath][profile][property]") {
    Contour blob;
    blob.closed = true;
    blob.append(geometry::BezierSpan{{0, 0}, {20, -10}, {40, 10}, {30, 30}});
    blob.append(geometry::BezierSpan{{30, 30}, {20, 45}, {-10, 30}, {0, 0}});
    for (ProfileSide side : {ProfileSide::Outside, ProfileSide::Inside}) {
        ProfileParams p;
        p.side = side;
        p.passes.cutDepth = 3.0;
        p.passes.passDepth = 3.0;
        const ProfileResult r = profileToolpath({blob}, test::endMill(6.0), p);
        REQUIRE(r.ok());
        CHECK(test::minCutDistance(r.toolpath, {blob}, 0.02) >= 3.0 - 1e-3);
    }
}
