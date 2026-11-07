#include "carvetto/viewmodel/snapping.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <iostream>

#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/intersect.hpp"

using namespace carvetto::document;
using namespace carvetto::geometry;
using namespace carvetto::viewmodel;
using Catch::Matchers::WithinAbs;

namespace {

/// Deterministic generator (no <random> distributions: their output differs between standard libraries).
struct Rng {
    std::uint64_t s{0x9E3779B97F4A7C15ULL};
    explicit Rng(std::uint64_t seed) : s(seed * 0x2545F4914F6CDD1DULL + 12345ULL) {}
    double next() {  // [0,1)
        s ^= s >> 12;
        s ^= s << 25;
        s ^= s >> 27;
        return static_cast<double>((s * 0x2545F4914F6CDD1DULL) >> 11) / 9007199254740992.0;
    }
    double range(double lo, double hi) { return lo + (hi - lo) * next(); }
};

struct Fixture {
    Document doc;
    ViewTransform view;
    explicit Fixture(double sizeMm = 1000.0) : doc(Job::singleSided(sizeMm, sizeMm, 18.0, Units::Millimetres)) {
        view.setViewport(1000.0, 1000.0);
        view.set(2.0, {100.0, 900.0});  // 2 px per mm
    }
    Vec2 px(Vec2 mm) const { return view.worldToScreen(mm); }

    ObjectId add(Contour c, CreationParams params = NoParams{}) {
        return doc.insertObject(Object::vector(std::move(c), doc.activeLayer(), doc.activeSheet(), std::move(params)));
    }
    ObjectId addOn(LayerId layer, Contour c) {
        return doc.insertObject(Object::vector(std::move(c), layer, doc.activeSheet()));
    }
    ObjectId line(Vec2 a, Vec2 b) {
        Contour c;
        c.append(LineSpan{a, b});
        return add(std::move(c));
    }
};

SnapSettings only(SnapKind kind, double tolPx = 10.0) {
    SnapSettings s;
    s.disableAll();
    s.setEnabled(kind, true);
    s.tolerancePx = tolPx;
    return s;
}

std::vector<Snap> ofKind(const std::vector<Snap>& all, SnapKind kind) {
    std::vector<Snap> out;
    for (const Snap& s : all) {
        if (s.kind == kind) {
            out.push_back(s);
        }
    }
    return out;
}

bool hasPoint(const std::vector<Snap>& snaps, Vec2 p, double tol = 1e-6) {
    return std::any_of(snaps.begin(), snaps.end(), [&](const Snap& s) { return distance(s.pointMm, p) <= tol; });
}

Contour polyline(std::initializer_list<Vec2> pts, bool closed) {
    Contour c;
    std::vector<Vec2> v(pts);
    for (std::size_t i = 0; i + 1 < v.size(); ++i) {
        c.append(LineSpan{v[i], v[i + 1]});
    }
    if (closed) {
        c.append(LineSpan{v.back(), v.front()});
    }
    c.closed = closed;
    return c;
}

}  // namespace

TEST_CASE("endpoint snaps to span ends within the pixel tolerance only", "[viewmodel][snap]") {
    Fixture f;
    const ObjectId id = f.line({10.0, 10.0}, {50.0, 10.0});
    const SnapIndex idx(f.doc);
    const SnapSettings s = only(SnapKind::Endpoint, 10.0);

    auto hit = idx.best(f.view, f.px({10.0, 10.0}) + Vec2{6.0, -6.0}, s);  // 8.49 px away
    REQUIRE(hit.has_value());
    CHECK(hit->kind == SnapKind::Endpoint);
    CHECK(hit->pointMm == Vec2{10.0, 10.0});
    CHECK_THAT(hit->distancePx, WithinAbs(std::hypot(6.0, 6.0), 1e-9));
    REQUIRE(hit->objects.size() == 1);
    CHECK(hit->objects[0] == id);

    CHECK_FALSE(idx.best(f.view, f.px({10.0, 10.0}) + Vec2{8.0, -8.0}, s).has_value());  // 11.3 px
    // The tolerance is in pixels: zooming in makes the same world offset too far.
    ViewTransform zoomed = f.view;
    zoomed.zoomAbout(f.px({10.0, 10.0}), 4.0);  // 8 px per mm
    CHECK(idx.best(zoomed, zoomed.worldToScreen({10.0, 10.0}) + Vec2{6.0, 0.0}, s).has_value());
    // The same world offset (3 mm) is 6 px at scale 2 (snaps) and 24 px at scale 8 (does not).
    CHECK(idx.best(f.view, f.px({13.0, 10.0}), s).has_value());
    CHECK_FALSE(idx.best(zoomed, zoomed.worldToScreen({13.0, 10.0}), s).has_value());
    // Zero tolerance snaps only exactly.
    SnapSettings zero = s;
    zero.tolerancePx = 0.0;
    CHECK(idx.best(f.view, f.px({50.0, 10.0}), zero).has_value());
    CHECK_FALSE(idx.best(f.view, f.px({50.0, 10.0}) + Vec2{0.01, 0.0}, zero).has_value());
}

TEST_CASE("midpoint centre and quadrant of lines, arcs, circles and parametric objects", "[viewmodel][snap]") {
    Fixture f;
    f.line({0.0, 0.0}, {40.0, 20.0});
    Contour arc;
    arc.append(ArcSpan{{100.0, 100.0}, 20.0, 0.25, 2.0, ArcDirection::CCW});  // contains 90 deg only
    f.add(arc);
    f.add(Contour::circle({200.0, 100.0}, 10.0), CircleParams{{200.0, 100.0}, 10.0});
    f.add(polyline({{300.0, 100.0}, {340.0, 100.0}, {340.0, 120.0}, {300.0, 120.0}}, true),
          RectangleParams{{320.0, 110.0}, 40.0, 20.0, RectangleParams::CornerStyle::Square, 0.0});
    const SnapIndex idx(f.doc);

    auto mid = idx.candidates(f.view, f.px({20.0, 10.0}), only(SnapKind::Midpoint));
    REQUIRE(mid.size() == 1);
    CHECK(mid[0].pointMm == Vec2{20.0, 10.0});

    // Arc midpoint is the arc-length midpoint (angle 1.125 rad).
    const Vec2 am = Vec2{100.0, 100.0} + Vec2::fromAngle(1.125, 20.0);
    auto amid = idx.candidates(f.view, f.px(am), only(SnapKind::Midpoint));
    REQUIRE(amid.size() == 1);
    CHECK(distance(amid[0].pointMm, am) < 1e-9);

    // Centre of an arc and of a circle (arc and params agree: one merged candidate), and of a rectangle.
    CHECK(hasPoint(idx.candidates(f.view, f.px({100.0, 100.0}), only(SnapKind::Centre)), {100.0, 100.0}));
    auto cc = idx.candidates(f.view, f.px({200.0, 100.0}), only(SnapKind::Centre));
    REQUIRE(cc.size() == 1);
    CHECK(cc[0].pointMm == Vec2{200.0, 100.0});
    CHECK(hasPoint(idx.candidates(f.view, f.px({320.0, 110.0}), only(SnapKind::Centre)), {320.0, 110.0}));

    // Quadrants: the arc has only the 90 deg point; the circle has four.
    auto q = idx.candidates(f.view, f.px({100.0, 120.0}), only(SnapKind::Quadrant));
    REQUIRE(q.size() == 1);
    CHECK(q[0].pointMm == Vec2{100.0, 120.0});
    CHECK(idx.candidates(f.view, f.px({120.0, 100.0}), only(SnapKind::Quadrant)).empty());  // 0 deg not on arc
    for (const Vec2 p : {Vec2{210.0, 100.0}, Vec2{200.0, 110.0}, Vec2{190.0, 100.0}, Vec2{200.0, 90.0}}) {
        CHECK(hasPoint(idx.candidates(f.view, f.px(p), only(SnapKind::Quadrant)), p));
    }
}

TEST_CASE("ellipse parameters give rotated quadrant points", "[viewmodel][snap]") {
    Fixture f;
    EllipseParams e{{50.0, 50.0}, 40.0, 20.0, 0.5};
    f.add(Contour::circle({50.0, 50.0}, 1.0), e);  // geometry is irrelevant here
    const SnapIndex idx(f.doc);
    const Vec2 tip = Vec2{50.0, 50.0} + Vec2::fromAngle(0.5, 20.0);
    CHECK(hasPoint(idx.candidates(f.view, f.px(tip), only(SnapKind::Quadrant)), tip, 1e-9));
    const Vec2 side = Vec2{50.0, 50.0} + Vec2::fromAngle(0.5 + kPi / 2.0, 10.0);
    CHECK(hasPoint(idx.candidates(f.view, f.px(side), only(SnapKind::Quadrant)), side, 1e-9));
}

TEST_CASE("bezier midpoint is the arc-length midpoint", "[viewmodel][snap]") {
    Fixture f;
    Contour c;
    c.append(BezierSpan{{0.0, 0.0}, {60.0, 80.0}, {10.0, -50.0}, {100.0, 10.0}});
    f.add(c);
    const BezierSpan b = std::get<BezierSpan>(c.spans[0]);
    const double half = 0.5 * b.length();
    double lo = 0.0;
    double hi = 1.0;
    for (int i = 0; i < 80; ++i) {
        const double m = 0.5 * (lo + hi);
        (b.split(m).first.length() < half ? lo : hi) = m;
    }
    const Vec2 expect = b.pointAt(0.5 * (lo + hi));
    const SnapIndex idx(f.doc);
    auto mid = idx.candidates(f.view, f.px(expect), only(SnapKind::Midpoint));
    REQUIRE(mid.size() == 1);
    CHECK(distance(mid[0].pointMm, expect) < 1e-6);
}

TEST_CASE("intersection snaps agree with intersect() and skip shared junctions", "[viewmodel][snap]") {
    Fixture f;
    const ObjectId a = f.line({0.0, 0.0}, {100.0, 100.0});
    const ObjectId b = f.line({0.0, 100.0}, {100.0, 0.0});
    Contour arc;
    arc.append(ArcSpan{{50.0, 50.0}, 30.0, 0.0, kPi, ArcDirection::CCW});
    const ObjectId c = f.add(arc);
    const SnapIndex idx(f.doc);
    const Tolerance tol;

    // Line x line at (50,50).
    auto x = idx.candidates(f.view, f.px({50.0, 50.0}), only(SnapKind::Intersection));
    REQUIRE(x.size() == 1);
    CHECK(distance(x[0].pointMm, {50.0, 50.0}) < 1e-9);
    CHECK(x[0].objects == std::vector<ObjectId>{a, b});

    // Line x arc: the same points intersect() reports.
    const auto r = intersect(Span{LineSpan{{0.0, 0.0}, {100.0, 100.0}}}, Span{arc.spans[0]}, tol);
    REQUIRE(!r.points.empty());
    for (const auto& hit : r.points) {
        auto got = idx.candidates(f.view, f.px(hit.point), only(SnapKind::Intersection));
        REQUIRE(!got.empty());
        CHECK(hasPoint(got, hit.point, 1e-9));
        const Snap& s = *std::find_if(got.begin(), got.end(), [&](const Snap& g) { return distance(g.pointMm, hit.point) < 1e-9; });
        CHECK(std::find(s.objects.begin(), s.objects.end(), a) != s.objects.end());
        CHECK(std::find(s.objects.begin(), s.objects.end(), c) != s.objects.end());
    }

    // The shared corner of a polyline is an endpoint, not an intersection.
    Fixture g;
    g.add(polyline({{0.0, 0.0}, {50.0, 0.0}, {50.0, 50.0}}, false));
    const SnapIndex idx2(g.doc);
    CHECK(idx2.candidates(g.view, g.px({50.0, 0.0}), only(SnapKind::Intersection)).empty());
    CHECK(hasPoint(idx2.candidates(g.view, g.px({50.0, 0.0}), only(SnapKind::Endpoint)), {50.0, 0.0}));
    // A closed square: wrap-around junction too.
    Fixture h;
    h.add(polyline({{0.0, 0.0}, {50.0, 0.0}, {50.0, 50.0}, {0.0, 50.0}}, true));
    const SnapIndex idx3(h.doc);
    for (const Vec2 p : {Vec2{0.0, 0.0}, Vec2{50.0, 0.0}, Vec2{50.0, 50.0}, Vec2{0.0, 50.0}}) {
        CHECK(idx3.candidates(h.view, h.px(p), only(SnapKind::Intersection)).empty());
    }
    // Bezier x line.
    Fixture k;
    Contour bz;
    bz.append(BezierSpan{{0.0, 0.0}, {30.0, 80.0}, {70.0, -80.0}, {100.0, 0.0}});
    k.add(bz);
    k.line({50.0, -50.0}, {50.0, 50.0});
    const SnapIndex idx4(k.doc);
    auto bx = idx4.candidates(k.view, k.px({50.0, 0.0}), only(SnapKind::Intersection));
    REQUIRE(bx.size() == 1);
    CHECK(distance(bx[0].pointMm, {50.0, 0.0}) < 1e-6);
    CHECK(bx[0].objects.size() == 2);
}

TEST_CASE("nearest on curve lies on the curve at the cursor's foot point", "[viewmodel][snap]") {
    Fixture f;
    f.line({0.0, 0.0}, {100.0, 0.0});
    f.add(Contour::circle({50.0, 60.0}, 20.0));
    const SnapIndex idx(f.doc);
    auto n = idx.best(f.view, f.px({30.0, 2.0}), only(SnapKind::Nearest));
    REQUIRE(n.has_value());
    CHECK(distance(n->pointMm, {30.0, 0.0}) < 1e-9);
    CHECK_THAT(n->distancePx, WithinAbs(4.0, 1e-9));
    auto c = idx.best(f.view, f.px({73.0, 60.0}), only(SnapKind::Nearest));
    REQUIRE(c.has_value());
    CHECK(distance(c->pointMm, {70.0, 60.0}) < 1e-9);
    CHECK_FALSE(idx.best(f.view, f.px({30.0, 10.0}), only(SnapKind::Nearest)).has_value());  // 20 px away
}

TEST_CASE("perpendicular from a pending start", "[viewmodel][snap]") {
    Fixture f;
    f.line({0.0, 0.0}, {100.0, 0.0});
    f.add(Contour::circle({200.0, 0.0}, 30.0));
    const SnapIndex idx(f.doc);
    SnapQuery q;
    q.pendingStartMm = Vec2{40.0, 50.0};
    // Line foot (40,0).
    auto p = idx.candidates(f.view, f.px({41.0, 0.5}), only(SnapKind::Perpendicular), q);
    REQUIRE(p.size() == 1);
    CHECK(distance(p[0].pointMm, {40.0, 0.0}) < 1e-12);
    // Foot beyond the segment end does not exist.
    q.pendingStartMm = Vec2{130.0, 50.0};
    CHECK(idx.candidates(f.view, f.px({130.0, 0.0}), only(SnapKind::Perpendicular), q).empty());
    // Circle: the near and the far point on the line through the centre.
    q.pendingStartMm = Vec2{200.0, 100.0};
    CHECK(hasPoint(idx.candidates(f.view, f.px({200.0, 30.0}), only(SnapKind::Perpendicular), q), {200.0, 30.0}));
    CHECK(hasPoint(idx.candidates(f.view, f.px({200.0, -30.0}), only(SnapKind::Perpendicular), q), {200.0, -30.0}));
    // Without a pending start nothing is offered.
    CHECK(idx.candidates(f.view, f.px({200.0, 30.0}), only(SnapKind::Perpendicular)).empty());
}

TEST_CASE("perpendicular and tangent points on Beziers satisfy their conditions", "[viewmodel][snap]") {
    Fixture f;
    Contour c;
    const BezierSpan b{{0.0, 0.0}, {40.0, 90.0}, {80.0, -60.0}, {120.0, 30.0}};
    c.append(b);
    f.add(c);
    const SnapIndex idx(f.doc);
    SnapQuery q;
    q.pendingStartMm = Vec2{60.0, 70.0};
    SnapSettings s;
    s.disableAll();
    s.setEnabled(SnapKind::Perpendicular, true);
    s.setEnabled(SnapKind::Tangent, true);
    s.tolerancePx = 6.0;
    int perp = 0;
    int tang = 0;
    for (int i = 0; i <= 400; ++i) {
        const Vec2 on = b.pointAt(i / 400.0);
        for (const Snap& sn : idx.candidates(f.view, f.px(on), s, q)) {
            const ClosestPoint cp = closestPoint(Span{b}, sn.pointMm);
            CHECK(cp.distance < 1e-6);
            const Vec2 tan = b.tangentAt(cp.t);
            const Vec2 d = sn.pointMm - *q.pendingStartMm;
            if (sn.kind == SnapKind::Perpendicular) {
                CHECK_THAT(d.normalized().dot(tan), WithinAbs(0.0, 1e-6));
                ++perp;
            } else {
                CHECK_THAT(d.normalized().cross(tan), WithinAbs(0.0, 1e-6));
                ++tang;
            }
        }
    }
    CHECK(perp > 0);
    CHECK(tang > 0);
}

TEST_CASE("tangent from a pending start to a circle and an arc", "[viewmodel][snap]") {
    Fixture f;
    f.add(Contour::circle({0.0, 0.0}, 10.0));
    const SnapIndex idx(f.doc);
    SnapQuery q;
    q.pendingStartMm = Vec2{20.0, 0.0};
    const double ang = std::acos(10.0 / 20.0);
    const Vec2 t1 = Vec2::fromAngle(ang, 10.0);
    const Vec2 t2 = Vec2::fromAngle(-ang, 10.0);
    auto got = idx.candidates(f.view, f.px(t1), only(SnapKind::Tangent), q);
    REQUIRE(got.size() == 1);
    CHECK(distance(got[0].pointMm, t1) < 1e-9);
    CHECK(hasPoint(idx.candidates(f.view, f.px(t2), only(SnapKind::Tangent), q), t2, 1e-9));
    // Pending start inside the circle: no tangent. On a line: none either.
    q.pendingStartMm = Vec2{3.0, 0.0};
    CHECK(idx.candidates(f.view, f.px(t1), only(SnapKind::Tangent), q).empty());
    Fixture g;
    g.line({0.0, 0.0}, {50.0, 0.0});
    q.pendingStartMm = Vec2{25.0, 20.0};
    CHECK(SnapIndex(g.doc).candidates(g.view, g.px({25.0, 0.0}), only(SnapKind::Tangent), q).empty());
}

TEST_CASE("grid and guides", "[viewmodel][snap]") {
    Fixture f;
    const SnapIndex idx(f.doc);
    SnapSettings s = only(SnapKind::Grid);
    s.gridSpacingMm = 5.0;
    s.gridOriginMm = {1.0, 2.0};
    auto g = idx.best(f.view, f.px({16.9, 11.8}), s);
    REQUIRE(g.has_value());
    CHECK(g->kind == SnapKind::Grid);
    CHECK(distance(g->pointMm, {16.0, 12.0}) < 1e-9);
    CHECK(g->objects.empty());
    s.tolerancePx = 4.0;
    CHECK_FALSE(idx.best(f.view, f.px({18.5, 14.5}), s).has_value());  // 3.5 mm = 7 px from (16,12)
    s.tolerancePx = 10.0;
    s.gridSpacingMm = 0.0;
    CHECK_FALSE(idx.best(f.view, f.px({16.0, 12.0}), s).has_value());

    SnapSettings gs = only(SnapKind::Guide);
    gs.guides = {Guide::horizontal(30.0), Guide::vertical(70.0), Guide{{0.0, 0.0}, {1.0, 1.0}}};
    auto onH = idx.best(f.view, f.px({20.0, 31.0}), gs);
    REQUIRE(onH.has_value());
    CHECK(onH->guide == 0);
    CHECK(distance(onH->pointMm, {20.0, 30.0}) < 1e-12);
    // Crossing of the two guides beats nothing else here; guide pair is reported.
    // (Near a crossing the single-guide feet are nearer to the cursor, so they come first.)
    auto crossings = idx.candidates(f.view, f.px({70.5, 30.0}), gs);
    const auto cross = std::find_if(crossings.begin(), crossings.end(), [](const Snap& x) { return x.guide2 >= 0; });
    REQUIRE(cross != crossings.end());
    CHECK(cross->guide == 0);
    CHECK(cross->guide2 == 1);
    CHECK(cross->pointMm == Vec2{70.0, 30.0});
    // Diagonal guide through the origin crosses the horizontal one at (30,30).
    auto diag = idx.candidates(f.view, f.px({30.0, 30.0}), gs);
    CHECK(std::any_of(diag.begin(), diag.end(), [](const Snap& x) { return x.guide == 0 && x.guide2 == 2; }));
    // Parallel guides never cross.
    gs.guides = {Guide::horizontal(30.0), Guide::horizontal(40.0)};
    gs.tolerancePx = 9.0;
    CHECK_FALSE(idx.best(f.view, f.px({20.0, 35.0}), gs).has_value());
}

TEST_CASE("disabled kinds never snap", "[viewmodel][snap]") {
    Fixture f;
    f.line({0.0, 0.0}, {40.0, 0.0});
    f.line({20.0, -20.0}, {20.0, 20.0});
    f.add(Contour::circle({100.0, 100.0}, 10.0));
    const SnapIndex idx(f.doc);
    SnapSettings all;
    all.gridSpacingMm = 7.0;
    all.guides = {Guide::horizontal(0.0)};
    SnapQuery q;
    q.pendingStartMm = Vec2{10.0, 40.0};
    for (const Vec2 p : {Vec2{0.0, 0.0}, Vec2{20.0, 0.0}, Vec2{20.5, 0.5}, Vec2{100.0, 110.0}, Vec2{110.0, 100.0}, Vec2{30.0, 1.0}}) {
        const auto base = idx.candidates(f.view, f.px(p), all, q);
        for (std::size_t k = 0; k < kSnapKindCount; ++k) {
            SnapSettings s = all;
            s.enabled[k] = false;
            for (const Snap& sn : idx.candidates(f.view, f.px(p), s, q)) {
                CHECK(static_cast<std::size_t>(sn.kind) != k);
            }
            // Disabling a kind removes exactly its candidates.
            std::size_t expect = 0;
            for (const Snap& sn : base) {
                expect += static_cast<std::size_t>(sn.kind) != k ? 1U : 0U;
            }
            CHECK(idx.candidates(f.view, f.px(p), s, q).size() == expect);
        }
        SnapSettings none = all;
        none.disableAll();
        CHECK(idx.candidates(f.view, f.px(p), none, q).empty());
    }
}

TEST_CASE("priority: kind first, then distance, then ids, then position", "[viewmodel][snap]") {
    Fixture f;
    const ObjectId a = f.line({0.0, 0.0}, {40.0, 0.0});
    f.line({20.0, -20.0}, {20.0, 20.0});
    const SnapIndex idx(f.doc);
    SnapSettings s;
    s.tolerancePx = 20.0;
    s.setEnabled(SnapKind::Grid, false);
    // Cursor at the crossing (20,0), which is also the midpoint of line a and of the vertical line.
    auto all = idx.candidates(f.view, f.px({20.0, 0.0}), s);
    REQUIRE(all.size() >= 3);
    CHECK(all[0].kind == SnapKind::Intersection);  // no endpoint within 20 px (20 mm = 40 px away)
    CHECK(all[1].kind == SnapKind::Midpoint);
    // Kinds are non-decreasing, distances non-decreasing within a kind.
    for (std::size_t i = 1; i < all.size(); ++i) {
        CHECK(all[i - 1].kind <= all[i].kind);
        if (all[i - 1].kind == all[i].kind) {
            CHECK(all[i - 1].distancePx <= all[i].distancePx + 1e-6);
        }
    }
    // A farther endpoint still beats a nearer nearest-on-curve snap.
    auto e = idx.best(f.view, f.px({2.0, 1.0}), s);
    REQUIRE(e.has_value());
    CHECK(e->kind == SnapKind::Endpoint);
    CHECK(e->pointMm == Vec2{0.0, 0.0});
    // Two endpoints at equal distance: lower position (x) wins; same object ids never matter.
    SnapSettings ep = only(SnapKind::Endpoint, 30.0);
    auto tie = idx.best(f.view, f.px({20.0, 20.0}), ep);  // the vertical line's top end: exact
    REQUIRE(tie.has_value());
    CHECK(tie->pointMm == Vec2{20.0, 20.0});
    Fixture g;
    g.line({0.0, 0.0}, {10.0, 0.0});
    g.line({20.0, 0.0}, {30.0, 0.0});
    const SnapIndex idg(g.doc);
    auto eq = idg.best(g.view, g.px({15.0, 0.0}), only(SnapKind::Endpoint, 20.0));
    REQUIRE(eq.has_value());
    CHECK(eq->pointMm == Vec2{10.0, 0.0});  // equal 10 px: smaller object id, then smaller x
    CHECK(eq->objects.size() == 1);
    // Coincident endpoints of two objects merge into one snap listing both ids.
    Fixture m;
    const ObjectId m1 = m.line({0.0, 0.0}, {10.0, 0.0});
    const ObjectId m2 = m.line({10.0, 0.0}, {10.0, 10.0});
    auto mg = SnapIndex(m.doc).candidates(m.view, m.px({10.0, 0.0}), only(SnapKind::Endpoint));
    REQUIRE(mg.size() == 1);
    CHECK(mg[0].objects == std::vector<ObjectId>{m1, m2});
    (void)a;
}

TEST_CASE("equal kind and distance: the smaller object id wins before position", "[viewmodel][snap]") {
    // Two endpoints exactly 10 px from the cursor; the one with the smaller id has the LARGER x,
    // so a position-only fallthrough would pick the other. Insertion order is swapped in the 2nd run.
    for (const bool swapped : {false, true}) {
        Fixture f;
        ObjectId first;
        ObjectId second;
        if (!swapped) {
            first = f.line({20.0, 0.0}, {20.0, 10.0});   // x = 20, smaller id
            second = f.line({10.0, 0.0}, {10.0, -10.0});  // x = 10, larger id
        } else {
            first = f.line({10.0, 0.0}, {10.0, -10.0});
            second = f.line({20.0, 0.0}, {20.0, 10.0});
        }
        REQUIRE(first < second);
        const SnapIndex idx(f.doc);
        auto got = idx.candidates(f.view, f.px({15.0, 0.0}), only(SnapKind::Endpoint, 20.0));
        const auto onlyAtAxis = ofKind(got, SnapKind::Endpoint);
        REQUIRE(onlyAtAxis.size() >= 2);
        CHECK(onlyAtAxis[0].objects == std::vector<ObjectId>{first});
        CHECK(onlyAtAxis[0].pointMm.x == (swapped ? 10.0 : 20.0));
    }
}

TEST_CASE("perpendicular foot coinciding with the pending start is dropped", "[viewmodel][snap]") {
    Fixture f;
    f.line({0.0, 0.0}, {100.0, 0.0});
    const SnapIndex idx(f.doc);
    SnapQuery q;
    q.pendingStartMm = Vec2{40.0, 0.0};  // the start lies on the line: its own foot
    CHECK(idx.candidates(f.view, f.px({40.0, 0.0}), only(SnapKind::Perpendicular), q).empty());
    q.pendingStartMm = Vec2{40.0, 0.5};  // off the line by more than the distance tolerance
    auto got = idx.candidates(f.view, f.px({40.0, 0.0}), only(SnapKind::Perpendicular), q);
    REQUIRE(got.size() == 1);
    CHECK(distance(got[0].pointMm, {40.0, 0.0}) < 1e-12);
}

TEST_CASE("results do not depend on insertion order", "[viewmodel][snap]") {
    auto build = [](bool reversed) {
        auto f = std::make_unique<Fixture>();
        std::vector<std::pair<Vec2, Vec2>> segs{{{0, 0}, {10, 0}}, {{10, 0}, {10, 10}}, {{10, 0}, {20, 5}}, {{5, -5}, {5, 5}}};
        if (reversed) {
            std::reverse(segs.begin(), segs.end());
        }
        for (const auto& [p, q] : segs) {
            f->line(p, q);
        }
        return f;
    };
    auto fa = build(false);
    auto fb = build(true);
    const SnapIndex ia(fa->doc);
    const SnapIndex ib(fb->doc);
    SnapSettings s;
    s.tolerancePx = 30.0;
    for (const Vec2 p : {Vec2{10.0, 0.0}, Vec2{5.0, 0.0}, Vec2{12.0, 3.0}}) {
        const auto ra = ia.candidates(fa->view, fa->px(p), s);
        const auto rb = ib.candidates(fb->view, fb->px(p), s);
        REQUIRE(ra.size() == rb.size());
        // Same snaps; only ties between equal-distance candidates may order differently (by id).
        for (std::size_t i = 0; i < ra.size(); ++i) {
            CHECK(ra[i].kind == rb[i].kind);
            CHECK(std::any_of(rb.begin(), rb.end(), [&](const Snap& o) {
                return o.kind == ra[i].kind && o.pointMm == ra[i].pointMm;
            }));
        }
    }
}

TEST_CASE("hidden and locked layers, other sheets, groups and exclusions", "[viewmodel][snap]") {
    Fixture f;
    Layer hidden;
    hidden.name = "hidden";
    hidden.visible = false;
    const LayerId hid = f.doc.insertLayer(hidden);
    Layer locked;
    locked.name = "locked";
    locked.locked = true;
    const LayerId lck = f.doc.insertLayer(locked);
    f.addOn(hid, polyline({{0.0, 0.0}, {10.0, 0.0}}, false));
    const ObjectId l = f.addOn(lck, polyline({{50.0, 0.0}, {60.0, 0.0}}, false));
    const ObjectId vis = f.line({100.0, 0.0}, {110.0, 0.0});
    const SnapSettings s = only(SnapKind::Endpoint);
    {
        const SnapIndex idx(f.doc);
        CHECK_FALSE(idx.best(f.view, f.px({0.0, 0.0}), s).has_value());
        CHECK_FALSE(idx.best(f.view, f.px({50.0, 0.0}), s).has_value());
        CHECK(idx.best(f.view, f.px({100.0, 0.0}), s).has_value());
    }
    {
        const SnapIndex idx(f.doc, true);  // includeLocked
        auto got = idx.best(f.view, f.px({50.0, 0.0}), s);
        REQUIRE(got.has_value());
        CHECK(got->objects == std::vector<ObjectId>{l});
        CHECK_FALSE(idx.best(f.view, f.px({0.0, 0.0}), s).has_value());  // hidden stays hidden
    }
    {
        const SnapIndex idx(f.doc);
        SnapQuery q;
        q.exclude = {vis};
        CHECK_FALSE(idx.best(f.view, f.px({100.0, 0.0}), s, q).has_value());
    }
    // A child on a visible layer inside a group on a hidden layer does not snap.
    Object group;
    group.layer = hid;
    group.sheet = f.doc.activeSheet();
    group.data = GroupObject{};
    const ObjectId gid = f.doc.insertObject(std::move(group));
    Object child = Object::vector(polyline({{200.0, 0.0}, {210.0, 0.0}}, false), f.doc.activeLayer(), f.doc.activeSheet());
    child.parent = gid;
    f.doc.insertObject(std::move(child));
    CHECK_FALSE(SnapIndex(f.doc).best(f.view, f.px({200.0, 0.0}), s).has_value());
    f.doc.findLayer(hid)->visible = true;
    auto g = SnapIndex(f.doc).best(f.view, f.px({200.0, 0.0}), s);
    REQUIRE(g.has_value());
    // Objects on another sheet never snap.
    Sheet other = Sheet::fromJob(f.doc.job(), "Sheet 2");
    const SheetId sid = f.doc.insertSheet(other);
    f.doc.insertObject(Object::vector(polyline({{300.0, 0.0}, {310.0, 0.0}}, false), f.doc.activeLayer(), sid));
    CHECK_FALSE(SnapIndex(f.doc).best(f.view, f.px({300.0, 0.0}), s).has_value());
}

TEST_CASE("generated scenes agree with brute-force oracles", "[viewmodel][snap][oracle]") {
    for (std::uint64_t seed = 1; seed <= 6; ++seed) {
        Rng rng(seed);
        Fixture f;
        std::vector<std::pair<ObjectId, Span>> all;
        const int n = 40;
        for (int i = 0; i < n; ++i) {
            const Vec2 a{rng.range(0.0, 200.0), rng.range(0.0, 200.0)};
            Contour c;
            const int type = i % 3;
            if (type == 0) {
                c.append(LineSpan{a, a + Vec2{rng.range(-60.0, 60.0), rng.range(-60.0, 60.0)}});
            } else if (type == 1) {
                const double a0 = rng.range(0.0, 6.0);
                c.append(ArcSpan{a, rng.range(5.0, 40.0), a0, a0 + rng.range(0.5, 5.5),
                                 (i % 2 == 0) ? ArcDirection::CCW : ArcDirection::CW});
            } else {
                c.append(BezierSpan{a, a + Vec2{rng.range(-50, 50), rng.range(-50, 50)},
                                    a + Vec2{rng.range(-50, 50), rng.range(-50, 50)},
                                    a + Vec2{rng.range(-60, 60), rng.range(-60, 60)}});
            }
            all.emplace_back(f.add(c), c.spans[0]);
        }
        const SnapIndex idx(f.doc);
        const Tolerance tol;
        SnapSettings s;
        s.tolerancePx = 12.0;
        s.disableAll();
        for (const SnapKind k : {SnapKind::Endpoint, SnapKind::Midpoint, SnapKind::Centre, SnapKind::Quadrant,
                                 SnapKind::Intersection, SnapKind::Nearest}) {
            s.setEnabled(k, true);
        }
        // Oracle intersections over every pair.
        std::vector<Vec2> xs;
        for (std::size_t i = 0; i < all.size(); ++i) {
            for (std::size_t j = i + 1; j < all.size(); ++j) {
                for (const auto& h : intersect(all[i].second, all[j].second, tol).points) {
                    xs.push_back(h.point);
                }
            }
        }
        REQUIRE(xs.size() > 3);
        for (int qn = 0; qn < 150; ++qn) {
            Vec2 cursorMm;
            const int mode = qn % 3;
            const auto& pick = all[static_cast<std::size_t>(rng.next() * n) % all.size()].second;
            if (mode == 0) {
                cursorMm = pointAt(pick, rng.next());
            } else if (mode == 1) {
                cursorMm = xs[static_cast<std::size_t>(rng.next() * static_cast<double>(xs.size())) % xs.size()];
            } else {
                cursorMm = {rng.range(0.0, 200.0), rng.range(0.0, 200.0)};
            }
            cursorMm = cursorMm + Vec2{rng.range(-3.0, 3.0), rng.range(-3.0, 3.0)};
            const Vec2 cpx = f.px(cursorMm);
            const auto got = idx.candidates(f.view, cpx, s);
            auto inReach = [&](Vec2 p) { return distance(p, cursorMm) * f.view.scale() <= s.tolerancePx; };

            // Never beyond the tolerance, sorted.
            for (const Snap& sn : got) {
                CHECK(sn.distancePx <= s.tolerancePx);
                CHECK_THAT(sn.distancePx, WithinAbs(distance(sn.pointMm, cursorMm) * f.view.scale(), 1e-9));
            }
            // Endpoints, midpoints (line/arc), centres, quadrants: exact sets.
            std::vector<Vec2> ends;
            std::vector<Vec2> mids;
            std::vector<Vec2> cens;
            std::vector<Vec2> quads;
            for (const auto& [id, sp] : all) {
                (void)id;
                ends.push_back(startPoint(sp));
                ends.push_back(endPoint(sp));
                if (!std::holds_alternative<BezierSpan>(sp)) {
                    mids.push_back(pointAt(sp, 0.5));
                }
                if (const auto* a = std::get_if<ArcSpan>(&sp)) {
                    cens.push_back(a->centre);
                    for (int k = 0; k < 4; ++k) {
                        if (a->containsAngle(k * kPi / 2.0)) {
                            quads.push_back(a->centre + Vec2::fromAngle(k * kPi / 2.0, a->radius));
                        }
                    }
                }
            }
            struct Oracle {
                SnapKind kind;
                const std::vector<Vec2>* pts;
            };
            for (const Oracle& o : {Oracle{SnapKind::Endpoint, &ends}, Oracle{SnapKind::Midpoint, &mids},
                                    Oracle{SnapKind::Centre, &cens}, Oracle{SnapKind::Quadrant, &quads},
                                    Oracle{SnapKind::Intersection, &xs}}) {
                const auto gk = ofKind(got, o.kind);
                for (const Vec2& p : *o.pts) {
                    if (inReach(p) && o.kind != SnapKind::Intersection) {
                        CHECK(hasPoint(gk, p, 1e-6));
                    }
                }
                if (o.kind == SnapKind::Intersection) {
                    for (const Vec2& p : *o.pts) {
                        if (inReach(p)) {
                            CHECK(hasPoint(gk, p, 1e-6));
                        }
                    }
                }
                for (const Snap& sn : gk) {
                    const bool listed = std::any_of(o.pts->begin(), o.pts->end(), [&](Vec2 p) {
                        return distance(p, sn.pointMm) <= 1e-6;
                    });
                    if (o.kind == SnapKind::Midpoint && !listed) {
                        continue;  // Bezier midpoints are verified separately
                    }
                    CHECK(listed);
                }
            }
            // Nearest: the best distance equals the brute-force minimum when within tolerance.
            double bestMm = 1e300;
            for (const auto& [id, sp] : all) {
                (void)id;
                bestMm = std::min(bestMm, closestPoint(sp, cursorMm).distance);
            }
            const auto nn = ofKind(got, SnapKind::Nearest);
            if (bestMm * f.view.scale() <= s.tolerancePx) {
                REQUIRE(!nn.empty());
                CHECK_THAT(nn.front().distancePx, WithinAbs(bestMm * f.view.scale(), 1e-6));
            } else {
                CHECK(nn.empty());
            }
        }
    }
}

namespace {
double cpuSeconds() {
    return static_cast<double>(std::clock()) / CLOCKS_PER_SEC;
}
}  // namespace

TEST_CASE("10k objects: queries touch only the neighbourhood of the cursor", "[viewmodel][snap][perf]") {
    Fixture f(3000.0);
    Rng rng(99);
    const int n = 10000;
    for (int i = 0; i < n; ++i) {
        const Vec2 c{rng.range(0.0, 2900.0), rng.range(0.0, 2900.0)};
        Contour k;
        switch (i % 4) {
            case 0:
                k.append(LineSpan{c, c + Vec2{rng.range(-30.0, 30.0), rng.range(-30.0, 30.0)}});
                break;
            case 1:
                k = Contour::circle(c, rng.range(2.0, 15.0));
                break;
            case 2:
                k.append(BezierSpan{c, c + Vec2{10, 20}, c + Vec2{25, -10}, c + Vec2{30, 5}});
                break;
            default:
                k = polyline({c, c + Vec2{20, 0}, c + Vec2{20, 15}, c + Vec2{0, 15}}, true);
                break;
        }
        f.add(std::move(k));
    }
    f.view.set(1.0, {0.0, 3000.0});
    f.view.setViewport(3000.0, 3000.0);
    const double t0 = cpuSeconds();
    const SnapIndex idx(f.doc);
    const double build = cpuSeconds() - t0;
    SnapSettings s;
    s.gridSpacingMm = 5.0;
    SnapQuery q;
    q.pendingStartMm = Vec2{10.0, 10.0};
    const int queries = 2000;
    std::size_t found = 0;
    const double t1 = cpuSeconds();
    for (int i = 0; i < queries; ++i) {
        const Vec2 cur{rng.range(0.0, 2900.0), rng.range(0.0, 2900.0)};
        found += idx.candidates(f.view, f.view.worldToScreen(cur), s, q).size();
    }
    const double perQuery = (cpuSeconds() - t1) / queries;
    std::cout << "[T-0203 perf] objects=" << n << " build=" << build << " s, query=" << perQuery * 1e6
              << " us avg, candidates=" << found << "\n";
    CHECK(found > 0);
    // Documented bound: 1 ms per query (Release ~20 us; the generous limit covers Debug builds
    // on a shared machine). A linear scan over 10k objects per move would exceed it.
    CHECK(perQuery < 1e-3);
}

TEST_CASE("dense drawing: hundreds of spans inside the tolerance window stay interactive", "[viewmodel][snap][perf][dense]") {
    // T-0204 criterion 3: 10k objects in a ~300 mm square; at 0.5 px/mm the 10 px window covers
    // 20 mm, so hundreds of spans overlap every cursor. All kinds on, with a pending start so the
    // perpendicular and tangent searches run too.
    Fixture f(400.0);
    Rng rng(7);
    const int n = 10000;
    for (int i = 0; i < n; ++i) {
        const Vec2 c{rng.range(50.0, 350.0), rng.range(50.0, 350.0)};
        Contour k;
        switch (i % 4) {
            case 0:
                k.append(LineSpan{c, c + Vec2{rng.range(-30.0, 30.0), rng.range(-30.0, 30.0)}});
                break;
            case 1:
                k = Contour::circle(c, rng.range(2.0, 15.0));
                break;
            case 2:
                k.append(BezierSpan{c, c + Vec2{10, 20}, c + Vec2{25, -10}, c + Vec2{30, 5}});
                break;
            default:
                k = polyline({c, c + Vec2{20, 0}, c + Vec2{20, 15}, c + Vec2{0, 15}}, true);
                break;
        }
        f.add(std::move(k));
    }
    f.view.set(0.5, {20.0, 220.0});
    const SnapIndex idx(f.doc);
    SnapSettings s;
    s.gridSpacingMm = 5.0;
    SnapQuery q;
    q.pendingStartMm = Vec2{200.0, 200.0};
    const int queries = 300;
    double worst = 0.0;
    double total = 0.0;
    std::size_t found = 0;
    for (int i = 0; i < queries; ++i) {
        const Vec2 cur{rng.range(100.0, 300.0), rng.range(100.0, 300.0)};
        const double t0 = cpuSeconds();
        found += idx.candidates(f.view, f.view.worldToScreen(cur), s, q).size();
        const double dt = cpuSeconds() - t0;
        total += dt;
        worst = std::max(worst, dt);
    }
    std::cout << "[T-0204 dense] objects=" << n << " query avg=" << total / queries * 1e6
              << " us, worst=" << worst * 1e6 << " us, candidates=" << found << "\n";
    CHECK(found > 0);
    // Documented bound: kMaxPairSpans^2 / 2 span-pair intersections per query. Measured on the
    // development machine: Release ~2.4 ms average / ~6 ms worst, Debug ~15 ms / ~70 ms (before the
    // bound the same scene needed hundreds of ms per query). The limits leave room for a loaded CI
    // machine in Debug builds.
    CHECK(total / queries < 0.05);
    CHECK(worst < 0.3);
}

TEST_CASE("dense window: the crossing nearest the cursor is still found among hundreds of spans", "[viewmodel][snap][dense]") {
    Fixture f(400.0);
    Rng rng(11);
    // 300 distractor lines through the 10 px window, all slightly off the cursor point.
    const Vec2 cursor{200.0, 200.0};
    for (int i = 0; i < 300; ++i) {
        const double a = rng.range(0.0, 3.14159);
        const Vec2 d = Vec2::fromAngle(a, 40.0);
        const Vec2 off = Vec2::fromAngle(a + 1.5707963, rng.range(2.0, 8.0));
        f.line(cursor + off - d, cursor + off + d);
    }
    // The true crossing exactly at the cursor.
    f.line({160.0, 200.0}, {240.0, 200.0});
    f.line({200.0, 160.0}, {200.0, 240.0});
    const SnapIndex idx(f.doc);
    const auto best = idx.best(f.view, f.px(cursor), only(SnapKind::Intersection));
    REQUIRE(best.has_value());
    CHECK(distance(best->pointMm, cursor) < 1e-9);
    REQUIRE(best->objects.size() == 2);
}
