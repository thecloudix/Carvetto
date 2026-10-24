#include "carvetto/geometry/polyline.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

using namespace carvetto::geometry;
using Catch::Matchers::WithinAbs;

namespace {
Polyline square() {
    Polyline p;
    p.points = {{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}};
    p.closed = true;
    return p;
}
}  // namespace

TEST_CASE("Polyline counts, length and bounds", "[geometry][polyline]") {
    Polyline empty;
    CHECK(empty.isEmpty());
    CHECK(empty.segmentCount() == 0);
    CHECK(empty.length() == 0.0);
    CHECK(empty.bounds().isEmpty());
    CHECK(empty.pointAt(0.5) == Vec2{});

    Polyline single;
    single.points = {{1.0, 2.0}};
    CHECK(single.segmentCount() == 0);
    CHECK(single.pointAt(0.3) == Vec2{1.0, 2.0});

    const Polyline sq = square();
    CHECK(sq.vertexCount() == 4);
    CHECK(sq.segmentCount() == 4);
    CHECK_THAT(sq.length(), WithinAbs(40.0, 1e-12));
    CHECK(sq.bounds() == Box2{{0.0, 0.0}, {10.0, 10.0}});
    CHECK(sq.segmentEnd(3) == Vec2{0.0, 0.0});

    Polyline open = sq;
    open.closed = false;
    CHECK(open.segmentCount() == 3);
    CHECK_THAT(open.length(), WithinAbs(30.0, 1e-12));
}

TEST_CASE("Polyline pointAt is linear in arc length", "[geometry][polyline]") {
    const Polyline sq = square();
    CHECK(approxEqual(sq.pointAt(0.0), {0.0, 0.0}));
    CHECK(approxEqual(sq.pointAt(0.25), {10.0, 0.0}));
    CHECK(approxEqual(sq.pointAt(0.375), {10.0, 5.0}));
    CHECK(approxEqual(sq.pointAt(1.0), {0.0, 0.0}));
    CHECK(approxEqual(sq.pointAt(-1.0), {0.0, 0.0}));
    CHECK(approxEqual(sq.pointAt(2.0), {0.0, 0.0}));
    Polyline open = sq;
    open.closed = false;
    CHECK(approxEqual(open.pointAt(1.0), {0.0, 10.0}));
}

TEST_CASE("Polyline reverse", "[geometry][polyline]") {
    Polyline open;
    open.points = {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
    open.reverse();
    CHECK(open.points == std::vector<Vec2>{{2.0, 0.0}, {1.0, 0.0}, {0.0, 0.0}});

    const Polyline sq = square();
    const Polyline r = sq.reversed();
    // Closed reversal keeps the start vertex and walks the loop the other way.
    CHECK(r.points == std::vector<Vec2>{{0.0, 0.0}, {0.0, 10.0}, {10.0, 10.0}, {10.0, 0.0}});
    CHECK_THAT(sq.signedArea(), WithinAbs(100.0, 1e-12));
    CHECK_THAT(r.signedArea(), WithinAbs(-100.0, 1e-12));
    Polyline openSq = sq;
    openSq.closed = false;
    CHECK(openSq.signedArea() == 0.0);
}

TEST_CASE("Polyline transform", "[geometry][polyline]") {
    const Polyline sq = square();
    const Polyline t = sq.transformed(Transform2::translation({1.0, -1.0}));
    CHECK(t.points.front() == Vec2{1.0, -1.0});
    CHECK(t.points[2] == Vec2{11.0, 9.0});
    CHECK(t.closed);
    Polyline m = sq;
    m.transform(Transform2::rotation(kPi / 2.0));
    CHECK(approxEqual(m.points[1], {0.0, 10.0}, 1e-12));
}

TEST_CASE("Polyline removeDuplicates", "[geometry][polyline]") {
    Polyline p;
    p.points = {{0.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}, {1.0, 1e-5}, {1.0, 1.0}, {0.0, 1.0}, {0.0, 0.0}};
    p.closed = true;
    p.removeDuplicates(1e-3);
    CHECK(p.points == std::vector<Vec2>{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}});
    Polyline open = p;
    open.closed = false;
    open.points.push_back({0.0, 0.0});
    open.removeDuplicates(1e-3);
    CHECK(open.points.size() == 5);  // open polylines keep a matching last vertex
}
