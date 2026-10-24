#include "carvetto/toolpath/toolpath.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/linking.hpp"

using namespace carvetto::toolpath;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::kPi;
using Catch::Matchers::WithinAbs;

namespace {
// A 10x10 square cut once at Z=-3 with safe Z=5, starting from the machine at (0,0,5).
Toolpath squareAtDepth() {
    Toolpath tp;
    tp.name = "square";
    LinkingOptions opts;
    opts.safeZ = 5.0;
    linkContour(Contour::rectangle(Box2{Vec2{0, 0}, Vec2{10, 10}}), -3.0, opts, tp);
    return tp;
}
}  // namespace

TEST_CASE("Empty toolpath queries", "[toolpath]") {
    const Toolpath tp;
    CHECK(tp.isEmpty());
    CHECK(tp.moveCount() == 0);
    CHECK_FALSE(tp.endPoint().has_value());
    CHECK_FALSE(tp.minZ().has_value());
    CHECK_FALSE(tp.maxZ().has_value());
    CHECK(tp.bounds().isEmpty());
    CHECK(tp.length() == 0.0);
    CHECK(tp.lengthByKind() == LengthByKind{});
    CHECK(tp.count(MoveKind::Feed) == 0);
}

TEST_CASE("AC3: length by kind for a 10x10 square at one depth is analytic", "[toolpath]") {
    const Toolpath tp = squareAtDepth();
    REQUIRE(tp.moveCount() == 7);  // rapid, plunge, 4 feeds, retract

    SECTION("first move measured from nothing has zero rapid length") {
        const LengthByKind l = tp.lengthByKind();
        CHECK_THAT(l.rapid, WithinAbs(0.0, 1e-9));
        CHECK_THAT(l.plunge, WithinAbs(8.0, 1e-9));
        CHECK_THAT(l.feed, WithinAbs(40.0, 1e-9));
        CHECK_THAT(l.retract, WithinAbs(8.0, 1e-9));
        CHECK_THAT(l.total(), WithinAbs(56.0, 1e-9));
        CHECK_THAT(tp.length(), WithinAbs(56.0, 1e-9));
    }
    SECTION("first move measured from an explicit start position") {
        const LengthByKind l = tp.lengthByKind(Vec3{-3.0, -4.0, 5.0});
        CHECK_THAT(l.rapid, WithinAbs(5.0, 1e-9));
        CHECK_THAT(l.total(), WithinAbs(61.0, 1e-9));
    }
}

TEST_CASE("Length by kind includes arcs under feed and ignores dwells", "[toolpath]") {
    Toolpath tp;
    tp.append(Move::rapid({10, 0, 5}));
    tp.append(Move::plunge({10, 0, 0}));
    tp.append(Move::arcCCW({10, 0, 0}, Vec2{0, 0}));  // full circle r=10
    tp.append(Move::dwell({10, 0, 0}, 3.0));
    tp.append(Move::retract({10, 0, 5}));
    const LengthByKind l = tp.lengthByKind();
    CHECK_THAT(l.feed, WithinAbs(20.0 * kPi, 1e-9));
    CHECK_THAT(l.plunge, WithinAbs(5.0, 1e-9));
    CHECK_THAT(l.retract, WithinAbs(5.0, 1e-9));
    CHECK(tp.count(MoveKind::Dwell) == 1);
    CHECK(tp.count(MoveKind::ArcCCW) == 1);
    CHECK(tp.count(MoveKind::ArcCW) == 0);
}

TEST_CASE("Bounds, Z range and end point", "[toolpath]") {
    const Toolpath tp = squareAtDepth();
    const Box2 b = tp.bounds();
    CHECK_THAT(b.min.x, WithinAbs(0.0, 1e-12));
    CHECK_THAT(b.min.y, WithinAbs(0.0, 1e-12));
    CHECK_THAT(b.max.x, WithinAbs(10.0, 1e-12));
    CHECK_THAT(b.max.y, WithinAbs(10.0, 1e-12));
    CHECK(tp.minZ() == -3.0);
    CHECK(tp.maxZ() == 5.0);
    CHECK(tp.endPoint() == Vec3{0, 0, 5});

    SECTION("arc bulge extends the bounds beyond the end points") {
        Toolpath arc;
        arc.append(Move::rapid({10, 0, 0}));
        arc.append(Move::arcCCW({-10, 0, 0}, Vec2{0, 0}));
        CHECK_THAT(arc.bounds().max.y, WithinAbs(10.0, 1e-9));
    }
    SECTION("explicit start position extends the bounds") {
        CHECK_THAT(tp.bounds(Vec3{-20, 0, 5}).min.x, WithinAbs(-20.0, 1e-12));
    }
}

TEST_CASE("startOf and append", "[toolpath]") {
    Toolpath tp;
    tp.append(Move::rapid({1, 1, 5}));
    tp.append(Move::plunge({1, 1, -1}));
    CHECK(tp.startOf(0) == Vec3{1, 1, 5});
    CHECK(tp.startOf(0, Vec3{0, 0, 0}) == Vec3{0, 0, 0});
    CHECK(tp.startOf(1) == Vec3{1, 1, 5});

    Toolpath other;
    other.tool.name = "ignored";
    other.append(Move::retract({1, 1, 5}));
    tp.append(other);
    CHECK(tp.moveCount() == 3);
    CHECK(tp.tool.name.empty());
    CHECK(tp.moves.back().kind == MoveKind::Retract);
}

TEST_CASE("ToolSnapshot is a plain value", "[toolpath]") {
    ToolSnapshot a{"6mm end mill", 1, 6.0, 1200.0, 300.0, 18000.0};
    ToolSnapshot b = a;
    CHECK(a == b);
    b.feedRate = 1000.0;
    CHECK_FALSE(a == b);
    Toolpath tp;
    tp.tool = a;
    CHECK(tp.tool.diameter == 6.0);
}
