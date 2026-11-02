#include "carvetto/toolpath/linking.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"

using namespace carvetto::toolpath;
using carvetto::geometry::ArcDirection;
using carvetto::geometry::ArcSpan;
using carvetto::geometry::BezierSpan;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::kPi;
using carvetto::geometry::LineSpan;
using Catch::Matchers::WithinAbs;

namespace {
std::vector<MoveKind> kinds(const Toolpath& tp) {
    std::vector<MoveKind> out;
    for (const Move& m : tp.moves) {
        out.push_back(m.kind);
    }
    return out;
}

LinkingOptions optsSafe(double safeZ) {
    LinkingOptions o;
    o.safeZ = safeZ;
    return o;
}
}  // namespace

TEST_CASE("AC2: linking two closed contours at -3", "[toolpath][linking]") {
    const std::vector<Contour> contours{Contour::rectangle(Box2{Vec2{0, 0}, Vec2{10, 10}}),
                                        Contour::rectangle(Box2{Vec2{20, 0}, Vec2{30, 10}})};
    Toolpath tp;
    linkContours(contours, -3.0, optsSafe(6.0), tp);

    const std::vector<MoveKind> want{
        MoveKind::Rapid, MoveKind::Plunge,  MoveKind::Feed,  MoveKind::Feed,    MoveKind::Feed,
        MoveKind::Feed,  MoveKind::Retract, MoveKind::Rapid, MoveKind::Plunge,  MoveKind::Feed,
        MoveKind::Feed,  MoveKind::Feed,    MoveKind::Feed,  MoveKind::Retract,
    };
    CHECK(kinds(tp) == want);

    // Geometry of the first block: rapid at safe Z above the start, plunge straight down, cut at -3,
    // retract straight up to safe Z above the end point (= start for a closed contour).
    CHECK(tp.moves[0].to == Vec3{0, 0, 6});
    CHECK(tp.moves[1].to == Vec3{0, 0, -3});
    for (std::size_t i = 2; i <= 5; ++i) {
        CHECK(tp.moves[i].to.z == -3.0);
    }
    CHECK(tp.moves[5].to == Vec3{0, 0, -3});
    CHECK(tp.moves[6].to == Vec3{0, 0, 6});
    // Second block starts above the second contour.
    CHECK(tp.moves[7].to == Vec3{20, 0, 6});
    CHECK(tp.moves[8].to == Vec3{20, 0, -3});
    CHECK(tp.moves[13].to == Vec3{20, 0, 6});

    // The whole thing moves at safe Z between contours: no XY travel below safe Z between retract and plunge.
    CHECK(tp.moves[7].to.z == 6.0);
}

TEST_CASE("Arc spans become arc moves with the same centre and direction", "[toolpath][linking]") {
    Toolpath tp;
    linkContour(Contour::circle(Vec2{5, 5}, 2.0, ArcDirection::CW), -1.0, optsSafe(4.0), tp);
    REQUIRE(tp.moveCount() == 4);
    CHECK(tp.moves[0] == Move::rapid({7, 5, 4}));
    CHECK(tp.moves[1] == Move::plunge({7, 5, -1}));
    CHECK(tp.moves[2].kind == MoveKind::ArcCW);
    CHECK(tp.moves[2].centre == Vec2{5, 5});
    CHECK(carvetto::geometry::approxEqual(tp.moves[2].to, Vec3{7, 5, -1}));
    CHECK_THAT(tp.lengthByKind().feed, WithinAbs(4.0 * kPi, 1e-9));
    CHECK(tp.moves[3].kind == MoveKind::Retract);

    Toolpath ccw;
    Contour half;
    half.append(ArcSpan::fromCentre(Vec2{0, 0}, Vec2{10, 0}, Vec2{-10, 0}, ArcDirection::CCW));
    linkContour(half, 0.0, optsSafe(1.0), ccw);
    CHECK(ccw.moves[2].kind == MoveKind::ArcCCW);
    CHECK_THAT(ccw.lengthByKind().feed, WithinAbs(10.0 * kPi, 1e-9));
}

TEST_CASE("Bezier spans are polygonised into feed moves within the chordal tolerance",
          "[toolpath][linking]") {
    Contour c;
    c.append(BezierSpan{{0, 0}, {0, 10}, {10, 10}, {10, 0}});
    LinkingOptions opts = optsSafe(2.0);
    opts.tolerance.chordal = 0.01;
    Toolpath tp;
    linkContour(c, -1.0, opts, tp);
    const std::size_t feeds = tp.count(MoveKind::Feed);
    CHECK(feeds > 4);
    CHECK(tp.count(MoveKind::ArcCW) + tp.count(MoveKind::ArcCCW) == 0);
    CHECK(carvetto::geometry::approxEqual(tp.moves.back().to, Vec3{10, 0, 2}));
    CHECK(carvetto::geometry::approxEqual(tp.moves[tp.moveCount() - 2].to, Vec3{10, 0, -1}));
    // Polyline length must approximate the true curve length from below, within a small margin.
    const double trueLength = c.length();
    const double feedLength = tp.lengthByKind().feed;
    CHECK(feedLength <= trueLength + 1e-9);
    CHECK(feedLength > trueLength * 0.999);

    // A coarser tolerance gives fewer moves.
    opts.tolerance.chordal = 1.0;
    Toolpath coarse;
    linkContour(c, -1.0, opts, coarse);
    CHECK(coarse.count(MoveKind::Feed) < feeds);
}

TEST_CASE("Zero-length spans produce no moves", "[toolpath][linking]") {
    Contour c;
    c.append(LineSpan{{0, 0}, {5, 0}});
    c.append(LineSpan{{5, 0}, {5, 0}});  // degenerate
    c.append(LineSpan{{5, 0}, {5, 5}});
    Toolpath tp;
    linkContour(c, -2.0, optsSafe(3.0), tp);
    CHECK(kinds(tp) == std::vector<MoveKind>{MoveKind::Rapid, MoveKind::Plunge, MoveKind::Feed,
                                             MoveKind::Feed, MoveKind::Retract});
    CHECK(tp.moves.back().to == Vec3{5, 5, 3});

    SECTION("a contour of only degenerate spans still gets positioned, plunged and retracted") {
        Contour d;
        d.append(LineSpan{{1, 1}, {1, 1}});
        Toolpath only;
        linkContour(d, -2.0, optsSafe(3.0), only);
        CHECK(kinds(only) == std::vector<MoveKind>{MoveKind::Rapid, MoveKind::Plunge, MoveKind::Retract});
    }
}

TEST_CASE("Empty inputs produce nothing", "[toolpath][linking]") {
    Toolpath tp;
    linkContour(Contour{}, -1.0, optsSafe(5.0), tp);
    CHECK(tp.isEmpty());
    linkContours({}, -1.0, optsSafe(5.0), tp);
    CHECK(tp.isEmpty());
    linkContours({Contour::rectangle(Box2{Vec2{0, 0}, Vec2{1, 1}})}, std::vector<double>{}, optsSafe(5.0),
                 tp);
    CHECK(tp.isEmpty());
    linkContours({Contour{}}, std::vector<double>{-1.0}, optsSafe(5.0), tp);
    CHECK(tp.isEmpty());
}

TEST_CASE("Single-level overload matches the multi-level overload with one level", "[toolpath][linking]") {
    const std::vector<Contour> contours{Contour::rectangle(Box2{Vec2{0, 0}, Vec2{10, 10}}),
                                        Contour::circle(Vec2{30, 5}, 3.0)};
    Toolpath a;
    Toolpath b;
    linkContours(contours, -3.0, optsSafe(6.0), a);
    linkContours(contours, std::vector<double>{-3.0}, optsSafe(6.0), b);
    CHECK(a == b);
}

TEST_CASE("Multi-level linking of a closed contour plunges between passes", "[toolpath][linking]") {
    const std::vector<Contour> contours{Contour::rectangle(Box2{Vec2{0, 0}, Vec2{10, 10}})};
    Toolpath tp;
    linkContours(contours, std::vector<double>{-2.0, -4.0}, optsSafe(6.0), tp);
    const std::vector<MoveKind> want{
        MoveKind::Rapid,  MoveKind::Plunge, MoveKind::Feed, MoveKind::Feed, MoveKind::Feed, MoveKind::Feed,
        MoveKind::Plunge, MoveKind::Feed,   MoveKind::Feed, MoveKind::Feed, MoveKind::Feed, MoveKind::Retract,
    };
    CHECK(kinds(tp) == want);
    CHECK(tp.moves[1].to == Vec3{0, 0, -2});
    CHECK(tp.moves[6].to == Vec3{0, 0, -4});
    CHECK(tp.moves[11].to == Vec3{0, 0, 6});
    CHECK(tp.count(MoveKind::Rapid) == 1);
    CHECK(tp.count(MoveKind::Retract) == 1);
}

TEST_CASE("Multi-level linking of an open contour alternates direction", "[toolpath][linking]") {
    Contour line;
    line.append(LineSpan{{0, 0}, {10, 0}});
    Toolpath tp;
    linkContours({line}, std::vector<double>{-1.0, -2.0, -3.0}, optsSafe(5.0), tp);
    const std::vector<MoveKind> want{
        MoveKind::Rapid, MoveKind::Plunge, MoveKind::Feed, MoveKind::Plunge,
        MoveKind::Feed,  MoveKind::Plunge, MoveKind::Feed, MoveKind::Retract,
    };
    CHECK(kinds(tp) == want);
    CHECK(tp.moves[2].to == Vec3{10, 0, -1});
    CHECK(tp.moves[3].to == Vec3{10, 0, -2});
    CHECK(tp.moves[4].to == Vec3{0, 0, -2});
    CHECK(tp.moves[5].to == Vec3{0, 0, -3});
    CHECK(tp.moves[6].to == Vec3{10, 0, -3});
    CHECK(tp.moves[7].to == Vec3{10, 0, 5});
    // Total feed is three lengths of the line, nothing more.
    CHECK_THAT(tp.lengthByKind().feed, WithinAbs(30.0, 1e-12));
}

TEST_CASE("Linking is deterministic", "[toolpath][linking]") {
    const std::vector<Contour> contours{Contour::circle(Vec2{3, 3}, 1.5),
                                        Contour::rectangle(Box2{Vec2{0, 0}, Vec2{7, 2}})};
    Toolpath a;
    Toolpath b;
    linkContours(contours, std::vector<double>{-1.0, -2.0}, optsSafe(4.0), a);
    linkContours(contours, std::vector<double>{-1.0, -2.0}, optsSafe(4.0), b);
    CHECK(a == b);
}
