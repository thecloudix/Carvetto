#include "dxf_curves.hpp"

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <cstddef>
#include <string>
#include <variant>
#include <vector>

#include "carvetto/geometry/span.hpp"

using namespace carvetto::geometry;
using namespace carvetto::io::detail;

TEST_CASE("unit-circle Bezier error bound matches measurement", "[io][dxf][curves]") {
    for (const double theta : {kPi / 2, kPi / 4, 0.1}) {
        const ArcSpan a{{0, 0}, 1.0, 0.0, theta, ArcDirection::CCW};
        const BezierSpan b = a.toBeziers().at(0);
        double worst = 0.0;
        for (int i = 0; i <= 2000; ++i) {
            worst = std::max(worst, std::fabs(b.pointAt(i / 2000.0).length() - 1.0));
        }
        CHECK(worst <= unitArcBezierError(theta) * 1.0001);
        CHECK(worst >= unitArcBezierError(theta) * 0.9);
    }
}

TEST_CASE("elliptical arcs: circular case is one exact arc, general case within tolerance",
          "[io][dxf][curves]") {
    std::vector<Span> out;
    REQUIRE(appendEllipticalArc({1, 2}, {0, 3}, {-3, 0}, 0.5, 1.0, 0.01, 10, out) == CurveStatus::Ok);
    REQUIRE(out.size() == 1);
    const auto& a = std::get<ArcSpan>(out[0]);
    CHECK_THAT(a.radius, Catch::Matchers::WithinAbs(3.0, 1e-15));
    CHECK(approxEqual(a.startPoint(), Vec2{1, 2} + Vec2{0, 3} * std::cos(0.5) + Vec2{-3, 0} * std::sin(0.5),
                      1e-12));
    CHECK(a.direction == ArcDirection::CCW);
    out.clear();
    for (const double tol : {0.1, 0.01, 1e-4}) {
        out.clear();
        REQUIRE(appendEllipticalArc({0, 0}, {50, 0}, {0, 20}, 0.0, -kTwoPi, tol, 100000, out) ==
                CurveStatus::Ok);
        double worst = 0.0;
        for (const Span& s : out) {
            for (int i = 0; i <= 100; ++i) {
                const Vec2 p = pointAt(s, i / 100.0);
                const double f = p.x * p.x / 2500.0 + p.y * p.y / 400.0 - 1.0;
                const double g = Vec2{2 * p.x / 2500.0, 2 * p.y / 400.0}.length();
                worst = std::max(worst, std::fabs(f) / g);
            }
        }
        CHECK(worst <= tol);
        CHECK(endPoint(out.back()) == startPoint(out.front()));
    }
    out.clear();
    CHECK(appendEllipticalArc({0, 0}, {50, 0}, {0, 20}, 0.0, kTwoPi, 1e-12, 5, out) ==
          CurveStatus::TooManySpans);
}

TEST_CASE("NURBS: degree-2 elevation is exact, rational conics within tolerance, budget enforced",
          "[io][dxf][curves]") {
    NurbsCurve q;
    q.degree = 2;
    q.knots = {0, 0, 0, 1, 1, 1};
    q.controls = {{0, 0}, {1, 2}, {2, 0}};
    std::vector<Span> out;
    std::string why;
    REQUIRE(nurbsToSpans(q, 0.01, 100, out, why) == CurveStatus::Ok);
    REQUIRE(out.size() == 1);
    const auto& b = std::get<BezierSpan>(out[0]);
    for (int i = 0; i <= 10; ++i) {
        const double t = i / 10.0;
        const Vec2 exact =
            q.controls[0] * ((1 - t) * (1 - t)) + q.controls[1] * (2 * t * (1 - t)) + q.controls[2] * (t * t);
        CHECK(approxEqual(b.pointAt(t), exact, 1e-14));
    }
    // Full circle as a rational quadratic NURBS with 9 control points (unit square corners).
    NurbsCurve circle;
    circle.degree = 2;
    circle.knots = {0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1};
    circle.controls = {{1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}, {1, 0}};
    const double w = std::sqrt(0.5);
    circle.weights = {1, w, 1, w, 1, w, 1, w, 1};
    for (const double tol : {0.01, 1e-6}) {
        out.clear();
        REQUIRE(nurbsToSpans(circle, tol, 100000, out, why) == CurveStatus::Ok);
        double worst = 0.0;
        for (const Span& s : out) {
            for (int i = 0; i <= 64; ++i) {
                worst = std::max(worst, std::fabs(pointAt(s, i / 64.0).length() - 1.0));
            }
        }
        CHECK(worst <= tol);
    }
    out.clear();
    CHECK(nurbsToSpans(circle, 1e-15, 50, out, why) == CurveStatus::TooManySpans);
    NurbsCurve bad = q;
    bad.knots = {0, 0, 1, 0.5, 1, 1};
    CHECK(nurbsToSpans(bad, 0.01, 100, out, why) == CurveStatus::Invalid);
    bad = q;
    bad.weights = {1, -1, 1};
    CHECK(nurbsToSpans(bad, 0.01, 100, out, why) == CurveStatus::Invalid);
}
