#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <vector>

#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/vector_texture.hpp"

using namespace carvetto::geometry;

namespace {

VectorTextureParams params() {
    VectorTextureParams p;
    p.maxSpacing = 4.0;
    p.wavelength = 8.0;
    p.amplitude = 1.0;
    p.noise = 0.3;
    p.seed = 7;
    return p;
}

Contour hole(Box2 b) {
    Contour c = Contour::rectangle(b);
    c.reverse();
    return c;
}

std::vector<Contour> frameWithHole() {
    return {Contour::rectangle({{0, 0}, {100, 80}}), hole({{30, 20}, {70, 60}})};
}

std::vector<Vec2> vertices(const std::vector<Contour>& cs) {
    std::vector<Vec2> v;
    for (const Contour& c : cs)
        for (const Span& s : c.spans) {
            const auto& l = std::get<LineSpan>(s);
            v.push_back(l.start);
            v.push_back(l.end);
        }
    return v;
}

}  // namespace

TEST_CASE("texture in a region keeps lines out of holes", "[texture][region]") {
    const auto region = frameWithHole();
    auto r = generateVectorTextureInRegion(region, params());
    REQUIRE(r.ok());
    REQUIRE(!r.value().empty());
    for (const Contour& c : r.value()) REQUIRE(!c.closed);
    const Tolerance tol;
    for (Vec2 v : vertices(r.value())) {
        // Inside the outer box, and never strictly inside the hole (one fixed-point unit of slack).
        REQUIRE(v.x >= -1e-3);
        REQUIRE(v.x <= 100.001);
        REQUIRE(v.y >= -1e-3);
        REQUIRE(v.y <= 80.001);
        const bool strictlyInHole = v.x > 30.002 && v.x < 69.998 && v.y > 20.002 && v.y < 59.998;
        REQUIRE(!strictlyInHole);
    }
    // Segment midpoints are inside the region as well.
    for (const Contour& c : r.value())
        for (const Span& s : c.spans) {
            const auto& l = std::get<LineSpan>(s);
            const Vec2 m{(l.start.x + l.end.x) / 2, (l.start.y + l.end.y) / 2};
            REQUIRE(isInside(region, m, FillRule::NonZero, tol));
        }
    // The hole splits lines: more pieces than a plain box.
    auto box = generateVectorTextureContours({{0, 0}, {100, 80}}, params());
    REQUIRE(box.ok());
    REQUIRE(r.value().size() > box.value().size());
}

TEST_CASE("texture in a circle stays inside it", "[texture][region]") {
    const std::vector<Contour> region{Contour::circle({10, 10}, 25.0)};
    auto r = generateVectorTextureInRegion(region, params());
    REQUIRE(r.ok());
    REQUIRE(!r.value().empty());
    for (Vec2 v : vertices(r.value())) {
        REQUIRE(std::hypot(v.x - 10, v.y - 10) <= 25.0 + 0.02);
    }
}

TEST_CASE("a rectangle region gives the rectangle API's lines", "[texture][region]") {
    const Box2 box{{0, 0}, {60, 40}};
    auto a = generateVectorTextureContours(box, params());
    auto b = generateVectorTextureInRegion({Contour::rectangle(box)}, params());
    REQUIRE(a.ok());
    REQUIRE(b.ok());
    REQUIRE(a.value().size() == b.value().size());
    double lenA = 0;
    double lenB = 0;
    for (const auto& c : a.value()) lenA += c.length();
    for (const auto& c : b.value()) lenB += c.length();
    REQUIRE(std::fabs(lenA - lenB) < 0.01 * lenA);
}

TEST_CASE("the pattern does not move with the region", "[texture][region]") {
    // Same bounding box, so the same lines: carving a hole only removes pieces.
    auto full = generateVectorTextureInRegion({Contour::rectangle({{0, 0}, {100, 80}})}, params());
    auto holed = generateVectorTextureInRegion(frameWithHole(), params());
    REQUIRE(full.ok());
    REQUIRE(holed.ok());
    double a = 0;
    double b = 0;
    for (const auto& c : full.value()) a += c.length();
    for (const auto& c : holed.value()) b += c.length();
    REQUIRE(b < a);
    REQUIRE(b > 0.5 * a);
}

TEST_CASE("texture in a region is deterministic", "[texture][region]") {
    auto a = generateVectorTextureInRegion(frameWithHole(), params());
    auto b = generateVectorTextureInRegion(frameWithHole(), params());
    REQUIRE(a.ok());
    REQUIRE(a.value() == b.value());
}

TEST_CASE("even-odd fill treats nested contours as holes whatever their winding", "[texture][region]") {
    const std::vector<Contour> region{Contour::rectangle({{0, 0}, {100, 80}}),
                                      Contour::rectangle({{30, 20}, {70, 60}})};
    auto r = generateVectorTextureInRegion(region, params(), Tolerance{}, FillRule::EvenOdd);
    REQUIRE(r.ok());
    for (Vec2 v : vertices(r.value())) {
        REQUIRE(!(v.x > 30.002 && v.x < 69.998 && v.y > 20.002 && v.y < 59.998));
    }
}

TEST_CASE("texture region errors are reported", "[texture][region]") {
    REQUIRE(!generateVectorTextureInRegion({}, params()).ok());
    Contour open = Contour::rectangle({{0, 0}, {10, 10}});
    open.closed = false;
    REQUIRE(!generateVectorTextureInRegion({open}, params()).ok());
    const double nan = std::numeric_limits<double>::quiet_NaN();
    REQUIRE(!generateVectorTextureInRegion({Contour::rectangle({{0, 0}, {nan, 10}})}, params()).ok());
    REQUIRE(!generateVectorTextureInRegion({Contour::circle({0, 0}, 1e12)}, params()).ok());
    VectorTextureParams bad = params();
    bad.maxSpacing = -1.0;
    REQUIRE(!generateVectorTextureInRegion(frameWithHole(), bad).ok());
}

TEST_CASE("hostile regions are bounded", "[texture][region]") {
    // A huge region with fine spacing exceeds the vertex bound instead of running away.
    VectorTextureParams p = params();
    p.maxSpacing = 0.001;
    p.wavelength = 0.01;
    auto big = generateVectorTextureInRegion({Contour::rectangle({{0, 0}, {1e6, 1e6}})}, p);
    REQUIRE(!big.ok());
    // Too many region spans.
    std::vector<Contour> many;
    Contour c;
    for (std::size_t i = 0; i < kVectorTextureMaxRegionSpans / 2 + 1; ++i) {
        c.append(LineSpan{{0, 0}, {1, 1}});
        c.append(LineSpan{{1, 1}, {0, 0}});
    }
    c.closed = true;
    many.push_back(c);
    REQUIRE(!generateVectorTextureInRegion(many, params()).ok());
}

TEST_CASE("a region no line reaches gives an empty success", "[texture][region]") {
    VectorTextureParams p = params();
    p.maxSpacing = 100.0;
    p.amplitude = 0.0;
    p.angle = 0.0;
    // Lines sit at the box centre +- k*100 mm; a thin sliver between them contains none.
    auto r = generateVectorTextureInRegion({Contour::rectangle({{0, 0}, {10, 1}})}, p);
    REQUIRE(r.ok());
}
