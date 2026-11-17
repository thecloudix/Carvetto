// T-0321: a mesh onto the cylinder of a rotary job (ADR-0114).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <numbers>

#include "carvetto/relief/rotary_raster.hpp"

using namespace carvetto::relief;
using carvetto::io::TriangleMesh;
using carvetto::io::Vec3;
using Catch::Approx;

namespace {

constexpr double kPi = std::numbers::pi;

/// Regular n-gon prism along X from x0 to x1, circumradius rc about (cy, cz), with end caps. Vertex k is at
/// cross-section angle 2 pi k / n (angle measured from +Z towards +Y, like the cylinder map).
TriangleMesh prism(int n, double rc, double x0, double x1, double cy = 0.0, double cz = 0.0) {
    TriangleMesh m;
    for (const double x : {x0, x1}) {
        for (int k = 0; k < n; ++k) {
            const double a = 2.0 * kPi * k / n;
            m.vertices.push_back({x, cy + rc * std::sin(a), cz + rc * std::cos(a)});
        }
    }
    m.vertices.push_back({x0, cy, cz});
    m.vertices.push_back({x1, cy, cz});
    const auto c0 = static_cast<std::uint32_t>(2 * n), c1 = c0 + 1;
    for (int k = 0; k < n; ++k) {
        const auto a = static_cast<std::uint32_t>(k), b = static_cast<std::uint32_t>((k + 1) % n);
        const auto a1 = a + static_cast<std::uint32_t>(n), b1 = b + static_cast<std::uint32_t>(n);
        m.triangles.push_back({a, b, b1});
        m.triangles.push_back({a, b1, a1});
        m.triangles.push_back({c0, b, a});
        m.triangles.push_back({c1, a1, b1});
    }
    return m;
}

/// Distance from the axis to the polygon along angle `theta` (the prism's analytic surface).
double polygonRadius(int n, double rc, double theta) {
    const double step = 2.0 * kPi / n;
    const double t = theta - step * std::floor(theta / step) - step / 2.0;
    return rc * std::cos(kPi / n) / std::cos(t);
}

RotaryRasterOptions optionsFor(double radius, double length, double cell, bool alongX = true) {
    RotaryRasterOptions o;
    o.map.alongX = alongX;
    o.map.radius = radius;
    const auto circ = static_cast<std::size_t>(std::ceil(2.0 * kPi * radius / cell));
    const auto len = static_cast<std::size_t>(std::ceil(length / cell));
    o.grid = alongX ? Grid{len, circ, cell, {0.5 * cell, 0.5 * cell}} : Grid{circ, len, cell, {0.5 * cell, 0.5 * cell}};
    return o;
}

float at(const RotaryRaster& r, std::size_t along, std::size_t circ, bool alongX = true) {
    return alongX ? r.field.at(along, circ) : r.field.at(circ, along);
}

}  // namespace

TEST_CASE("rotary raster: a prism along X gives its analytic radius at every cell", "[relief][rotary][mesh]") {
    const int n = 12;
    const TriangleMesh mesh = prism(n, 10.0, 0.0, 20.0, 3.0, -2.0);
    RotaryRasterOptions o = optionsFor(10.0, 20.0, 0.5);
    o.axisPoint = {3.0, -2.0};
    auto r = meshToRotaryField(mesh, o);
    REQUIRE(r.ok());
    const RotaryRaster& rr = r.value();
    const std::size_t nCirc = o.grid.height;
    CHECK(rr.hitCells == o.grid.width * nCirc);
    for (std::size_t j = 0; j < nCirc; ++j) {
        const double theta = (0.25 + 0.5 * static_cast<double>(j)) / 10.0;
        for (std::size_t i = 0; i < o.grid.width; ++i) {
            CHECK(at(rr, i, j) == Approx(polygonRadius(n, 10.0, theta)).margin(1e-5));
        }
    }
    CHECK(rr.maxRadius == Approx(10.0).margin(0.2));
    // The counter: the side triangles are tested only for their own angular slice (and the caps for none of the
    // columns), far less than every triangle against every cell.
    const std::uint64_t all = mesh.triangles.size() * o.grid.width * nCirc;
    CHECK(rr.rayTests > 0);
    CHECK(rr.rayTests < all / 4);
    auto again = meshToRotaryField(mesh, o);
    REQUIRE(again.ok());
    CHECK(again.value().rayTests == rr.rayTests);  // deterministic
}

TEST_CASE("rotary raster: along Y is the transpose; flip reverses the angle", "[relief][rotary][mesh]") {
    const int n = 7;  // odd: not symmetric under theta -> -theta
    const TriangleMesh mesh = prism(n, 8.0, 0.0, 12.0);
    RotaryRasterOptions ox = optionsFor(8.0, 12.0, 0.5);
    auto rx = meshToRotaryField(mesh, ox);
    REQUIRE(rx.ok());
    TriangleMesh swapped = mesh;
    for (Vec3& v : swapped.vertices) {
        // Along Y: the cross-section is (X, Z); the angle runs from +Z towards +X, so X takes the old Y role.
        v = {v.y, v.x, v.z};
    }
    RotaryRasterOptions oy = optionsFor(8.0, 12.0, 0.5, false);
    auto ry = meshToRotaryField(swapped, oy);
    REQUIRE(ry.ok());
    for (std::size_t j = 0; j < ox.grid.height; ++j) {
        for (std::size_t i = 0; i < ox.grid.width; ++i) CHECK(at(ry.value(), i, j, false) == at(rx.value(), i, j));
    }
    ox.map.flip = true;
    auto rf = meshToRotaryField(mesh, ox);
    REQUIRE(rf.ok());
    for (std::size_t j = 0; j < ox.grid.height; ++j) {
        const double theta = -(0.25 + 0.5 * static_cast<double>(j)) / 8.0;
        CHECK(at(rf.value(), 3, j) == Approx(polygonRadius(n, 8.0, theta)).margin(1e-5));
    }
}

TEST_CASE("rotary raster: placement along the axis, axis movement, zero radius", "[relief][rotary][mesh]") {
    // A cube of half size 5 centred on the axis.
    TriangleMesh cube = prism(4, 5.0 * std::sqrt(2.0), 0.0, 10.0);
    // The prism is a diamond (vertices on the axes); turn it into a square by sampling: use a 4-gon rotated by
    // 45 degrees instead.
    for (Vec3& v : cube.vertices) {
        const double y = v.y, z = v.z;
        v.y = (y + z) / std::sqrt(2.0);
        v.z = (z - y) / std::sqrt(2.0);
    }
    RotaryRasterOptions o = optionsFor(5.0, 30.0, 0.5);
    o.alongStart = 5.0;  // mesh x 0 lands at flat x 5
    auto r = meshToRotaryField(cube, o);
    REQUIRE(r.ok());
    const std::size_t i0 = static_cast<std::size_t>(5.0 / 0.5), i1 = static_cast<std::size_t>(15.0 / 0.5);
    for (std::size_t i = 0; i < o.grid.width; ++i) {
        if (i < i0 || i >= i1) {
            CHECK(std::isnan(at(r.value(), i, 0)));
        } else {
            CHECK_FALSE(std::isnan(at(r.value(), i, 0)));
        }
    }
    // Face distances: straight up (theta 0 is row 0 at c = 0.25, close to 0) is half size / cos(theta).
    const double th0 = 0.25 / 5.0;
    CHECK(at(r.value(), 15, 0) == Approx(5.0 / std::cos(th0)).margin(1e-5));
    // Move the axis 1 mm towards +Y: the +Y face is nearer, the -Y face farther. theta = pi/2 is c = pi r / 2.
    o.alongStart = 0.0;
    o.axisPoint = {1.0, 0.0};
    auto moved = meshToRotaryField(cube, o);
    REQUIRE(moved.ok());
    const std::size_t quarter = static_cast<std::size_t>(std::lround((kPi * 5.0 / 2.0 - 0.25) / 0.5));
    const double thq = (0.25 + 0.5 * static_cast<double>(quarter)) / 5.0;
    // +Y face at distance (5 - 1) along the ray at angle thq from +Z: (4) / sin(thq).
    CHECK(at(moved.value(), 5, quarter) == Approx(4.0 / std::sin(thq)).margin(1e-5));
    // Zero radius and discard.
    o.axisPoint = {0.0, 0.0};
    o.zeroRadius = 6.0;
    auto z1 = meshToRotaryField(cube, o);
    REQUIRE(z1.ok());
    CHECK(at(z1.value(), 5, 0) == 0.0F);  // below the zero radius: clamped
    // maxRadius is the distance from the axis, not the (clamped) height above the zero radius.
    CHECK(z1.value().maxRadius > 6.0);
    CHECK(z1.value().maxRadius == Approx(5.0 * std::sqrt(2.0)).margin(0.1));
    o.discardBelowZero = false;
    auto z2 = meshToRotaryField(cube, o);
    REQUIRE(z2.ok());
    CHECK(at(z2.value(), 5, 0) == Approx(5.0 / std::cos(th0) - 6.0).margin(1e-5));
}

TEST_CASE("rotary raster: a slanted triangle over the axis is tested against the whole ring, exact counter", "[relief][rotary][mesh]") {
    // Plane a = 5 + s / 2 through the axis region, triangle (a, s, u): its projection contains the origin.
    TriangleMesh m;
    m.vertices = {{0, -10, -5}, {10, 10, -5}, {5, 0, 8}};
    m.triangles = {{0, 1, 2}};
    RotaryRasterOptions o = optionsFor(4.0, 12.0, 0.5);
    auto r = meshToRotaryField(m, o);
    REQUIRE(r.ok());
    const std::size_t nCirc = o.grid.height;
    std::uint64_t columns = 0;
    for (std::size_t i = 0; i < o.grid.width; ++i) {
        const double a = 0.25 + 0.5 * static_cast<double>(i);
        if (a >= 0.0 && a <= 10.0) ++columns;
        for (std::size_t j = 0; j < nCirc; ++j) {
            const double theta = (0.25 + 0.5 * static_cast<double>(j)) / 4.0;
            const double s = std::sin(theta), u = std::cos(theta);
            // Hit where a = 5 + t s / 2.
            float expect = kTransparent;
            if (std::fabs(s) > 1e-9) {
                const double t = (a - 5.0) * 2.0 / s;
                if (t >= 0.0) {
                    const double ps = t * s, pu = t * u;
                    const auto side = [](double ax, double ay, double bx, double by, double px, double py) {
                        return (bx - ax) * (py - ay) - (by - ay) * (px - ax);
                    };
                    const double d0 = side(-10, -5, 10, -5, ps, pu), d1 = side(10, -5, 0, 8, ps, pu), d2 = side(0, 8, -10, -5, ps, pu);
                    if ((d0 >= 0 && d1 >= 0 && d2 >= 0) || (d0 <= 0 && d1 <= 0 && d2 <= 0)) expect = static_cast<float>(t);
                }
            }
            const float got = at(r.value(), i, j);
            if (std::isnan(expect)) {
                CHECK(std::isnan(got));
            } else {
                CHECK(got == Approx(expect).margin(1e-4));
            }
        }
    }
    CHECK(r.value().rayTests == columns * nCirc);
}

TEST_CASE("rotary raster: counter of a small triangle equals its cells in range", "[relief][rotary][mesh]") {
    TriangleMesh m;
    m.vertices = {{2, 5, 5}, {6, 5, 5}, {2, 6, 5}};
    m.triangles = {{0, 1, 2}};
    RotaryRasterOptions o = optionsFor(10.0, 12.0, 0.25);
    auto r = meshToRotaryField(m, o);
    REQUIRE(r.ok());
    const double th0 = std::atan2(5.0, 5.0), th1 = std::atan2(6.0, 5.0);
    std::uint64_t expect = 0;
    for (std::size_t j = 0; j < o.grid.height; ++j) {
        const double theta = (0.125 + 0.25 * static_cast<double>(j)) / 10.0;
        if (theta < th0 - 1e-6 || theta > th1 + 1e-6) continue;
        for (std::size_t i = 0; i < o.grid.width; ++i) {
            const double a = 0.125 + 0.25 * static_cast<double>(i);
            if (a >= 2.0 - 1e-6 && a <= 6.0 + 1e-6) ++expect;
        }
    }
    CHECK(expect > 0);
    CHECK(r.value().rayTests == expect);
    CHECK(r.value().hitCells <= expect);
    CHECK(r.value().hitCells > 0);
}

TEST_CASE("rotary raster: refused inputs, work cap and cancel", "[relief][rotary][mesh]") {
    const TriangleMesh good = prism(8, 5.0, 0.0, 6.0);
    RotaryRasterOptions o = optionsFor(5.0, 6.0, 0.5);
    CHECK(meshToRotaryField(good, o).ok());
    CHECK_FALSE(meshToRotaryField(TriangleMesh{}, o).ok());
    TriangleMesh badIndex = good;
    badIndex.triangles[0][0] = 9999;
    CHECK_FALSE(meshToRotaryField(badIndex, o).ok());
    TriangleMesh nan = good;
    nan.vertices[0].x = std::nan("");
    CHECK_FALSE(meshToRotaryField(nan, o).ok());
    TriangleMesh huge = good;  // out of range: refused, not silently dropped or cast out of range
    huge.vertices[0].x = -1e300;
    huge.vertices[1].x = 1e300;
    const auto hugeResult = meshToRotaryField(huge, o);
    CHECK_FALSE(hugeResult.ok());
    CHECK_FALSE(hugeResult.error().empty());
    RotaryRasterOptions hugeAxis = o;
    hugeAxis.axisPoint = {1e300, 0.0};
    CHECK_FALSE(meshToRotaryField(good, hugeAxis).ok());
    RotaryRasterOptions badRadius = o;
    badRadius.map.radius = 0.0;
    CHECK_FALSE(meshToRotaryField(good, badRadius).ok());
    RotaryRasterOptions badGrid = o;
    badGrid.grid.cellSize = 0.0;
    CHECK_FALSE(meshToRotaryField(good, badGrid).ok());
    RotaryRasterOptions capped = o;
    capped.maxRayTests = 1;
    const auto few = meshToRotaryField(good, capped);
    REQUIRE_FALSE(few.ok());
    CHECK(few.error().find("ray tests") != std::string::npos);
    const std::uint64_t needed = meshToRotaryField(good, o).value().rayTests;
    capped.maxRayTests = needed;
    CHECK(meshToRotaryField(good, capped).ok());  // exactly the work needed is allowed
    capped.maxRayTests = needed - 1;
    CHECK_FALSE(meshToRotaryField(good, capped).ok());
    RotaryRasterOptions cancel = o;
    cancel.cancel = [] { return true; };
    const auto c = meshToRotaryField(good, cancel);
    REQUIRE_FALSE(c.ok());
    CHECK(c.error() == "cancelled");
}

TEST_CASE("rotary raster: the farthest surface wins where the ray meets several", "[relief][rotary][mesh]") {
    // An inner prism listed after the outer one: either order must give the outer radius.
    for (const bool innerLast : {true, false}) {
        TriangleMesh outer = prism(16, 8.0, 0.0, 10.0);
        TriangleMesh inner = prism(16, 3.0, 0.0, 10.0);
        TriangleMesh m = innerLast ? outer : inner;
        const TriangleMesh& add = innerLast ? inner : outer;
        const auto base = static_cast<std::uint32_t>(m.vertices.size());
        for (const Vec3& v : add.vertices) m.vertices.push_back(v);
        for (const auto& t : add.triangles) m.triangles.push_back({t[0] + base, t[1] + base, t[2] + base});
        RotaryRasterOptions o = optionsFor(8.0, 10.0, 0.5);
        auto r = meshToRotaryField(m, o);
        REQUIRE(r.ok());
        for (std::size_t j = 0; j < o.grid.height; ++j) {
            const double theta = (0.25 + 0.5 * static_cast<double>(j)) / 8.0;
            CHECK(at(r.value(), 5, j) == Approx(polygonRadius(16, 8.0, theta)).margin(1e-5));
        }
    }
}

TEST_CASE("rotary raster: a ray exactly on the shared edge of two triangles still hits", "[relief][rotary][mesh]") {
    for (const double k : {0.1, 0.3, 0.7, 1.1, 1.3, 2.9}) {
        // The wall u = 5k, s in [-5k, 5k], a in [0, 10k], split on the diagonal s = a - 5k, which the ray at angle 0
        // from the axis point a = 5k crosses exactly at (5k, 0, 5k).
        TriangleMesh m;
        m.vertices = {{0, -5 * k, 5 * k}, {10 * k, 5 * k, 5 * k}, {10 * k, -5 * k, 5 * k}, {0, 5 * k, 5 * k}};
        m.triangles = {{0, 1, 2}, {0, 1, 3}};
        RotaryRasterOptions o;
        o.map.radius = 4.0;
        o.grid = Grid{21, 3, k / 2.0, {0.0, 0.0}};  // column 10 is at a = 5k, row 0 at circumference 0
        auto r = meshToRotaryField(m, o);
        REQUIRE(r.ok());
        CHECK_FALSE(std::isnan(r.value().field.at(10, 0)));
        CHECK(r.value().field.at(10, 0) == Approx(5.0 * k).margin(1e-9));
    }
}

TEST_CASE("rotary raster: rays along a prism's edges (vertex angles) are never lost to rounding", "[relief][rotary][mesh]") {
    const int n = 12;
    const TriangleMesh mesh = prism(n, 5.0, 0.0, 6.0);
    for (int k = 0; k < n; ++k) {
        RotaryRasterOptions o;
        o.map.radius = 1.0;
        o.grid = Grid{24, 1, 0.25, {0.125, 2.0 * kPi * k / n}};  // one row exactly at vertex angle k
        auto r = meshToRotaryField(mesh, o);
        REQUIRE(r.ok());
        for (std::size_t i = 0; i < 24; ++i) {
            REQUIRE_FALSE(std::isnan(r.value().field.at(i, 0)));
            CHECK(r.value().field.at(i, 0) == Approx(5.0).margin(1e-6));  // the vertex itself: the circumradius
        }
    }
}

TEST_CASE("rotary raster: huge but in-range triangles over a tiny cell stay well defined", "[relief][rotary][mesh]") {
    // Index arithmetic reaches 1e20 here, beyond long long: it must clamp, not cast out of range.
    TriangleMesh m;
    m.vertices = {{-1e11, -1e11, 1e-9}, {1e11, -1e11, 1e-9}, {0.0, 1e11, 1e-9}};
    m.triangles = {{0, 1, 2}};
    RotaryRasterOptions o;
    o.map.alongX = true;
    o.map.radius = 1e-9;
    o.grid = Grid{4, 7, 1e-9, {0.5e-9, 0.5e-9}};
    const auto r = meshToRotaryField(m, o);
    REQUIRE(r.ok());
    CHECK(r.value().hitCells > 0);
    CHECK_FALSE(std::isnan(at(r.value(), 0, 0)));
}
