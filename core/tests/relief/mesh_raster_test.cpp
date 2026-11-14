#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>

#include "carvetto/relief/mesh_raster.hpp"

using namespace carvetto;
using namespace carvetto::relief;
using io::TriangleMesh;
using io::Vec3;

namespace {

void addTri(TriangleMesh& m, std::uint32_t a, std::uint32_t b, std::uint32_t c) {
    m.triangles.push_back({{a, b, c}});
}

// Closed axis-aligned box, outward CCW.
void addBox(TriangleMesh& m, double x0, double y0, double z0, double x1, double y1, double z1) {
    const auto o = static_cast<std::uint32_t>(m.vertices.size());
    m.vertices.push_back({x0, y0, z0});  // 0
    m.vertices.push_back({x1, y0, z0});  // 1
    m.vertices.push_back({x1, y1, z0});  // 2
    m.vertices.push_back({x0, y1, z0});  // 3
    m.vertices.push_back({x0, y0, z1});  // 4
    m.vertices.push_back({x1, y0, z1});  // 5
    m.vertices.push_back({x1, y1, z1});  // 6
    m.vertices.push_back({x0, y1, z1});  // 7
    const std::uint32_t f[12][3] = {{0, 3, 2}, {0, 2, 1}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
                                    {1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
    for (const auto& t : f) addTri(m, o + t[0], o + t[1], o + t[2]);
}

TriangleMesh box(double x0, double y0, double z0, double x1, double y1, double z1) {
    TriangleMesh m;
    addBox(m, x0, y0, z0, x1, y1, z1);
    return m;
}

TriangleMesh pyramid() {
    TriangleMesh m;
    m.vertices = {{-4, -4, 0}, {4, -4, 0}, {4, 4, 0}, {-4, 4, 0}, {0, 0, 4}};
    addTri(m, 0, 3, 2);
    addTri(m, 0, 2, 1);
    addTri(m, 0, 1, 4);
    addTri(m, 1, 2, 4);
    addTri(m, 2, 3, 4);
    addTri(m, 3, 0, 4);
    return m;
}

TriangleMesh sphere(double r, int seg, int rings) {
    TriangleMesh m;
    m.vertices.push_back({0, 0, r});  // north pole 0
    for (int i = 1; i < rings; ++i) {
        const double th = 3.14159265358979323846 * i / rings;
        for (int k = 0; k < seg; ++k) {
            const double ph = 2 * 3.14159265358979323846 * k / seg;
            m.vertices.push_back({r * std::sin(th) * std::cos(ph), r * std::sin(th) * std::sin(ph), r * std::cos(th)});
        }
    }
    m.vertices.push_back({0, 0, -r});
    const auto south = static_cast<std::uint32_t>(m.vertices.size() - 1);
    auto idx = [&](int ring, int k) { return static_cast<std::uint32_t>(1 + (ring - 1) * seg + (k % seg)); };
    for (int k = 0; k < seg; ++k) addTri(m, 0, idx(1, k), idx(1, k + 1));
    for (int i = 1; i < rings - 1; ++i)
        for (int k = 0; k < seg; ++k) {
            addTri(m, idx(i, k), idx(i + 1, k), idx(i + 1, k + 1));
            addTri(m, idx(i, k), idx(i + 1, k + 1), idx(i, k + 1));
        }
    for (int k = 0; k < seg; ++k) addTri(m, south, idx(rings - 1, k + 1), idx(rings - 1, k));
    return m;
}

MeshRaster run(const TriangleMesh& m, const MeshRasterOptions& o) {
    auto r = meshToHeightField(m, o);
    REQUIRE(r.ok());
    return r.value();
}

MeshRasterOptions opts(double cell) {
    MeshRasterOptions o;
    o.cellSize = cell;
    return o;
}

}  // namespace

TEST_CASE("box gives its height in every cell, including cells on triangle diagonals") {
    // Cell centres at 0.5, 1.5, ... : the (0,0)-(4,4) diagonal passes through centres exactly.
    const MeshRaster r = run(box(0, 0, 0, 4, 4, 3), opts(1.0));
    REQUIRE(r.top.width() == 4);
    REQUIRE(r.top.height() == 4);
    CHECK(r.top.origin().x == 0.5);
    for (std::size_t j = 0; j < 4; ++j)
        for (std::size_t i = 0; i < 4; ++i) CHECK(r.top.at(i, j) == 3.0f);
    CHECK(r.maxHeight == 3.0);
    CHECK(r.undercutCount == 0);
    CHECK(r.bottom.empty());
}

TEST_CASE("box with cell centres on its edges and vertices has no cracks") {
    MeshRasterOptions o = opts(1.0);
    o.grid = Grid{6, 6, 1.0, {-1.0, -1.0}};  // centres at integer coordinates, on the box boundary
    const MeshRaster r = run(box(0, 0, 0, 4, 4, 2), o);
    // Each interior or owned-boundary centre is covered exactly once; interior is certainly covered.
    for (std::size_t j = 2; j <= 4; ++j)
        for (std::size_t i = 2; i <= 4; ++i) CHECK(r.top.at(i, j) == 2.0f);
    // Outside the footprint stays transparent.
    CHECK(std::isnan(r.top.at(0, 0)));
    CHECK(std::isnan(r.top.at(5, 5)));
    // Every centre on a shared diagonal or edge is hit by exactly one top triangle: no undercut flags.
    CHECK(r.undercutCount == 0);
    // Ownership is the top-left rule: lower/left boundary (x=0 or y=0) belongs to the box, the far
    // boundary (x=4 or y=4) does not.
    CHECK(r.top.at(1, 1) == 2.0f);
    CHECK(std::isnan(r.top.at(5, 3)));
    CHECK(std::isnan(r.top.at(3, 5)));
}

TEST_CASE("pyramid matches the analytic height") {
    const MeshRaster r = run(pyramid(), opts(0.5));
    REQUIRE(r.top.width() == 16);
    double maxErr = 0;
    for (std::size_t j = 0; j < r.top.height(); ++j)
        for (std::size_t i = 0; i < r.top.width(); ++i) {
            const double x = r.top.origin().x + static_cast<double>(i) * 0.5;
            const double y = r.top.origin().y + static_cast<double>(j) * 0.5;
            const double expect = 4.0 - std::max(std::fabs(x), std::fabs(y));
            REQUIRE(!std::isnan(r.top.at(i, j)));
            maxErr = std::max(maxErr, std::fabs(static_cast<double>(r.top.at(i, j)) - expect));
        }
    CHECK(maxErr < 1e-5);
    CHECK(r.maxHeight == r.top.at(8, 8));  // centre-most cell is the highest
    CHECK(r.undercutCount == 0);
}

TEST_CASE("sphere matches the analytic height within the faceting tolerance") {
    const double R = 5.0;
    MeshRasterOptions o = opts(0.25);
    const MeshRaster r = run(sphere(R, 96, 48), o);
    double maxErr = 0;
    int checked = 0;
    for (std::size_t j = 0; j < r.top.height(); ++j)
        for (std::size_t i = 0; i < r.top.width(); ++i) {
            const double x = r.top.origin().x + static_cast<double>(i) * 0.25;
            const double y = r.top.origin().y + static_cast<double>(j) * 0.25;
            const double d2 = x * x + y * y;
            if (d2 > 4.5 * 4.5) continue;
            const double expect = std::sqrt(R * R - d2);
            REQUIRE(!std::isnan(r.top.at(i, j)));
            maxErr = std::max(maxErr, std::fabs(static_cast<double>(r.top.at(i, j)) - expect));
            ++checked;
        }
    CHECK(checked > 900);
    CHECK(maxErr < 0.03);
    // Zero plane at the equator: the lower hemisphere is discarded and nothing is hidden.
    CHECK(r.undercutCount == 0);
    // Corner cells outside the circle but inside the bounds have no surface.
    CHECK(std::isnan(r.top.at(0, 0)));
}

TEST_CASE("zero plane shifts heights; discard clamps, keeping negatives otherwise") {
    TriangleMesh m = box(0, 0, 1, 2, 2, 3);
    MeshRasterOptions o = opts(1.0);
    o.zeroPlane = 2.0;
    MeshRaster r = run(m, o);
    CHECK(r.top.at(0, 0) == 1.0f);
    o.zeroPlane = 5.0;
    r = run(m, o);
    CHECK(r.top.at(0, 0) == 0.0f);  // discarded below the plane
    CHECK(r.maxHeight == 0.0);
    o.zeroPlane = 3.5;  // top at -0.5: a small negative is clamped too
    r = run(m, o);
    CHECK(r.top.at(0, 0) == 0.0f);
    o.zeroPlane = 5.0;
    o.discardBelowZero = false;
    r = run(m, o);
    CHECK(r.top.at(0, 0) == -2.0f);
    CHECK(r.maxHeight == 0.0);  // max over cells starts at 0
}

TEST_CASE("undercut mask on an overhang mesh") {
    // Base slab z 0..1 over x 0..6, roof slab z 2..3 over x 3..9 (overhang on x 6..9, hidden
    // material on x 3..6 under the roof). Cells are 1 mm wide, centres at 0.5 ... 8.5; y 0..1.
    TriangleMesh m;
    addBox(m, 0, 0, 0, 6, 1, 1);
    addBox(m, 3, 0, 2, 9, 1, 3);
    MeshRasterOptions o = opts(1.0);
    o.discardBelowZero = false;  // material below the plane stays: hidden material under the roof
    MeshRaster r = run(m, o);
    REQUIRE(r.top.width() == 9);
    for (std::size_t i = 0; i < 9; ++i) {
        const bool expectUc = i >= 3 && i <= 5;  // roof over base
        CHECK(r.undercut[i] == (expectUc ? 1 : 0));
        CHECK(r.top.at(i, 0) == (i >= 3 ? 3.0f : 1.0f));
    }
    CHECK(r.undercutCount == 3);

    // With the plane at 0 and data below discarded, the overhang x 6..9 is a cavity too.
    o.discardBelowZero = true;
    r = run(m, o);
    for (std::size_t i = 0; i < 9; ++i) CHECK(r.undercut[i] == (i >= 3 ? 1 : 0));
    CHECK(r.undercutCount == 6);

    // Raising the plane above the base slab removes the hidden material region from view.
    o.zeroPlane = 2.0;
    r = run(m, o);
    CHECK(r.undercutCount == 0);

    // Detection can be switched off.
    o.detectUndercuts = false;
    r = run(m, o);
    CHECK(r.undercut.empty());
    CHECK(r.undercutCount == 0);
}

TEST_CASE("a box resting on the plane is not an undercut; a floating one is") {
    MeshRasterOptions o = opts(1.0);
    CHECK(run(box(0, 0, 0, 3, 3, 2), o).undercutCount == 0);
    CHECK(run(box(0, 0, 0.5, 3, 3, 2), o).undercutCount == 9);
    // Without discarding, a lone floating slab hides no material (nothing below it).
    o.discardBelowZero = false;
    CHECK(run(box(0, 0, 0.5, 3, 3, 2), o).undercutCount == 0);
}

TEST_CASE("inverted winding still rasterises and does not flag undercuts") {
    TriangleMesh m = box(0, 0, 0, 3, 3, 2);
    for (auto& t : m.triangles) std::swap(t[1], t[2]);
    const MeshRaster r = run(m, opts(1.0));
    CHECK(r.top.at(1, 1) == 2.0f);
    CHECK(r.undercutCount == 0);
}

TEST_CASE("two-sided: bottom side is the depth below the plane") {
    // Box from z -2 to 3 with the plane at 0.
    MeshRasterOptions o = opts(1.0);
    o.bothSides = true;
    const MeshRaster r = run(box(0, 0, -2, 3, 3, 3), o);
    CHECK(r.top.at(1, 1) == 3.0f);
    CHECK(r.bottom.at(1, 1) == 2.0f);
    CHECK(r.maxHeight == 3.0);
    CHECK(r.maxDepth == 2.0);
    // A box entirely above the plane has depth 0 on the bottom side.
    const MeshRaster r2 = run(box(0, 0, 1, 3, 3, 3), o);
    CHECK(r2.bottom.at(1, 1) == 0.0f);
    CHECK(r2.maxDepth == 0.0);
}

TEST_CASE("explicit grid: cells outside the mesh are transparent, model is sampled at centres") {
    MeshRasterOptions o = opts(1.0);
    o.grid = Grid{6, 3, 1.0, {-1.5, 0.5}};
    const MeshRaster r = run(box(0, 0, 0, 3, 3, 2), o);
    REQUIRE(r.top.width() == 6);
    CHECK(std::isnan(r.top.at(0, 0)));
    CHECK(std::isnan(r.top.at(1, 0)));  // x = -0.5
    CHECK(r.top.at(2, 0) == 2.0f);      // x = 0.5
    CHECK(r.top.at(4, 2) == 2.0f);      // x = 2.5, y = 2.5
    CHECK(std::isnan(r.top.at(5, 0)));  // x = 3.5
}

TEST_CASE("fit-to-mesh grid size and non-integer extents") {
    const MeshRaster r = run(box(0, 0, 0, 2.2, 1.0, 1), opts(1.0));
    CHECK(r.top.width() == 3);
    CHECK(r.top.height() == 1);
    CHECK(r.top.at(1, 0) == 1.0f);
    CHECK(std::isnan(r.top.at(2, 0)));  // centre 2.5 is past the mesh at 2.2
}

TEST_CASE("deterministic") {
    const TriangleMesh m = sphere(5.0, 40, 20);
    const MeshRaster a = run(m, opts(0.3)), b = run(m, opts(0.3));
    REQUIRE(a.top.data().size() == b.top.data().size());
    for (std::size_t i = 0; i < a.top.data().size(); ++i) {
        const float x = a.top.data()[i], y = b.top.data()[i];
        CHECK((std::isnan(x) ? std::isnan(y) : x == y));
    }
    CHECK(a.undercut == b.undercut);
}

TEST_CASE("cancel is polled per row and aborts") {
    MeshRasterOptions o = opts(0.1);
    int calls = 0;
    o.cancel = [&] { return ++calls > 3; };
    auto r = meshToHeightField(box(0, 0, 0, 10, 10, 1), o);
    REQUIRE(!r.ok());
    CHECK(r.error() == "cancelled");
    CHECK(calls == 4);
    // A cancel that never fires lets it finish after exactly one poll per row.
    calls = 0;
    o.cancel = [&] { ++calls; return false; };
    REQUIRE(meshToHeightField(box(0, 0, 0, 10, 10, 1), o).ok());
    CHECK(calls == 100);
}

TEST_CASE("bad input is rejected") {
    TriangleMesh empty;
    CHECK(!meshToHeightField(empty, opts(1.0)).ok());
    CHECK(!meshToHeightField(box(0, 0, 0, 1, 1, 1), opts(0.0)).ok());
    CHECK(!meshToHeightField(box(0, 0, 0, 1, 1, 1), opts(std::nan(""))).ok());
    MeshRasterOptions o = opts(1.0);
    o.zeroPlane = std::nan("");
    CHECK(!meshToHeightField(box(0, 0, 0, 1, 1, 1), o).ok());
    TriangleMesh m = box(0, 0, 0, 1, 1, 1);
    m.triangles[0][0] = 99;
    CHECK(!meshToHeightField(m, opts(1.0)).ok());
    m = box(0, 0, 0, 1, 1, 1);
    m.vertices[0].z = std::nan("");
    CHECK(!meshToHeightField(m, opts(1.0)).ok());
    // Too many cells.
    CHECK(!meshToHeightField(box(0, 0, 0, 1e5, 1e5, 1), opts(1.0)).ok());
    // Grid override that is invalid.
    o = opts(1.0);
    o.grid = Grid{0, 4, 1.0, {0, 0}};
    CHECK(!meshToHeightField(box(0, 0, 0, 1, 1, 1), o).ok());
}

TEST_CASE("vertical and degenerate triangles are ignored; mesh outside the grid is skipped") {
    TriangleMesh m = box(0, 0, 0, 2, 2, 1);
    m.vertices.push_back({1, 1, 5});
    m.vertices.push_back({1, 1, 6});
    m.vertices.push_back({1, 1, 7});
    m.triangles.push_back({{8, 9, 10}});  // zero footprint
    MeshRasterOptions o = opts(1.0);
    o.grid = Grid{2, 2, 1.0, {50.5, 50.5}};
    const MeshRaster far = run(m, o);
    CHECK(std::isnan(far.top.at(0, 0)));
    const MeshRaster r = run(m, opts(1.0));
    CHECK(r.top.at(0, 0) == 1.0f);
}
