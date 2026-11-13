#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <random>
#include <string>

#include "carvetto/io/mesh.hpp"

using namespace carvetto::io;

namespace {

// Unit tetrahedron-ish: 4 vertices, 4 triangles.
TriangleMesh tetra() {
    TriangleMesh m;
    m.vertices = {{0, 0, 0}, {2, 0, 0}, {0, 3, 0}, {0, 0, 4}};
    m.triangles = {{{0, 2, 1}}, {{0, 1, 3}}, {{0, 3, 2}}, {{1, 2, 3}}};
    return m;
}

void putU32(std::string& s, std::uint32_t v) {
    for (int i = 0; i < 4; ++i) s.push_back(static_cast<char>((v >> (8 * i)) & 0xFF));
}
void putF(std::string& s, float f) {
    std::uint32_t u;
    std::memcpy(&u, &f, 4);
    putU32(s, u);
}

std::string binaryStl(const TriangleMesh& m, const std::string& header = "binary header") {
    std::string s = header;
    s.resize(80, ' ');
    putU32(s, static_cast<std::uint32_t>(m.triangles.size()));
    for (const auto& t : m.triangles) {
        for (int i = 0; i < 3; ++i) putF(s, 0.0f);  // normal
        for (std::uint32_t idx : t) {
            putF(s, static_cast<float>(m.vertices[idx].x));
            putF(s, static_cast<float>(m.vertices[idx].y));
            putF(s, static_cast<float>(m.vertices[idx].z));
        }
        s.push_back(0);
        s.push_back(0);
    }
    return s;
}

std::string asciiStl(const TriangleMesh& m) {
    std::string s = "solid test\n";
    for (const auto& t : m.triangles) {
        s += "facet normal 0 0 0\n outer loop\n";
        for (std::uint32_t idx : t)
            s += "  vertex " + std::to_string(m.vertices[idx].x) + " " + std::to_string(m.vertices[idx].y) + " " +
                 std::to_string(m.vertices[idx].z) + "\n";
        s += " endloop\nendfacet\n";
    }
    return s + "endsolid test\n";
}

bool sameMesh(const TriangleMesh& a, const TriangleMesh& b) {
    return a.vertices == b.vertices && a.triangles == b.triangles;
}

}  // namespace

TEST_CASE("binary STL round trip welds vertices", "[mesh]") {
    const TriangleMesh m = tetra();
    auto r = readStl(binaryStl(m));
    REQUIRE(r.ok());
    REQUIRE(r.value().vertices.size() == 4);
    REQUIRE(r.value().triangles.size() == 4);
    // First-appearance order: relabel by comparing triangle corner coordinates.
    for (std::size_t t = 0; t < 4; ++t)
        for (std::size_t k = 0; k < 3; ++k)
            CHECK(r.value().vertices[r.value().triangles[t][k]] == m.vertices[m.triangles[t][k]]);
}

TEST_CASE("binary STL whose header starts with solid is still binary", "[mesh]") {
    auto r = readStl(binaryStl(tetra(), "solid fooled you"));
    REQUIRE(r.ok());
    CHECK(r.value().triangles.size() == 4);
}

TEST_CASE("ASCII STL matches binary STL", "[mesh]") {
    auto a = readStl(asciiStl(tetra()));
    auto b = readStl(binaryStl(tetra()));
    REQUIRE(a.ok());
    REQUIRE(b.ok());
    CHECK(sameMesh(a.value(), b.value()));
}

TEST_CASE("ASCII STL accepts upper case, plus signs and exponents", "[mesh]") {
    const std::string s =
        "SOLID x\nFACET NORMAL 0 0 1\nOUTER LOOP\nVERTEX +0 0 0\nVERTEX 1e1 0 0\nVERTEX 0 2.5E0 0\nENDLOOP\nENDFACET\nENDSOLID x";
    auto r = readStl(s);
    REQUIRE(r.ok());
    CHECK(r.value().vertices[1].x == 10.0);
    CHECK(r.value().vertices[2].y == 2.5);
}

TEST_CASE("STL malformed classes return errors", "[mesh]") {
    const std::string bin = binaryStl(tetra());
    CHECK_FALSE(readStl("").ok());
    CHECK_FALSE(readStl("hello").ok());
    CHECK_FALSE(readStl(bin.substr(0, 83)).ok());
    CHECK_FALSE(readStl(bin.substr(0, bin.size() - 1)).ok());  // truncated
    CHECK_FALSE(readStl(bin + "x").ok());                      // trailing byte, count mismatch
    std::string huge = bin;
    huge[80] = huge[81] = huge[82] = huge[83] = static_cast<char>(0xFF);  // 4 G triangles claimed
    CHECK_FALSE(readStl(huge).ok());
    std::string nan = bin;
    const float f = std::nanf("");
    std::memcpy(&nan[84 + 12], &f, 4);
    CHECK_FALSE(readStl(nan).ok());
    std::string inf = bin;
    const float g = INFINITY;
    std::memcpy(&inf[84 + 12], &g, 4);
    CHECK_FALSE(readStl(inf).ok());
    std::string big = bin;
    const float h = 1e30f;
    std::memcpy(&big[84 + 12], &h, 4);
    CHECK_FALSE(readStl(big).ok());

    const std::string a = asciiStl(tetra());
    CHECK_FALSE(readStl(a.substr(0, a.size() - 12)).ok());  // no endsolid
    CHECK_FALSE(readStl(a.substr(0, a.size() / 2)).ok());
    CHECK_FALSE(readStl("solid x\nfacet normal 0 0 0\nouter loop\nvertex 0 0 nan\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid").ok());
    CHECK_FALSE(readStl("solid x\nfacet normal 0 0 0\nouter loop\nvertex 0 0 inf\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid").ok());
    CHECK_FALSE(readStl("solid x\nfacet normal 0 0 0\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nendloop\nendfacet\nendsolid").ok());
    CHECK_FALSE(readStl("solid x\nfacet normal 0 0 0\nouter loop\nvertex 0 0 0\nvertex 1 0 abc\nvertex 0 1 0\nendloop\nendfacet\nendsolid").ok());
}

TEST_CASE("STL caps", "[mesh]") {
    MeshLimits l;
    l.maxTriangles = 3;
    CHECK_FALSE(readStl(binaryStl(tetra()), l).ok());
    CHECK_FALSE(readStl(asciiStl(tetra()), l).ok());
    MeshLimits v;
    v.maxVertices = 3;
    CHECK_FALSE(readStl(binaryStl(tetra()), v).ok());
    MeshLimits b;
    b.maxBytes = 100;
    CHECK_FALSE(readStl(binaryStl(tetra()), b).ok());
}

TEST_CASE("OBJ reads triangles, quads, relative indices and ignores the rest", "[mesh]") {
    const std::string s =
        "# comment\nmtllib a.mtl\no thing\nv 0 0 0\nv 1 0 0\nv 1 1 0 1.0\nv 0 1 0\nvt 0 0\nvn 0 0 1\nusemtl m\ns off\n"
        "f 1/1/1 2/2/1 3/3/1 4//1\nf -4 -3 -2\nl 1 2\n";
    auto r = readObj(s);
    REQUIRE(r.ok());
    CHECK(r.value().vertices.size() == 4);
    REQUIRE(r.value().triangles.size() == 3);
    CHECK(r.value().triangles[0] == std::array<std::uint32_t, 3>{0, 1, 2});
    CHECK(r.value().triangles[1] == std::array<std::uint32_t, 3>{0, 2, 3});
    CHECK(r.value().triangles[2] == std::array<std::uint32_t, 3>{0, 1, 2});
}

TEST_CASE("OBJ with CRLF line ends", "[mesh]") {
    auto r = readObj("v 0 0 0\r\nv 1 0 0\r\nv 0 1 0\r\nf 1 2 3\r\n");
    REQUIRE(r.ok());
    CHECK(r.value().triangles.size() == 1);
}

TEST_CASE("OBJ malformed classes return errors", "[mesh]") {
    const std::string vs = "v 0 0 0\nv 1 0 0\nv 0 1 0\n";
    CHECK_FALSE(readObj(vs + "f 1 2 4\n").ok());     // out of range
    CHECK_FALSE(readObj(vs + "f 0 1 2\n").ok());     // zero index
    CHECK_FALSE(readObj(vs + "f -4 1 2\n").ok());    // relative out of range
    CHECK_FALSE(readObj(vs + "f 1 2\n").ok());       // too few
    CHECK_FALSE(readObj(vs + "f 1 2 x\n").ok());     // junk
    CHECK_FALSE(readObj("f 1 2 3\n").ok());          // before the vertices
    CHECK_FALSE(readObj("v 0 0\n").ok());            // short vertex
    CHECK_FALSE(readObj("v 0 0 nan\n").ok());
    CHECK_FALSE(readObj("v 0 0 inf\n").ok());
    CHECK_FALSE(readObj("v 0 0 1e30\n").ok());
    CHECK_FALSE(readObj("v 0 0 abc\n").ok());
    MeshLimits l;
    l.maxTriangles = 0;
    CHECK_FALSE(readObj(vs + "f 1 2 3\n", l).ok());
    l = {};
    l.maxVertices = 2;
    CHECK_FALSE(readObj(vs, l).ok());
    l = {};
    l.maxBytes = 5;
    CHECK_FALSE(readObj(vs, l).ok());
}

TEST_CASE("Format from path and file reading", "[mesh]") {
    MeshFormat f;
    CHECK(meshFormatFromPath("a/b.STL", f));
    CHECK(f == MeshFormat::Stl);
    CHECK(meshFormatFromPath("x.obj", f));
    CHECK(f == MeshFormat::Obj);
    CHECK_FALSE(meshFormatFromPath("x.3ds", f));
    CHECK_FALSE(meshFormatFromPath("noext", f));
    CHECK_FALSE(readMeshFile("does-not-exist.stl").ok());
    CHECK_FALSE(readMeshFile("x.skp").ok());
}

TEST_CASE("Fuzz: random and mutated input never crashes", "[mesh][fuzz]") {
    std::mt19937 rng(12345);
    const std::string seeds[] = {binaryStl(tetra()), asciiStl(tetra()), "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\nf -1 -2 -3\n"};
    MeshLimits lim;
    lim.maxTriangles = 1000;
    lim.maxVertices = 1000;
    for (int iter = 0; iter < 3000; ++iter) {
        std::string s = seeds[rng() % 3];
        const int edits = 1 + static_cast<int>(rng() % 6);
        for (int e = 0; e < edits && !s.empty(); ++e) {
            switch (rng() % 3) {
                case 0: s[rng() % s.size()] = static_cast<char>(rng()); break;
                case 1: s.resize(rng() % (s.size() + 1)); break;
                default: s.insert(rng() % (s.size() + 1), 1, static_cast<char>(rng())); break;
            }
        }
        auto a = readStl(s, lim);
        auto b = readObj(s, lim);
        if (a.ok()) {
            for (const auto& t : a.value().triangles)
                for (auto i : t) REQUIRE(i < a.value().vertices.size());
        }
        if (b.ok()) {
            for (const auto& t : b.value().triangles)
                for (auto i : t) REQUIRE(i < b.value().vertices.size());
        }
    }
}

TEST_CASE("Bounds, units, centre", "[mesh]") {
    TriangleMesh m = tetra();
    MeshBounds b = meshBounds(m);
    REQUIRE_FALSE(b.empty);
    CHECK(b.size() == Vec3{2, 3, 4});
    CHECK(meshBounds(TriangleMesh{}).empty);
    CHECK(applyMeshUnits(m, MeshUnits::Inches).empty());
    CHECK(meshBounds(m).max.x == 50.8);
    centerMesh(m);
    b = meshBounds(m);
    CHECK(b.min.x == -b.max.x);
    CHECK(b.min.y == -b.max.y);
    CHECK(b.min.z == -b.max.z);
    TriangleMesh far;
    far.vertices = {{9e8, 0, 0}};
    CHECK_FALSE(applyMeshUnits(far, MeshUnits::Inches).empty());
}

TEST_CASE("Orientation goldens: each top face lands on +Z", "[mesh]") {
    struct Case { TopFace f; Vec3 dir; };
    const Case cases[] = {{TopFace::PlusZ, {0, 0, 1}},  {TopFace::MinusZ, {0, 0, -1}}, {TopFace::PlusX, {1, 0, 0}},
                          {TopFace::MinusX, {-1, 0, 0}}, {TopFace::PlusY, {0, 1, 0}},  {TopFace::MinusY, {0, -1, 0}}};
    for (const Case& c : cases) {
        TriangleMesh m;
        m.vertices = {c.dir};
        REQUIRE(orientMesh(m, {c.f, 0}).empty());
        CHECK(m.vertices[0] == Vec3{0, 0, 1});
    }
}

TEST_CASE("Orientation goldens: proper rotations keep winding and Z turns", "[mesh]") {
    // Signed volume of the tetra stays the same under every orientation (no mirroring).
    auto volume = [](const TriangleMesh& m) {
        double v = 0;
        for (const auto& t : m.triangles) {
            const Vec3 &a = m.vertices[t[0]], &b = m.vertices[t[1]], &c = m.vertices[t[2]];
            v += a.x * (b.y * c.z - b.z * c.y) - a.y * (b.x * c.z - b.z * c.x) + a.z * (b.x * c.y - b.y * c.x);
        }
        return v / 6.0;
    };
    const double v0 = volume(tetra());
    for (int f = 0; f < 6; ++f)
        for (int q = 0; q < 4; ++q) {
            TriangleMesh m = tetra();
            REQUIRE(orientMesh(m, {static_cast<TopFace>(f), q}).empty());
            CHECK(volume(m) == v0);
        }
    TriangleMesh m;
    m.vertices = {{1, 2, 3}};
    orientMesh(m, {TopFace::PlusZ, 1});
    CHECK(m.vertices[0] == Vec3{-2, 1, 3});
    m.vertices = {{1, 2, 3}};
    orientMesh(m, {TopFace::PlusZ, 2});
    CHECK(m.vertices[0] == Vec3{-1, -2, 3});
    m.vertices = {{1, 2, 3}};
    orientMesh(m, {TopFace::PlusZ, 3});
    CHECK(m.vertices[0] == Vec3{2, -1, 3});
    CHECK_FALSE(orientMesh(m, {TopFace::PlusZ, 4}).empty());
    CHECK_FALSE(orientMesh(m, {TopFace::PlusZ, -1}).empty());
}

TEST_CASE("Arbitrary rotation", "[mesh]") {
    using Catch::Matchers::WithinAbs;
    const double pi = std::acos(-1.0);
    TriangleMesh m;
    m.vertices = {{1, 0, 0}};
    REQUIRE(rotateMesh(m, {0, 0, 2}, pi / 2).empty());
    CHECK_THAT(m.vertices[0].x, WithinAbs(0, 1e-12));
    CHECK_THAT(m.vertices[0].y, WithinAbs(1, 1e-12));
    m.vertices = {{0, 1, 0}};
    REQUIRE(rotateMesh(m, {1, 0, 0}, pi / 2).empty());
    CHECK_THAT(m.vertices[0].z, WithinAbs(1, 1e-12));
    CHECK_FALSE(rotateMesh(m, {0, 0, 0}, 1).empty());
    CHECK_FALSE(rotateMesh(m, {0, 0, 1}, std::nan("")).empty());
}

TEST_CASE("Scaling to size with and without lock ratio", "[mesh]") {
    TriangleMesh m = tetra();  // size 2 x 3 x 4
    REQUIRE(scaleMeshToSize(m, {0, 0, 8}, true, 2).empty());
    CHECK(meshBounds(m).size() == Vec3{4, 6, 8});
    m = tetra();
    REQUIRE(scaleMeshToSize(m, {1, 6, 2}, false, 0).empty());
    CHECK(meshBounds(m).size() == Vec3{1, 6, 2});
    m = tetra();
    CHECK_FALSE(scaleMeshToSize(m, {0, 0, 0}, true, 0).empty());
    CHECK_FALSE(scaleMeshToSize(m, {1, 1, 1}, true, 3).empty());
    CHECK_FALSE(scaleMeshToSize(m, {1e12, 1, 1}, true, 0).empty());
    CHECK(meshBounds(m).size() == Vec3{2, 3, 4});  // unchanged on error
    TriangleMesh flat;
    flat.vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    CHECK_FALSE(scaleMeshToSize(flat, {1, 1, 1}, false, 0).empty());
    CHECK(scaleMeshToSize(flat, {2, 0, 0}, true, 0).empty());
    TriangleMesh none;
    CHECK_FALSE(scaleMeshToSize(none, {1, 1, 1}, true, 0).empty());
}
