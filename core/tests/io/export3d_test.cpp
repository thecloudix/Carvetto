#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <map>
#include <string>
#include <utility>

#include "carvetto/io/export3d.hpp"
#include "carvetto/io/raster.hpp"

using namespace carvetto::io;
using carvetto::relief::Grid;
using carvetto::relief::HeightField;
using carvetto::relief::kTransparent;

namespace {

HeightField makeField(std::size_t w, std::size_t h, double cell, double ox, double oy,
                      std::vector<float> data) {
    HeightField f;
    REQUIRE(HeightField::fromData(Grid{w, h, cell, {ox, oy}}, std::move(data), f) ==
            carvetto::relief::Status::Ok);
    return f;
}

// Every undirected edge must be used twice, once in each direction.
struct Edges {
    bool closed = true;
    std::size_t count = 0;
};
Edges edgeCheck(const TriangleMesh& m) {
    std::map<std::pair<std::uint32_t, std::uint32_t>, int> dir;
    for (const auto& t : m.triangles)
        for (int k = 0; k < 3; ++k) ++dir[{t[static_cast<std::size_t>(k)], t[static_cast<std::size_t>((k + 1) % 3)]}];
    Edges e;
    for (const auto& [ed, n] : dir) {
        auto rev = dir.find({ed.second, ed.first});
        if (n != 1 || rev == dir.end() || rev->second != 1) e.closed = false;
    }
    e.count = dir.size() / 2;
    return e;
}

double signedVolume(const TriangleMesh& m) {
    double v = 0;
    for (const auto& t : m.triangles) {
        const Vec3 &a = m.vertices[t[0]], &b = m.vertices[t[1]], &c = m.vertices[t[2]];
        v += (a.x * (b.y * c.z - b.z * c.y) - a.y * (b.x * c.z - b.z * c.x) + a.z * (b.x * c.y - b.y * c.x)) / 6;
    }
    return v;
}

std::uint32_t be32(const std::string& s, std::size_t p) {
    return (std::uint32_t(std::uint8_t(s[p])) << 24) | (std::uint32_t(std::uint8_t(s[p + 1])) << 16) |
           (std::uint32_t(std::uint8_t(s[p + 2])) << 8) | std::uint32_t(std::uint8_t(s[p + 3]));
}

std::uint32_t refCrc(const std::string& s) {
    std::uint32_t c = 0xFFFFFFFFu;
    for (char sc : s) {
        c ^= static_cast<unsigned char>(sc);
        for (int b = 0; b < 8; ++b) c = (c & 1u) ? (c >> 1) ^ 0xEDB88320u : (c >> 1);
    }
    return ~c;
}

struct Png {
    std::size_t w = 0, h = 0;
    std::vector<std::uint16_t> px;
};
// Minimal decoder for our own output: stored blocks only, checks CRCs, Adler and framing.
Png decodePng(const std::string& s) {
    Png p;
    REQUIRE(s.compare(0, 8, "\x89PNG\r\n\x1a\n", 8) == 0);
    std::size_t pos = 8;
    std::string idat;
    bool end = false;
    int chunks = 0;
    while (!end) {
        const std::uint32_t len = be32(s, pos);
        const std::string type = s.substr(pos + 4, 4);
        const std::string body = s.substr(pos + 8, len);
        REQUIRE(be32(s, pos + 8 + len) == refCrc(type + body));
        if (chunks++ == 0) {
            REQUIRE(type == "IHDR");
            p.w = be32(body, 0);
            p.h = be32(body, 4);
            REQUIRE(body.substr(8) == std::string("\x10\0\0\0\0", 5));
        } else if (type == "IDAT") {
            idat += body;
        } else {
            REQUIRE(type == "IEND");
            REQUIRE(len == 0);
            end = true;
        }
        pos += 12 + len;
    }
    REQUIRE(pos == s.size());
    REQUIRE(idat.substr(0, 2) == "\x78\x01");
    std::size_t q = 2;
    std::string raw;
    for (bool last = false; !last;) {
        last = idat[q] == 1;
        REQUIRE((idat[q] == 0 || idat[q] == 1));
        const std::size_t n = std::uint8_t(idat[q + 1]) | (std::size_t(std::uint8_t(idat[q + 2])) << 8);
        const std::size_t nn = std::uint8_t(idat[q + 3]) | (std::size_t(std::uint8_t(idat[q + 4])) << 8);
        REQUIRE((n ^ 0xFFFF) == nn);
        raw.append(idat, q + 5, n);
        q += 5 + n;
    }
    std::uint32_t a = 1, b = 0;
    for (char rc : raw) {
        const auto c = static_cast<unsigned char>(rc);
        a = (a + c) % 65521u;
        b = (b + a) % 65521u;
    }
    REQUIRE(be32(idat, q) == ((b << 16) | a));
    REQUIRE(q + 4 == idat.size());
    REQUIRE(raw.size() == (p.w * 2 + 1) * p.h);
    for (std::size_t r = 0; r < p.h; ++r) {
        REQUIRE(raw[r * (p.w * 2 + 1)] == 0);
        for (std::size_t i = 0; i < p.w; ++i) {
            const std::size_t o = r * (p.w * 2 + 1) + 1 + i * 2;
            p.px.push_back(static_cast<std::uint16_t>((std::uint8_t(raw[o]) << 8) | std::uint8_t(raw[o + 1])));
        }
    }
    return p;
}

HeightField ramp(std::size_t w, std::size_t h) {
    std::vector<float> d;
    for (std::size_t j = 0; j < h; ++j)
        for (std::size_t i = 0; i < w; ++i) d.push_back(static_cast<float>(1.5 * static_cast<double>(i) + 0.25 * static_cast<double>(j * j) + 2));
    return makeField(w, h, 0.5, -1.0, 3.0, d);
}

}  // namespace

TEST_CASE("surface mesh geometry, counts and winding", "[export3d]") {
    auto f = ramp(4, 3);
    auto r = buildSurfaceMesh(f);
    REQUIRE(r.ok());
    const auto& m = r.value();
    CHECK(m.vertices.size() == 12);
    CHECK(m.triangles.size() == 12);  // 3x2 squares
    const Vec3 v = m.vertices[0];
    CHECK(v == Vec3{-1.0, 3.0, 2.0});
    for (const auto& t : m.triangles) {
        const Vec3 &a = m.vertices[t[0]], &b = m.vertices[t[1]], &c = m.vertices[t[2]];
        const double nz = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        CHECK(nz > 0);  // counter-clockwise from +Z
        // vertex heights equal the field
        for (const Vec3* p : {&a, &b, &c}) {
            const auto i = static_cast<std::size_t>(std::lround((p->x + 1.0) / 0.5));
            const auto j = static_cast<std::size_t>(std::lround((p->y - 3.0) / 0.5));
            CHECK(p->z == static_cast<double>(f.at(i, j)));
        }
    }
    // Open surface: the area projected on XY equals the field footprint.
    double area = 0;
    for (const auto& t : m.triangles) {
        const Vec3 &a = m.vertices[t[0]], &b = m.vertices[t[1]], &c = m.vertices[t[2]];
        area += ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)) / 2;
    }
    CHECK(area == 3 * 2 * 0.25);
    CHECK_FALSE(edgeCheck(m).closed);
}

TEST_CASE("watertight option yields a closed manifold", "[export3d]") {
    auto f = ramp(5, 4);
    SurfaceMeshOptions o;
    o.back = BackFace::Flat;
    o.backZ = -1.0;
    auto r = buildSurfaceMesh(f, o);
    REQUIRE(r.ok());
    const auto& m = r.value();
    // squares 4x3=12: top 24 + back 24 + walls 2*(4+3)*2 = 28
    CHECK(m.triangles.size() == 24 + 24 + 28);
    const Edges e = edgeCheck(m);
    CHECK(e.closed);
    // Euler: V - E + F = 2 for a closed genus-0 surface.
    CHECK(static_cast<long long>(m.vertices.size()) - static_cast<long long>(e.count) +
              static_cast<long long>(m.triangles.size()) == 2);
    CHECK(m.vertices.size() == 40);
    // Outward orientation: positive volume, equal to column integral of (z - backZ) over cell area.
    double expect = 0;
    for (std::size_t j = 0; j + 1 < 4; ++j)
        for (std::size_t i = 0; i + 1 < 5; ++i) {
            double s = 0;
            for (auto [di, dj] : {std::pair{0, 0}, {1, 0}, {1, 1}, {0, 1}})
                s += static_cast<double>(f.at(i + std::size_t(di), j + std::size_t(dj))) + 1.0;
            expect += s / 4 * 0.25;
        }
    CHECK(signedVolume(m) == Catch::Approx(expect).epsilon(1e-12));
    // Back vertices lie at backZ, walls vertical (every bottom vertex has a top partner).
    std::size_t atBack = 0;
    for (const auto& v : m.vertices) atBack += v.z == -1.0;
    CHECK(atBack == 20);
}

TEST_CASE("watertight with holes closes around them", "[export3d]") {
    std::vector<float> d(25, 1.0f);
    d[2 * 5 + 2] = kTransparent;  // hole in the middle
    d[0] = kTransparent;          // missing corner
    auto f = makeField(5, 5, 1.0, 0, 0, d);
    SurfaceMeshOptions o;
    o.back = BackFace::Flat;
    o.backZ = 0.0;
    auto r = buildSurfaceMesh(f, o);
    REQUIRE(r.ok());
    CHECK(edgeCheck(r.value()).closed);
    CHECK(signedVolume(r.value()) > 0);
    // 11 squares (16 - 4 around the hole - 1 corner): 44 top+back, 24 wall edges (16 outer + 8 hole)
    CHECK(r.value().triangles.size() == 44 + 48);
    // Vertex (1,1) is a pinch (squares (1,0) and (0,1) touch only there): split top and back.
    CHECK(r.value().vertices.size() == 2 * (23 + 1));
    // Open version has exactly 22 top triangles.
    auto ro = buildSurfaceMesh(f);
    REQUIRE(ro.ok());
    CHECK(ro.value().triangles.size() == 22);
}

TEST_CASE("surface mesh errors", "[export3d]") {
    CHECK_FALSE(buildSurfaceMesh(makeField(1, 4, 1, 0, 0, {1, 2, 3, 4})).ok());
    CHECK_FALSE(buildSurfaceMesh(makeField(2, 2, 1, 0, 0, {1, 2, 3, kTransparent})).ok());
    auto f = ramp(3, 3);
    SurfaceMeshOptions o;
    o.back = BackFace::Flat;
    o.backZ = 2.0;  // equals the lowest cell
    CHECK_FALSE(buildSurfaceMesh(f, o).ok());
    o.backZ = 1.999;
    CHECK(buildSurfaceMesh(f, o).ok());
    o.backZ = std::nan("");
    CHECK_FALSE(buildSurfaceMesh(f, o).ok());
    SurfaceMeshOptions cap;
    cap.maxTriangles = 7;  // 4 squares = 8 triangles
    CHECK_FALSE(buildSurfaceMesh(f, cap).ok());
    cap.maxTriangles = 8;
    CHECK(buildSurfaceMesh(f, cap).ok());
    cap.back = BackFace::Flat;
    cap.backZ = 0;
    cap.maxTriangles = 8 + 8 + 16 - 1;  // walls 8 edges * 2
    CHECK_FALSE(buildSurfaceMesh(f, cap).ok());
    cap.maxTriangles = 32;
    CHECK(buildSurfaceMesh(f, cap).ok());
}

TEST_CASE("STL round trip reproduces the surface", "[export3d]") {
    auto f = ramp(6, 5);
    auto mesh = buildSurfaceMesh(f);
    REQUIRE(mesh.ok());
    auto bytes = writeStlBinary(mesh.value());
    REQUIRE(bytes.ok());
    CHECK(bytes.value().data.size() == 84 + 50 * mesh.value().triangles.size());
    auto back = readStl(bytes.value().data);
    REQUIRE(back.ok());
    CHECK(back.value().triangles.size() == mesh.value().triangles.size());
    CHECK(back.value().vertices.size() == mesh.value().vertices.size());
    for (std::size_t k = 0; k < mesh.value().triangles.size(); ++k)
        for (std::size_t c = 0; c < 3; ++c) {
            const Vec3& a = mesh.value().vertices[mesh.value().triangles[k][c]];
            const Vec3& b = back.value().vertices[back.value().triangles[k][c]];
            CHECK(static_cast<float>(a.x) == static_cast<float>(b.x));
            CHECK(static_cast<float>(a.y) == static_cast<float>(b.y));
            CHECK(static_cast<float>(a.z) == static_cast<float>(b.z));
        }
    // Watertight survives the round trip.
    SurfaceMeshOptions o;
    o.back = BackFace::Flat;
    o.backZ = 0;
    auto wt = buildSurfaceMesh(f, o);
    auto wtb = readStl(writeStlBinary(wt.value()).value().data);
    REQUIRE(wtb.ok());
    CHECK(edgeCheck(wtb.value()).closed);
}

TEST_CASE("STL normals and header", "[export3d]") {
    TriangleMesh m;
    m.vertices = {{0, 0, 0}, {3, 0, 0}, {0, 2, 0}};
    m.triangles = {{{0, 1, 2}}};
    const std::string s = writeStlBinary(m).value().data;
    REQUIRE(s.size() == 134);
    auto f32 = [&](std::size_t p) {
        std::uint32_t u = std::uint8_t(s[p]) | (std::uint8_t(s[p + 1]) << 8) | (std::uint8_t(s[p + 2]) << 16) |
                          (std::uint32_t(std::uint8_t(s[p + 3])) << 24);
        float x;
        std::memcpy(&x, &u, 4);
        return x;
    };
    CHECK(s[80] == 1);
    CHECK(s[81] == 0);
    CHECK(f32(84) == 0.0f);
    CHECK(f32(88) == 0.0f);
    CHECK(f32(92) == 1.0f);
    CHECK(f32(96 + 12) == 3.0f);
    CHECK(f32(96 + 28) == 2.0f);
    CHECK(s[132] == 0);
    CHECK(s[133] == 0);
    CHECK(s.compare(0, 5, "solid") != 0);  // a binary header must not look like ASCII STL
    TriangleMesh bad = m;
    bad.triangles[0][2] = 3;
    CHECK_FALSE(writeStlBinary(bad).ok());
    CHECK_FALSE(writeObj(bad).ok());
}

TEST_CASE("OBJ round trip is exact", "[export3d]") {
    TriangleMesh m;
    m.vertices = {{0.1, -2.5, 1e-7}, {3, 0, 1.0 / 3.0}, {0, 2, 4}, {1, 1, 1}};
    m.triangles = {{{0, 1, 2}}, {{3, 2, 1}}};
    auto s = writeObj(m);
    REQUIRE(s.ok());
    auto b = readObj(s.value().data);
    REQUIRE(b.ok());
    CHECK(b.value().vertices == m.vertices);
    CHECK(b.value().triangles == m.triangles);
    CHECK(s.value().data.find("f 1 2 3\n") != std::string::npos);
    CHECK(s.value().data.find("f 4 3 2\n") != std::string::npos);
}

TEST_CASE("heightmap scaling, orientation and transparency", "[export3d]") {
    // 3x2: row j=0 (bottom) = 0, 5, 10 ; row j=1 (top) = 10, kTransparent, 2.5
    auto f = makeField(3, 2, 1, 0, 0, {0, 5, 10, 10, kTransparent, 2.5f});
    HeightmapOptions o;
    auto r = writeHeightmap(f, o);
    REQUIRE(r.ok());
    auto p = decodePng(r.value().data);
    REQUIRE(p.w == 3);
    REQUIRE(p.h == 2);
    // image row 0 is the largest y
    CHECK(p.px[0] == 65535);
    CHECK(p.px[1] == 0);  // transparent
    CHECK(p.px[2] == 16384);  // 0.25 * 65535 = 16383.75
    CHECK(p.px[3] == 0);
    CHECK(p.px[4] == 32768);  // 32767.5 rounds half away
    CHECK(p.px[5] == 65535);

    o.format = HeightmapFormat::Pgm16;
    auto g = writeHeightmap(f, o);
    REQUIRE(g.ok());
    const std::string& s = g.value().data;
    const std::string head = "P5\n3 2\n65535\n";
    REQUIRE(s.compare(0, head.size(), head) == 0);
    REQUIRE(s.size() == head.size() + 12);
    auto px = [&](std::size_t k) {
        return static_cast<std::uint16_t>((std::uint8_t(s[head.size() + 2 * k]) << 8) | std::uint8_t(s[head.size() + 2 * k + 1]));
    };
    for (std::size_t k = 0; k < 6; ++k) CHECK(px(k) == p.px[k]);
    // decodePnm reads it back (scaled to 8 bits)
    auto img = decodePnm(s);
    REQUIRE(img.ok());
    CHECK(img.value().rgb[0] == 255);
}

TEST_CASE("heightmap is linear and exact at the ends", "[export3d]") {
    std::vector<float> d;
    for (int k = 0; k <= 100; ++k) d.push_back(static_cast<float>(-3.0 + 0.07 * k));
    auto f = makeField(101, 1, 1, 0, 0, d);
    auto p = decodePng(writeHeightmap(f).value().data);
    CHECK(p.px[0] == 0);
    CHECK(p.px[100] == 65535);
    for (int k = 0; k <= 100; ++k) {
        const double expect = (static_cast<double>(d[static_cast<std::size_t>(k)]) - d[0]) / (static_cast<double>(d[100]) - d[0]) * 65535.0;
        CHECK(std::fabs(p.px[static_cast<std::size_t>(k)] - expect) <= 0.5 + 1e-9);
    }
    // Explicit range clamps and rescales.
    HeightmapOptions o;
    o.rangeMin = -3.0;
    o.rangeMax = 8.0;
    auto q = decodePng(writeHeightmap(f, o).value().data);
    CHECK(q.px[0] == 0);
    CHECK(q.px[100] == 41704);  // 7/11 * 65535 = 41704.09
    o.rangeMin = -1.0;
    o.rangeMax = 2.0;  // both ends clamp
    auto c = decodePng(writeHeightmap(f, o).value().data);
    CHECK(c.px[0] == 0);
    CHECK(c.px[100] == 65535);
}

TEST_CASE("heightmap edge cases", "[export3d]") {
    auto flat = makeField(2, 2, 1, 0, 0, {3, 3, 3, 3});
    auto p = decodePng(writeHeightmap(flat).value().data);
    for (auto v : p.px) CHECK(v == 0);
    CHECK_FALSE(writeHeightmap(makeField(2, 1, 1, 0, 0, {kTransparent, kTransparent})).ok());
    HeightmapOptions o;
    o.rangeMin = 5;
    o.rangeMax = 1;
    CHECK_FALSE(writeHeightmap(flat, o).ok());
    o = {};
    o.rangeMax = std::nan("");
    CHECK_FALSE(writeHeightmap(flat, o).ok());
    CHECK_FALSE(writeHeightmap(HeightField{}).ok());
    // Wide image: stored deflate needs several blocks (> 65535 bytes of scanlines).
    std::vector<float> d(40000);
    for (std::size_t k = 0; k < d.size(); ++k) d[k] = static_cast<float>(k % 977);
    auto big = decodePng(writeHeightmap(makeField(200, 200, 1, 0, 0, d)).value().data);
    CHECK(big.px.size() == 40000);
    CHECK(big.px[199 * 200 + 0] == 0);  // bottom-left is cell (0,0) = 0
    CHECK(big.px[0] == static_cast<std::uint16_t>(std::lround(static_cast<double>(d[199 * 200]) / 976.0 * 65535.0)));
}
