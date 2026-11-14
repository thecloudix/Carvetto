#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "carvetto/document/document.hpp"
#include "carvetto/relief/bitmap_relief.hpp"

using namespace carvetto::relief;
using Catch::Approx;

namespace {

std::vector<std::uint8_t> bytesOf(const std::string& s) { return std::vector<std::uint8_t>(s.begin(), s.end()); }

std::string pgm16(int w, int h, const std::vector<unsigned>& v, unsigned maxval = 65535) {
    std::string s = "P5\n# c\n" + std::to_string(w) + " " + std::to_string(h) + "\n" + std::to_string(maxval) + "\n";
    for (unsigned x : v) {
        if (maxval > 255) s.push_back(static_cast<char>(x >> 8));
        s.push_back(static_cast<char>(x & 255));
    }
    return s;
}

GrayImage16 img(int w, int h, std::vector<std::uint16_t> px) {
    GrayImage16 g;
    g.width = w;
    g.height = h;
    g.px = std::move(px);
    return g;
}

BitmapReliefParams params(double w = 10.0, double z = 5.0) {
    BitmapReliefParams p;
    p.widthMm = w;
    p.zHeightMm = z;
    return p;
}

}  // namespace

TEST_CASE("decodeGray16 keeps 16-bit precision", "[relief][bitmap]") {
    const std::vector<unsigned> v = {0, 1, 256, 40000, 65534, 65535};
    auto r = decodeGray16(bytesOf(pgm16(3, 2, v)));
    REQUIRE(r.ok());
    REQUIRE(r.value().width == 3);
    for (std::size_t i = 0; i < v.size(); ++i) CHECK(r.value().px[i] == v[i]);
}

TEST_CASE("decodeGray16 scales odd maxvals and reads ascii and colour", "[relief][bitmap]") {
    auto r = decodeGray16(bytesOf("P2 2 1 100\n0 50\n"));
    REQUIRE(r.ok());
    CHECK(r.value().px[0] == 0);
    CHECK(r.value().px[1] == 32768);  // 50*65535/100 rounded
    auto c = decodeGray16(bytesOf("P3 1 1 255\n255 255 255\n"));
    REQUIRE(c.ok());
    CHECK(c.value().px[0] == 65535);
    auto b = decodeGray16(bytesOf(pgm16(2, 1, {0, 255}, 255)));
    REQUIRE(b.ok());
    CHECK(b.value().px[1] == 65535);
    auto red = decodeGray16(bytesOf(std::string("P6 1 1 255\n") + std::string("\xff\x00\x00", 3)));
    REQUIRE(red.ok());
    CHECK(red.value().px[0] == 19595);  // 0.299 * 65535
}

TEST_CASE("decodeGray16 hostile input", "[relief][bitmap]") {
    CHECK_FALSE(decodeGray16(bytesOf("P5 4 4 65535\n\x01\x02")).ok());
    CHECK_FALSE(decodeGray16(bytesOf("P5 99999 99999 255\n")).ok());
    CHECK_FALSE(decodeGray16(bytesOf("P5 0 4 255\n")).ok());
    CHECK_FALSE(decodeGray16(bytesOf("P5 2 2 0\n")).ok());
    CHECK_FALSE(decodeGray16(bytesOf("P5 2 2 70000\n")).ok());
    CHECK_FALSE(decodeGray16(bytesOf("P2 2 2 10\n1 2 3\n")).ok());
    CHECK_FALSE(decodeGray16(bytesOf("P2 1 1 10\n11\n")).ok());
    CHECK_FALSE(decodeGray16(bytesOf("P5")).ok());
    CHECK_FALSE(decodeGray16(bytesOf("P5 x")).ok());
    CHECK_FALSE(decodeGray16({}).ok());
    CHECK_FALSE(decodeGray16(bytesOf("\x89PNG....")).ok());  // no decoder supplied
}

TEST_CASE("decodeGray16 falls back to the decoder for other formats", "[relief][bitmap]") {
    carvetto::document::BitmapDecoder dec = [](std::span<const std::uint8_t>) {
        carvetto::io::RasterImage r;
        r.width = 2;
        r.height = 1;
        r.rgb = {0, 0, 0, 255, 255, 255};
        return carvetto::geometry::ShapeResult<carvetto::io::RasterImage>(r);
    };
    auto g = decodeGray16(bytesOf("PNGDATA"), dec);
    REQUIRE(g.ok());
    CHECK(g.value().px[0] == 0);
    CHECK(g.value().px[1] == 65535);
    // PBM goes through the 8-bit path
    auto p = decodeGray16(bytesOf("P1 2 1\n0 1\n"));
    REQUIRE(p.ok());
    CHECK(p.value().px[0] == 65535);
    CHECK(p.value().px[1] == 0);
}

TEST_CASE("grayFromRaster rejects invalid images and is exact for gray", "[relief][bitmap]") {
    carvetto::io::RasterImage bad;
    CHECK_FALSE(grayFromRaster(bad).ok());
    carvetto::io::RasterImage r;
    r.width = 1;
    r.height = 1;
    r.rgb = {7, 7, 7};
    auto g = grayFromRaster(r);
    REQUIRE(g.ok());
    CHECK(g.value().px[0] == 7 * 257);
}

TEST_CASE("bitmap store and file loading", "[relief][bitmap]") {
    carvetto::document::Document doc;
    auto bytes = bytesOf(pgm16(2, 1, {1000, 60000}));
    auto asset = carvetto::document::makeBitmapAsset(bytes);
    carvetto::document::BitmapObject obj;
    obj.sourceRef = asset.key;
    CHECK_FALSE(loadBitmapObjectGray16(doc, obj).ok());
    REQUIRE(doc.insertBitmap(asset));
    auto r = loadBitmapObjectGray16(doc, obj);
    REQUIRE(r.ok());
    CHECK(r.value().px[1] == 60000);

    const std::string path = "bitmap_relief_test_tmp.pgm";
    {
        std::ofstream o(path, std::ios::binary);
        o.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    }
    auto f = loadFileGray16(path);
    REQUIRE(f.ok());
    CHECK(f.value().px[0] == 1000);
    std::remove(path.c_str());
    CHECK_FALSE(loadFileGray16(path).ok());
    {
        std::ofstream o(path, std::ios::binary);
    }
    CHECK_FALSE(loadFileGray16(path).ok());
    std::remove(path.c_str());
}

TEST_CASE("heightFieldFromBitmap maps brightness to height with full precision", "[relief][bitmap]") {
    HeightField f;
    // Two pixels differing by one 16-bit step must differ in height.
    REQUIRE(heightFieldFromBitmap(img(2, 1, {65534, 65535}), params(2.0, 1.0), f) == Status::Ok);
    CHECK(f.at(1, 0) == Approx(1.0).margin(1e-7));
    CHECK(f.at(0, 0) == Approx(65534.0 / 65535.0).margin(1e-7));
    CHECK(f.at(0, 0) < f.at(1, 0));
    CHECK(f.cellSize() == 1.0);
}

TEST_CASE("heightFieldFromBitmap geometry: top row is highest Y, corner placement", "[relief][bitmap]") {
    BitmapReliefParams p = params(4.0, 2.0);
    p.corner = {10.0, 20.0};
    HeightField f;
    REQUIRE(heightFieldFromBitmap(img(2, 2, {65535, 0, 0, 0}), p, f) == Status::Ok);
    CHECK(f.cellSize() == 2.0);
    CHECK(f.origin().x == 11.0);
    CHECK(f.origin().y == 21.0);
    CHECK(f.at(0, 1) == 2.0f);  // pixel row 0 -> top row
    CHECK(f.at(0, 0) == 0.0f);
    CHECK(f.at(1, 1) == 0.0f);
}

TEST_CASE("uniform image gives a flat field", "[relief][bitmap]") {
    HeightField f;
    for (bool a : {false, true}) {
        BitmapReliefParams p = params();
        p.autoScale = a;
        REQUIRE(heightFieldFromBitmap(img(3, 3, std::vector<std::uint16_t>(9, 30000)), p, f) == Status::Ok);
        float lo = 0;
        float hi = 0;
        REQUIRE(f.minMax(lo, hi));
        CHECK(lo == hi);
    }
    BitmapReliefParams p = params();
    REQUIRE(heightFieldFromBitmap(img(3, 3, std::vector<std::uint16_t>(9, 30000)), p, f) == Status::Ok);
    CHECK(f.at(1, 1) == Approx(5.0 * 30000.0 / 65535.0).margin(1e-6));
}

TEST_CASE("invert and auto-scale", "[relief][bitmap]") {
    HeightField f;
    BitmapReliefParams p = params(3.0, 4.0);
    p.invert = true;
    REQUIRE(heightFieldFromBitmap(img(3, 1, {0, 65535, 16384}), p, f) == Status::Ok);
    CHECK(f.at(0, 0) == Approx(4.0).margin(1e-6));
    CHECK(f.at(1, 0) == 0.0f);
    CHECK(f.at(2, 0) == Approx(4.0 * (65535 - 16384) / 65535.0).margin(1e-6));

    p = params(3.0, 4.0);
    p.autoScale = true;
    REQUIRE(heightFieldFromBitmap(img(3, 1, {10000, 30000, 20000}), p, f) == Status::Ok);
    CHECK(f.at(0, 0) == 0.0f);
    CHECK(f.at(1, 0) == Approx(4.0).margin(1e-6));
    CHECK(f.at(2, 0) == Approx(2.0).margin(1e-6));

    p.invert = true;  // brightest becomes lowest
    REQUIRE(heightFieldFromBitmap(img(3, 1, {10000, 30000, 20000}), p, f) == Status::Ok);
    CHECK(f.at(0, 0) == Approx(4.0).margin(1e-6));
    CHECK(f.at(1, 0) == 0.0f);
    CHECK(f.at(2, 0) == Approx(2.0).margin(1e-6));
}

TEST_CASE("degenerate bitmap relief input errors cleanly", "[relief][bitmap]") {
    HeightField f;
    const GrayImage16 ok = img(2, 2, {1, 2, 3, 4});
    CHECK(heightFieldFromBitmap(GrayImage16{}, params(), f) == Status::BadParameter);
    CHECK(heightFieldFromBitmap(img(2, 2, {1}), params(), f) == Status::BadParameter);
    CHECK(heightFieldFromBitmap(ok, params(0.0), f) == Status::BadParameter);
    CHECK(heightFieldFromBitmap(ok, params(-1.0), f) == Status::BadParameter);
    CHECK(heightFieldFromBitmap(ok, params(NAN), f) == Status::BadParameter);
    CHECK(heightFieldFromBitmap(ok, params(INFINITY), f) == Status::BadParameter);
    CHECK(heightFieldFromBitmap(ok, params(1.0, 0.0), f) == Status::BadParameter);
    CHECK(heightFieldFromBitmap(ok, params(1.0, NAN), f) == Status::BadParameter);
    BitmapReliefParams p = params();
    p.corner = {NAN, 0.0};
    CHECK(heightFieldFromBitmap(ok, p, f) == Status::BadParameter);
    CHECK(f.empty());
    Component c;
    CHECK(makeBitmapComponent(GrayImage16{}, params(), c) == Status::BadParameter);
}

TEST_CASE("bitmap component defaults and composes", "[relief][bitmap]") {
    BitmapReliefParams p = params(2.0, 1.0);
    p.name = "Pic";
    p.mode = CombineMode::MergeHigh;
    Component c;
    REQUIRE(makeBitmapComponent(img(2, 1, {0, 65535}), p, c) == Status::Ok);
    CHECK(c.name == "Pic");
    CHECK(c.mode == CombineMode::MergeHigh);
    CHECK(c.shapeHeightPercent == 100.0);
    CHECK(c.baseHeight == 0.0);
    Level l;
    l.components.push_back(c);
    HeightField m;
    REQUIRE(compose({l}, c.field.grid(), m) == Status::Ok);
    CHECK(m.at(1, 0) == 1.0f);
    // default mode is Add
    REQUIRE(makeBitmapComponent(img(2, 1, {0, 65535}), params(2.0, 1.0), c) == Status::Ok);
    CHECK(c.mode == CombineMode::Add);
}

TEST_CASE("zero plane covers the job rectangle", "[relief][bitmap]") {
    Component z;
    REQUIRE(makeZeroPlane(100.0, 50.0, 3.0, {5.0, -2.0}, z) == Status::Ok);
    CHECK(z.mode == CombineMode::Add);
    const Grid& g = z.field.grid();
    CHECK(g.width == 35);  // ceil(100/3)+1
    CHECK(g.height == 18);
    CHECK(g.origin.x == 5.0);
    CHECK(g.origin.y == -2.0);
    CHECK(z.field.sample(5.0, -2.0) == 0.0f);
    CHECK(z.field.sample(105.0, 48.0) == 0.0f);  // opposite job corner is inside the field
    CHECK(5.0 + (static_cast<double>(g.width) - 1.0) * g.cellSize >= 105.0);
    CHECK(-2.0 + (static_cast<double>(g.height) - 1.0) * g.cellSize >= 48.0);
    CHECK((static_cast<double>(g.width) - 2.0) * g.cellSize < 100.0);  // not oversized
    // Exact multiple: no extra cell beyond +1.
    REQUIRE(makeZeroPlane(10.0, 10.0, 5.0, {0, 0}, z) == Status::Ok);
    CHECK(z.field.width() == 3);
    CHECK(z.field.height() == 3);
    float lo = 1;
    float hi = 1;
    REQUIRE(z.field.minMax(lo, hi));
    CHECK(lo == 0.0f);
    CHECK(hi == 0.0f);
}

TEST_CASE("zero plane errors", "[relief][bitmap]") {
    Component z;
    CHECK(makeZeroPlane(0.0, 5.0, 1.0, {}, z) == Status::BadParameter);
    CHECK(makeZeroPlane(5.0, -1.0, 1.0, {}, z) == Status::BadParameter);
    CHECK(makeZeroPlane(5.0, 5.0, 0.0, {}, z) == Status::BadParameter);
    CHECK(makeZeroPlane(NAN, 5.0, 1.0, {}, z) == Status::BadParameter);
    CHECK(makeZeroPlane(5.0, 5.0, 1.0, {INFINITY, 0.0}, z) == Status::BadParameter);
    CHECK(makeZeroPlane(1e9, 1e9, 1.0, {}, z) == Status::TooLarge);
    CHECK(makeZeroPlane(1e30, 1.0, 1.0, {}, z) == Status::TooLarge);
    CHECK(makeZeroPlane(1.0, 1e30, 1.0, {}, z) == Status::TooLarge);
}

namespace {
HeightField mk(std::vector<float> d, std::size_t w = 3, std::size_t h = 1) {
    HeightField f;
    REQUIRE(HeightField::fromData(Grid{w, h, 1.0, {0, 0}}, std::move(d), f) == Status::Ok);
    return f;
}
}  // namespace

TEST_CASE("scaleModelZ percent", "[relief][bitmap]") {
    HeightField f = mk({1.0f, kTransparent, 4.0f});
    double k = 0;
    REQUIRE(scaleModelZ(f, {ScaleZ::Kind::Percent, 50.0, false}, &k) == Status::Ok);
    CHECK(k == 0.5);
    CHECK(f.at(0, 0) == 0.5f);
    CHECK(std::isnan(f.at(1, 0)));
    CHECK(f.at(2, 0) == 2.0f);
    REQUIRE(scaleModelZ(f, {ScaleZ::Kind::Percent, 200.0, false}) == Status::Ok);
    CHECK(f.at(2, 0) == 4.0f);
}

TEST_CASE("scaleModelZ exact height hits the target", "[relief][bitmap]") {
    HeightField f = mk({0.0f, 1.234567f, 3.3f});
    REQUIRE(scaleModelZ(f, {ScaleZ::Kind::ExactHeight, 7.3, false}) == Status::Ok);
    float lo = 0;
    float hi = 0;
    REQUIRE(f.minMax(lo, hi));
    CHECK(std::fabs(static_cast<double>(hi) - 7.3) < 1e-6);
    CHECK(lo == 0.0f);
    CHECK(f.at(1, 0) == Approx(1.234567 * 7.3 / 3.3).margin(1e-6));
}

TEST_CASE("scaleModelZ top-only leaves the bottom side; both sides scale it", "[relief][bitmap]") {
    HeightField top = mk({-2.0f, 0.0f, 4.0f});
    REQUIRE(scaleModelZ(top, {ScaleZ::Kind::Percent, 50.0, false}) == Status::Ok);
    CHECK(top.at(0, 0) == -2.0f);
    CHECK(top.at(2, 0) == 2.0f);

    HeightField both = mk({-2.0f, 0.0f, 4.0f});
    REQUIRE(scaleModelZ(both, {ScaleZ::Kind::Percent, 50.0, true}) == Status::Ok);
    CHECK(both.at(0, 0) == -1.0f);
    CHECK(both.at(2, 0) == 2.0f);

    // Exact: top-only -> max hits the target, bottom untouched.
    HeightField e1 = mk({-2.0f, 0.0f, 4.0f});
    REQUIRE(scaleModelZ(e1, {ScaleZ::Kind::ExactHeight, 8.0, false}) == Status::Ok);
    CHECK(e1.at(2, 0) == 8.0f);
    CHECK(e1.at(0, 0) == -2.0f);
    // Exact both sides: total extent (4 - -2 = 6) becomes 3.
    HeightField e2 = mk({-2.0f, 0.0f, 4.0f});
    REQUIRE(scaleModelZ(e2, {ScaleZ::Kind::ExactHeight, 3.0, true}) == Status::Ok);
    CHECK(e2.at(0, 0) == -1.0f);
    CHECK(e2.at(2, 0) == 2.0f);
    // All-negative field, both sides: extent measured from the zero plane (0 - -2 = 2).
    HeightField e3 = mk({-2.0f, -1.0f, -0.5f});
    REQUIRE(scaleModelZ(e3, {ScaleZ::Kind::ExactHeight, 4.0, true}) == Status::Ok);
    CHECK(e3.at(0, 0) == -4.0f);
}

TEST_CASE("scaleModelZ degenerate input errors and leaves the field alone", "[relief][bitmap]") {
    HeightField f = mk({0.0f, 0.0f, 0.0f});
    CHECK(scaleModelZ(f, {ScaleZ::Kind::ExactHeight, 5.0, false}) == Status::BadParameter);  // flat zero
    CHECK(scaleModelZ(f, {ScaleZ::Kind::Percent, 0.0, false}) == Status::BadParameter);
    CHECK(scaleModelZ(f, {ScaleZ::Kind::Percent, -5.0, false}) == Status::BadParameter);
    CHECK(scaleModelZ(f, {ScaleZ::Kind::Percent, NAN, false}) == Status::BadParameter);
    CHECK(scaleModelZ(f, {ScaleZ::Kind::ExactHeight, INFINITY, false}) == Status::BadParameter);
    CHECK(scaleModelZ(f, {ScaleZ::Kind::Percent, 1e300, false}) == Status::Ok);  // 0 stays 0
    HeightField all = mk({kTransparent, kTransparent, kTransparent});
    CHECK(scaleModelZ(all, {ScaleZ::Kind::ExactHeight, 5.0, false}) == Status::BadParameter);
    HeightField neg = mk({-1.0f, -2.0f, -3.0f});
    CHECK(scaleModelZ(neg, {ScaleZ::Kind::ExactHeight, 5.0, false}) == Status::BadParameter);  // no top side
    CHECK(neg.at(0, 0) == -1.0f);
    HeightField empty;
    CHECK(scaleModelZ(empty, {ScaleZ::Kind::Percent, 50.0, false}) == Status::BadParameter);
}
