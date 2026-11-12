#include "carvetto/io/bitmap_trace.hpp"

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <numbers>
#include <string>

#include <algorithm>
#include <atomic>

#include "carvetto/geometry/cancel.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/io/raster.hpp"

using namespace carvetto;
using namespace carvetto::io;

namespace {

RasterImage blank(int w, int h, std::uint8_t v = 255) {
    RasterImage im;
    im.width = w;
    im.height = h;
    im.rgb.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 3, v);
    return im;
}
void setPx(RasterImage& im, int x, int y, std::uint8_t r, std::uint8_t g, std::uint8_t b) {
    const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(im.width) + static_cast<std::size_t>(x)) * 3;
    im.rgb[i] = r;
    im.rgb[i + 1] = g;
    im.rgb[i + 2] = b;
}
void fillRect(RasterImage& im, int x0, int y0, int x1, int y1, std::uint8_t v = 0) {
    for (int y = y0; y < y1; ++y) for (int x = x0; x < x1; ++x) setPx(im, x, y, v, v, v);
}
RasterImage disc(int size, double r) {
    RasterImage im = blank(size, size);
    const double c = size / 2.0;
    for (int y = 0; y < size; ++y)
        for (int x = 0; x < size; ++x)
            if (std::hypot(x + 0.5 - c, y + 0.5 - c) < r) setPx(im, x, y, 0, 0, 0);
    return im;
}
double netArea(const TraceLayer& l) {
    double a = 0;
    for (const auto& c : l.contours) a += geometry::signedArea(c);
    return a;
}
std::size_t spans(const TraceLayer& l) {
    std::size_t n = 0;
    for (const auto& c : l.contours) n += c.spans.size();
    return n;
}
std::size_t darkCount(const RasterImage& im) {
    std::size_t n = 0;
    for (std::size_t i = 0; i < im.rgb.size(); i += 3) n += im.rgb[i] < 128;
    return n;
}
std::vector<TraceLayer> trace(const RasterImage& im, TraceParams p = {}) {
    auto r = traceBitmap(im, p);
    INFO((r.ok() ? std::string() : r.error()));
    REQUIRE(r.ok());
    return r.value();
}

}  // namespace

TEST_CASE("disc traces to one smooth contour with the right area", "[trace]") {
    const RasterImage im = disc(100, 30.0);
    const auto layers = trace(im);
    REQUIRE(layers.size() == 1);
    REQUIRE(layers[0].contours.size() == 1);
    const double expect = static_cast<double>(darkCount(im));
    CHECK(std::fabs(netArea(layers[0]) - expect) / expect < 0.02);
    CHECK(std::fabs(netArea(layers[0]) - std::numbers::pi * 900.0) / (std::numbers::pi * 900.0) < 0.02);
    CHECK(layers[0].contours[0].closed);
    CHECK(layers[0].contours[0].isContinuous(geometry::Tolerance::defaults()));
    CHECK(spans(layers[0]) <= 24);  // fitted curves, not a 190-edge staircase
    CHECK(geometry::signedArea(layers[0].contours[0]) > 0.0);
}

TEST_CASE("scale and origin place the result in y-up millimetres", "[trace]") {
    RasterImage im = blank(20, 10);
    fillRect(im, 2, 1, 6, 4);  // x 2..6, rows 1..4 from the top
    TraceParams p;
    p.mmPerPixel = 0.5;
    p.origin = {10.0, 20.0};
    const auto layers = trace(im, p);
    REQUIRE(layers.size() == 1);
    const auto b = layers[0].contours[0].bounds();
    CHECK(b.min.x == 10.0 + 1.0);
    CHECK(b.max.x == 10.0 + 3.0);
    CHECK(b.min.y == 20.0 + (10 - 4) * 0.5);
    CHECK(b.max.y == 20.0 + (10 - 1) * 0.5);
    CHECK(netArea(layers[0]) == 4 * 3 * 0.25);
}

TEST_CASE("letters keep their corners and counters", "[trace]") {
    RasterImage im = blank(80, 60);
    // "E": stem and three arms; "O": ring with a hole.
    fillRect(im, 5, 5, 15, 55);
    fillRect(im, 15, 5, 35, 15);
    fillRect(im, 15, 25, 30, 35);
    fillRect(im, 15, 45, 35, 55);
    fillRect(im, 45, 5, 75, 55);
    fillRect(im, 55, 15, 65, 45, 255);
    const auto layers = trace(im);
    REQUIRE(layers.size() == 1);
    CHECK(layers[0].contours.size() == 3);  // E outline, O outer, O counter
    const double expect = static_cast<double>(darkCount(im));
    CHECK(std::fabs(netArea(layers[0]) - expect) / expect < 0.02);
    int holes = 0;
    for (const auto& c : layers[0].contours) holes += geometry::signedArea(c) < 0;
    CHECK(holes == 1);
    // Straight-edged letters need only their corners: E has 12 edges, O 4 + 4.
    CHECK(spans(layers[0]) <= 24);
}

TEST_CASE("invert and threshold choose the foreground", "[trace]") {
    RasterImage im = blank(20, 20, 200);
    fillRect(im, 5, 5, 15, 15, 100);
    TraceParams p;
    p.threshold = 150;
    CHECK(netArea(trace(im, p)[0]) == 100.0);
    p.threshold = 90;
    CHECK(trace(im, p).empty());
    p.threshold = 150;
    p.invert = true;
    CHECK(netArea(trace(im, p)[0]) == 400.0 - 100.0);
}

TEST_CASE("noise filter drops specks and fills pinholes", "[trace]") {
    RasterImage im = blank(40, 40);
    fillRect(im, 10, 10, 30, 30);
    setPx(im, 2, 2, 0, 0, 0);       // speck
    setPx(im, 20, 20, 255, 255, 255);  // pinhole
    TraceParams p;
    p.noiseFilter = 0;
    CHECK(trace(im, p)[0].contours.size() == 3);
    p.noiseFilter = 4;
    const auto l = trace(im, p);
    REQUIRE(l[0].contours.size() == 1);
    CHECK(netArea(l[0]) == 400.0);
}

TEST_CASE("diagonal pixels stay separate", "[trace]") {
    RasterImage im = blank(4, 4);
    setPx(im, 1, 1, 0, 0, 0);
    setPx(im, 2, 2, 0, 0, 0);
    TraceParams p;
    p.noiseFilter = 0;
    const auto l = trace(im, p);
    CHECK(l[0].contours.size() == 2);
    CHECK(netArea(l[0]) == 2.0);
}

TEST_CASE("colour mode gives one layer per colour", "[trace]") {
    RasterImage im = blank(60, 30);
    fillRect(im, 0, 0, 30, 30, 255);
    for (int y = 0; y < 30; ++y) for (int x = 0; x < 20; ++x) setPx(im, x, y, 220, 30, 30);       // red
    for (int y = 0; y < 30; ++y) for (int x = 40; x < 60; ++x) setPx(im, x, y, 20, 20, 200);      // blue
    TraceParams p;
    p.mode = TraceMode::Colour;
    p.colourCount = 3;
    const auto l = trace(im, p);
    REQUIRE(l.size() == 3);
    std::size_t total = 0;
    double area = 0;
    for (const auto& layer : l) { total += layer.pixelCount; area += netArea(layer); }
    CHECK(total == 1800);
    CHECK(area == 1800.0);
    CHECK(l[0].pixelCount >= l[1].pixelCount);
    bool red = false, blue = false;
    for (const auto& layer : l) {
        red |= layer.r > 200 && layer.g < 60;
        blue |= layer.b > 180 && layer.r < 60;
        CHECK(layer.pixelCount >= 600);
    }
    CHECK(red);
    CHECK(blue);
    // The same input gives the same output.
    const auto again = trace(im, p);
    CHECK(again.size() == l.size());
    CHECK(again[0].contours == l[0].contours);
}

TEST_CASE("limits and invalid parameters are refused", "[trace]") {
    RasterImage big;
    big.width = 100000;
    big.height = 100000;
    CHECK_FALSE(traceBitmap(big, {}).ok());
    CHECK_FALSE(traceBitmap(blank(0, 0), {}).ok());
    RasterImage lying = blank(4, 4);
    lying.rgb.pop_back();
    CHECK_FALSE(traceBitmap(lying, {}).ok());
    TraceParams p;
    p.mmPerPixel = 0.0;
    CHECK_FALSE(traceBitmap(blank(4, 4), p).ok());
    p = {};
    p.cornerFit = 1.5;
    CHECK_FALSE(traceBitmap(blank(4, 4), p).ok());
    p = {};
    p.mode = TraceMode::Colour;
    p.colourCount = 17;
    CHECK_FALSE(traceBitmap(blank(4, 4), p).ok());
    p.colourCount = 1;
    CHECK_FALSE(traceBitmap(blank(4, 4), p).ok());
    p = {};
    p.noiseFilter = -1;
    CHECK_FALSE(traceBitmap(blank(4, 4), p).ok());
}

TEST_CASE("contour count is capped on a hostile checkerboard", "[trace]") {
    RasterImage im = blank(600, 600);
    for (int y = 0; y < 600; ++y) for (int x = 0; x < 600; ++x) if ((x + y) % 2 == 0) setPx(im, x, y, 0, 0, 0);
    TraceParams p;
    p.noiseFilter = 0;
    const auto r = traceBitmap(im, p);  // 180000 single-pixel contours
    REQUIRE_FALSE(r.ok());
    CHECK(r.error() == "too many contours");
}

TEST_CASE("cornerFit trades roundness for corners", "[trace]") {
    // A 45-degree diagonal edge ending in square corners: tight keeps more spans than loose.
    RasterImage im = blank(60, 60);
    for (int y = 10; y < 50; ++y) for (int x = 10; x < 10 + (y - 10) + 1; ++x) setPx(im, x, y, 0, 0, 0);
    TraceParams loose, tight;
    loose.cornerFit = 0.0;
    tight.cornerFit = 1.0;
    const auto a = trace(im, loose);
    const auto b = trace(im, tight);
    const double expect = static_cast<double>(darkCount(im));
    CHECK(std::fabs(netArea(a[0]) - expect) / expect < 0.03);
    CHECK(std::fabs(netArea(b[0]) - expect) / expect < 0.03);
    CHECK(a[0].contours != b[0].contours);
}

TEST_CASE("PNM decoding", "[trace][pnm]") {
    SECTION("P1 ASCII bitmap, 1 is black") {
        auto r = decodePnm("P1\n# c\n3 2\n1 0 1\n0 1 0\n");
        REQUIRE(r.ok());
        CHECK(r.value().width == 3);
        CHECK(r.value().height == 2);
        CHECK(r.value().rgb[0] == 0);
        CHECK(r.value().rgb[3] == 255);
        CHECK(r.value().rgb[(1 * 3 + 1) * 3] == 0);
    }
    SECTION("P4 packed bitmap with row padding") {
        const std::string d = std::string("P4\n9 2\n") + std::string("\x80\x80\x00\x00", 4);
        auto r = decodePnm(d);
        REQUIRE(r.ok());
        CHECK(r.value().rgb[0] == 0);
        CHECK(r.value().rgb[1 * 3] == 255);
        CHECK(r.value().rgb[8 * 3] == 0);
        CHECK(r.value().rgb[9 * 3] == 255);
    }
    SECTION("P2 and P5 gray scale to 8 bits") {
        auto a = decodePnm("P2 2 1 15 0 15");
        REQUIRE(a.ok());
        CHECK(a.value().rgb[0] == 0);
        CHECK(a.value().rgb[3] == 255);
        auto b = decodePnm(std::string("P5\n2 1\n255\n") + std::string("\x10\xf0", 2));
        REQUIRE(b.ok());
        CHECK(b.value().rgb[0] == 0x10);
        CHECK(b.value().rgb[5] == 0xf0);
    }
    SECTION("P3 and P6 colour, 16-bit samples") {
        auto a = decodePnm("P3 1 1 255 1 2 3");
        REQUIRE(a.ok());
        CHECK(a.value().rgb == std::vector<std::uint8_t>{1, 2, 3});
        auto b = decodePnm(std::string("P6 1 1 65535\n") + std::string("\xff\xff\x00\x00\x80\x00", 6));
        REQUIRE(b.ok());
        CHECK(b.value().rgb[0] == 255);
        CHECK(b.value().rgb[1] == 0);
        CHECK(b.value().rgb[2] == 128);
    }
    SECTION("hostile input") {
        CHECK_FALSE(decodePnm("").ok());
        CHECK_FALSE(decodePnm("P7\n1 1\n").ok());
        CHECK_FALSE(decodePnm("P5\n0 4\n255\n").ok());
        CHECK_FALSE(decodePnm("P5\n99999 99999\n255\n").ok());   // no allocation, just refused
        CHECK_FALSE(decodePnm("P5\n10000 10000\n255\nxx").ok());  // within caps but truncated
        CHECK_FALSE(decodePnm("P6\n2 2\n255\n\x01\x02\x03").ok());
        CHECK_FALSE(decodePnm("P2 2 1 255 1").ok());
        CHECK_FALSE(decodePnm("P2 1 1 0 0").ok());
        CHECK_FALSE(decodePnm("P1 2 1 1 2").ok());
        auto sat = decodePnm("P2 1 1 255 99999999999999999999");  // saturates, then clamps to maxval
        REQUIRE(sat.ok());
        CHECK(sat.value().rgb[0] == 255);
    }
    SECTION("decoded bitmap traces") {
        auto r = decodePnm("P1 4 4 0 0 0 0 0 1 1 0 0 1 1 0 0 0 0 0");
        REQUIRE(r.ok());
        TraceParams p;
        p.noiseFilter = 0;
        CHECK(netArea(trace(r.value(), p)[0]) == 4.0);
    }
}

TEST_CASE("trace: boundary-edge budget is exact and refuses before fitting (T-0260)", "[trace][budget]") {
    RasterImage im = blank(20, 20);
    fillRect(im, 3, 3, 13, 9);  // 10x6 rectangle: 32 boundary edges
    TraceParams p;
    p.noiseFilter = 0;
    const auto reference = traceBitmap(im, p);
    REQUIRE(reference.ok());
    p.maxBoundaryEdges = 32;
    const auto exact = traceBitmap(im, p);
    REQUIRE(exact.ok());
    REQUIRE(exact.value().size() == reference.value().size());
    CHECK(exact.value()[0].contours[0].spans.size() == reference.value()[0].contours[0].spans.size());
    CHECK(netArea(exact.value()[0]) == netArea(reference.value()[0]));
    p.maxBoundaryEdges = 31;
    const auto over = traceBitmap(im, p);
    REQUIRE_FALSE(over.ok());
    CHECK(over.error().find("too complex") != std::string::npos);
}

TEST_CASE("trace: the default budget refuses a 4000x4000 serpentine quickly (T-0260)", "[trace][budget]") {
    // Alternating one-pixel walls: one huge boundary, valid input, ~16 M edges.
    RasterImage im = blank(4000, 4000);
    for (int y = 0; y < 4000; y += 2) fillRect(im, 0, y, 3999, y + 1);
    for (int y = 1; y < 4000; y += 4) fillRect(im, 3998, y, 3999, std::min(y + 2, 4000));
    for (int y = 3; y < 4000; y += 4) fillRect(im, 0, y, 1, std::min(y + 2, 4000));
    TraceParams p;
    const auto r = traceBitmap(im, p);
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().find("too complex") != std::string::npos);
}

TEST_CASE("trace: cancel flag stops the trace (T-0260)", "[trace][cancel]") {
    const RasterImage im = disc(200, 80);
    TraceParams p;
    const auto reference = traceBitmap(im, p);
    REQUIRE(reference.ok());
    std::atomic<bool> flag{true};
    {
        geometry::CancelScope scope(&flag);
        const auto r = traceBitmap(im, p);
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().find("cancelled") != std::string::npos);
        p.mode = TraceMode::Colour;
        CHECK_FALSE(traceBitmap(im, p).ok());
    }
    // A scope whose flag never reads true changes nothing.
    std::atomic<bool> quiet{false};
    geometry::CancelScope scope(&quiet);
    p.mode = TraceMode::BlackWhite;
    const auto again = traceBitmap(im, p);
    REQUIRE(again.ok());
    REQUIRE(again.value().size() == reference.value().size());
    CHECK(again.value()[0].contours.size() == reference.value()[0].contours.size());
    CHECK(spans(again.value()[0]) == spans(reference.value()[0]));
    CHECK(netArea(again.value()[0]) == netArea(reference.value()[0]));
}
