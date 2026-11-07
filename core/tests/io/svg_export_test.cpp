#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <string>
#include <variant>
#include <vector>

#include "carvetto/geometry/span.hpp"
#include "carvetto/io/svg.hpp"
#include "svg_test_util.hpp"

using namespace carvetto::geometry;
using namespace carvetto::io;
using namespace svgtest;
using Catch::Approx;

namespace {

std::string text(const SvgResult<Bytes>& r) {
    REQUIRE(r.ok());
    return std::string(r.value().begin(), r.value().end());
}

Contour polyline(std::initializer_list<Vec2> pts, bool closed) {
    Contour c;
    std::vector<Vec2> v(pts);
    for (std::size_t i = 0; i + 1 < v.size(); ++i) {
        c.spans.emplace_back(LineSpan{v[i], v[i + 1]});
    }
    if (closed) {
        c.spans.emplace_back(LineSpan{v.back(), v.front()});
    }
    c.closed = closed;
    return c;
}

Contour arcContour(Vec2 centre, double r, double a0, double a1, ArcDirection dir) {
    Contour c;
    ArcSpan a;
    a.centre = centre;
    a.radius = r;
    a.startAngle = a0;
    a.endAngle = a1;
    a.direction = dir;
    c.spans.emplace_back(a);
    return c;
}

/// Exports then imports, returning the re-imported contours.
SvgDrawing roundTrip(const std::vector<SvgLayer>& layers, const std::vector<SvgContour>& contours,
                     const SvgExportOptions& o = {}) {
    const auto out = exportSvg(layers, contours, o);
    REQUIRE(out.ok());
    const auto back = importSvg(out.value());
    REQUIRE(back.ok());
    return back.value();
}

}  // namespace

TEST_CASE("svg export: document structure", "[svg][io]") {
    const std::vector<SvgLayer> layers{{"Outline"}, {"Pockets & <more>"}};
    const std::vector<SvgContour> contours{
        {0, polyline({{0, 0}, {10, 0}, {10, 5}}, true)},
        {1, Contour::circle({20, 20}, 3)},
    };
    const std::string s = text(exportSvg(layers, contours));
    CHECK(s.find("<?xml version=\"1.0\" encoding=\"UTF-8\"?>") == 0);
    CHECK(s.find("width=\"23mm\" height=\"23mm\" viewBox=\"0 0 23 23\"") != std::string::npos);
    CHECK(s.find("inkscape:label=\"Outline\"") != std::string::npos);
    CHECK(s.find("inkscape:label=\"Pockets &amp; &lt;more&gt;\"") != std::string::npos);
    CHECK(s.find("<path d=\"M0 23L10 23L10 18L0 23Z\"/>") != std::string::npos);
    CHECK(s.find("<circle cx=\"20\" cy=\"3\" r=\"3\"/>") != std::string::npos);
    CHECK(s.find("data-origin-mm=\"0 0\"") != std::string::npos);
    // Same input, same bytes.
    CHECK(s == text(exportSvg(layers, contours)));
}

TEST_CASE("svg export: argument errors", "[svg][io]") {
    const std::vector<SvgLayer> layers{{"a"}};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(exportSvg(layers, std::vector<SvgContour>{{3, polyline({{0, 0}, {1, 1}}, false)}}).ok());
    CHECK_FALSE(exportSvg(layers, std::vector<SvgContour>{{0, polyline({{0, 0}, {nan, 1}}, false)}}).ok());
    CHECK_FALSE(exportSvg(layers, std::vector<SvgContour>{{0, polyline({{0, 0}, {1e300, 1}}, false)}}).ok());
    SvgExportOptions o;
    o.margin = -1.0;
    CHECK_FALSE(exportSvg(layers, std::vector<SvgContour>{}, o).ok());
    const auto r = exportSvg(layers, std::vector<SvgContour>{{3, polyline({{0, 0}, {1, 1}}, false)}});
    CHECK(r.error().code == SvgError::Code::InvalidArgument);
}

TEST_CASE("svg export: empty and degenerate input", "[svg][io]") {
    const auto back = roundTrip({{"a"}, {"b"}}, {});
    REQUIRE(back.layers.size() == 2);
    CHECK(back.contours.empty());
    CHECK(back.layers[1].name == "b");
    // A single horizontal line has zero height: the page is padded to 1 mm.
    const auto line = roundTrip({{"a"}}, {{0, polyline({{5, 5}, {15, 5}}, false)}});
    CHECK(line.heightMm == Approx(1.0));
    REQUIRE(line.contours.size() == 1);
    CHECK((line.contours[0].contour.startPoint() - Vec2{5, 5}).length() < 1e-9);
}

TEST_CASE("svg export: round trip within tolerance", "[svg][io]") {
    Contour bez;
    bez.spans.emplace_back(BezierSpan{{-20, -5}, {-10, 15}, {0, -15}, {10, 5}});
    bez.spans.emplace_back(LineSpan{{10, 5}, {10, -20}});
    Contour mixed;
    mixed.spans.emplace_back(LineSpan{{30, 0}, {40, 0}});
    mixed.spans.emplace_back(ArcSpan{{40, 5}, 5.0, -kPi / 2, kPi / 2, ArcDirection::CCW});
    mixed.spans.emplace_back(LineSpan{{40, 10}, {30, 10}});
    mixed.spans.emplace_back(LineSpan{{30, 10}, {30, 0}});
    mixed.closed = true;

    const std::vector<SvgLayer> layers{{"Layer A"}, {"Layer B"}};
    // Export groups contours by layer, so the input is already grouped.
    const std::vector<SvgContour> in{
        {0, polyline({{-30, -30}, {-10, -30}, {-10, -10}, {-30, -10}}, true)},
        {0, polyline({{0, 0}, {5, 5}, {10, 0}}, false)},
        {0, arcContour({60, 0}, 10, 0.3, 2.0, ArcDirection::CCW)},                 // small CCW
        {0, arcContour({60, 30}, 10, 0.3, 2.0, ArcDirection::CW)},                 // large CW (sweep 4.58)
        {0, Contour::circle({90, 90}, 4)},                                          // full circle
        {1, bez},
        {1, mixed},
        {1, arcContour({60, 60}, 10, 1.0, 1.0 + 4.0, ArcDirection::CCW)},          // > 180 degrees
        {1, arcContour({60, 90}, 10, 1.0, 1.0 - 5.9, ArcDirection::CW)},           // > 270: split in two
        {1, arcContour({-60, 0}, 10, 3.0, 3.0 + 3.14159265358979, ArcDirection::CCW)},  // about a half turn
    };
    const SvgDrawing back = roundTrip(layers, in);
    REQUIRE(back.layers.size() == 2);
    CHECK(back.layers[0].name == "Layer A");
    CHECK(back.layers[1].name == "Layer B");
    REQUIRE(back.contours.size() == in.size());
    for (std::size_t i = 0; i < in.size(); ++i) {
        INFO("contour " << i);
        CHECK(back.contours[i].layer == in[i].layer);
        CHECK(back.contours[i].contour.closed == in[i].contour.closed);
        CHECK(outlineDistance(back.contours[i].contour, in[i].contour) < 1e-6);
    }
    // Span counts are kept for lines and Beziers; arcs stay exact arcs.
    CHECK(back.contours[0].contour.spans.size() == 4);
    CHECK(back.contours[5].contour.spans.size() == 2);
    CHECK(std::holds_alternative<BezierSpan>(back.contours[5].contour.spans[0]));
    REQUIRE(back.contours[6].contour.spans.size() == 4);
    const auto* arc = std::get_if<ArcSpan>(&back.contours[6].contour.spans[1]);
    REQUIRE(arc != nullptr);
    CHECK(arc->radius == Approx(5.0));
    CHECK(arc->direction == ArcDirection::CCW);
    const auto* cw = std::get_if<ArcSpan>(&back.contours[3].contour.spans[0]);
    REQUIRE(cw != nullptr);
    CHECK(cw->direction == ArcDirection::CW);
    CHECK(cw->radius == Approx(10.0));
    CHECK(back.contours[8].contour.spans.size() == 2);  // the 338 degree arc is written as two commands
    const auto* circle = std::get_if<ArcSpan>(&back.contours[4].contour.spans[0]);
    REQUIRE(circle != nullptr);
    CHECK(circle->isFullCircle());
    CHECK(circle->radius == Approx(4.0));
}

TEST_CASE("svg export: positions survive the round trip", "[svg][io]") {
    const std::vector<SvgLayer> layers{{"a"}};
    for (const double margin : {0.0, 2.5}) {
        SvgExportOptions o;
        o.margin = margin;
        const auto c = polyline({{-120.5, 33.25}, {-100.0, 80.0}, {-90.0, 33.25}}, true);
        const SvgDrawing back = roundTrip(layers, {{0, c}}, o);
        REQUIRE(back.contours.size() == 1);
        CHECK(outlineDistance(back.contours[0].contour, c) < 1e-9);
        CHECK(back.widthMm == Approx(30.5 + 2.0 * margin));
        CHECK(back.heightMm == Approx(46.75 + 2.0 * margin));
    }
}

TEST_CASE("svg export: duplicate layer names are made unique", "[svg][io]") {
    const auto back = roundTrip({{"x"}, {"x"}, {""}}, {{0, polyline({{0, 0}, {1, 1}}, false)},
                                                         {1, polyline({{0, 0}, {2, 2}}, false)},
                                                         {2, polyline({{0, 0}, {3, 3}}, false)}});
    REQUIRE(back.layers.size() == 3);
    CHECK(back.layers[0].name == "x");
    CHECK(back.layers[1].name == "x_2");
    CHECK(back.layers[2].name == "Layer");
    CHECK(back.contours[0].layer == 0);
    CHECK(back.contours[1].layer == 1);
}

TEST_CASE("svg export: ellipse and imported shapes survive", "[svg][io]") {
    // Import the shapes fixture, export, import again: same outlines.
    const auto first = importSvg(readFixture("shapes.svg"));
    REQUIRE(first.ok());
    std::vector<SvgLayer> layers = first.value().layers;
    const auto out = exportSvg(layers, first.value().contours);
    REQUIRE(out.ok());
    const auto second = importSvg(out.value());
    REQUIRE(second.ok());
    REQUIRE(second.value().contours.size() == first.value().contours.size());
    for (std::size_t i = 0; i < first.value().contours.size(); ++i) {
        INFO("contour " << i);
        CHECK(outlineDistance(first.value().contours[i].contour, second.value().contours[i].contour) < 1e-6);
    }
}
