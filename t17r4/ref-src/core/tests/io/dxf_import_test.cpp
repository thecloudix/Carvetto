#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <variant>
#include <vector>

#include "carvetto/geometry/span.hpp"
#include "carvetto/io/dxf.hpp"
#include "dxf_test_util.hpp"

using namespace carvetto::geometry;
using namespace carvetto::io;
using namespace dxftest;
using Catch::Matchers::WithinAbs;

// Fixtures (core/tests/fixtures/dxf, hand-written) are valid files of their declared version:
// all_entities.dxf and inches.dxf are DXF R2000 (handles, owners, subclass markers),
// nested_inserts.dxf and r12_entities.dxf are DXF R12 with R12 entities only. Each loads in
// ezdxf 1.4.5 (checked out of tree; not a dependency); its audit is clean except for the
// deliberate self-inserting block and undefined block in nested_inserts.dxf.

namespace {

constexpr double kExact = 1e-6;  // AC1: analytic geometry within 1e-6 mm
constexpr double kChordal = 0.01;

bool near(Vec2 a, Vec2 b, double tol = kExact) {
    return distance(a, b) <= tol;
}

const LineSpan& line(const Contour& c, std::size_t i) {
    REQUIRE(i < c.spans.size());
    REQUIRE(std::holds_alternative<LineSpan>(c.spans[i]));
    return std::get<LineSpan>(c.spans[i]);
}

const ArcSpan& arc(const Contour& c, std::size_t i) {
    REQUIRE(i < c.spans.size());
    REQUIRE(std::holds_alternative<ArcSpan>(c.spans[i]));
    return std::get<ArcSpan>(c.spans[i]);
}

void checkLine(const Contour& c, std::size_t i, Vec2 a, Vec2 b) {
    const LineSpan& l = line(c, i);
    CHECK(near(l.start, a));
    CHECK(near(l.end, b));
}

void checkArc(const Contour& c, std::size_t i, Vec2 centre, double r, Vec2 start, Vec2 end,
              ArcDirection dir) {
    const ArcSpan& a = arc(c, i);
    CHECK(near(a.centre, centre));
    CHECK_THAT(a.radius, WithinAbs(r, kExact));
    CHECK(near(a.startPoint(), start));
    CHECK(near(a.endPoint(), end));
    CHECK(a.direction == dir);
}

/// Samples every span of `c` (also at its ends) and returns the worst value of `deviation`.
template <class F>
double worstDeviation(const Contour& c, F deviation, int samples = 64) {
    double worst = 0.0;
    for (const Span& s : c.spans) {
        for (int i = 0; i <= samples; ++i) {
            worst = std::max(worst, deviation(pointAt(s, static_cast<double>(i) / samples)));
        }
    }
    return worst;
}

/// Distance from p to the ellipse centred at c with semi-axes a (along x) and b (along y),
/// to first order (|F| / |grad F|), accurate for the small deviations measured here.
double ellipseDistance(Vec2 p, Vec2 c, double a, double b) {
    const Vec2 d = p - c;
    const double f = d.x * d.x / (a * a) + d.y * d.y / (b * b) - 1.0;
    const Vec2 g{2.0 * d.x / (a * a), 2.0 * d.y / (b * b)};
    return std::fabs(f) / g.length();
}

/// Independent B-spline evaluation (Cox-de Boor basis functions).
double basis(const std::vector<double>& U, std::size_t i, int p, double u) {
    if (p == 0) {
        const bool last = u == U.back() && U[i] < U[i + 1] && U[i + 1] == U.back();
        return (U[i] <= u && u < U[i + 1]) || last ? 1.0 : 0.0;
    }
    const auto up = static_cast<std::size_t>(p);
    double v = 0.0;
    if (U[i + up] > U[i]) {
        v += (u - U[i]) / (U[i + up] - U[i]) * basis(U, i, p - 1, u);
    }
    if (U[i + up + 1] > U[i + 1]) {
        v += (U[i + up + 1] - u) / (U[i + up + 1] - U[i + 1]) * basis(U, i + 1, p - 1, u);
    }
    return v;
}

Vec2 deBoor(const std::vector<double>& U, const std::vector<Vec2>& P, int p, double u) {
    Vec2 sum;
    for (std::size_t i = 0; i < P.size(); ++i) {
        sum += P[i] * basis(U, i, p, u);
    }
    return sum;
}

double distanceToPolyline(Vec2 p, const std::vector<Vec2>& poly) {
    double best = 1e300;
    for (std::size_t i = 0; i + 1 < poly.size(); ++i) {
        const Vec2 a = poly[i];
        const Vec2 ab = poly[i + 1] - a;
        const double t = std::clamp((p - a).dot(ab) / ab.lengthSquared(), 0.0, 1.0);
        best = std::min(best, distance(p, a + ab * t));
    }
    return best;
}

DxfDrawing load(const std::string& fixture, const DxfImportOptions& options = {}) {
    const Bytes bytes = readFixture(fixture);
    REQUIRE_FALSE(bytes.empty());
    auto r = importDxf(bytes, options);
    if (!r.ok()) {
        FAIL(r.error().describe());
    }
    return r.value();
}

}  // namespace

// ------------------------------------------------------------------------------- AC1

TEST_CASE("AC1: every supported entity imports to its analytic geometry", "[io][dxf][ac1]") {
    const DxfDrawing d = load("all_entities.dxf");
    REQUIRE(d.contours.size() == 16);
    CHECK(d.unitScale == 1.0);
    CHECK(d.unitSource == DxfUnitSource::Insunits);
    CHECK(d.version == "AC1015");
    const auto& C = d.contours;
    auto layerName = [&](std::size_t i) { return d.layers[C[i].layer].name; };

    SECTION("LINE") {
        CHECK(layerName(0) == "Lines");
        REQUIRE(C[0].contour.spans.size() == 1);
        checkLine(C[0].contour, 0, {0, 0}, {10, 5});
        CHECK_FALSE(C[0].contour.closed);
    }
    SECTION("ARC, CIRCLE and an ARC in a mirrored OCS") {
        checkArc(C[1].contour, 0, {20, 0}, 5, {25, 0}, {20, 5}, ArcDirection::CCW);
        const ArcSpan& circle = arc(C[2].contour, 0);
        CHECK(circle.isFullCircle());
        CHECK(near(circle.centre, {40, 0}));
        CHECK(circle.radius == 3.0);
        CHECK(C[2].contour.closed);
        // Extrusion (0,0,-1): OCS X is world -X, so the CCW arc becomes clockwise.
        checkArc(C[3].contour, 0, {-5, 0}, 1, {-6, 0}, {-5, 1}, ArcDirection::CW);
    }
    SECTION("ELLIPSE: full, partial, and circular") {
        const Contour& full = C[4].contour;
        CHECK(full.closed);
        CHECK(full.isContinuous(Tolerance{}));
        CHECK(near(full.startPoint(), {70, 0}));
        for (const Span& s : full.spans) {
            CHECK(std::holds_alternative<BezierSpan>(s));
        }
        CHECK(worstDeviation(full, [](Vec2 p) { return ellipseDistance(p, {60, 0}, 10, 5); }) <= kChordal);
        const Contour& part = C[5].contour;
        CHECK_FALSE(part.closed);
        CHECK(near(part.startPoint(), {60, 28}));
        CHECK(near(part.endPoint(), {58, 20}));
        CHECK(worstDeviation(part, [](Vec2 p) { return ellipseDistance(p, {60, 20}, 2, 8); }) <= kChordal);
        REQUIRE(C[6].contour.spans.size() == 1);
        checkArc(C[6].contour, 0, {80, 0}, 4, {84, 0}, {76, 0}, ArcDirection::CCW);
    }
    SECTION("LWPOLYLINE bulges are exact arcs; closing segment uses the last bulge") {
        CHECK(layerName(7) == "Polys");
        const Contour& c = C[7].contour;
        REQUIRE(c.spans.size() == 4);
        CHECK(c.closed);
        checkLine(c, 0, {0, 20}, {10, 20});
        checkArc(c, 1, {10, 25}, 5, {10, 20}, {10, 30}, ArcDirection::CCW);
        CHECK(near(arc(c, 1).pointAt(0.5), {15, 25}));
        checkLine(c, 2, {10, 30}, {0, 30});
        checkArc(c, 3, {-5, 25}, std::sqrt(50.0), {0, 30}, {0, 20}, ArcDirection::CW);
        CHECK_THAT(std::fabs(arc(c, 3).sweepAngle()), WithinAbs(kPi / 2, 1e-12));
        CHECK(hasWarning(d, "polyline widths ignored"));
    }
    SECTION("POLYLINE/VERTEX/SEQEND 2D, 3D projected, mesh skipped") {
        const Contour& c = C[8].contour;
        REQUIRE(c.spans.size() == 2);
        checkLine(c, 0, {0, 40}, {10, 40});
        checkArc(c, 1, {15, 40}, 5, {10, 40}, {20, 40}, ArcDirection::CW);
        CHECK(near(arc(c, 1).pointAt(0.5), {15, 45}));
        checkLine(C[9].contour, 0, {30, 40}, {35, 45});
        CHECK(hasWarning(d, "3D POLYLINE projected"));
        CHECK(hasWarning(d, "polyface POLYLINE skipped"));
    }
    SECTION("SPLINE: non-rational cubic converts exactly") {
        const std::vector<double> U{0, 0, 0, 0, 0.5, 1, 1, 1, 1};
        const std::vector<Vec2> P{{0, 50}, {5, 60}, {10, 50}, {15, 60}, {20, 50}};
        const Contour& c = C[10].contour;
        REQUIRE(c.spans.size() == 2);
        for (std::size_t seg = 0; seg < 2; ++seg) {
            const auto& b = std::get<BezierSpan>(c.spans[seg]);
            for (int i = 0; i <= 20; ++i) {
                const double t = i / 20.0;
                const double u = 0.5 * (static_cast<double>(seg) + t);
                CHECK(near(b.pointAt(t), deBoor(U, P, 3, u), 1e-9));
            }
        }
    }
    SECTION("SPLINE: rational quadratic quarter circle within the chordal tolerance") {
        const Contour& c = C[11].contour;
        CHECK(near(c.startPoint(), {100, 0}));
        CHECK(near(c.endPoint(), {90, 10}));
        CHECK(c.isContinuous(Tolerance{}));
        CHECK(worstDeviation(c, [](Vec2 p) { return std::fabs(distance(p, {90, 0}) - 10.0); }) <= kChordal);
    }
    SECTION("SPLINE: degree 4 within the chordal tolerance") {
        const std::vector<double> U{0, 0, 0, 0, 0, 0.4, 1, 1, 1, 1, 1};
        const std::vector<Vec2> P{{30, 50}, {32, 58}, {38, 44}, {44, 60}, {50, 46}, {54, 52}};
        std::vector<Vec2> dense;
        for (int i = 0; i <= 20000; ++i) {
            dense.push_back(deBoor(U, P, 4, i / 20000.0));
        }
        const Contour& c = C[12].contour;
        CHECK(near(c.startPoint(), P.front()));
        CHECK(near(c.endPoint(), P.back()));
        CHECK(worstDeviation(c, [&](Vec2 p) { return distanceToPolyline(p, dense); }) <= kChordal);
    }
    SECTION("SPLINE: fit points only are interpolated through every point") {
        const Contour& c = C[13].contour;
        REQUIRE(c.spans.size() == 3);
        const std::vector<Vec2> fit{{0, 60}, {10, 65}, {20, 60}, {30, 65}};
        for (std::size_t i = 0; i < 3; ++i) {
            CHECK(startPoint(c.spans[i]) == fit[i]);
            CHECK(endPoint(c.spans[i]) == fit[i + 1]);
        }
        CHECK(hasWarning(d, "fit points interpolated"));
    }
    SECTION("INSERT/BLOCK: layer-0 entities take the INSERT's layer") {
        CHECK(layerName(14) == "Blocks");
        const Contour& sq = C[14].contour;
        REQUIRE(sq.spans.size() == 4);
        CHECK(sq.closed);
        checkLine(sq, 0, {52, 48}, {52, 52});
        checkLine(sq, 1, {52, 52}, {48, 52});
        checkLine(sq, 2, {48, 52}, {48, 48});
        checkLine(sq, 3, {48, 48}, {52, 48});
        CHECK(layerName(15) == "Inner");
        checkLine(C[15].contour, 0, {52, 48}, {48, 52});
    }
    SECTION("TEXT and MTEXT become text items") {
        REQUIRE(d.texts.size() == 3);
        const DxfText& t = d.texts[0];
        CHECK(t.kind == DxfTextKind::Text);
        CHECK(t.text == "Hello \xC2\xB0");
        CHECK(t.style == "MyStyle");
        CHECK(t.font == "arial.ttf");
        CHECK_THAT(t.height, WithinAbs(2.5, 1e-12));
        CHECK(near(t.insertion, {5, 80}));
        CHECK_THAT(t.rotation, WithinAbs(kPi / 6, 1e-12));
        CHECK(t.hAlign == DxfHAlign::Left);
        CHECK(t.vAlign == DxfVAlign::Baseline);
        CHECK_FALSE(t.alignmentPoint.has_value());
        CHECK(d.layers[t.layer].name == "Text");

        const DxfText& c = d.texts[1];
        CHECK(c.text == "Centred");
        CHECK(c.style == "Standard");
        CHECK(c.hAlign == DxfHAlign::Center);
        CHECK(c.vAlign == DxfVAlign::Middle);
        REQUIRE(c.alignmentPoint.has_value());
        CHECK(near(*c.alignmentPoint, {15, 90}));

        const DxfText& m = d.texts[2];
        CHECK(m.kind == DxfTextKind::MText);
        CHECK(m.text == "Bold line\nnext 1/2");
        CHECK(m.hAlign == DxfHAlign::Center);
        CHECK(m.vAlign == DxfVAlign::Middle);
        CHECK_THAT(m.referenceWidth, WithinAbs(50.0, 1e-12));
        CHECK_THAT(m.height, WithinAbs(3.0, 1e-12));
        CHECK(m.font == "arial.ttf");
    }
    SECTION("unsupported entities are counted warnings, not errors") {
        CHECK(hasWarning(d, "unsupported entity HATCH skipped", 2));
        CHECK(hasWarning(d, "unsupported entity POINT skipped", 1));
    }
    SECTION("layers: table order, colours, visibility, then referenced-only layers") {
        REQUIRE(d.layers.size() >= 6);
        CHECK(d.layers[0].name == "0");
        CHECK(d.layers[1].name == "Lines");
        CHECK(d.layers[1].aci == 1);
        CHECK(d.layers[1].rgb == 0xFF0000u);
        CHECK(d.layers[2].rgb == 0x0000FFu);
        CHECK(d.layers[3].visible);
        CHECK_FALSE(d.layers[4].visible);  // negative colour = off
        CHECK(d.layers[4].aci == 4);
        CHECK_FALSE(d.layers[5].visible);  // frozen
        CHECK(layerIndex(d, "Polys").has_value());
        CHECK(layerIndex(d, "Splines").has_value());
    }
}

TEST_CASE("binary DXF (R13+ and R12 layouts) imports the same as ASCII", "[io][dxf][binary]") {
    const Bytes ascii = readFixture("all_entities.dxf");
    const DxfDrawing a = importDxf(ascii).value();
    for (const bool wide : {true, false}) {
        const Bytes bin = toBinary(toString(ascii), wide);
        auto r = importDxf(bin);
        REQUIRE(r.ok());
        CHECK(r.value().binary);
        CHECK(r.value().contours == a.contours);
        CHECK(r.value().texts == a.texts);
        CHECK(r.value().layers == a.layers);
    }
    // Truncated binary input fails cleanly.
    const Bytes bin = toBinary(toString(ascii), true);
    auto r = importDxf(std::span<const std::uint8_t>(bin.data(), bin.size() / 2));
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == DxfError::Code::Truncated);
}

// ------------------------------------------------------------------------------- AC4

namespace {

Vec2 rot(Vec2 p, double degrees) {
    const double a = degrees * kPi / 180.0;
    return {p.x * std::cos(a) - p.y * std::sin(a), p.x * std::sin(a) + p.y * std::cos(a)};
}

// Hand-written chain for the fixture: LEAF is inserted into MID at (2,0), rotated 90 degrees,
// scaled (-2, 2); MID (base point (1,1)) is inserted at (10,10), rotated 30 degrees, scaled 1.5.
Vec2 leafToMid(Vec2 q) {
    q = {-2.0 * q.x, 2.0 * q.y};
    q = rot(q, 90.0);
    return q + Vec2{2.0, 0.0};
}
Vec2 midToWorld(Vec2 q) {
    q = q - Vec2{1.0, 1.0};
    q = q * 1.5;
    q = rot(q, 30.0);
    return q + Vec2{10.0, 10.0};
}

}  // namespace

TEST_CASE("AC4: nested INSERTs with rotation, scale and mirror flatten correctly", "[io][dxf][ac4]") {
    const DxfDrawing d = load("nested_inserts.dxf");
    REQUIRE(d.contours.size() == 17);
    auto layerName = [&](std::size_t i) { return d.layers[d.contours[i].layer].name; };
    const auto& C = d.contours;

    // LEAF line (layer 0 in LEAF, inserted on layer 0 in MID) inherits MID's insert layer "Top".
    CHECK(layerName(0) == "Top");
    checkLine(C[0].contour, 0, midToWorld(leafToMid({0, 0})), midToWorld(leafToMid({1, 0})));
    // Hand-evaluated values of the same points.
    CHECK(near(line(C[0].contour, 0).start,
               {10.0 + 1.5 * std::cos(kPi / 6) + 1.5 * 0.5, 10.0 + 1.5 * 0.5 - 1.5 * std::cos(kPi / 6)}));
    CHECK(near(line(C[0].contour, 0).end, {13.549038105676658, 6.852885682970026}));

    // LEAF arc: one mirror in the chain, uniform magnitudes -> still an exact arc, now clockwise.
    CHECK(layerName(1) == "LeafLayer");
    checkArc(C[1].contour, 0, midToWorld(leafToMid({1, 0})), 0.5 * 2.0 * 1.5, midToWorld(leafToMid({1.5, 0})),
             midToWorld(leafToMid({1, 0.5})), ArcDirection::CW);
    CHECK(near(arc(C[1].contour, 0).startPoint(), {14.299038105676658, 5.553847577293368}));
    CHECK(near(arc(C[1].contour, 0).endPoint(), {12.25, 6.102885682970026}));

    CHECK(layerName(2) == "MidLayer");
    checkLine(C[2].contour, 0, {10, 10}, {9.25, 10.0 + 1.5 * std::cos(kPi / 6)});

    SECTION("MINSERT arrays: rows x columns cells, spacing in the insert's rotated frame") {
        std::size_t k = 3;
        for (int r = 0; r < 2; ++r) {
            for (int c = 0; c < 2; ++c) {
                const Vec2 o{100.0 + 5.0 * c, 3.0 * r};
                CHECK(layerName(k) == "Arr");
                checkLine(C[k].contour, 0, o, o + Vec2{1, 0});
                checkArc(C[k + 1].contour, 0, o + Vec2{1, 0}, 0.5, o + Vec2{1.5, 0}, o + Vec2{1, 0.5},
                         ArcDirection::CCW);
                k += 2;
            }
        }
        checkLine(C[11].contour, 0, {200, 0}, {200, 1});
        checkLine(C[13].contour, 0, {200, 5}, {200, 6});
    }
    SECTION("a block inserting itself is expanded once and warned about") {
        CHECK(layerName(15) == "Loop");
        checkLine(C[15].contour, 0, {300, 0}, {300, 1});
        CHECK(hasWarning(d, "inserts itself"));
    }
    SECTION("non-uniform scale turns a circle into an ellipse within the chordal tolerance") {
        const Contour& ring = C[16].contour;
        CHECK(ring.closed);
        CHECK(ring.isContinuous(Tolerance{}));
        CHECK(std::holds_alternative<BezierSpan>(ring.spans[0]));
        CHECK(worstDeviation(ring, [](Vec2 p) { return ellipseDistance(p, {400, 0}, 30, 10); }) <= kChordal);
    }
    CHECK(hasWarning(d, "undefined block 'NOSUCH'"));
}

// ------------------------------------------------------------------------------- AC3

TEST_CASE("an R12 file with R12 entities only", "[io][dxf][r12]") {
    const DxfDrawing d = load("r12_entities.dxf");
    CHECK(d.version == "AC1009");
    CHECK(d.unitSource == DxfUnitSource::Default);
    REQUIRE(d.contours.size() == 5);
    const auto& C = d.contours;
    checkLine(C[0].contour, 0, {0, 0}, {10, 0});
    checkArc(C[1].contour, 0, {0, 0}, 2, {0, 2}, {-2, 0}, ArcDirection::CCW);
    CHECK(arc(C[2].contour, 0).isFullCircle());
    const Contour& poly = C[3].contour;
    REQUIRE(poly.spans.size() == 3);
    CHECK(poly.closed);
    checkArc(poly, 0, {2, 10}, 2, {0, 10}, {4, 10}, ArcDirection::CCW);
    CHECK(near(arc(poly, 0).pointAt(0.5), {2, 8}));
    checkLine(poly, 1, {4, 10}, {4, 14});
    checkLine(poly, 2, {4, 14}, {0, 10});
    checkLine(C[4].contour, 0, {20, 0}, {20, 2});
    CHECK(d.layers[C[4].layer].name == "B");
    REQUIRE(d.texts.size() == 1);
    CHECK(d.texts[0].text == "R12 text");
    CHECK(d.warnings.empty());
}

TEST_CASE("AC3: $INSUNITS inches scale to millimetres", "[io][dxf][ac3][units]") {
    const DxfDrawing d = load("inches.dxf");
    CHECK(d.insunits == 1);
    CHECK(d.unitScale == 25.4);
    CHECK(d.unitSource == DxfUnitSource::Insunits);
    REQUIRE(d.contours.size() == 2);
    checkLine(d.contours[0].contour, 0, {0, 0}, {25.4, 50.8});
    const ArcSpan& c = arc(d.contours[1].contour, 0);
    CHECK(near(c.centre, {50.8, 0}));
    CHECK_THAT(c.radius, WithinAbs(12.7, 1e-12));
    REQUIRE(d.texts.size() == 1);
    CHECK_THAT(d.texts[0].height, WithinAbs(6.35, 1e-12));
    CHECK(near(d.texts[0].insertion, {25.4, 25.4}));
}

TEST_CASE("units: other $INSUNITS codes, $MEASUREMENT fallback, unitless default, forced scale",
          "[io][dxf][units]") {
    auto file = [](const std::string& header) {
        return "0\nSECTION\n2\nHEADER\n" + header +
               "0\nENDSEC\n0\nSECTION\n2\nENTITIES\n0\nLINE\n8\n0\n10\n0\n20\n0\n11\n1\n21\n0\n0\nENDSEC\n0\n"
               "EOF\n";
    };
    auto lengthOf = [](const DxfDrawing& d) { return d.contours.at(0).contour.length(); };
    struct Case {
        std::string header;
        double mm;
        DxfUnitSource source;
    };
    const std::vector<Case> cases{
        {"9\n$INSUNITS\n70\n2\n", 304.8, DxfUnitSource::Insunits},
        {"9\n$INSUNITS\n70\n4\n", 1.0, DxfUnitSource::Insunits},
        {"9\n$INSUNITS\n70\n5\n", 10.0, DxfUnitSource::Insunits},
        {"9\n$INSUNITS\n70\n6\n", 1000.0, DxfUnitSource::Insunits},
        {"9\n$INSUNITS\n70\n6\n9\n$MEASUREMENT\n70\n0\n", 1000.0, DxfUnitSource::Insunits},
        {"9\n$INSUNITS\n70\n0\n9\n$MEASUREMENT\n70\n0\n", 25.4, DxfUnitSource::Measurement},
        {"9\n$MEASUREMENT\n70\n1\n", 1.0, DxfUnitSource::Measurement},
        {"9\n$INSUNITS\n70\n0\n", 1.0, DxfUnitSource::Default},
        {"", 1.0, DxfUnitSource::Default},
    };
    for (const Case& c : cases) {
        auto r = importText(file(c.header));
        REQUIRE(r.ok());
        CHECK_THAT(lengthOf(r.value()), WithinAbs(c.mm, 1e-9));
        CHECK(r.value().unitSource == c.source);
    }
    DxfImportOptions inchDefault;
    inchDefault.unitlessScale = 25.4;
    CHECK_THAT(lengthOf(importText(file(""), inchDefault).value()), WithinAbs(25.4, 1e-12));
    DxfImportOptions forced;
    forced.forcedScale = 2.0;
    const DxfDrawing f = importText(file("9\n$INSUNITS\n70\n1\n"), forced).value();
    CHECK_THAT(lengthOf(f), WithinAbs(2.0, 1e-12));
    CHECK(f.unitSource == DxfUnitSource::Forced);
    const DxfDrawing unknown = importText(file("9\n$INSUNITS\n70\n19\n")).value();
    CHECK(hasWarning(unknown, "$INSUNITS 19"));
    CHECK(unknown.unitSource == DxfUnitSource::Default);
}

// ----------------------------------------------------------------- other behaviour

TEST_CASE("joinContiguous option joins touching consecutive entities; default keeps them apart",
          "[io][dxf][join]") {
    const std::string text =
        "0\nSECTION\n2\nENTITIES\n"
        "0\nLINE\n8\nA\n10\n0\n20\n0\n11\n10\n21\n0\n"
        "0\nARC\n8\nA\n10\n10\n20\n5\n40\n5\n50\n-90\n51\n90\n"
        "0\nLINE\n8\nA\n10\n0\n20\n10\n11\n10\n21\n10\n"  // reversed: ends at the arc's end
        "0\nLINE\n8\nA\n10\n0\n20\n10\n11\n0\n21\n0\n"
        "0\nLINE\n8\nB\n10\n0\n20\n0\n11\n5\n21\n5\n"
        "0\nENDSEC\n0\nEOF\n";
    CHECK(importText(text).value().contours.size() == 5);
    DxfImportOptions o;
    o.joinContiguous = true;
    const DxfDrawing d = importText(text, o).value();
    REQUIRE(d.contours.size() == 2);
    const Contour& c = d.contours[0].contour;
    CHECK(c.spans.size() == 4);
    CHECK(c.closed);
    CHECK(c.isContinuous(Tolerance{}));
    CHECK(d.layers[d.contours[1].layer].name == "B");
}

TEST_CASE("text decoding: control codes, MTEXT formatting, code pages and \\U+ escapes", "[io][dxf][text]") {
    CHECK(stripMTextFormatting("\\A1;{\\H2.5x;\\C1;Red} \\~x\\P\\LUnder\\l \\{brace\\} \\\\") ==
          "Red \xC2\xA0x\nUnder {brace} \\");
    CHECK(stripMTextFormatting("\\S+0.1^-0.2; and \\S3#4;") == "+0.1/-0.2 and 3/4");
    CHECK(stripMTextFormatting("\\U+00E9t\\U+00E9") == "\xC3\xA9t\xC3\xA9");
    const std::string text =
        "0\nSECTION\n2\nHEADER\n9\n$ACADVER\n1\nAC1015\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n"
        "0\nTEXT\n8\nL\xE9\n10\n0\n20\n0\n40\n1\n1\n%%c10 %%p0.1 100%%% \xB0 \\U+2713 %%uU\n"
        "0\nENDSEC\n0\nEOF\n";
    const DxfDrawing d = importText(text).value();
    REQUIRE(d.texts.size() == 1);
    CHECK(d.texts[0].text ==
          "\xE2\x8C\x80"
          "10 \xC2\xB1"
          "0.1 100% \xC2\xB0 \xE2\x9C\x93 U");
    CHECK(d.layers[d.texts[0].layer].name == "L\xC3\xA9");  // Windows-1252 0xE9
    // R2007+ files are UTF-8.
    const std::string utf8 =
        "0\nSECTION\n2\nHEADER\n9\n$ACADVER\n1\nAC1032\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n"
        "0\nTEXT\n8\n0\n10\n0\n20\n0\n40\n1\n1\n\xC3\xA9\xFF\n0\nENDSEC\n0\nEOF\n";
    CHECK(importText(utf8).value().texts.at(0).text == "\xC3\xA9\xEF\xBF\xBD");
}

TEST_CASE("text placement follows block transforms (rotation, scale, mirror)", "[io][dxf][text]") {
    const std::string text =
        "0\nSECTION\n2\nBLOCKS\n0\nBLOCK\n2\nT\n10\n0\n20\n0\n70\n0\n"
        "0\nTEXT\n8\n0\n10\n1\n20\n0\n40\n2\n1\nx\n"
        "0\nENDBLK\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n"
        "0\nINSERT\n8\nTL\n2\nT\n10\n10\n20\n0\n41\n-3\n42\n3\n50\n90\n"
        "0\nINSERT\n8\nTL\n2\nT\n10\n0\n20\n0\n41\n2\n42\n1\n"
        "0\nENDSEC\n0\nEOF\n";
    const DxfDrawing d = importText(text).value();
    REQUIRE(d.texts.size() == 2);
    const DxfText& a = d.texts[0];
    CHECK(near(a.insertion, {10, -3}));  // (1,0) -> (-3,0) -> rotated 90 -> (0,-3) -> +(10,0)
    CHECK_THAT(a.height, WithinAbs(6.0, 1e-12));
    CHECK_THAT(a.widthFactor, WithinAbs(1.0, 1e-12));
    CHECK(a.mirrored);
    CHECK_THAT(a.rotation, WithinAbs(-kPi / 2, 1e-12));
    CHECK(d.layers[a.layer].name == "TL");
    const DxfText& b = d.texts[1];
    CHECK_THAT(b.height, WithinAbs(2.0, 1e-12));
    CHECK_THAT(b.widthFactor, WithinAbs(2.0, 1e-12));
    CHECK_FALSE(b.mirrored);
}

TEST_CASE("Carvetto-specific checks: colours, budgets, depth and malformed structure", "[io][dxf]") {
    SECTION("ACI palette") {
        CHECK(aciToRgb(1) == 0xFF0000u);
        CHECK(aciToRgb(3) == 0x00FF00u);
        CHECK(aciToRgb(5) == 0x0000FFu);
        CHECK(aciToRgb(7) == 0xFFFFFFu);
        CHECK(aciToRgb(10) == 0xFF0000u);
        CHECK(aciToRgb(250) == 0x333333u);
        CHECK(aciToRgb(0) == 0xFFFFFFu);
        for (int i = 1; i <= 255; ++i) {
            CHECK(aciToRgb(nearestAci(aciToRgb(i))) == aciToRgb(i));
        }
    }
    SECTION("span budget") {
        DxfImportOptions o;
        o.maxSpans = 3;
        auto r = importDxf(readFixture("all_entities.dxf"), o);
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().code == DxfError::Code::ResourceLimit);
    }
    SECTION("exponential block nesting hits the instance budget") {
        std::string blocks = "0\nSECTION\n2\nBLOCKS\n";
        blocks += "0\nBLOCK\n2\nB0\n70\n0\n10\n0\n20\n0\n0\nENDBLK\n";
        for (int i = 1; i <= 12; ++i) {
            blocks += "0\nBLOCK\n2\nB" + std::to_string(i) + "\n70\n0\n10\n0\n20\n0\n";
            for (int k = 0; k < 10; ++k) {
                blocks += "0\nINSERT\n8\n0\n2\nB" + std::to_string(i - 1) + "\n10\n" + std::to_string(k) +
                          "\n20\n0\n";
            }
            blocks += "0\nENDBLK\n";
        }
        blocks +=
            "0\nENDSEC\n0\nSECTION\n2\nENTITIES\n0\nINSERT\n8\n0\n2\nB12\n10\n0\n20\n0\n0\nENDSEC\n0\nEOF\n";
        auto r = importText(blocks);
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().code == DxfError::Code::ResourceLimit);
        // A huge MINSERT array likewise.
        auto m = importText(
            "0\nSECTION\n2\nBLOCKS\n0\nBLOCK\n2\nE\n70\n0\n10\n0\n20\n0\n0\nENDBLK\n0\nENDSEC\n"
            "0\nSECTION\n2\nENTITIES\n0\nINSERT\n2\nE\n10\n0\n20\n0\n70\n32767\n71\n32767\n0\nENDSEC\n0\nEOF"
            "\n");
        REQUIRE_FALSE(m.ok());
        CHECK(m.error().code == DxfError::Code::ResourceLimit);
    }
    SECTION("nesting deeper than maxBlockDepth is skipped with a warning") {
        std::string blocks = "0\nSECTION\n2\nBLOCKS\n";
        for (int i = 0; i < 100; ++i) {
            blocks += "0\nBLOCK\n2\nD" + std::to_string(i) +
                      "\n70\n0\n10\n0\n20\n0\n0\nLINE\n8\n0\n10\n0\n20\n" + std::to_string(i) +
                      "\n11\n1\n21\n" + std::to_string(i) + "\n0\nINSERT\n2\nD" + std::to_string(i + 1) +
                      "\n10\n0\n20\n0\n0\nENDBLK\n";
        }
        blocks += "0\nENDSEC\n0\nSECTION\n2\nENTITIES\n0\nINSERT\n2\nD0\n10\n0\n20\n0\n0\nENDSEC\n0\nEOF\n";
        DxfImportOptions o;
        o.maxBlockDepth = 10;
        const DxfDrawing d = importText(blocks, o).value();
        CHECK(d.contours.size() == 10);
        CHECK(hasWarning(d, "nested deeper than 10"));
        CHECK(hasWarning(d, "undefined block 'D100'") == false);
    }
    SECTION("structural errors are reported, never thrown") {
        CHECK(importText("").error().code == DxfError::Code::InvalidFormat);
        CHECK(importText("hello world\n").error().code == DxfError::Code::InvalidFormat);
        CHECK(importText("0\nSECTION\n2\nENTITIES\n0\nLINE\n10\n").error().code == DxfError::Code::Truncated);
        CHECK(importText("0\nSECTION\n2\nENTITIES\n0\nLINE\n10\nabc\n0\nENDSEC\n0\nEOF\n").error().code ==
              DxfError::Code::InvalidFormat);
        const auto noEof = importText("0\nSECTION\n2\nENTITIES\n0\nENDSEC\n");
        REQUIRE(noEof.ok());
        CHECK(hasWarning(noEof.value(), "no EOF"));
        const DxfError e = importText("0\nSECTION\n2\nENTITIES\n0\nLINE\n10\n1e999\n").error();
        CHECK(e.line == 8);  // the value line
        CHECK(e.describe().find("line 8") == 0);
    }
    SECTION("out-of-range curves and degenerate entities are skipped with warnings") {
        const DxfDrawing d = importText(
                                 "0\nSECTION\n2\nENTITIES\n"
                                 "0\nELLIPSE\n10\n0\n20\n0\n11\n1e300\n21\n0\n40\n0.5\n"
                                 "0\nELLIPSE\n10\n1e13\n20\n0\n11\n1\n21\n0\n40\n0.5\n"
                                 "0\nARC\n10\n0\n20\n0\n40\n1e200\n50\n0\n51\n90\n"
                                 "0\nSPLINE\n71\n3\n40\n0\n40\n0\n40\n0\n40\n0\n40\n1\n40\n1\n40\n1\n40\n1\n"
                                 "10\n2\n20\n3\n10\n2\n20\n3\n10\n2\n20\n3\n10\n2\n20\n3\n"
                                 "0\nSPLINE\n71\n3\n11\n0\n21\n0\n11\n1e300\n21\n0\n"
                                 "0\nLWPOLYLINE\n90\n2\n10\n1\n20\n1\n10\n1\n20\n1\n"
                                 "0\nLINE\n10\n0\n20\n0\n11\n1\n21\n0\n"
                                 "0\nENDSEC\n0\nEOF\n")
                                 .value();
        REQUIRE(d.contours.size() == 1);  // only the good LINE
        CHECK(hasWarning(d, "coordinates out of range skipped", 4));
        CHECK(hasWarning(d, "zero-length SPLINE skipped"));
        CHECK(hasWarning(d, "zero-length polyline skipped"));
    }
    SECTION("invalid splines and tilted entities are warnings") {
        const DxfDrawing d = importText(
                                 "0\nSECTION\n2\nENTITIES\n"
                                 "0\nSPLINE\n71\n1000\n10\n0\n20\n0\n"
                                 "0\nSPLINE\n71\n3\n40\n0\n40\n1\n10\n0\n20\n0\n10\n1\n20\n0\n"
                                 "0\nCIRCLE\n10\n0\n20\n0\n40\n1\n210\n1\n220\n0\n230\n0\n"
                                 "0\nCIRCLE\n10\n0\n20\n0\n40\n-1\n"
                                 "0\nENDSEC\n0\nEOF\n")
                                 .value();
        CHECK(d.contours.empty());
        CHECK(hasWarning(d, "invalid SPLINE skipped: unsupported degree 1000"));
        CHECK(hasWarning(d, "invalid SPLINE skipped: too few control points"));
        CHECK(hasWarning(d, "CIRCLE not parallel"));
        CHECK(hasWarning(d, "non-positive radius"));
    }
}
