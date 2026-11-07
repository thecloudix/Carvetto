#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <string>
#include <variant>

#include "carvetto/geometry/span.hpp"
#include "carvetto/io/svg.hpp"
#include "svg_test_util.hpp"

using namespace carvetto::geometry;
using namespace carvetto::io;
using namespace svgtest;
using Catch::Approx;

namespace {

SvgDrawing load(const std::string& text, const SvgImportOptions& o = {}) {
    auto r = importText(text, o);
    REQUIRE(r.ok());
    return r.value();
}

SvgDrawing loadPage(const std::string& body, const std::string& extra = {}) {
    return load(page(body, extra));
}

bool isNear(Vec2 a, Vec2 b, double tol = 1e-9) {
    return (a - b).length() <= tol;
}

}  // namespace

TEST_CASE("svg import: rect, circle, ellipse, line, polyline, polygon", "[svg][io]") {
    auto r = importSvg(readFixture("shapes.svg"));
    REQUIRE(r.ok());
    const SvgDrawing& d = r.value();
    CHECK(d.widthMm == Approx(100.0));
    CHECK(d.heightMm == Approx(80.0));
    REQUIRE(d.layers.size() == 1);
    CHECK(d.layers[0].name == "0");
    REQUIRE(d.contours.size() == 7);

    const Contour& plain = d.contours[0].contour;
    CHECK(plain.closed);
    CHECK(plain.spans.size() == 4);
    CHECK(isNear(plain.bounds().min, {5.0, 65.0}));
    CHECK(isNear(plain.bounds().max, {25.0, 75.0}));

    const Contour& rounded = d.contours[1].contour;
    CHECK(rounded.closed);
    CHECK(rounded.spans.size() == 8);  // four lines + four elliptical corners
    CHECK(isNear(rounded.bounds().min, {30.0, 65.0}, 1e-3));
    CHECK(rounded.isContinuous(Tolerance{}));

    const Contour& circle = d.contours[2].contour;
    REQUIRE(circle.spans.size() == 1);
    const auto* arc = std::get_if<ArcSpan>(&circle.spans[0]);
    REQUIRE(arc != nullptr);
    CHECK(arc->radius == Approx(10.0));
    CHECK(isNear(arc->centre, {70.0, 60.0}));
    CHECK(arc->isFullCircle());
    CHECK(circle.closed);

    const Contour& ellipse = d.contours[3].contour;
    CHECK(ellipse.closed);
    CHECK(isNear(ellipse.bounds().min, {5.0, 22.0}, 0.01));
    CHECK(isNear(ellipse.bounds().max, {35.0, 38.0}, 0.01));

    const Contour& line = d.contours[4].contour;
    CHECK_FALSE(line.closed);
    REQUIRE(line.spans.size() == 1);
    CHECK(isNear(line.startPoint(), {45.0, 40.0}));
    CHECK(isNear(line.endPoint(), {60.0, 25.0}));

    CHECK_FALSE(d.contours[5].contour.closed);
    CHECK(d.contours[5].contour.spans.size() == 2);
    CHECK(d.contours[6].contour.closed);
    CHECK(d.contours[6].contour.spans.size() == 3);
    CHECK(d.warnings.empty());
}

TEST_CASE("svg import: every path command, absolute and relative", "[svg][io]") {
    auto r = importSvg(readFixture("paths.svg"));
    REQUIRE(r.ok());
    const SvgDrawing& d = r.value();
    REQUIRE(d.contours.size() == 2);
    const Contour& a = d.contours[0].contour;
    const Contour& b = d.contours[1].contour;
    CHECK(a.closed);
    CHECK(b.closed);
    // M L H V C S Q T A Z -> 1 + 1 + 1 + 1 + 1 + 1 + 1 + 1 spans plus the closing line.
    CHECK(a.spans.size() == b.spans.size());
    // The relative path is the absolute path shifted by (0, 50) in user space.
    const Transform2 shift = Transform2::translation({0.0, -50.0});
    CHECK(outlineDistance(a.transformed(shift), b) < 1e-9);
    CHECK(a.isContinuous(Tolerance{}));
    CHECK(isNear(a.startPoint(), {10.0, 90.0}));
    CHECK(std::holds_alternative<ArcSpan>(a.spans[a.spans.size() - 2]));
}

TEST_CASE("svg import: path command details", "[svg][io]") {
    SECTION("H and V, implicit lineto after moveto") {
        const auto d = loadPage("<path d=\"M10 10 20 10 20 20\"/><path d=\"M10 10 H30 V30\"/>");
        REQUIRE(d.contours.size() == 2);
        CHECK(d.contours[0].contour.spans.size() == 2);
        CHECK(isNear(d.contours[0].contour.endPoint(), {20.0, 80.0}));
        CHECK(isNear(d.contours[1].contour.endPoint(), {30.0, 70.0}));
    }
    SECTION("relative moveto after close starts at the subpath start") {
        const auto d = loadPage("<path d=\"M10 10 h10 v10 z m5 5 h5\"/>");
        REQUIRE(d.contours.size() == 2);
        CHECK(isNear(d.contours[1].contour.startPoint(), {15.0, 85.0}));
    }
    SECTION("lineto after close continues from the subpath start") {
        const auto d = loadPage("<path d=\"M10 10 h10 v10 z l5 0\"/>");
        REQUIRE(d.contours.size() == 2);
        CHECK(isNear(d.contours[1].contour.startPoint(), {10.0, 90.0}));
    }
    SECTION("smooth cubic reflects the previous control point") {
        const auto d = loadPage("<path d=\"M0 0 C 10 0 20 10 30 10 S 50 20 60 20\"/>");
        const auto* b = std::get_if<BezierSpan>(&d.contours[0].contour.spans[1]);
        REQUIRE(b != nullptr);
        // Reflection of (20,10) about (30,10) is (40,10); y flips.
        CHECK(isNear(b->p1, {40.0, 90.0}));
        CHECK(isNear(b->p2, {50.0, 80.0}));
    }
    SECTION("S without a previous cubic uses the current point") {
        const auto d = loadPage("<path d=\"M10 10 S 20 20 30 10\"/>");
        const auto* b = std::get_if<BezierSpan>(&d.contours[0].contour.spans[0]);
        REQUIRE(b != nullptr);
        CHECK(isNear(b->p1, {10.0, 90.0}));
    }
    SECTION("quadratics are raised exactly to cubics") {
        const auto d = loadPage("<path d=\"M0 0 Q 15 30 30 0 T 60 0\"/>");
        const Contour& c = d.contours[0].contour;
        REQUIRE(c.spans.size() == 2);
        const auto* b0 = std::get_if<BezierSpan>(&c.spans[0]);
        const auto* b1 = std::get_if<BezierSpan>(&c.spans[1]);
        REQUIRE(b0 != nullptr);
        REQUIRE(b1 != nullptr);
        CHECK(b0->pointAt(0.5).y == Approx(100.0 - 15.0));  // quadratic midpoint is 0.5*control height
        CHECK(isNear(b1->p1, {40.0, 100.0 + 20.0}, 1e-9));    // control (45,-30) reflected, raised to cubic
    }
    SECTION("compact number forms") {
        const auto d = loadPage("<path d=\"M1e1,1e1.5.5-.5-.5\"/><path d=\"M+10 10L2.5e+1-.5e1\"/>");
        REQUIRE(d.contours.size() == 2);
        CHECK(d.contours[0].contour.spans.size() == 2);
        CHECK(isNear(d.contours[1].contour.endPoint(), {25.0, 105.0}));
    }
    SECTION("zero-length segments are dropped") {
        const auto d = loadPage("<path d=\"M10 10 L10 10 L20 10\"/>");
        REQUIRE(d.contours.size() == 1);
        CHECK(d.contours[0].contour.spans.size() == 1);
    }
    SECTION("a lone moveto draws nothing") {
        CHECK(loadPage("<path d=\"M10 10\"/><path d=\"M10 10 z\"/><path/>").contours.empty());
    }
    SECTION("close adds a line only when needed") {
        const auto d = loadPage("<path d=\"M0 0 H10 V10 H0 V0 Z\"/><path d=\"M0 0 H10 V10 Z\"/>");
        CHECK(d.contours[0].contour.spans.size() == 4);
        CHECK(d.contours[1].contour.spans.size() == 3);
    }
}

TEST_CASE("svg import: elliptical arcs", "[svg][io]") {
    SECTION("the four flag combinations pick the four centres") {
        struct Case {
            const char* flags;
            double cx;
            double cy;
            double sweepSign;  // sign of the sweep in the Y-up result
            bool large;
        };
        // Chord from (0,0) to (10,0) with radius 10: centres at (5, +-8.660254).
        const Case cases[] = {
            {"0 0", 5.0, -std::sqrt(75.0), 1.0, false},   // small, negative angle direction
            {"0 1", 5.0, std::sqrt(75.0), -1.0, false},
            {"1 0", 5.0, std::sqrt(75.0), 1.0, true},
            {"1 1", 5.0, -std::sqrt(75.0), -1.0, true},
        };
        for (const Case& c : cases) {
            const auto d = loadPage(std::string("<path d=\"M40 50 a10 10 0 ") + c.flags + " 10 0\"/>");
            REQUIRE(d.contours.size() == 1);
            REQUIRE(d.contours[0].contour.spans.size() == 1);
            const auto* a = std::get_if<ArcSpan>(&d.contours[0].contour.spans[0]);
            REQUIRE(a != nullptr);
            INFO(c.flags);
            CHECK(a->radius == Approx(10.0));
            CHECK(isNear(a->centre, {45.0, 50.0 - c.cy}, 1e-9));
            CHECK(isNear(a->startPoint(), {40.0, 50.0}, 1e-9));
            CHECK(isNear(a->endPoint(), {50.0, 50.0}, 1e-9));
            const bool largeResult = std::fabs(a->sweepAngle()) > kPi;
            CHECK(largeResult == c.large);
        }
    }
    SECTION("flags may be written without separators") {
        const auto d = loadPage("<path d=\"M10 50a5 5 0 0110 0\"/><path d=\"M10 50a5,5,0,1,0,10,0\"/>");
        REQUIRE(d.contours.size() == 2);
        CHECK(std::holds_alternative<ArcSpan>(d.contours[0].contour.spans[0]));
        CHECK(isNear(d.contours[0].contour.endPoint(), {20.0, 50.0}));
    }
    SECTION("radii too small are scaled up to a half circle") {
        const auto d = loadPage("<path d=\"M10 50 A1 1 0 0 1 30 50\"/>");
        const auto* a = std::get_if<ArcSpan>(&d.contours[0].contour.spans[0]);
        REQUIRE(a != nullptr);
        CHECK(a->radius == Approx(10.0));
        CHECK(std::fabs(a->sweepAngle()) == Approx(kPi));
    }
    SECTION("negative radii use their absolute value; zero radius makes a line") {
        const auto d = loadPage("<path d=\"M10 50 A-5 -5 0 0 1 20 50\"/><path d=\"M10 50 A0 5 0 0 1 20 50\"/>");
        REQUIRE(d.contours.size() == 2);
        CHECK(std::holds_alternative<ArcSpan>(d.contours[0].contour.spans[0]));
        CHECK(std::holds_alternative<LineSpan>(d.contours[1].contour.spans[0]));
    }
    SECTION("an arc to the current point is omitted") {
        const auto d = loadPage("<path d=\"M10 50 A5 5 0 0 1 10 50 L20 50\"/>");
        REQUIRE(d.contours[0].contour.spans.size() == 1);
        CHECK(std::holds_alternative<LineSpan>(d.contours[0].contour.spans[0]));
    }
    SECTION("rotated ellipse arcs become Beziers within the tolerance") {
        SvgImportOptions o;
        o.tolerance.chordal = 0.001;
        const auto d = load(page("<path d=\"M30 50 A20 10 30 1 1 70 50\"/>"), o);
        const Contour& c = d.contours[0].contour;
        REQUIRE(!c.spans.empty());
        for (const Span& s : c.spans) {
            CHECK(std::holds_alternative<BezierSpan>(s));
        }
        CHECK(isNear(c.startPoint(), {30.0, 50.0}, 1e-9));
        CHECK(isNear(c.endPoint(), {70.0, 50.0}, 1e-9));
        SvgImportOptions fine;
        fine.tolerance.chordal = 1e-6;
        const auto ref = load(page("<path d=\"M30 50 A20 10 30 1 1 70 50\"/>"), fine);
        CHECK(outlineDistance(c, ref.contours[0].contour) < 0.002);
    }
    SECTION("a huge radius becomes a line instead of overflowing") {
        const auto d = loadPage("<path d=\"M10 50 A1e300 1e300 0 0 1 20 50\"/>");
        REQUIRE(d.contours.size() == 1);
        CHECK(std::holds_alternative<LineSpan>(d.contours[0].contour.spans[0]));
        CHECK(hasWarning(d, "arcs with unusable radii"));
    }
    SECTION("a large but legal radius stays an arc") {
        const auto d = loadPage("<path d=\"M10 50 A1e6 1e6 0 0 1 20 50\"/>");
        const auto* a = std::get_if<ArcSpan>(&d.contours[0].contour.spans[0]);
        REQUIRE(a != nullptr);
        CHECK(a->radius == Approx(1e6).epsilon(1e-6));
    }
}

TEST_CASE("svg import: path errors keep what came before", "[svg][io]") {
    SECTION("garbage in the middle") {
        const auto d = loadPage("<path d=\"M10 10 L20 10 L30 x L40 40\"/>");
        REQUIRE(d.contours.size() == 1);
        CHECK(d.contours[0].contour.spans.size() == 1);
        CHECK(hasWarning(d, "path data error"));
    }
    SECTION("overflowing numbers are errors, underflow is zero") {
        auto d = loadPage("<path d=\"M10 10 L1e999 5\"/>");
        CHECK(d.contours.empty());
        CHECK(hasWarning(d, "path data error"));
        d = loadPage("<path d=\"M10 10 L1e-999 5\"/>");
        REQUIRE(d.contours.size() == 1);
        CHECK(isNear(d.contours[0].contour.endPoint(), {0.0, 95.0}));
        d = loadPage("<path d=\"M10 10 L0.0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001 5\"/>");
        REQUIRE(d.contours.size() == 1);
    }
    SECTION("NaN, Infinity and stray commas") {
        CHECK(loadPage("<path d=\"M NaN 5 L 10 10\"/>").contours.empty());
        CHECK(loadPage("<path d=\"M 5 5 L Infinity 10\"/>").contours.empty());
        CHECK(loadPage("<path d=\"M,5 5 L 10 10\"/>").contours.empty());
        CHECK(loadPage("<path d=\"L 5 5 L 10 10\"/>").contours.empty());
        CHECK(loadPage("<path d=\"M 5 5 L 10\"/>").contours.empty());
        CHECK(loadPage("<path d=\"M 5 5 Z 10 10\"/>").contours.empty());
    }
    SECTION("bad arc flags") {
        const auto d = loadPage("<path d=\"M10 10 L20 10 A5 5 0 2 1 30 10\"/>");
        REQUIRE(d.contours.size() == 1);
        CHECK(d.contours[0].contour.spans.size() == 1);
        CHECK(hasWarning(d, "arc flag"));
    }
    SECTION("coordinates beyond the supported range drop the contour") {
        const auto d = loadPage("<path d=\"M0 0 L1e300 5\"/><path d=\"M0 0 L5 5\"/>");
        REQUIRE(d.contours.size() == 1);
        CHECK(hasWarning(d, "out-of-range"));
    }
}

TEST_CASE("svg import: transforms", "[svg][io]") {
    SECTION("each transform function") {
        // Point (10,0) of the path M0 0 L10 0 under each transform; Y flips at the end (y_up = 100 - y).
        struct Case {
            const char* t;
            Vec2 end;
        };
        const Case cases[] = {
            {"translate(5 7)", {15.0, 93.0}},
            {"translate(5)", {15.0, 100.0}},
            {"scale(2)", {20.0, 100.0}},
            {"scale(2 3)", {20.0, 100.0}},
            {"rotate(90)", {0.0, 90.0}},
            {"rotate(90 10 0)", {10.0, 100.0}},
            {"matrix(1 0 0 1 3 4)", {13.0, 96.0}},
            {"matrix(0 1 -1 0 0 0)", {0.0, 90.0}},
            {"skewX(45)", {10.0, 100.0}},
            {"skewY(45)", {10.0, 90.0}},
            {"translate(1,2) scale(2)", {21.0, 98.0}},
            {"scale(2) translate(1,2)", {22.0, 96.0}},
        };
        for (const Case& c : cases) {
            INFO(c.t);
            const auto d = loadPage(std::string("<path transform=\"") + c.t + "\" d=\"M0 0 L10 0\"/>");
            REQUIRE(d.contours.size() == 1);
            CHECK(isNear(d.contours[0].contour.endPoint(), c.end, 1e-9));
        }
    }
    SECTION("skew moves points along the other axis") {
        const auto d = loadPage("<path transform=\"skewX(45)\" d=\"M0 10 L0 0\"/>");
        CHECK(isNear(d.contours[0].contour.startPoint(), {10.0, 90.0}, 1e-9));
    }
    SECTION("nested groups multiply outermost first") {
        const auto d = loadPage(
            "<g transform=\"translate(10 0)\"><g transform=\"scale(2)\"><path d=\"M1 0 L2 0\"/></g></g>");
        CHECK(isNear(d.contours[0].contour.startPoint(), {12.0, 100.0}));
        CHECK(isNear(d.contours[0].contour.endPoint(), {14.0, 100.0}));
    }
    SECTION("the transforms fixture") {
        const auto r = importSvg(readFixture("transforms.svg"));
        REQUIRE(r.ok());
        const SvgDrawing& d = r.value();
        CHECK(d.widthMm == Approx(50.8));
        CHECK(d.heightMm == Approx(25.4));
        REQUIRE(d.contours.size() == 3);
        // 200 user units span 50.8 mm: 0.254 mm per unit; first path ends rotated 90 degrees, scaled 2, moved.
        const double u = 0.254;
        CHECK(isNear(d.contours[0].contour.startPoint(), {10.0 * u, 25.4 - 20.0 * u}, 1e-9));
        CHECK(isNear(d.contours[0].contour.pointAt(0.0), {10.0 * u, 25.4 - 20.0 * u}, 1e-9));
        // (10,0) --rotate90--> (0,10) --scale2--> (0,20) --translate--> (10,40)
        const auto& s0 = d.contours[0].contour.spans[0];
        CHECK(isNear(endPoint(s0), {10.0 * u, 25.4 - 40.0 * u}, 1e-9));
        // matrix(1 0 0.5 1 0 0): (100,50) -> (125, 50)
        CHECK(isNear(d.contours[1].contour.startPoint(), {125.0 * u, 25.4 - 50.0 * u}, 1e-9));
        // skewX(45): (0,80) -> (80,80)
        CHECK(isNear(d.contours[2].contour.startPoint(), {80.0 * u, 25.4 - 80.0 * u}, 1e-9));
    }
    SECTION("a conformal transform keeps circles as arcs") {
        const auto d = loadPage("<g transform=\"rotate(30) scale(2)\"><circle r=\"3\" cx=\"4\"/></g>");
        const auto* a = std::get_if<ArcSpan>(&d.contours[0].contour.spans[0]);
        REQUIRE(a != nullptr);
        CHECK(a->radius == Approx(6.0));
    }
    SECTION("a non-uniform scale turns circles into Beziers within tolerance") {
        const auto d = loadPage("<g transform=\"scale(2 1)\"><circle r=\"10\" cx=\"20\" cy=\"50\"/></g>");
        const Contour& c = d.contours[0].contour;
        REQUIRE(!c.spans.empty());
        CHECK(std::holds_alternative<BezierSpan>(c.spans[0]));
        const Box2 b = c.bounds();
        CHECK(b.min.x == Approx(20.0).margin(0.01));
        CHECK(b.max.x == Approx(60.0).margin(0.01));
    }
    SECTION("an invalid transform skips the element with a warning") {
        auto d = loadPage("<path transform=\"scale(\" d=\"M0 0 L1 1\"/><path transform=\"foo(1)\" d=\"M0 0 L1 1\"/>");
        CHECK(d.contours.empty());
        CHECK(hasWarning(d, "invalid transform"));
        d = loadPage("<path transform=\"scale(1e300)\" d=\"M0 0 L1 1\"/>");
        CHECK(d.contours.empty());
        d = loadPage("<path transform=\"skewX(90)\" d=\"M0 0 L1 1\"/>");
        CHECK(d.contours.empty());
    }
    SECTION("a mirroring transform reverses arc direction but not geometry") {
        const auto a = loadPage("<path d=\"M10 50 a5 5 0 0 1 10 0\"/>");
        const auto b = loadPage("<path transform=\"translate(30 0) scale(-1 1)\" d=\"M10 50 a5 5 0 0 1 10 0\"/>");
        const auto* aa = std::get_if<ArcSpan>(&a.contours[0].contour.spans[0]);
        const auto* ab = std::get_if<ArcSpan>(&b.contours[0].contour.spans[0]);
        REQUIRE(aa != nullptr);
        REQUIRE(ab != nullptr);
        CHECK(aa->direction != ab->direction);
        CHECK(isNear(ab->startPoint(), {20.0, 50.0}, 1e-9));
    }
}

TEST_CASE("svg import: viewBox and units", "[svg][io]") {
    auto size = [](const std::string& attrs) {
        return load("<svg xmlns=\"http://www.w3.org/2000/svg\" " + attrs + "><path d=\"M10 10 L20 10\"/></svg>");
    };
    SECTION("unit suffixes") {
        auto d = size("width=\"10cm\" height=\"5cm\"");
        CHECK(d.widthMm == Approx(100.0));
        CHECK(d.heightMm == Approx(50.0));
        d = size("width=\"1in\" height=\"72pt\"");
        CHECK(d.widthMm == Approx(25.4));
        CHECK(d.heightMm == Approx(25.4));
        d = size("width=\"6pc\" height=\"96px\"");
        CHECK(d.widthMm == Approx(25.4));
        CHECK(d.heightMm == Approx(25.4));
        d = size("width=\"96\" height=\"48\"");
        CHECK(d.widthMm == Approx(25.4));
        CHECK(d.heightMm == Approx(12.7));
        // The path M10 10 L20 10 in px: x = 10 px = 2.6458 mm.
        CHECK(isNear(d.contours[0].contour.startPoint(), {10.0 * 25.4 / 96.0, 12.7 - 10.0 * 25.4 / 96.0}, 1e-9));
    }
    SECTION("viewBox scales to the declared size") {
        const auto d = size("width=\"50mm\" height=\"50mm\" viewBox=\"0 0 100 100\"");
        CHECK(isNear(d.contours[0].contour.startPoint(), {5.0, 45.0}, 1e-9));
        CHECK(isNear(d.contours[0].contour.endPoint(), {10.0, 45.0}, 1e-9));
    }
    SECTION("viewBox without size uses pixels") {
        const auto d = size("viewBox=\"0 0 96 96\"");
        CHECK(d.widthMm == Approx(25.4));
        CHECK(d.heightMm == Approx(25.4));
    }
    SECTION("viewBox origin offsets the geometry") {
        const auto d = size("width=\"100mm\" height=\"100mm\" viewBox=\"10 10 100 100\"");
        CHECK(isNear(d.contours[0].contour.startPoint(), {0.0, 100.0}, 1e-9));
    }
    SECTION("only one of width/height: the other follows the aspect ratio") {
        const auto d = size("width=\"100mm\" viewBox=\"0 0 200 100\"");
        CHECK(d.heightMm == Approx(50.0));
    }
    SECTION("preserveAspectRatio") {
        auto d = size("width=\"100mm\" height=\"50mm\" viewBox=\"0 0 50 50\"");  // meet, xMidYMid
        CHECK(isNear(d.contours[0].contour.startPoint(), {25.0 + 10.0, 50.0 - 10.0}, 1e-9));
        d = size("width=\"100mm\" height=\"50mm\" viewBox=\"0 0 50 50\" preserveAspectRatio=\"xMinYMin meet\"");
        CHECK(isNear(d.contours[0].contour.startPoint(), {10.0, 40.0}, 1e-9));
        d = size("width=\"100mm\" height=\"50mm\" viewBox=\"0 0 50 50\" preserveAspectRatio=\"none\"");
        CHECK(isNear(d.contours[0].contour.startPoint(), {20.0, 40.0}, 1e-9));
        d = size("width=\"100mm\" height=\"50mm\" viewBox=\"0 0 50 50\" preserveAspectRatio=\"xMinYMin slice\"");
        CHECK(isNear(d.contours[0].contour.startPoint(), {20.0, 30.0}, 1e-9));
    }
    SECTION("percentages and nonsense sizes fall back with a warning") {
        auto d = size("width=\"100%\" height=\"100%\" viewBox=\"0 0 96 96\"");
        CHECK(d.widthMm == Approx(25.4));
        d = size("width=\"-5\" height=\"abc\" viewBox=\"0 0 96 96\"");
        CHECK(d.widthMm == Approx(25.4));
        CHECK(hasWarning(d, "invalid width"));
        d = size("viewBox=\"0 0 0 10\"");
        CHECK(hasWarning(d, "invalid viewBox"));
        d = size("");
        CHECK(hasWarning(d, "neither a size nor a viewBox"));
        CHECK(d.widthMm == 0.0);
    }
    SECTION("the dpi option") {
        SvgImportOptions o;
        o.dpi = 72.0;
        const auto d = load("<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"72\" height=\"72\"/>", o);
        CHECK(d.widthMm == Approx(25.4));
    }
    SECTION("lengths with units inside shapes") {
        const auto d = loadPage("<rect x=\"1cm\" y=\"0\" width=\"1cm\" height=\"5\"/>");
        // Absolute units are CSS-based: 1cm is 96/2.54 user units (here: mm).
        const double cm = 96.0 / 2.54;
        CHECK(isNear(d.contours[0].contour.bounds().min, {cm, 95.0}, 1e-9));
        CHECK(isNear(d.contours[0].contour.bounds().max, {2.0 * cm, 100.0}, 1e-9));
    }
    SECTION("percentage lengths refer to the viewBox") {
        const auto d = loadPage("<rect x=\"10%\" y=\"0\" width=\"50%\" height=\"5\"/>");
        CHECK(isNear(d.contours[0].contour.bounds().min, {10.0, 95.0}, 1e-9));
        CHECK(isNear(d.contours[0].contour.bounds().max, {60.0, 100.0}, 1e-9));
    }
    SECTION("data-origin-mm shifts the result") {
        const auto d = loadPage("<path d=\"M10 10 L20 10\"/>", "data-origin-mm=\"-5 7\"");
        CHECK(isNear(d.contours[0].contour.startPoint(), {5.0, 97.0}, 1e-9));
    }
}

TEST_CASE("svg import: rect corner radii", "[svg][io]") {
    SECTION("only rx given: ry follows") {
        const auto d = loadPage("<rect x=\"10\" y=\"10\" width=\"30\" height=\"20\" rx=\"4\"/>");
        const Contour& c = d.contours[0].contour;
        CHECK(c.spans.size() == 8);
        const auto* a = std::get_if<ArcSpan>(&c.spans[1]);
        REQUIRE(a != nullptr);
        CHECK(a->radius == Approx(4.0));
    }
    SECTION("radii are clamped to half the size") {
        const auto d = loadPage("<rect x=\"10\" y=\"10\" width=\"10\" height=\"10\" rx=\"50\"/>");
        const Contour& c = d.contours[0].contour;
        CHECK(c.bounds().max.x - c.bounds().min.x == Approx(10.0));
        for (const Span& s : c.spans) {
            if (const auto* a = std::get_if<ArcSpan>(&s)) {
                CHECK(a->radius == Approx(5.0));
            }
        }
    }
    SECTION("zero radius gives a plain rectangle") {
        CHECK(loadPage("<rect width=\"10\" height=\"10\" rx=\"0\" ry=\"3\"/>").contours[0].contour.spans.size() == 4);
    }
    SECTION("invalid and empty rectangles") {
        auto d = loadPage("<rect width=\"-1\" height=\"10\"/>");
        CHECK(d.contours.empty());
        CHECK(hasWarning(d, "invalid <rect>"));
        d = loadPage("<rect width=\"0\" height=\"10\"/><rect height=\"10\"/><circle r=\"0\"/>");
        CHECK(d.contours.empty());
    }
    SECTION("circle with bad radius") {
        auto d = loadPage("<circle r=\"-2\"/><ellipse rx=\"3\" ry=\"abc\"/>");
        CHECK(d.contours.empty());
        CHECK(hasWarning(d, "invalid <circle>"));
        CHECK(hasWarning(d, "invalid <ellipse>"));
    }
    SECTION("odd point lists") {
        const auto d = loadPage("<polyline points=\"0,0 10,0 10\"/><polyline points=\"1,2\"/><polygon points=\"0 0 5 0 5 5 x\"/>");
        REQUIRE(d.contours.size() == 2);
        CHECK(d.contours[0].contour.spans.size() == 1);
        CHECK(d.contours[1].contour.closed);
        CHECK(hasWarning(d, "points list"));
    }
}

TEST_CASE("svg import: layers, visibility and use", "[svg][io]") {
    SECTION("the layers fixture") {
        auto r = importSvg(readFixture("layers.svg"));
        REQUIRE(r.ok());
        const SvgDrawing& d = r.value();
        REQUIRE(d.layers.size() == 3);
        CHECK(d.layers[0].name == "Outline");
        CHECK(d.layers[1].name == "Holes");
        CHECK(d.layers[2].name == "0");
        // Outline: rect + circle; Holes: two uses of the tile; the hidden layer, defs, text, image
        // and the clipped rect (outside any layer: default layer "0" would be a third layer).
        std::size_t outline = 0;
        std::size_t holes = 0;
        std::size_t other = 0;
        for (const SvgContour& c : d.contours) {
            (c.layer == 0 ? outline : (c.layer == 1 ? holes : other)) += 1;
        }
        CHECK(outline == 2);
        CHECK(holes == 2);
        CHECK(other == 1);  // the clipped rectangle, on layer "0"
    }
    SECTION("warnings are reported per skipped element type") {
        auto r = importSvg(readFixture("layers.svg"));
        REQUIRE(r.ok());
        const SvgDrawing& d = r.value();
        CHECK(hasWarning(d, "<text> element skipped"));
        CHECK(hasWarning(d, "<image> element skipped"));
        CHECK(hasWarning(d, "<linearGradient> element skipped"));
        CHECK(hasWarning(d, "<clipPath> element skipped"));
        CHECK(hasWarning(d, "clip-path attribute ignored"));
    }
    SECTION("top-level groups with an id become layers, nested groups inherit") {
        const auto d = loadPage(
            "<path d=\"M0 0 L1 1\"/><g id=\"top\"><g id=\"inner\"><path d=\"M0 0 L2 2\"/></g></g><g><path d=\"M0 0 L3 3\"/></g>");
        REQUIRE(d.layers.size() == 2);
        CHECK(d.layers[0].name == "0");
        CHECK(d.layers[1].name == "top");
        REQUIRE(d.contours.size() == 3);
        CHECK(d.contours[0].layer == 0);
        CHECK(d.contours[1].layer == 1);
        CHECK(d.contours[2].layer == 0);
    }
    SECTION("same layer name merges; unnamed Inkscape layers get a name") {
        const auto d = loadPage(
            "<g inkscape:groupmode=\"layer\" inkscape:label=\"A\"><path d=\"M0 0 L1 1\"/></g>"
            "<g inkscape:groupmode=\"layer\" inkscape:label=\"A\"><path d=\"M0 0 L2 2\"/></g>"
            "<g inkscape:groupmode=\"layer\"><path d=\"M0 0 L2 3\"/></g>");
        REQUIRE(d.layers.size() == 2);
        CHECK(d.contours[0].layer == d.contours[1].layer);
        CHECK(d.layers[1].name == "Layer 2");
    }
    SECTION("a label wins over the id") {
        const auto d = loadPage("<g id=\"x\" inkscape:label=\"Label\"><path d=\"M0 0 L1 1\"/></g>");
        CHECK(d.layers[0].name == "Label");
    }
    SECTION("hidden elements") {
        const auto d = loadPage(
            "<path display=\"none\" d=\"M0 0 L1 1\"/>"
            "<path style=\"fill:none; display : none ;stroke:red\" d=\"M0 0 L2 2\"/>"
            "<g style=\"display:none\"><path d=\"M0 0 L3 3\"/></g>"
            "<g visibility=\"hidden\"><path d=\"M0 0 L4 4\"/><path visibility=\"visible\" d=\"M0 0 L5 5\"/></g>"
            "<path style=\"display:inline\" display=\"none\" d=\"M0 0 L6 6\"/>");
        REQUIRE(d.contours.size() == 2);
        CHECK(isNear(d.contours[0].contour.endPoint(), {5.0, 95.0}));
        CHECK(isNear(d.contours[1].contour.endPoint(), {6.0, 94.0}));
        CHECK(d.warnings.empty());
    }
    SECTION("use of a defs element, symbol, group; with transform and offset") {
        const auto d = loadPage(
            "<defs><path id=\"p\" d=\"M0 0 L1 0\"/><g id=\"g\"><path d=\"M0 0 L0 1\"/><path d=\"M0 0 L0 2\"/></g>"
            "<symbol id=\"s\"><path d=\"M0 0 L3 0\"/></symbol></defs>"
            "<use xlink:href=\"#p\" x=\"10\" y=\"20\"/>"
            "<use href=\"#g\" transform=\"translate(50 50)\"/>"
            "<use href=\"#s\" x=\"5\"/>"
            "<use href=\"#p\" transform=\"scale(10)\" x=\"1\"/>");
        REQUIRE(d.contours.size() == 5);
        CHECK(isNear(d.contours[0].contour.startPoint(), {10.0, 80.0}));
        CHECK(isNear(d.contours[1].contour.startPoint(), {50.0, 50.0}));
        CHECK(isNear(d.contours[3].contour.endPoint(), {8.0, 100.0}));
        CHECK(isNear(d.contours[4].contour.startPoint(), {10.0, 100.0}));
        CHECK(isNear(d.contours[4].contour.endPoint(), {20.0, 100.0}));
    }
    SECTION("symbols and defs are not drawn directly") {
        CHECK(loadPage("<defs><path d=\"M0 0 L1 1\"/></defs><symbol><path d=\"M0 0 L1 1\"/></symbol>").contours.empty());
    }
    SECTION("use problems") {
        auto d = loadPage("<use href=\"#missing\"/><use href=\"http://example.com/a.svg#x\"/><use/>");
        CHECK(d.contours.empty());
        CHECK(hasWarning(d, "unknown id"));
        CHECK(hasWarning(d, "external"));
        d = loadPage("<g id=\"a\"><path d=\"M0 0 L1 1\"/><use href=\"#a\"/></g>");
        REQUIRE(d.contours.size() == 1);
        CHECK(hasWarning(d, "refers to itself"));
        d = loadPage("<use id=\"u\" href=\"#u\"/>");
        CHECK(d.contours.empty());
        d = loadPage("<g id=\"a\"><use href=\"#b\"/></g><g id=\"b\"><use href=\"#a\"/></g><use href=\"#a\"/>");
        CHECK(d.contours.empty());
    }
    SECTION("exponential use expansion is cut off") {
        std::string defs = "<defs><path id=\"l0\" d=\"M0 0 L1 1\"/>";
        for (int i = 1; i <= 12; ++i) {
            defs += "<g id=\"l" + std::to_string(i) + "\">";
            for (int k = 0; k < 8; ++k) {
                defs += "<use href=\"#l" + std::to_string(i - 1) + "\"/>";
            }
            defs += "</g>";
        }
        defs += "</defs><use href=\"#l12\"/>";
        SvgImportOptions o;
        o.maxUseDepth = 64;
        auto r = importText(page(defs), o);
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().code == SvgError::Code::ResourceLimit);
        // With the default use depth the nesting itself is refused with a warning.
        r = importText(page(defs));
        if (r.ok()) {
            CHECK(hasWarning(r.value(), "nests too deeply"));
        } else {
            CHECK(r.error().code == SvgError::Code::ResourceLimit);
        }
    }
    SECTION("skipped and unknown elements") {
        const auto d = loadPage(
            "<metadata/><desc>d</desc><foo/><sodipodi:namedview xmlns:sodipodi=\"x\"/><switch/><style>a{}</style>"
            "<a><path d=\"M0 0 L1 1\"/></a>");
        CHECK(d.contours.size() == 1);
        CHECK(hasWarning(d, "unknown element <foo>"));
        CHECK(hasWarning(d, "<switch>"));
        CHECK(hasWarning(d, "<style>"));
        CHECK_FALSE(hasWarning(d, "metadata"));
        CHECK_FALSE(hasWarning(d, "namedview"));
    }
    SECTION("svg-prefixed element names") {
        const auto d = load(
            "<svg:svg xmlns:svg=\"http://www.w3.org/2000/svg\" width=\"10mm\" height=\"10mm\"><svg:path d=\"M0 0 L1 1\"/></svg:svg>");
        CHECK(d.contours.size() == 1);
    }
    SECTION("repeated warnings merge with a count") {
        const auto d = loadPage("<text/><text/><text/>");
        REQUIRE(d.warnings.size() == 1);
        CHECK(d.warnings[0].count == 3);
        CHECK(d.warnings[0].describe().find("(x3)") != std::string::npos);
    }
}

TEST_CASE("svg import: the px fixture without viewBox", "[svg][io]") {
    const auto r = importSvg(readFixture("px_arcs.svg"));
    REQUIRE(r.ok());
    const Contour& c = r.value().contours[0].contour;
    CHECK(r.value().widthMm == Approx(50.8));
    CHECK(c.closed);
    REQUIRE(c.spans.size() == 4);
    const double u = 25.4 / 96.0;
    CHECK(isNear(c.startPoint(), {48.0 * u, 25.4}, 1e-9));
    const auto* a = std::get_if<ArcSpan>(&c.spans[0]);
    REQUIRE(a != nullptr);
    CHECK(a->radius == Approx(48.0 * u));
    CHECK(isNear(a->centre, {48.0 * u, 25.4 - 48.0 * u}, 1e-9));
}

TEST_CASE("svg import: determinism and limits", "[svg][io]") {
    const Bytes in = readFixture("shapes.svg");
    const auto a = importSvg(in);
    const auto b = importSvg(in);
    REQUIRE(a.ok());
    REQUIRE(b.ok());
    CHECK(a.value().contours == b.value().contours);
    CHECK(a.value().layers == b.value().layers);

    SvgImportOptions o;
    o.maxSpans = 5;
    const auto r = importSvg(in, o);
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == SvgError::Code::ResourceLimit);

    o = SvgImportOptions{};
    o.maxLayers = 1;
    const auto lr = importText(page("<g id=\"a\"><path d=\"M0 0 L1 1\"/></g><g id=\"b\"><path d=\"M0 0 L1 1\"/></g>"), o);
    REQUIRE_FALSE(lr.ok());

    o = SvgImportOptions{};
    o.dpi = 0.0;
    CHECK_FALSE(importText(page(""), o).ok());
}

TEST_CASE("svg import: closing Z uses the distance tolerance", "[svg][io]") {
    SECTION("near-closed hand-written path adds no sliver") {
        const auto d = loadPage("<path d=\"M10 10 H30 V30 H10 V10.000001 Z\"/>");
        REQUIRE(d.contours.size() == 1);
        CHECK(d.contours[0].contour.closed);
        CHECK(d.contours[0].contour.spans.size() == 4);
    }
    SECTION("a point beyond the tolerance still gets its closing line") {
        // Default distance tolerance is 0.001 mm; 0.002 is twice that.
        const auto d = loadPage("<path d=\"M10 10 H30 V30 H10 V10.002 Z\"/>");
        REQUIRE(d.contours.size() == 1);
        CHECK(d.contours[0].contour.spans.size() == 5);
        const auto& last = d.contours[0].contour.spans.back();
        CHECK(std::holds_alternative<LineSpan>(last));
        CHECK(isNear(endPoint(last), {10.0, 90.0}, 1e-9));
    }
    SECTION("the tolerance option is honoured") {
        SvgImportOptions o;
        o.tolerance.distance = 0.01;
        const auto d = load(page("<path d=\"M10 10 H30 V30 H10 V10.002 Z\"/>"), o);
        CHECK(d.contours[0].contour.spans.size() == 4);
    }
    SECTION("two half arcs and a four-Bezier ellipse close without a sliver") {
        const auto a = loadPage("<path d=\"M10 50 A10 10 0 0 1 30 50 A10 10 0 0 1 10 50 Z\"/>");
        REQUIRE(a.contours.size() == 1);
        CHECK(a.contours[0].contour.spans.size() == 2);
        const auto b = loadPage("<path d=\"M10 50 A10 5 0 0 1 30 50 A10 5 0 0 1 10 50 Z\"/>");
        REQUIRE(b.contours.size() == 1);
        for (const Span& s : b.contours[0].contour.spans) {
            CHECK(std::holds_alternative<BezierSpan>(s));
        }
        const auto e = loadPage("<ellipse cx=\"50\" cy=\"50\" rx=\"20\" ry=\"10\"/>");
        REQUIRE(e.contours.size() == 1);
        CHECK(e.contours[0].contour.spans.size() == 4);
    }
}

TEST_CASE("svg import: smaller diagnostics", "[svg][io]") {
    SECTION("UTF-16 input") {
        Bytes b{0xFF, 0xFE, '<', 0, 's', 0, 'v', 0, 'g', 0, '/', 0, '>', 0};
        auto r = importSvg(b);
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().message == "UTF-16 input is not supported (convert to UTF-8)");
        b = {'<', 0, 's', 0, 'v', 0, 'g', 0};
        r = importSvg(b);
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().message.find("UTF-16") != std::string::npos);
        b = {0, '<', 0, 's'};
        CHECK(importSvg(b).error().message.find("UTF-16") != std::string::npos);
    }
    SECTION("ellipse with only one radius") {
        auto d = loadPage("<ellipse rx=\"5\"/><ellipse ry=\"5\"/>");
        CHECK(d.contours.empty());
        REQUIRE(d.warnings.size() == 1);
        CHECK(d.warnings[0].count == 2);
        CHECK(hasWarning(d, "only one of rx and ry"));
    }
    SECTION("zero scale") {
        const auto d = loadPage("<path transform=\"scale(0)\" d=\"M0 0 L1 1\"/><g transform=\"matrix(0 0 0 0 1 1)\"><path d=\"M0 0 L1 1\"/></g>");
        CHECK(d.contours.empty());
        REQUIRE(d.warnings.size() == 2);
        CHECK(hasWarning(d, "non-invertible"));
    }
    SECTION("default caps") {
        const SvgImportOptions o;
        CHECK(o.maxSpans == 1'000'000);
        CHECK(o.maxInputBytes == 32U * 1024U * 1024U);
    }
}

TEST_CASE("svg import: the closing tolerance is evaluated in mm after the transform", "[svg][io]") {
    auto spans = [](const std::string& svg, const SvgImportOptions& o = {}) {
        const auto d = load(svg, o);
        REQUIRE(d.contours.size() == 1);
        return d.contours[0].contour.spans.size();
    };
    const std::string tail = " Z\"/></svg>";
    SECTION("scaled viewBox: 10 units over 100 mm") {
        auto doc = [&](const char* off) {
            return std::string("<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"100mm\" height=\"100mm\" viewBox=\"0 0 10 10\">"
                               "<path d=\"M1 1 H3 V3 H1 V") + off + tail;
        };
        CHECK(spans(doc("1.00009")) == 4);  // 0.9e-3 mm: closes without a line
        CHECK(spans(doc("1.0002")) == 5);   // 2e-3 mm, although only 2e-4 user units
    }
    SECTION("px units: 1 px is 0.2646 mm") {
        auto doc = [&](const char* off) {
            return std::string("<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"96\" height=\"96\">"
                               "<path d=\"M10 10 H30 V30 H10 V") + off + tail;
        };
        CHECK(spans(doc("10.0037")) == 4);  // 0.00098 mm
        CHECK(spans(doc("10.004")) == 5);   // 0.00106 mm
    }
    SECTION("exact boundary: equal closes, just above gets a line") {
        SvgImportOptions o;
        o.tolerance.distance = 0.5;
        auto doc = [&](const char* off) {
            return page(std::string("<path d=\"M10 10 H30 V30 H10 V") + off + " Z\"/>");
        };
        CHECK(spans(doc("10.5"), o) == 4);
        CHECK(spans(doc("10.5000001"), o) == 5);
    }
}
