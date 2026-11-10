// Dimensions (T-0246): measuring each kind exactly, text options, drawing geometry, associativity
// under scale/move/mirror, undo, and the .carv round trip.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "carv_zip.hpp"
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/dimension.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"

using namespace carvetto;
using namespace carvetto::document;
using Catch::Approx;
using geometry::Vec2;

namespace {

constexpr double kPi = geometry::kPi;

DimensionObject make(DimensionKind kind, Vec2 a, Vec2 b, Vec2 c = {}) {
    DimensionObject d;
    d.kind = kind;
    d.points[0].pos = a;
    d.points[1].pos = b;
    d.points[2].pos = c;
    return d;
}

double measured(const Document& doc, const DimensionObject& d) {
    const auto v = measureDimension(doc, d);
    REQUIRE(v.has_value());
    return *v;
}

geometry::Box2 rectBox(double x0, double y0, double x1, double y1) {
    geometry::Box2 b;
    b.expand(Vec2{x0, y0});
    b.expand(Vec2{x1, y1});
    return b;
}

// A 40 x 30 rectangle with its lower left corner at (10, 20); returns its id.
ObjectId addRect(Document& doc) {
    return doc.insertObject(Object::vector(geometry::Contour::rectangle(rectBox(10, 20, 50, 50)),
                                           doc.activeLayer(), doc.activeSheet()));
}

}  // namespace

TEST_CASE("each kind measures a known figure exactly", "[dimension][document]") {
    Document doc;
    CHECK(measured(doc, make(DimensionKind::Aligned, {0, 0}, {3, 4})) == Approx(5.0).margin(1e-12));
    DimensionObject lin = make(DimensionKind::Linear, {1, 1}, {4, 5});
    CHECK(measured(doc, lin) == Approx(3.0).margin(1e-12));
    lin.vertical = true;
    CHECK(measured(doc, lin) == Approx(4.0).margin(1e-12));
    CHECK(measured(doc, make(DimensionKind::Angular, {1, 1}, {6, 1}, {1, 6})) == Approx(kPi / 2).margin(1e-12));
    CHECK(measured(doc, make(DimensionKind::Angular, {0, 0}, {1, 0}, {1, 1})) == Approx(kPi / 4).margin(1e-12));
    // The smaller angle whichever arm is picked first.
    CHECK(measured(doc, make(DimensionKind::Angular, {0, 0}, {1, 1}, {1, 0})) == Approx(kPi / 4).margin(1e-12));
    CHECK(measured(doc, make(DimensionKind::Angular, {0, 0}, {1, 0}, {-1, 0})) == Approx(kPi).margin(1e-12));
    DimensionObject rad = make(DimensionKind::Radial, {2, 3}, {7, 3});
    CHECK(measured(doc, rad) == Approx(5.0).margin(1e-12));
    rad.diameter = true;
    CHECK(measured(doc, rad) == Approx(10.0).margin(1e-12));
}

TEST_CASE("degenerate dimensions measure nothing", "[dimension][document]") {
    Document doc;
    CHECK_FALSE(measureDimension(doc, make(DimensionKind::Aligned, {1, 1}, {1, 1})));
    CHECK_FALSE(measureDimension(doc, make(DimensionKind::Linear, {0, 0}, {0, 5})));  // no X extent
    CHECK_FALSE(measureDimension(doc, make(DimensionKind::Angular, {0, 0}, {0, 0}, {1, 0})));
    CHECK_FALSE(measureDimension(doc, make(DimensionKind::Angular, {0, 0}, {1, 0}, {2, 0})));  // 0 degrees
    CHECK_FALSE(measureDimension(doc, make(DimensionKind::Radial, {2, 2}, {2, 2})));
    CHECK_FALSE(measureDimension(doc, make(DimensionKind::Aligned, {0, 0}, {std::nan(""), 1})));
    DimensionObject d = make(DimensionKind::Aligned, {1, 1}, {1, 1});
    CHECK(dimensionText(doc, d, Units::Millimetres).empty());
    CHECK_FALSE(dimensionGeometry(doc, d).valid);
}

TEST_CASE("dimension text honours decimals, units, custom text", "[dimension][document]") {
    Document doc;
    DimensionObject d = make(DimensionKind::Aligned, {0, 0}, {25.4, 0});
    CHECK(dimensionText(doc, d, Units::Millimetres) == "25.40");
    CHECK(dimensionText(doc, d, Units::Inches) == "1.00");
    d.decimals = 0;
    CHECK(dimensionText(doc, d, Units::Millimetres) == "25");
    d.decimals = 3;
    CHECK(dimensionText(doc, d, Units::Millimetres) == "25.400");

    DimensionObject ang = make(DimensionKind::Angular, {0, 0}, {1, 0}, {0, 1});
    CHECK(dimensionText(doc, ang, Units::Millimetres) == "90.00\xC2\xB0");
    ang.angleDecimals = 0;
    CHECK(dimensionText(doc, ang, Units::Millimetres) == "90\xC2\xB0");
    ang.angleDecimals = 6;
    CHECK(dimensionText(doc, ang, Units::Inches) == "90.000000\xC2\xB0");  // angles ignore display units

    DimensionObject rad = make(DimensionKind::Radial, {0, 0}, {5, 0});
    CHECK(dimensionText(doc, rad, Units::Millimetres) == "R5.00");
    rad.diameter = true;
    CHECK(dimensionText(doc, rad, Units::Millimetres) == "\xC3\x98" "10.00");

    d.useCustomText = true;
    d.customText = "THICK";
    CHECK(dimensionText(doc, d, Units::Millimetres) == "THICK");
    d.useCustomText = false;
    CHECK(dimensionText(doc, d, Units::Millimetres) == "25.400");
}

TEST_CASE("dimension parameter checks", "[dimension][document]") {
    DimensionObject d = make(DimensionKind::Aligned, {0, 0}, {1, 0});
    CHECK(dimensionProblem(d).empty());
    d.decimals = 9;
    CHECK_FALSE(dimensionProblem(d).empty());
    d.decimals = 2;
    d.angleDecimals = 7;
    CHECK_FALSE(dimensionProblem(d).empty());
    d.angleDecimals = -1;
    CHECK_FALSE(dimensionProblem(d).empty());
    d.angleDecimals = 2;
    d.textHeightMm = 0.0;
    CHECK_FALSE(dimensionProblem(d).empty());
    d.textHeightMm = std::nan("");
    CHECK_FALSE(dimensionProblem(d).empty());
    d.textHeightMm = 3.0;
    d.offsetMm = INFINITY;
    CHECK_FALSE(dimensionProblem(d).empty());
    d.offsetMm = 1.0;
    d.points[1].pos.x = std::nan("");
    CHECK_FALSE(dimensionProblem(d).empty());
    d.points[1].pos.x = 1.0;
    d.customText.assign(kMaxDimensionTextBytes + 1, 'x');
    CHECK_FALSE(dimensionProblem(d).empty());
}

TEST_CASE("dimension geometry places lines, arrows and text", "[dimension][document]") {
    Document doc;
    DimensionObject d = make(DimensionKind::Linear, {0, 0}, {10, 0});
    d.offsetMm = 5.0;
    d.textHeightMm = 2.0;
    DimensionGeometry g = dimensionGeometry(doc, d);
    REQUIRE(g.valid);
    // Two extension lines, the dimension line and two arrowheads.
    CHECK(g.contours.size() == 5);
    geometry::Box2 box;
    for (const auto& c : g.contours) {
        box.expand(c.bounds());
    }
    CHECK(box.min.y == Approx(0.0).margin(1e-12));
    // The dimension line is at y = 5; the arrowheads are 2/3 of the text height wide (+-h/6).
    CHECK(box.max.y == Approx(5.0 + 2.0 / 6.0).margin(1e-12));
    CHECK(box.min.x == Approx(0.0).margin(1e-12));
    CHECK(box.max.x == Approx(10.0).margin(1e-12));
    CHECK(g.textCentre.x == Approx(5.0).margin(1e-12));
    CHECK(g.textCentre.y == Approx(5.0 + 0.75 * 2.0).margin(1e-12));
    CHECK(g.textAngle == Approx(0.0).margin(1e-12));

    // A negative offset puts the line on the other side.
    d.offsetMm = -5.0;
    g = dimensionGeometry(doc, d);
    CHECK(g.textCentre.y == Approx(-5.0 + 1.5).margin(1e-12));

    // Vertical linear: the dimension line is parallel to Y, text reads upright.
    DimensionObject v = make(DimensionKind::Linear, {0, 0}, {3, 10});
    v.vertical = true;
    v.offsetMm = 4.0;
    g = dimensionGeometry(doc, v);
    REQUIRE(g.valid);
    CHECK(g.textCentre.x == Approx(1.5 - 4.0 - 0.75 * 3.0).margin(1e-12));
    CHECK(std::fabs(g.textAngle) <= kPi / 2 + 1e-12);

    // Text never reads upside down for an aligned dimension pointing left.
    g = dimensionGeometry(doc, make(DimensionKind::Aligned, {10, 0}, {0, 1}));
    CHECK(g.textAngle > -kPi / 2);
    CHECK(g.textAngle <= kPi / 2);

    // Angular: arc of the dimension radius about the vertex.
    DimensionObject a = make(DimensionKind::Angular, {0, 0}, {10, 0}, {0, 10});
    a.offsetMm = 6.0;
    g = dimensionGeometry(doc, a);
    REQUIRE(g.valid);
    geometry::Box2 ab;
    for (const auto& c : g.contours) {
        ab.expand(c.bounds());
    }
    CHECK(ab.max.x >= 6.0 - 1e-9);
    CHECK(g.textCentre.x == Approx(std::cos(kPi / 4) * (6.0 + 0.75 * 3.0)).margin(1e-9));

    // Radial: leader runs from the centre through the circle point and beyond.
    DimensionObject r = make(DimensionKind::Radial, {0, 0}, {5, 0});
    r.offsetMm = 3.0;
    g = dimensionGeometry(doc, r);
    REQUIRE(g.valid);
    CHECK(g.contours.size() == 2);  // leader + one arrowhead
    r.diameter = true;
    g = dimensionGeometry(doc, r);
    CHECK(g.contours.size() == 3);  // leader + two arrowheads
    CHECK(g.textCentre.x == Approx(8.0 + 0.75 * 3.0).margin(1e-12));
}

TEST_CASE("offset for a placement click", "[dimension][document]") {
    const auto pts = [](const DimensionObject& d) {
        return std::array<Vec2, 3>{d.points[0].pos, d.points[1].pos, d.points[2].pos};
    };
    DimensionObject lin = make(DimensionKind::Linear, {0, 0}, {10, 0});
    CHECK(*offsetForPlacement(lin, pts(lin), {3, 7}) == Approx(7.0));
    CHECK(*offsetForPlacement(lin, pts(lin), {3, -2}) == Approx(-2.0));
    DimensionObject al = make(DimensionKind::Aligned, {0, 0}, {0, 10});  // normal is -X
    CHECK(*offsetForPlacement(al, pts(al), {-4, 5}) == Approx(4.0));
    DimensionObject ang = make(DimensionKind::Angular, {1, 1}, {5, 1}, {1, 5});
    CHECK(*offsetForPlacement(ang, pts(ang), {4, 5}) == Approx(5.0));
    DimensionObject rad = make(DimensionKind::Radial, {0, 0}, {5, 0});
    CHECK(*offsetForPlacement(rad, pts(rad), {9, 2}) == Approx(4.0));
    CHECK(*offsetForPlacement(rad, pts(rad), {2, 0}) == Approx(0.0));  // never negative
    CHECK_FALSE(offsetForPlacement(make(DimensionKind::Aligned, {1, 1}, {1, 1}), {}, {0, 0}));

    // The offset it returns reproduces the placement.
    Document doc;
    lin.offsetMm = *offsetForPlacement(lin, pts(lin), {3, 7});
    DimensionGeometry g = dimensionGeometry(doc, lin);
    geometry::Box2 b;
    for (const auto& c : g.contours) {
        b.expand(c.bounds());
    }
    CHECK(b.max.y == Approx(7.0 + 3.0 / 6.0));  // line at 7, arrowheads add half their width
}

TEST_CASE("anchors follow the part when it is scaled or moved", "[dimension][document][associative]") {
    Document doc;
    const ObjectId rect = addRect(doc);
    // Width: the start of span 0 (10,20) to the start of span 1 (50,20); the rectangle goes CCW.
    DimensionObject d = make(DimensionKind::Aligned, {10, 20}, {50, 20});
    d.points[0].anchor = DimensionAnchor{rect, 0, 0, DimensionAnchor::Which::Start};
    d.points[1].anchor = DimensionAnchor{rect, 0, 1, DimensionAnchor::Which::Start};
    REQUIRE(measured(doc, d) == Approx(40.0));
    const auto id = addDimension(doc, d, doc.activeSheet());
    REQUIRE(id);
    const auto text = [&] { return dimensionText(doc, *doc.findObject(*id)->asDimension(), Units::Millimetres); };
    CHECK(text() == "40.00");

    // Scaling the part (only the part is selected) updates the dimension text.
    REQUIRE(doc.execute(std::make_unique<TransformObjectsCommand>(std::vector<ObjectId>{rect},
                                                                   geometry::Transform2::scaling(2.0))));
    CHECK(text() == "80.00");
    // Moving does not change a length; rotating does not either.
    REQUIRE(doc.execute(std::make_unique<TransformObjectsCommand>(
        std::vector<ObjectId>{rect}, geometry::Transform2::translation({100, -7}))));
    CHECK(text() == "80.00");
    REQUIRE(doc.execute(std::make_unique<TransformObjectsCommand>(
        std::vector<ObjectId>{rect}, geometry::Transform2::rotation(0.7))));
    CHECK(text() == "80.00");
    // Non-uniform scale: the anchors follow the corners.
    REQUIRE(doc.execute(std::make_unique<TransformObjectsCommand>(
        std::vector<ObjectId>{rect}, geometry::Transform2::scaling(0.5, 1.0))));
    const auto resolved = resolveDimensionPoints(doc, *doc.findObject(*id)->asDimension());
    CHECK(geometry::distance(resolved[0], resolved[1]) > 0.0);

    // Consecutive transforms of one selection coalesce, so one undo reverts them all and redo
    // restores the final state.
    const std::string finalText = text();
    REQUIRE(doc.undo());
    CHECK(text() == "40.00");
    REQUIRE(doc.redo());
    CHECK(text() == finalText);
}

TEST_CASE("an angular dimension anchored to a part follows non-uniform scaling", "[dimension][associative]") {
    Document doc;
    const ObjectId rect = addRect(doc);
    // Angle at the lower-left corner between its bottom edge and its left edge: 90 degrees.
    DimensionObject d = make(DimensionKind::Angular, {10, 20}, {50, 20}, {10, 50});
    d.points[0].anchor = DimensionAnchor{rect, 0, 0, DimensionAnchor::Which::Start};
    d.points[1].anchor = DimensionAnchor{rect, 0, 1, DimensionAnchor::Which::Start};
    d.points[2].anchor = DimensionAnchor{rect, 0, 3, DimensionAnchor::Which::Start};
    CHECK(measured(doc, d) == Approx(kPi / 2));
    // A shear-free non-uniform scale keeps right angles; a rotation does too.
    REQUIRE(doc.execute(std::make_unique<TransformObjectsCommand>(std::vector<ObjectId>{rect},
                                                                   geometry::Transform2::scaling(3.0, 0.5))));
    CHECK(measured(doc, d) == Approx(kPi / 2));
    // A shear changes it.
    REQUIRE(doc.execute(std::make_unique<TransformObjectsCommand>(
        std::vector<ObjectId>{rect}, geometry::Transform2(1.0, 1.0, 0.0, 1.0, 0.0, 0.0))));
    CHECK(measured(doc, d) != Approx(kPi / 2));
}

TEST_CASE("a radial dimension anchors to a circle's centre and start", "[dimension][associative]") {
    Document doc;
    const ObjectId circle = doc.insertObject(Object::vector(geometry::Contour::circle({20, 20}, 6.0),
                                                            doc.activeLayer(), doc.activeSheet()));
    DimensionObject d = make(DimensionKind::Radial, {20, 20}, {26, 20});
    d.points[0].anchor = DimensionAnchor{circle, 0, 0, DimensionAnchor::Which::Centre};
    d.points[1].anchor = DimensionAnchor{circle, 0, 0, DimensionAnchor::Which::Start};
    CHECK(measured(doc, d) == Approx(6.0));
    d.diameter = true;
    CHECK(measured(doc, d) == Approx(12.0));
    REQUIRE(doc.execute(std::make_unique<TransformObjectsCommand>(std::vector<ObjectId>{circle},
                                                                   geometry::Transform2::scaling(1.5))));
    CHECK(measured(doc, d) == Approx(18.0));
}

TEST_CASE("unresolvable anchors fall back to the stored position", "[dimension][associative]") {
    Document doc;
    const ObjectId rect = addRect(doc);
    DimensionObject d = make(DimensionKind::Aligned, {0, 0}, {3, 4});
    d.points[0].anchor = DimensionAnchor{rect, 0, 0, DimensionAnchor::Which::Start};  // (10, 20)
    CHECK(resolveDimensionPoints(doc, d)[0] == Vec2{10, 20});
    d.points[0].anchor = DimensionAnchor{rect, 0, 0, DimensionAnchor::Which::Centre};  // a line: none
    CHECK(resolveDimensionPoints(doc, d)[0] == Vec2{0, 0});
    d.points[0].anchor = DimensionAnchor{rect, 7, 0, DimensionAnchor::Which::Start};
    CHECK(resolveDimensionPoints(doc, d)[0] == Vec2{0, 0});
    d.points[0].anchor = DimensionAnchor{rect, 0, 9, DimensionAnchor::Which::Start};
    CHECK(resolveDimensionPoints(doc, d)[0] == Vec2{0, 0});
    d.points[0].anchor = DimensionAnchor{ObjectId{999}, 0, 0, DimensionAnchor::Which::Start};
    CHECK(resolveDimensionPoints(doc, d)[0] == Vec2{0, 0});
    CHECK(measured(doc, d) == Approx(5.0));
    // Deleting the part leaves the dimension at its last position and the document consistent.
    d.points[0].anchor = DimensionAnchor{rect, 0, 0, DimensionAnchor::Which::End};
    REQUIRE(addDimension(doc, d, doc.activeSheet()));
    REQUIRE(doc.execute(std::make_unique<RemoveObjectsCommand>(std::vector<ObjectId>{rect})));
    CHECK(doc.checkIntegrity().empty());
}

TEST_CASE("a dimension transformed with its part scales its own geometry", "[dimension][document]") {
    Document doc;
    DimensionObject d = make(DimensionKind::Aligned, {0, 0}, {10, 0});
    d.offsetMm = 4.0;
    const auto id = addDimension(doc, d, doc.activeSheet());
    REQUIRE(id);
    REQUIRE(doc.execute(std::make_unique<TransformObjectsCommand>(std::vector<ObjectId>{*id},
                                                                   geometry::Transform2::scaling(3.0))));
    const DimensionObject* s = doc.findObject(*id)->asDimension();
    CHECK(measured(doc, *s) == Approx(30.0));
    CHECK(s->offsetMm == Approx(12.0));
    CHECK(s->points[1].pos == Vec2{30, 0});
    // The text height is a drawing property and is not scaled.
    CHECK(s->textHeightMm == Approx(3.0));
    REQUIRE(doc.undo());
    CHECK(*doc.findObject(*id)->asDimension() == d);
}

TEST_CASE("the dimension line stays on the same side of the part under rotation and mirroring",
          "[dimension][document]") {
    const auto sideOf = [](const Document& doc, const DimensionObject& d, Vec2 partCentre) {
        const DimensionGeometry g = dimensionGeometry(doc, d);
        return g.textCentre - partCentre;
    };
    Document doc;
    const Vec2 centre{5, 0};
    for (const DimensionKind kind : {DimensionKind::Aligned, DimensionKind::Linear}) {
        DimensionObject d = make(kind, {0, 0}, {10, 0});
        d.offsetMm = 6.0;
        const geometry::Transform2 maps[] = {geometry::Transform2::mirrorY(0.0), geometry::Transform2::mirrorX(5.0),
                                             geometry::Transform2::rotation(kPi / 2, {5, 0}),
                                             geometry::Transform2::scaling(2.0, 2.0, {5, 0})};
        for (const auto& m : maps) {
            DimensionObject t = d;
            t.transform(m);
            // The text centre of the transformed dimension lies where the original's maps to,
            // for the conformal maps with an axis-aligned result (aligned) or on the same side
            // (linear: the line keeps its normal direction).
            const Vec2 before = sideOf(doc, d, centre);
            const Vec2 after = sideOf(doc, t, m.apply(centre));
            if (kind == DimensionKind::Aligned) {
                const Vec2 expect = m.applyVector(before);
                // Text clearance does not scale, so compare directions.
                CHECK(after.normalized().dot(expect.normalized()) == Approx(1.0).margin(1e-9));
            } else {
                // Linear keeps measuring along X; its line is on the side the mapped line anchor is.
                const Vec2 lineAnchor = m.apply(Vec2{5, 6});
                CHECK((after.dot(Vec2{0, 1})) * (lineAnchor - m.apply(centre)).dot(Vec2{0, 1}) >= 0.0);
            }
        }
    }
}

TEST_CASE("addDimension uses the Dimensions layer and is one undo step", "[dimension][document]") {
    Document doc;
    const std::size_t layersBefore = doc.layers().size();
    DimensionObject d = make(DimensionKind::Aligned, {0, 0}, {5, 0});
    const auto first = addDimension(doc, d, doc.activeSheet());
    REQUIRE(first);
    REQUIRE(doc.layers().size() == layersBefore + 1);
    const Layer* layer = doc.findLayer(doc.findObject(*first)->layer);
    REQUIRE(layer != nullptr);
    CHECK(layer->name == "Dimensions");
    CHECK(doc.findObject(*first)->isDimension());
    CHECK(doc.checkIntegrity().empty());

    // A second one reuses the layer.
    const auto second = addDimension(doc, d, doc.activeSheet());
    REQUIRE(second);
    CHECK(doc.layers().size() == layersBefore + 1);
    CHECK(doc.findObject(*second)->layer == layer->id);

    REQUIRE(doc.undo());
    CHECK_FALSE(doc.hasObject(*second));
    CHECK(doc.layers().size() == layersBefore + 1);
    REQUIRE(doc.undo());  // the layer and the first dimension go together
    CHECK_FALSE(doc.hasObject(*first));
    CHECK(doc.layers().size() == layersBefore);
    REQUIRE(doc.redo());
    CHECK(doc.hasObject(*first));
    CHECK(doc.layers().size() == layersBefore + 1);

    // An explicit layer is used as given; bad input is refused without a trace.
    const LayerId plain = doc.defaultLayer();
    const auto third = addDimension(doc, d, doc.activeSheet(), plain);
    REQUIRE(third);
    CHECK(doc.findObject(*third)->layer == plain);
    const std::size_t count = doc.objectCount();
    DimensionObject bad = d;
    bad.decimals = 20;
    CHECK_FALSE(addDimension(doc, bad, doc.activeSheet()));
    CHECK_FALSE(addDimension(doc, d, SheetId{12345}));
    CHECK_FALSE(addDimension(doc, d, doc.activeSheet(), LayerId{12345}));
    CHECK(doc.objectCount() == count);
}

TEST_CASE("SetDimensionCommand edits options with undo", "[dimension][document]") {
    Document doc;
    DimensionObject d = make(DimensionKind::Aligned, {0, 0}, {5, 0});
    const auto id = addDimension(doc, d, doc.activeSheet());
    REQUIRE(id);
    DimensionObject e = d;
    e.decimals = 4;
    e.useCustomText = true;
    e.customText = "5 typ";
    e.textHeightMm = 2.5;
    REQUIRE(doc.execute(std::make_unique<SetDimensionCommand>(*id, e)));
    CHECK(*doc.findObject(*id)->asDimension() == e);
    CHECK(dimensionText(doc, e, Units::Millimetres) == "5 typ");
    // A no-op, a bad value and a non-dimension are all refused.
    CHECK_FALSE(doc.execute(std::make_unique<SetDimensionCommand>(*id, e)));
    DimensionObject bad = e;
    bad.textHeightMm = -1.0;
    CHECK_FALSE(doc.execute(std::make_unique<SetDimensionCommand>(*id, bad)));
    const ObjectId rect = addRect(doc);
    CHECK_FALSE(doc.execute(std::make_unique<SetDimensionCommand>(rect, e)));
    REQUIRE(doc.undo());
    CHECK(*doc.findObject(*id)->asDimension() == d);
    REQUIRE(doc.redo());
    CHECK(*doc.findObject(*id)->asDimension() == e);
}

TEST_CASE("findAnchorNear picks vertices and arc centres", "[dimension][document]") {
    Document doc;
    const ObjectId rect = addRect(doc);
    const ObjectId circle = doc.insertObject(Object::vector(geometry::Contour::circle({100, 100}, 5.0),
                                                            doc.activeLayer(), doc.activeSheet()));
    auto hit = findAnchorNear(doc, doc.activeSheet(), {10.2, 19.9}, 0.5);
    REQUIRE(hit);
    CHECK(hit->anchor.object == rect);
    CHECK(hit->anchor.contour == 0);
    CHECK(hit->anchor.span == 0);
    CHECK(hit->pos == Vec2{10, 20});
    hit = findAnchorNear(doc, doc.activeSheet(), {100.1, 100.1}, 0.5);
    REQUIRE(hit);
    CHECK(hit->anchor.object == circle);
    CHECK(hit->anchor.which == DimensionAnchor::Which::Centre);
    hit = findAnchorNear(doc, doc.activeSheet(), {105.0, 100.0}, 0.5);
    REQUIRE(hit);
    CHECK(hit->anchor.which == DimensionAnchor::Which::Start);
    CHECK_FALSE(findAnchorNear(doc, doc.activeSheet(), {30, 35}, 1.0));
    CHECK_FALSE(findAnchorNear(doc, doc.activeSheet(), {10, 20}, -1.0));
    CHECK_FALSE(findAnchorNear(doc, SheetId{999}, {10, 20}, 1.0));
    // Exactly on the tolerance counts.
    CHECK(findAnchorNear(doc, doc.activeSheet(), {10, 21}, 1.0));
}

TEST_CASE("object bounds of a dimension cover its drawing", "[dimension][document]") {
    Document doc;
    DimensionObject d = make(DimensionKind::Linear, {0, 0}, {10, 0});
    d.offsetMm = 5.0;
    const auto id = addDimension(doc, d, doc.activeSheet());
    REQUIRE(id);
    const auto box = doc.objectBounds(*id);
    REQUIRE(box);
    CHECK(box->min.x == Approx(0.0).margin(1e-12));
    CHECK(box->max.x == Approx(10.0).margin(1e-12));
    CHECK(box->max.y >= 5.0);
    // A degenerate dimension still reports its points.
    DimensionObject deg = make(DimensionKind::Aligned, {2, 3}, {2, 3});
    const auto id2 = addDimension(doc, deg, doc.activeSheet());
    REQUIRE(id2);
    CHECK(doc.objectBounds(*id2)->min == Vec2{2, 3});
}

TEST_CASE("dimensions round trip through .carv byte for byte", "[dimension][document][carv]") {
    Document doc;
    const ObjectId rect = addRect(doc);
    DimensionObject d = make(DimensionKind::Aligned, {10, 20}, {50, 20});
    d.points[0].anchor = DimensionAnchor{rect, 0, 0, DimensionAnchor::Which::Start};
    d.offsetMm = -7.25;
    d.decimals = 3;
    d.useCustomText = true;
    d.customText = "40 \xC2\xB1 0.1";
    d.fontKey = "0123456789abcdef";
    REQUIRE(addDimension(doc, d, doc.activeSheet()));
    DimensionObject a = make(DimensionKind::Angular, {10, 20}, {50, 20}, {10, 50});
    a.angleDecimals = 4;
    a.points[2].anchor = DimensionAnchor{rect, 0, 3, DimensionAnchor::Which::End};
    REQUIRE(addDimension(doc, a, doc.activeSheet()));
    DimensionObject r = make(DimensionKind::Radial, {1, 2}, {4, 6});
    r.diameter = true;
    r.vertical = true;
    REQUIRE(addDimension(doc, r, doc.activeSheet()));

    auto saved = saveCarv(doc);
    REQUIRE(saved.ok());
    auto loaded = loadCarv(std::span<const std::uint8_t>(saved.value()));
    INFO((loaded.ok() ? std::string() : loaded.error().message));
    REQUIRE(loaded.ok());
    CHECK(loaded.value().formatMinor == kCarvFormatMinor);
    const Document& back = loaded.value().document;
    REQUIRE(back.objectCount() == doc.objectCount());
    for (const auto& [id, obj] : doc.objects()) {
        const Object* other = back.findObject(id);
        REQUIRE(other != nullptr);
        CHECK(*other == obj);
    }
    CHECK(back.checkIntegrity().empty());
    auto again = saveCarv(back, loaded.value().extras);
    REQUIRE(again.ok());
    CHECK(again.value() == saved.value());
}

TEST_CASE("documents without dimensions keep their older format minor", "[dimension][document][carv]") {
    Document doc;
    addRect(doc);
    auto saved = saveCarv(doc);
    REQUIRE(saved.ok());
    auto loaded = loadCarv(std::span<const std::uint8_t>(saved.value()));
    REQUIRE(loaded.ok());
    CHECK(loaded.value().formatMinor == kCarvFormatMinorBase);
}

TEST_CASE("hostile dimension records are refused on load", "[dimension][document][carv]") {
    Document doc;
    DimensionObject d = make(DimensionKind::Aligned, {0, 0}, {10, 0});
    REQUIRE(addDimension(doc, d, doc.activeSheet()));
    auto saved = saveCarv(doc);
    REQUIRE(saved.ok());
    auto zip = detail::readZip(saved.value());
    REQUIRE(zip.ok);
    const auto mutate = [&](const std::string& from, const std::string& to) {
        auto members = zip.members;
        bool found = false;
        for (auto& m : members) {
            if (m.name == "job.json") {
                std::string t(m.data.begin(), m.data.end());
                const auto pos = t.find(from);
                if (pos != std::string::npos) {
                    found = true;
                    t.replace(pos, from.size(), to);
                    m.data.assign(t.begin(), t.end());
                }
            }
        }
        REQUIRE(found);
        auto w = detail::writeZip(members);
        REQUIRE(w.ok);
        return loadCarv(std::span<const std::uint8_t>(w.bytes));
    };
    CHECK(mutate("\"decimals\": 2", "\"decimals\": 2").ok());
    CHECK_FALSE(mutate("\"decimals\": 2", "\"decimals\": 9").ok());
    CHECK_FALSE(mutate("\"decimals\": 2", "\"decimals\": -1").ok());
    CHECK_FALSE(mutate("\"angle_decimals\": 2", "\"angle_decimals\": 7").ok());
    CHECK_FALSE(mutate("\"kind\": \"aligned\"", "\"kind\": \"ordinate\"").ok());
    CHECK_FALSE(mutate("\"text_height_mm\": 3", "\"text_height_mm\": 0").ok());
    CHECK_FALSE(mutate("\"text_height_mm\": 3", "\"text_height_mm\": \"NaN\"").ok());
    CHECK_FALSE(mutate("\"offset_mm\": 5", "\"offset_mm\": \"Infinity\"").ok());
    CHECK_FALSE(mutate("\"type\": \"dimension\"", "\"type\": \"vector\"").ok());
    CHECK_FALSE(mutate("\"vertical\": false", "\"vertical\": 1").ok());
}
