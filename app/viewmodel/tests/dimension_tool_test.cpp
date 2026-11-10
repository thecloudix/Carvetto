// Dimension tool (T-0246): picking points, anchoring to the part, placement, one undo step, edit.

#include <cmath>
#include <memory>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/dimension.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/viewmodel/canvas_scene.hpp"
#include "carvetto/viewmodel/dimension_tool.hpp"
#include "carvetto/viewmodel/dxf_exchange.hpp"
#include "carvetto/viewmodel/view_transform.hpp"

using namespace carvetto;
using carvetto::geometry::Vec2;
using carvetto::viewmodel::DimensionOptions;
using carvetto::viewmodel::DimensionTool;
using Catch::Approx;

namespace {

document::ObjectId addRect(document::Document& doc) {
    geometry::Box2 b;
    b.expand(Vec2{10, 20});
    b.expand(Vec2{50, 50});
    return doc.insertObject(
        document::Object::vector(geometry::Contour::rectangle(b), doc.activeLayer(), doc.activeSheet()));
}

}  // namespace

TEST_CASE("the dimension tool picks two points and a placement, anchored to the part", "[dimension][viewmodel]") {
    document::Document doc;
    const auto rect = addRect(doc);
    DimensionTool tool(&doc);
    DimensionOptions o;
    o.kind = document::DimensionKind::Aligned;
    REQUIRE(tool.setOptions(o).ok);

    CHECK(tool.click({10.1, 20.1}, 0.5).draftChanged);  // snaps to the corner (10, 20)
    CHECK(tool.pickedCount() == 1);
    tool.move({30, 10});
    CHECK_FALSE(tool.preview().contours.empty());       // rubber band
    CHECK(tool.click({49.9, 19.9}, 0.5).ok);            // corner (50, 20)
    CHECK(tool.pickedCount() == 2);
    tool.move({30, 10});
    CHECK(tool.preview().valid);                        // the dimension at the cursor
    const auto r = tool.click({30, 10}, 0.5);           // placement below the bottom edge
    REQUIRE(r.committed);
    REQUIRE(r.created);
    CHECK(tool.pickedCount() == 0);
    CHECK(doc.selection().single() == *r.created);

    const document::DimensionObject* d = doc.findObject(*r.created)->asDimension();
    REQUIRE(d != nullptr);
    REQUIRE(d->points[0].anchor);
    CHECK(d->points[0].anchor->object == rect);
    CHECK(d->points[0].pos == Vec2{10, 20});
    CHECK(d->points[1].pos == Vec2{50, 20});
    CHECK(d->offsetMm == Approx(-10.0));  // normal of 0 -> 1 is +Y; the click is below
    CHECK(document::dimensionText(doc, *d, document::Units::Millimetres) == "40.00");

    // Scaling the part updates the text.
    doc.execute(std::make_unique<document::TransformObjectsCommand>(std::vector<document::ObjectId>{rect},
                                                                    geometry::Transform2::scaling(2.0)));
    CHECK(document::dimensionText(doc, *doc.findObject(*r.created)->asDimension(),
                                  document::Units::Millimetres) == "80.00");

    // The whole dimension (layer + object) is one undo step.
    const std::size_t layers = doc.layers().size();
    doc.undo();  // the scale
    doc.undo();
    CHECK_FALSE(doc.hasObject(*r.created));
    CHECK(doc.layers().size() == layers - 1);
}

TEST_CASE("angular and radial placement clicks", "[dimension][viewmodel]") {
    document::Document doc;
    DimensionTool tool(&doc);
    DimensionOptions o;
    o.kind = document::DimensionKind::Angular;
    REQUIRE(tool.setOptions(o).ok);
    tool.click({0, 0}, 0.1);
    tool.click({10, 0}, 0.1);
    tool.click({0, 10}, 0.1);
    CHECK(tool.pickedCount() == 3);
    auto r = tool.click({6, 8}, 0.1);
    REQUIRE(r.committed);
    const auto* a = doc.findObject(*r.created)->asDimension();
    CHECK(a->offsetMm == Approx(10.0));
    CHECK(document::dimensionText(doc, *a, document::Units::Millimetres) == "90.00\xC2\xB0");

    o.kind = document::DimensionKind::Radial;
    o.diameter = true;
    REQUIRE(tool.setOptions(o).ok);
    tool.click({20, 20}, 0.1);
    tool.click({25, 20}, 0.1);
    r = tool.click({30, 20}, 0.1);
    REQUIRE(r.committed);
    const auto* rad = doc.findObject(*r.created)->asDimension();
    CHECK(rad->offsetMm == Approx(5.0));
    CHECK(document::dimensionText(doc, *rad, document::Units::Millimetres) == "\xC3\x98" "10.00");
}

TEST_CASE("a degenerate pick is refused and the draft resets", "[dimension][viewmodel]") {
    document::Document doc;
    DimensionTool tool(&doc);
    tool.click({5, 5}, 0.1);
    tool.click({5, 5}, 0.1);
    const auto r = tool.click({9, 9}, 0.1);
    CHECK_FALSE(r.ok);
    CHECK_FALSE(r.committed);
    CHECK(tool.pickedCount() == 0);
    CHECK(doc.objectCount() == 0);
    // Escape drops a draft without touching the document.
    tool.click({0, 0}, 0.1);
    tool.cancel();
    CHECK(tool.pickedCount() == 0);
    CHECK(tool.preview().contours.empty());
    // Bad options are refused and change nothing.
    DimensionOptions bad;
    bad.decimals = 12;
    CHECK_FALSE(tool.setOptions(bad).ok);
    CHECK(tool.options().decimals == 2);
}

TEST_CASE("editing a dimension replaces its options in one undo step", "[dimension][viewmodel]") {
    document::Document doc;
    DimensionTool tool(&doc);
    tool.click({0, 0}, 0.1);
    tool.click({10, 0}, 0.1);
    const auto id = *tool.click({5, 4}, 0.1).created;
    auto o = tool.optionsOf(id);
    REQUIRE(o);
    CHECK(o->decimals == 2);
    o->decimals = 4;
    o->textHeightMm = 5.0;
    o->useCustomText = true;
    o->customText = "TEN";
    o->kind = document::DimensionKind::Radial;  // the kind cannot change
    const auto r = tool.edit(id, *o);
    REQUIRE(r.committed);
    const auto* d = doc.findObject(id)->asDimension();
    CHECK(d->kind == document::DimensionKind::Aligned);
    CHECK(d->decimals == 4);
    CHECK(d->customText == "TEN");
    CHECK(d->offsetMm == Approx(4.0));
    doc.undo();
    CHECK(doc.findObject(id)->asDimension()->decimals == 2);
    // Refusals.
    CHECK_FALSE(tool.edit(document::ObjectId{999}, *o).ok);
    o->textHeightMm = 0.0;
    CHECK_FALSE(tool.edit(id, *o).ok);
    CHECK_FALSE(tool.optionsOf(document::ObjectId{999}));
}

TEST_CASE("the canvas scene draws dimension lines and DXF export explodes them", "[dimension][viewmodel]") {
    document::Document doc;
    document::DimensionObject d;
    d.points[0].pos = {0, 0};
    d.points[1].pos = {10, 0};
    d.offsetMm = 5.0;
    REQUIRE(document::addDimension(doc, d, doc.activeSheet()));
    viewmodel::ViewTransform view;
    view.setViewport(200, 200);
    view.set(10.0, {50.0, 150.0});
    const viewmodel::CanvasScene scene = viewmodel::buildCanvasScene(doc, view);
    CHECK(scene.paths.size() == 5);  // two extension lines, the dimension line, two arrowheads

    const auto out = viewmodel::exportDocumentDxf(doc, false);
    REQUIRE(out.ok);
    CHECK(out.contoursWritten == 5);
}
