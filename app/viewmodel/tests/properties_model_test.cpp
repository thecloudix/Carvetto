#include "carvetto/viewmodel/properties_model.hpp"

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/shape_params.hpp"
#include "carvetto/document/text_object.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/text/font.hpp"
#include "carvetto/viewmodel/layers_sheets.hpp"
#include "carvetto/viewmodel/starter_font.hpp"

using namespace carvetto::document;
using namespace carvetto::viewmodel;
using carvetto::geometry::Contour;

namespace {

ObjectId add(Document& doc, CreationParams params, bool select = true) {
    auto built = buildContours(params);
    REQUIRE(built.ok());
    auto cmd = std::make_unique<AddObjectCommand>(
        Object::vector(built.value(), doc.activeLayer(), doc.activeSheet(), params));
    AddObjectCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    if (select) {
        doc.selection().assign({raw->id()});
    }
    return raw->id();
}

const PropertyField* find(const PropertyView& v, const std::string& id) {
    for (const PropertyField& f : v.fields) {
        if (f.id == id) {
            return &f;
        }
    }
    return nullptr;
}

ObjectId addText(Document& doc, const std::string& text) {
    const auto bytes = starterFontBytes();
    FontAsset asset{fontKeyFor({bytes.begin(), bytes.end()}), {bytes.begin(), bytes.end()}};
    TextParams p;
    p.text = text;
    p.fontKey = asset.key;
    p.sizeMm = 10.0;
    p.placement = carvetto::geometry::Transform2::translation({10, 10});
    REQUIRE(doc.insertFont(asset));
    auto font = carvetto::text::Font::load(asset.bytes);
    REQUIRE(font.ok());
    const TextBuild b = buildTextContours(font.font, p);
    REQUIRE(b.ok());
    auto cmd = std::make_unique<AddObjectCommand>(Object::vector(b.contours, doc.activeLayer(), doc.activeSheet(), p));
    AddObjectCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    doc.selection().assign({raw->id()});
    return raw->id();
}

}  // namespace

TEST_CASE("no selection: no fields", "[viewmodel][properties]") {
    Document doc;
    const PropertyView v = describeSelection(doc);
    CHECK(v.title == "No selection");
    CHECK(v.fields.empty());
    CHECK_FALSE(setProperty(doc, "x", "1").ok);
}

TEST_CASE("rectangle fields reflect and edit the parameters", "[viewmodel][properties]") {
    Document doc;
    const ObjectId id = add(doc, RectangleParams{{50, 40}, 20, 10});
    PropertyView v = describeSelection(doc);
    CHECK(v.title == "Rectangle");
    REQUIRE(find(v, "width") != nullptr);
    CHECK(find(v, "width")->text == "20.00");
    CHECK(find(v, "x")->text == "40.00");
    CHECK(find(v, "y")->text == "35.00");
    CHECK(find(v, "layer")->choice == 0);

    const std::size_t steps = doc.undoStack().count();
    REQUIRE(setProperty(doc, "width", "30").ok);
    CHECK(doc.undoStack().count() == steps + 1);
    CHECK(std::get<RectangleParams>(doc.findObject(id)->asVector()->params).widthMm == 30.0);
    CHECK(doc.findObject(id)->asVector()->bounds().width() == 30.0);
    // Two edits in a row are two steps.
    REQUIRE(setProperty(doc, "width", "31").ok);
    REQUIRE(setProperty(doc, "height", "12").ok);
    CHECK(doc.undoStack().count() == steps + 3);

    REQUIRE(setProperty(doc, "corners", "1").ok);
    REQUIRE(setProperty(doc, "cornerRadius", "2").ok);
    CHECK(std::get<RectangleParams>(doc.findObject(id)->asVector()->params).corners ==
          RectangleParams::CornerStyle::Rounded);

    REQUIRE(doc.undo());
    REQUIRE(doc.undo());
    const auto& now = std::get<RectangleParams>(doc.findObject(id)->asVector()->params);
    CHECK(now.heightMm == 12.0);
    CHECK(now.corners == RectangleParams::CornerStyle::Square);
}

TEST_CASE("bad numeric entries are refused without touching the document", "[viewmodel][properties]") {
    Document doc;
    const ObjectId id = add(doc, RectangleParams{{50, 40}, 20, 10});
    const Object before = *doc.findObject(id);
    const std::size_t steps = doc.undoStack().count();
    for (const char* bad : {"", "abc", "nan", "inf", "-5", "0", "1e30", "5 5", "--3", "0x10"}) {
        INFO(bad);
        CHECK_FALSE(setProperty(doc, "width", bad).ok);
    }
    CHECK_FALSE(setProperty(doc, "corners", "9").ok);
    CHECK_FALSE(setProperty(doc, "corners", "-1").ok);
    CHECK_FALSE(setProperty(doc, "layer", "99").ok);
    CHECK_FALSE(setProperty(doc, "sides", "5").ok);  // not a rectangle property
    CHECK_FALSE(setProperty(doc, "x", "nan").ok);
    CHECK_FALSE(setProperty(doc, "x", "1e12").ok);
    CHECK(*doc.findObject(id) == before);
    CHECK(doc.undoStack().count() == steps);
}

TEST_CASE("units: lengths follow the document's display units", "[viewmodel][properties]") {
    Job job = Job::singleSided(12, 12, 1, Units::Inches);
    Document doc(job);
    const ObjectId id = add(doc, CircleParams{{50.8, 50.8}, 12.7});
    const PropertyView v = describeSelection(doc);
    CHECK(find(v, "diameter")->text == "1.000");
    CHECK(find(v, "diameter")->suffix == "in");
    REQUIRE(setProperty(doc, "diameter", "2").ok);
    CHECK(std::abs(std::get<CircleParams>(doc.findObject(id)->asVector()->params).radiusMm - 25.4) < 1e-9);
    REQUIRE(setProperty(doc, "diameter", "10 mm").ok);
    CHECK(std::abs(std::get<CircleParams>(doc.findObject(id)->asVector()->params).radiusMm - 5.0) < 1e-9);
}

TEST_CASE("polygon, star and ellipse per-kind fields", "[viewmodel][properties]") {
    Document doc;
    const ObjectId poly = add(doc, PolygonParams{{50, 50}, 6, 10, 0});
    CHECK(find(describeSelection(doc), "sides")->text == "6");
    REQUIRE(setProperty(doc, "sides", "8").ok);
    REQUIRE(setProperty(doc, "rotation", "22.5").ok);
    const auto& p = std::get<PolygonParams>(doc.findObject(poly)->asVector()->params);
    CHECK(p.sides == 8);
    CHECK(std::abs(p.angle - 22.5 * 3.14159265358979323846 / 180.0) < 1e-12);
    CHECK_FALSE(setProperty(doc, "sides", "2").ok);
    CHECK_FALSE(setProperty(doc, "sides", "2.5").ok);
    CHECK_FALSE(setProperty(doc, "sides", "1000000").ok);

    const ObjectId star = add(doc, StarParams{{50, 50}, 5, 20, 8, 0});
    REQUIRE(setProperty(doc, "points", "7").ok);
    REQUIRE(setProperty(doc, "outerRadius", "25").ok);
    REQUIRE(setProperty(doc, "innerRadius", "10").ok);
    CHECK(std::get<StarParams>(doc.findObject(star)->asVector()->params).points == 7);

    add(doc, EllipseParams{{50, 50}, 30, 20, 0});
    REQUIRE(setProperty(doc, "rotation", "-30").ok);
    REQUIRE(setProperty(doc, "height", "25").ok);
    CHECK(doc.checkIntegrity().empty());
}

TEST_CASE("name and layer edits", "[viewmodel][properties]") {
    Document doc;
    REQUIRE(addLayer(doc, "Second").ok);
    const ObjectId id = add(doc, CircleParams{{50, 50}, 5});
    REQUIRE(setProperty(doc, "name", "Hole 1").ok);
    CHECK(doc.findObject(id)->name == "Hole 1");
    CHECK(find(describeSelection(doc), "name")->text == "Hole 1");
    CHECK_FALSE(setProperty(doc, "name", "bad\nname").ok);

    // The new layer became the active one, so the circle was drawn on it.
    CHECK(find(describeSelection(doc), "layer")->text == "Second");
    REQUIRE(setProperty(doc, "layer", "0").ok);
    CHECK(doc.findObject(id)->layer == doc.layers()[0].id);
    CHECK(doc.undoStack().undoText() == "Move to layer");
    CHECK(find(describeSelection(doc), "layer")->text == "Layer 1");

    // Mixed layers: no current choice.
    const ObjectId other = add(doc, CircleParams{{80, 80}, 5}, false);  // on the active layer "Second"
    doc.selection().assign({id, other});
    const PropertyView v = describeSelection(doc);
    CHECK(v.title == "2 objects");
    CHECK(find(v, "layer")->choice == -1);
    CHECK(find(v, "name") == nullptr);
}

TEST_CASE("locked layers are not editable", "[viewmodel][properties]") {
    Document doc;
    const ObjectId id = add(doc, RectangleParams{{50, 50}, 20, 10});
    REQUIRE(setLayerLocked(doc, doc.layers().front().id, true).ok);
    const PropertyView v = describeSelection(doc);
    CHECK_FALSE(find(v, "name")->editable);
    CHECK_FALSE(find(v, "layer")->editable);
    CHECK(find(v, "x") == nullptr);  // no transform targets
    const Object before = *doc.findObject(id);
    CHECK_FALSE(setProperty(doc, "width", "5").ok);
    CHECK_FALSE(setProperty(doc, "name", "x").ok);
    CHECK_FALSE(setProperty(doc, "x", "5").ok);
    CHECK(*doc.findObject(id) == before);
}

TEST_CASE("position size and rotation of plain geometry", "[viewmodel][properties]") {
    Document doc;
    auto cmd = std::make_unique<AddObjectCommand>(Object::vector(
        Contour::rectangle(carvetto::geometry::Box2{{10, 10}, {30, 20}}), doc.activeLayer(), doc.activeSheet()));
    AddObjectCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    doc.selection().assign({raw->id()});

    PropertyView v = describeSelection(doc);
    CHECK(v.title == "Vector");
    REQUIRE(find(v, "x") != nullptr);
    REQUIRE(find(v, "width") != nullptr);
    REQUIRE(find(v, "rotation") != nullptr);
    CHECK(find(v, "x")->text == "10.00");
    CHECK(find(v, "width")->text == "20.00");
    CHECK(find(v, "rotation")->text == "0.00");

    const std::size_t steps = doc.undoStack().count();
    REQUIRE(setProperty(doc, "x", "100").ok);
    REQUIRE(setProperty(doc, "y", "50").ok);
    CHECK(doc.undoStack().count() == steps + 2);  // x then y: two steps, not coalesced
    auto b = doc.findObject(raw->id())->asVector()->bounds();
    CHECK(std::abs(b.min.x - 100) < 1e-9);
    CHECK(std::abs(b.min.y - 50) < 1e-9);

    REQUIRE(setProperty(doc, "width", "40").ok);
    b = doc.findObject(raw->id())->asVector()->bounds();
    CHECK(std::abs(b.width() - 40) < 1e-9);
    CHECK(std::abs(b.height() - 10) < 1e-9);
    CHECK(std::abs(b.min.x - 100) < 1e-9);  // scaled from the lower-left corner

    REQUIRE(setProperty(doc, "rotation", "90").ok);
    b = doc.findObject(raw->id())->asVector()->bounds();
    CHECK(std::abs(b.width() - 10) < 1e-9);
    CHECK(std::abs(b.height() - 40) < 1e-9);
    CHECK(find(describeSelection(doc), "rotation")->text.empty() == false);

    // One undo per edit.
    for (int i = 0; i < 4; ++i) {
        REQUIRE(doc.undo());
    }
    b = doc.findObject(raw->id())->asVector()->bounds();
    CHECK(std::abs(b.min.x - 10) < 1e-9);
    CHECK(std::abs(b.width() - 20) < 1e-9);
}

TEST_CASE("text fields re-lay out the text as one undo step", "[viewmodel][properties]") {
    Document doc;
    const ObjectId id = addText(doc, "Hi");
    const PropertyView v = describeSelection(doc);
    CHECK(v.title == "Text");
    CHECK(find(v, "text")->text == "Hi");
    CHECK(find(v, "text")->multiline);
    CHECK(find(v, "width") == nullptr);  // text has no generic size field

    const double w0 = doc.findObject(id)->asVector()->bounds().width();
    REQUIRE(setProperty(doc, "text", "Hello\nWorld").ok);
    CHECK(std::get<TextParams>(doc.findObject(id)->asVector()->params).text == "Hello\nWorld");
    CHECK(doc.findObject(id)->asVector()->bounds().width() > w0);
    CHECK(doc.undoStack().undoText() == "Edit text");
    REQUIRE(setProperty(doc, "size", "20").ok);
    REQUIRE(setProperty(doc, "align", "2").ok);
    REQUIRE(setProperty(doc, "lineSpacing", "120").ok);
    REQUIRE(setProperty(doc, "kerning", "0").ok);
    REQUIRE(setProperty(doc, "letterSpacing", "1").ok);
    CHECK_FALSE(setProperty(doc, "size", "0").ok);
    CHECK_FALSE(setProperty(doc, "size", "nan").ok);
    CHECK_FALSE(setProperty(doc, "lineSpacing", "-5").ok);
    CHECK_FALSE(setProperty(doc, "text", std::string(300 * 1024, 'x')).ok);

    REQUIRE(doc.undo());
    REQUIRE(doc.undo());
    REQUIRE(doc.undo());
    REQUIRE(doc.undo());
    REQUIRE(doc.undo());
    REQUIRE(doc.undo());
    CHECK(std::get<TextParams>(doc.findObject(id)->asVector()->params).text == "Hi");
    CHECK(doc.checkIntegrity().empty());
}

TEST_CASE("stale selection after the document was replaced is harmless", "[viewmodel][properties]") {
    auto doc = std::make_unique<Document>();
    add(*doc, CircleParams{{50, 50}, 5});
    doc = std::make_unique<Document>();
    CHECK(describeSelection(*doc).fields.empty());
    CHECK_FALSE(setProperty(*doc, "name", "x").ok);
}
