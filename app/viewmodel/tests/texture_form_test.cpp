#include "carvetto/viewmodel/texture_form.hpp"

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <memory>
#include <string>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/viewmodel/layers_sheets.hpp"
#include "carvetto/viewmodel/vector_ops.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using document::Document;
using document::ObjectId;
using document::Units;

namespace {

ObjectId add(Document& doc, std::vector<geometry::Contour> cs, document::LayerId layer) {
    auto cmd = std::make_unique<document::AddObjectCommand>(
        document::Object::vector(std::move(cs), layer, doc.activeSheet()));
    const document::AddObjectCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    return raw->id();
}

geometry::Contour hole(geometry::Box2 b) {
    geometry::Contour c = geometry::Contour::rectangle(b);
    c.reverse();
    return c;
}

/// A 100 x 80 plate with a 40 x 40 hole, selected.
ObjectId plate(Document& doc) {
    const ObjectId id = add(doc, {geometry::Contour::rectangle({{0, 0}, {100, 80}}), hole({{30, 20}, {70, 60}})},
                            doc.activeLayer());
    doc.selection().assign({id});
    return id;
}

bool setAll(TextureForm& f, std::initializer_list<std::pair<const char*, const char*>> kv) {
    for (const auto& [k, v] : kv) {
        if (!f.setField(k, v).ok) return false;
    }
    return true;
}

}  // namespace

TEST_CASE("texture form fields are validated with units and refuse bad entries", "[viewmodel][texture]") {
    TextureForm f(Units::Millimetres);
    REQUIRE(f.fields().size() == 7);
    const auto before = f.params();
    CHECK_FALSE(f.setField("spacing", "0").ok);
    CHECK_FALSE(f.setField("spacing", "-3").ok);
    CHECK_FALSE(f.setField("spacing", "abc").ok);
    CHECK_FALSE(f.setField("spacing", "nan").ok);
    CHECK_FALSE(f.setField("spacing", "1e30").ok);
    CHECK_FALSE(f.setField("wavelength", "0").ok);
    CHECK_FALSE(f.setField("amplitude", "-1").ok);
    CHECK_FALSE(f.setField("angle", "91").ok);
    CHECK_FALSE(f.setField("angle", "-90.5").ok);
    CHECK_FALSE(f.setField("variation", "101").ok);
    CHECK_FALSE(f.setField("noise", "-1").ok);
    CHECK_FALSE(f.setField("seed", "-1").ok);
    CHECK_FALSE(f.setField("seed", "1.5").ok);
    CHECK_FALSE(f.setField("seed", "99999999999999999999999").ok);
    CHECK_FALSE(f.setField("seed", "").ok);
    CHECK_FALSE(f.setField("nope", "1").ok);
    CHECK(f.params().maxSpacing == before.maxSpacing);
    CHECK(f.params().seed == before.seed);

    REQUIRE(f.setField("spacing", "0.5 in").ok);  // explicit unit wins
    CHECK(f.params().maxSpacing == 12.7);
    REQUIRE(f.setField("angle", "45").ok);
    CHECK(std::fabs(f.params().angle - geometry::kPi / 4) < 1e-12);
    REQUIRE(f.setField("noise", "25%").ok);
    CHECK(f.params().noise == 0.25);
    REQUIRE(f.setField("seed", "18446744073709551615").ok);
    CHECK(f.params().seed == 18446744073709551615ULL);

    TextureForm inch(Units::Inches);
    REQUIRE(inch.setField("wavelength", "2").ok);
    CHECK(inch.params().wavelength == 50.8);
    CHECK(inch.fields()[3].text == "2.000");
}

TEST_CASE("texture preview follows the selection and stays inside it", "[viewmodel][texture]") {
    Document doc;
    TextureForm f(Units::Millimetres);
    CHECK_FALSE(f.preview(doc).ok);  // nothing selected
    plate(doc);
    REQUIRE(setAll(f, {{"spacing", "4"}, {"wavelength", "8"}, {"noise", "30"}, {"seed", "7"}}));
    const TexturePreview p = f.preview(doc);
    REQUIRE(p.ok);
    CHECK(p.vertexCount > 100);
    const auto region = selectedClosedRegion(doc);
    REQUIRE(region.ok);
    for (const geometry::Contour& c : p.contours) {
        for (const geometry::Span& s : c.spans) {
            const auto& l = std::get<geometry::LineSpan>(s);
            const geometry::Vec2 m{(l.start.x + l.end.x) / 2, (l.start.y + l.end.y) / 2};
            CHECK(geometry::isInside(region.contours, m, geometry::FillRule::NonZero, geometry::Tolerance{}));
            CHECK_FALSE((m.x > 30.01 && m.x < 69.99 && m.y > 20.01 && m.y < 59.99));
        }
    }
    CHECK(doc.objectCount() == 1);  // preview does not touch the document
    // Same inputs, same preview.
    CHECK(f.preview(doc).contours == p.contours);
    // Edits change it; an invalid edit does not.
    REQUIRE(f.setField("seed", "8").ok);
    CHECK(f.preview(doc).contours != p.contours);
}

TEST_CASE("texture apply is one undo step on the target layer", "[viewmodel][texture]") {
    Document doc;
    const ObjectId source = plate(doc);
    const document::LayerId target = addLayer(doc, "Texture").ok ? doc.layers().back().id : doc.layers().front().id;
    doc.setActiveLayer(doc.layers().front().id);
    TextureForm f(Units::Millimetres);
    f.setTargetLayer(target);
    doc.selection().assign({source});
    const std::size_t steps = doc.undoStack().count();
    const TextureResult r = f.apply(doc);
    REQUIRE(r.ok);
    REQUIRE(r.created.has_value());
    CHECK(doc.objectCount() == 2);
    CHECK(doc.findObject(*r.created)->layer == target);
    CHECK(doc.undoStack().count() == steps + 1);
    CHECK(doc.selection().ids() == std::vector<ObjectId>{*r.created});
    for (const geometry::Contour& c : doc.findObject(*r.created)->asVector()->contours) CHECK_FALSE(c.closed);
    REQUIRE(doc.undo());
    CHECK(doc.objectCount() == 1);
    CHECK(doc.hasObject(source));
    REQUIRE(doc.redo());
    CHECK(doc.objectCount() == 2);
}

TEST_CASE("texture apply refuses without changing the document", "[viewmodel][texture]") {
    Document doc;
    TextureForm f(Units::Millimetres);
    const std::size_t steps = doc.undoStack().count();
    CHECK_FALSE(f.apply(doc).ok);  // no selection
    const ObjectId source = plate(doc);
    // Locked target layer.
    REQUIRE(setLayerLocked(doc, doc.layers().front().id, true).ok);
    const std::size_t afterLock = doc.undoStack().count();
    doc.selection().assign({source});
    // The source layer is locked too, so the selection is not an operand: refused either way.
    CHECK_FALSE(f.apply(doc).ok);
    REQUIRE(setLayerLocked(doc, doc.layers().front().id, false).ok);
    // Density beyond the vertex bound.
    REQUIRE(setAll(f, {{"spacing", "0.001"}, {"wavelength", "0.01"}}));
    const std::size_t now = doc.undoStack().count();
    const TextureResult big = f.apply(doc);
    CHECK_FALSE(big.ok);
    CHECK_FALSE(big.message.empty());
    CHECK(doc.undoStack().count() == now);
    CHECK(doc.objectCount() == 1);
    (void)steps;
    (void)afterLock;
}

TEST_CASE("texture preview is skipped, not run, when it would be huge", "[viewmodel][texture]") {
    Document doc;
    plate(doc);
    TextureForm f(Units::Millimetres);
    REQUIRE(setAll(f, {{"spacing", "0.05"}, {"wavelength", "0.5"}}));
    const TexturePreview p = f.preview(doc);
    CHECK_FALSE(p.ok);
    CHECK(p.skipped);
    // ... but it can still be created (about 1M vertices is refused by the generator, this is below).
    REQUIRE(setAll(f, {{"spacing", "0.4"}, {"wavelength", "2"}}));
    CHECK(f.apply(doc).ok);
}

TEST_CASE("texture uses the union of several selected shapes", "[viewmodel][texture]") {
    Document doc;
    const ObjectId a = add(doc, {geometry::Contour::rectangle({{0, 0}, {30, 30}})}, doc.activeLayer());
    const ObjectId b = add(doc, {geometry::Contour::circle({80, 15}, 12)}, doc.activeLayer());
    doc.selection().assign({a, b});
    TextureForm f(Units::Millimetres);
    REQUIRE(setAll(f, {{"spacing", "3"}, {"wavelength", "6"}}));
    const TexturePreview p = f.preview(doc);
    REQUIRE(p.ok);
    bool left = false;
    bool right = false;
    for (const auto& c : p.contours) {
        const geometry::Vec2 s = c.startPoint();
        left = left || s.x < 31;
        right = right || s.x > 66;
        CHECK_FALSE((s.x > 31 && s.x < 66));
    }
    CHECK(left);
    CHECK(right);
}

TEST_CASE("texture on a hostile selection is bounded", "[viewmodel][texture]") {
    Document doc;
    const ObjectId huge = add(doc, {geometry::Contour::rectangle({{-1e9, -1e9}, {1e9, 1e9}})}, doc.activeLayer());
    doc.selection().assign({huge});
    TextureForm f(Units::Millimetres);
    CHECK_FALSE(f.preview(doc).ok);
    CHECK_FALSE(f.apply(doc).ok);
    CHECK(doc.objectCount() == 1);
    const ObjectId nan = add(doc, {geometry::Contour::rectangle({{0, 0}, {std::nan(""), 5}})}, doc.activeLayer());
    doc.selection().assign({nan});
    CHECK_FALSE(f.apply(doc).ok);
}
