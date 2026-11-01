#include "carvetto/document/document.hpp"

#include <catch2/catch_test_macros.hpp>

#include "carvetto/geometry/contour.hpp"

using namespace carvetto::document;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::Vec2;

namespace {

Object square(const Document& doc, double x, double y, double size = 10.0) {
    return Object::vector(Contour::rectangle(Box2{Vec2{x, y}, Vec2{x + size, y + size}}), doc.activeLayer(),
                          doc.activeSheet());
}

}  // namespace

TEST_CASE("a new document has one sheet and one layer sized from the job", "[document]") {
    const Job job = Job::singleSided(10.0, 8.0, 0.5, Units::Inches);
    Document doc(job);
    CHECK(doc.job() == job);
    REQUIRE(doc.sheets().size() == 1);
    REQUIRE(doc.layers().size() == 1);
    CHECK(doc.sheets().front().sizeMm == job.sizeMm);
    CHECK(doc.activeSheet() == doc.sheets().front().id);
    CHECK(doc.activeLayer() == doc.layers().front().id);
    CHECK(doc.defaultLayer() == doc.layers().front().id);
    CHECK(doc.objectCount() == 0);
    CHECK(doc.selection().empty());
    CHECK(doc.variables().empty());
    CHECK_FALSE(doc.undoStack().canUndo());

    Document plain;
    CHECK(plain.job().sizeMm == JobSize{300.0, 300.0, 18.0});
}

TEST_CASE("layers are ordered, reorderable and protected when last or non-empty", "[document][layer]") {
    Document doc;
    const LayerId base = doc.defaultLayer();
    const LayerId top = doc.insertLayer(Layer{.name = "Top"});
    const LayerId middle = doc.insertLayer(Layer{.name = "Middle"}, 1);
    REQUIRE(doc.layers().size() == 3);
    CHECK(doc.layerIndex(base) == 0);
    CHECK(doc.layerIndex(middle) == 1);
    CHECK(doc.layerIndex(top) == 2);
    CHECK_FALSE(doc.layerIndex(LayerId{999}).has_value());
    CHECK(doc.findLayer(top)->name == "Top");
    CHECK(doc.findLayer(LayerId{999}) == nullptr);

    CHECK(doc.moveLayer(top, 0));
    CHECK(doc.layers()[0].id == top);
    CHECK(doc.layers()[1].id == base);
    CHECK(doc.defaultLayer() == top);  // the default layer is whatever sits at the bottom
    CHECK_FALSE(doc.moveLayer(top, 3));
    CHECK_FALSE(doc.moveLayer(LayerId{999}, 0));

    CHECK(doc.setActiveLayer(middle));
    CHECK_FALSE(doc.setActiveLayer(LayerId{999}));
    const ObjectId obj = doc.insertObject(square(doc, 0, 0));
    CHECK(doc.findObject(obj)->layer == middle);
    CHECK_FALSE(doc.layerIsEmpty(middle));
    CHECK(doc.objectsOnLayer(middle) == std::vector<ObjectId>{obj});

    CHECK_FALSE(doc.takeLayer(middle).has_value());  // non-empty
    doc.takeObject(obj);
    const auto taken = doc.takeLayer(middle);
    REQUIRE(taken.has_value());
    CHECK(taken->name == "Middle");
    CHECK(doc.activeLayer() == doc.defaultLayer());  // fell back
    CHECK(doc.takeLayer(base).has_value());
    CHECK_FALSE(doc.takeLayer(top).has_value());  // last layer stays
}

TEST_CASE("sheets are protected when last or non-empty", "[document][sheet]") {
    Document doc;
    const SheetId first = doc.activeSheet();
    const SheetId second = doc.insertSheet(Sheet{.name = "Back"});
    CHECK(doc.sheetIndex(second) == 1);
    CHECK(doc.setActiveSheet(second));
    CHECK_FALSE(doc.setActiveSheet(SheetId{999}));

    Object obj = square(doc, 0, 0);
    obj.sheet = second;
    const ObjectId id = doc.insertObject(obj);
    CHECK(doc.objectsOnSheet(second) == std::vector<ObjectId>{id});
    CHECK_FALSE(doc.takeSheet(second).has_value());
    doc.takeObject(id);
    CHECK(doc.takeSheet(second).has_value());
    CHECK(doc.activeSheet() == first);
    CHECK_FALSE(doc.takeSheet(first).has_value());
}

TEST_CASE("objects get monotonically increasing ids and iterate in id order", "[document][object]") {
    Document doc;
    const ObjectId a = doc.insertObject(square(doc, 0, 0));
    const ObjectId b = doc.insertObject(square(doc, 20, 0));
    CHECK(a < b);
    CHECK(doc.objectIds() == std::vector<ObjectId>{a, b});
    CHECK(doc.hasObject(a));
    CHECK(doc.findObject(ObjectId{12345}) == nullptr);

    doc.selection().add(a);
    const auto taken = doc.takeObject(a);
    REQUIRE(taken.has_value());
    CHECK(taken->id == a);
    CHECK_FALSE(doc.selection().contains(a));  // removal deselects
    CHECK_FALSE(doc.takeObject(a).has_value());

    // Re-inserting with its old id (what undo does) keeps the identity and never collides.
    CHECK(doc.insertObject(*taken) == a);
    const ObjectId c = doc.insertObject(square(doc, 40, 0));
    CHECK(c > b);
}

TEST_CASE("object kinds, payload accessors and bounds", "[document][object]") {
    Document doc;
    const ObjectId v = doc.insertObject(square(doc, 0, 0));
    CHECK(doc.findObject(v)->kind() == ObjectKind::Vector);
    CHECK(doc.findObject(v)->isVector());
    CHECK(doc.findObject(v)->asVector() != nullptr);
    CHECK(doc.findObject(v)->asBitmap() == nullptr);
    CHECK(std::holds_alternative<NoParams>(doc.findObject(v)->asVector()->params));
    CHECK(doc.objectBounds(v) == Box2{Vec2{0, 0}, Vec2{10, 10}});

    Object bitmap;
    bitmap.layer = doc.activeLayer();
    bitmap.sheet = doc.activeSheet();
    BitmapObject raster;
    raster.widthPx = 64;
    raster.heightPx = 32;
    raster.placementMm = Box2{Vec2{50, 50}, Vec2{70, 60}};
    bitmap.data = raster;
    const ObjectId b = doc.insertObject(bitmap);
    CHECK(doc.findObject(b)->kind() == ObjectKind::Bitmap);
    CHECK(doc.objectBounds(b) == Box2{Vec2{50, 50}, Vec2{70, 60}});

    Object group;
    group.layer = doc.activeLayer();
    group.sheet = doc.activeSheet();
    group.data = GroupObject{{v, b, ObjectId{999}}};  // the unknown child is ignored
    const ObjectId g = doc.insertObject(group);
    CHECK(doc.findObject(g)->kind() == ObjectKind::Group);
    CHECK(doc.objectBounds(g) == Box2{Vec2{0, 0}, Vec2{70, 60}});
    CHECK_FALSE(doc.objectBounds(ObjectId{999}).has_value());

    // Parametric object keeps its creation parameters.
    const CircleParams params{.centreMm = {5, 5}, .radiusMm = 5};
    const ObjectId circle = doc.insertObject(
        Object::vector(Contour::circle({5, 5}, 5), doc.activeLayer(), doc.activeSheet(), params));
    REQUIRE(std::holds_alternative<CircleParams>(doc.findObject(circle)->asVector()->params));
    CHECK(std::get<CircleParams>(doc.findObject(circle)->asVector()->params) == params);
}

TEST_CASE("copyObjects deep-copies groups with their descendants", "[document][object]") {
    Document doc;
    const ObjectId a = doc.insertObject(square(doc, 0, 0));
    const ObjectId b = doc.insertObject(square(doc, 20, 0));
    const ObjectId loose = doc.insertObject(square(doc, 40, 0));
    Object group;
    group.layer = doc.activeLayer();
    group.sheet = doc.activeSheet();
    group.data = GroupObject{{a, b}};
    const ObjectId g = doc.insertObject(group);
    doc.findObject(a)->parent = g;
    doc.findObject(b)->parent = g;

    const auto clip = doc.copyObjects({g, loose, g, ObjectId{999}});
    REQUIRE(clip.size() == 4);
    CHECK(clip[0].id == a);
    CHECK(clip[1].id == b);
    CHECK(clip[2].id == loose);
    CHECK(clip[3].id == g);
    CHECK(clip[0].parent == g);

    // Copying a child alone detaches it from the parent that was not copied.
    const auto childOnly = doc.copyObjects({a});
    REQUIRE(childOnly.size() == 1);
    CHECK_FALSE(childOnly[0].parent.has_value());
}
