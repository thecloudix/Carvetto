#include "carvetto/document/object_property_commands.hpp"

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/shape_params.hpp"

using namespace carvetto::document;
using carvetto::geometry::Vec2;

namespace {

ObjectId addShape(Document& doc, CreationParams params) {
    auto built = buildContours(params);
    REQUIRE(built.ok());
    auto cmd = std::make_unique<AddObjectCommand>(
        Object::vector(built.value(), doc.activeLayer(), doc.activeSheet(), params));
    AddObjectCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    return raw->id();
}

}  // namespace

TEST_CASE("nameProblem bounds names", "[document][properties]") {
    CHECK(nameProblem("Layer 2", true).empty());
    CHECK(nameProblem("caf\xc3\xa9", true).empty());
    CHECK_FALSE(nameProblem("", true).empty());
    CHECK_FALSE(nameProblem("   ", true).empty());
    CHECK(nameProblem("", false).empty());
    CHECK_FALSE(nameProblem("a\nb", false).empty());
    CHECK_FALSE(nameProblem(std::string("a\0b", 3), false).empty());
    CHECK_FALSE(nameProblem(std::string(kMaxNameBytes + 1, 'x'), false).empty());
    CHECK(nameProblem(std::string(kMaxNameBytes, 'x'), true).empty());
}

TEST_CASE("RenameObjectCommand renames in one undoable step", "[document][properties]") {
    Document doc;
    const ObjectId id = addShape(doc, CircleParams{{10, 10}, 5});
    REQUIRE(doc.execute(std::make_unique<RenameObjectCommand>(id, "Wheel")));
    CHECK(doc.findObject(id)->name == "Wheel");
    CHECK(doc.undoStack().undoText() == "Rename object");
    CHECK_FALSE(doc.execute(std::make_unique<RenameObjectCommand>(id, "Wheel")));  // no change
    CHECK_FALSE(doc.execute(std::make_unique<RenameObjectCommand>(id, "bad\tname")));
    CHECK_FALSE(doc.execute(std::make_unique<RenameObjectCommand>(ObjectId{9999}, "x")));
    CHECK(doc.findObject(id)->name == "Wheel");
    REQUIRE(doc.undo());
    CHECK(doc.findObject(id)->name.empty());
    REQUIRE(doc.redo());
    CHECK(doc.findObject(id)->name == "Wheel");
}

TEST_CASE("SetShapeParamsCommand rebuilds contours and undoes exactly", "[document][properties]") {
    Document doc;
    const ObjectId id = addShape(doc, RectangleParams{{50, 50}, 20, 10});
    const VectorObject original = *doc.findObject(id)->asVector();

    RectangleParams next{{50, 50}, 40, 10};
    REQUIRE(doc.execute(std::make_unique<SetShapeParamsCommand>(id, next)));
    const VectorObject* now = doc.findObject(id)->asVector();
    CHECK(std::get<RectangleParams>(now->params).widthMm == 40.0);
    CHECK(now->bounds().width() == 40.0);

    REQUIRE(doc.undo());
    CHECK(*doc.findObject(id)->asVector() == original);
    REQUIRE(doc.redo());
    CHECK(doc.findObject(id)->asVector()->bounds().width() == 40.0);
}

TEST_CASE("SetShapeParamsCommand refuses bad input without changing anything", "[document][properties]") {
    Document doc;
    const ObjectId rect = addShape(doc, RectangleParams{{0, 0}, 20, 10});
    const ObjectId poly = addShape(doc, PolygonParams{{0, 0}, 6, 10, 0});
    const std::size_t steps = doc.undoStack().count();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    auto refused = [&](ObjectId id, CreationParams p) {
        const Object* now = doc.findObject(id);
        const std::optional<Object> before = now != nullptr ? std::optional<Object>(*now) : std::nullopt;
        const bool ok = doc.execute(std::make_unique<SetShapeParamsCommand>(id, std::move(p)));
        const Object* after = doc.findObject(id);
        const bool same = before.has_value() ? (after != nullptr && *after == *before) : after == nullptr;
        return !ok && same && doc.undoStack().count() == steps;
    };
    CHECK(refused(rect, RectangleParams{{0, 0}, nan, 10}));
    CHECK(refused(rect, RectangleParams{{inf, 0}, 20, 10}));
    CHECK(refused(rect, RectangleParams{{0, 0}, 2e9, 10}));
    CHECK(refused(rect, RectangleParams{{0, 0}, -5, 10}));
    CHECK(refused(rect, RectangleParams{{0, 0}, 20, 10}));              // unchanged
    CHECK(refused(rect, CircleParams{{0, 0}, 5}));                      // other kind
    CHECK(refused(rect, TextParams{}));                                 // text
    CHECK(refused(poly, PolygonParams{{0, 0}, 2, 10, 0}));              // too few sides
    CHECK(refused(poly, PolygonParams{{0, 0}, 1000000, 10, 0}));        // too many sides
    CHECK(refused(poly, PolygonParams{{0, 0}, 6, 10, nan}));
    CHECK(refused(ObjectId{9999}, RectangleParams{}));
}

TEST_CASE("SetShapeParamsCommand refuses plain geometry", "[document][properties]") {
    Document doc;
    auto cmd = std::make_unique<AddObjectCommand>(Object::vector(
        carvetto::geometry::Contour::circle({0, 0}, 5), doc.activeLayer(), doc.activeSheet()));
    AddObjectCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    CHECK_FALSE(doc.execute(std::make_unique<SetShapeParamsCommand>(raw->id(), CircleParams{{0, 0}, 6})));
}

TEST_CASE("UndoStack::stepText lists the history", "[document][properties]") {
    Document doc;
    const ObjectId id = addShape(doc, CircleParams{{0, 0}, 5});
    REQUIRE(doc.execute(std::make_unique<RenameObjectCommand>(id, "A")));
    REQUIRE(doc.undo());
    CHECK(doc.undoStack().count() == 2);
    CHECK(doc.undoStack().index() == 1);
    CHECK(doc.undoStack().stepText(0) == "Add object");
    CHECK(doc.undoStack().stepText(1) == "Rename object");
    CHECK(doc.undoStack().stepText(2).empty());
}
