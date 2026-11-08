#include "carvetto/document/node_edit_command.hpp"

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <memory>
#include <vector>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/undo_stack.hpp"

using namespace carvetto::document;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::LineSpan;
using carvetto::geometry::Vec2;

namespace {

ObjectId addRect(Document& doc, CreationParams params = NoParams{}) {
    auto cmd = std::make_unique<AddObjectCommand>(Object::vector(
        Contour::rectangle(Box2{Vec2{0, 0}, Vec2{10, 5}}), doc.activeLayer(), doc.activeSheet(), params));
    AddObjectCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    return raw->id();
}

Contour triangle(double apexY) {
    Contour c;
    c.append(LineSpan{{0, 0}, {10, 0}});
    c.append(LineSpan{{10, 0}, {5, apexY}});
    c.append(LineSpan{{5, apexY}, {0, 0}});
    c.closed = true;
    return c;
}

}  // namespace

TEST_CASE("SetObjectContoursCommand replaces contours, drops params and undoes exactly", "[document][node]") {
    Document doc;
    RectangleParams rp;
    rp.widthMm = 10;
    rp.heightMm = 5;
    rp.centreMm = {5, 2.5};
    const ObjectId id = addRect(doc, rp);
    const Object before = *doc.findObject(id);

    REQUIRE(doc.execute(std::make_unique<SetObjectContoursCommand>(
        std::vector<ContourReplacement>{{id, {triangle(7)}}})));
    CHECK(doc.findObject(id)->asVector()->contours == std::vector<Contour>{triangle(7)});
    CHECK(std::holds_alternative<NoParams>(doc.findObject(id)->asVector()->params));

    REQUIRE(doc.undoStack().undo(doc));
    CHECK(*doc.findObject(id) == before);  // params restored exactly
    REQUIRE(doc.undoStack().redo(doc));
    CHECK(doc.findObject(id)->asVector()->contours == std::vector<Contour>{triangle(7)});
}

TEST_CASE("SetObjectContoursCommand is all or nothing", "[document][node]") {
    Document doc;
    const ObjectId a = addRect(doc);
    const ObjectId b = addRect(doc);
    const std::size_t steps = doc.undoStack().count();
    const Object snapshotA = *doc.findObject(a);

    auto run = [&](std::vector<ContourReplacement> items) {
        return doc.execute(std::make_unique<SetObjectContoursCommand>(std::move(items)));
    };
    CHECK_FALSE(run({}));
    CHECK_FALSE(run({{a, {triangle(7)}}, {ObjectId{999}, {triangle(7)}}}));  // unknown id
    CHECK_FALSE(run({{a, {triangle(7)}}, {a, {triangle(8)}}}));              // repeated id
    CHECK_FALSE(run({{a, {triangle(std::numeric_limits<double>::quiet_NaN())}}, {b, {triangle(3)}}}));
    CHECK_FALSE(run({{a, {triangle(std::numeric_limits<double>::infinity())}}}));
    CHECK_FALSE(run({{a, {triangle(2e9)}}}));
    CHECK_FALSE(run({{a, doc.findObject(a)->asVector()->contours}}));  // no change at all
    CHECK(doc.undoStack().count() == steps);
    CHECK(*doc.findObject(a) == snapshotA);

    // A group is not a vector object.
    auto grp = std::make_unique<GroupObjectsCommand>(std::vector<ObjectId>{a, b});
    GroupObjectsCommand* rawGrp = grp.get();
    REQUIRE(doc.execute(std::move(grp)));
    CHECK_FALSE(run({{rawGrp->groupId(), {triangle(7)}}}));
}

TEST_CASE("SetObjectContoursCommand coalesces a drag into one step", "[document][node]") {
    Document doc;
    const ObjectId id = addRect(doc);
    const Object before = *doc.findObject(id);
    const std::size_t steps = doc.undoStack().count();

    doc.undoStack().breakCoalescing();
    for (int i = 1; i <= 20; ++i) {
        REQUIRE(doc.execute(std::make_unique<SetObjectContoursCommand>(
            std::vector<ContourReplacement>{{id, {triangle(5.0 + i)}}})));
    }
    doc.undoStack().breakCoalescing();
    CHECK(doc.undoStack().count() == steps + 1);
    CHECK(doc.findObject(id)->asVector()->contours == std::vector<Contour>{triangle(25.0)});

    REQUIRE(doc.undoStack().undo(doc));
    CHECK(*doc.findObject(id) == before);
    REQUIRE(doc.undoStack().redo(doc));
    CHECK(doc.findObject(id)->asVector()->contours == std::vector<Contour>{triangle(25.0)});

    // A different id set does not merge.
    const ObjectId other = addRect(doc);
    doc.undoStack().breakCoalescing();
    REQUIRE(doc.execute(std::make_unique<SetObjectContoursCommand>(
        std::vector<ContourReplacement>{{id, {triangle(30)}}})));
    REQUIRE(doc.execute(std::make_unique<SetObjectContoursCommand>(
        std::vector<ContourReplacement>{{other, {triangle(30)}}})));
    CHECK(doc.undoStack().count() == steps + 1 + 1 + 2);
}

TEST_CASE("UndoStack::undoAndDiscard reverts and forgets the latest step", "[document][undo]") {
    Document doc;
    const ObjectId id = addRect(doc);
    doc.undoStack().setClean();
    const Object before = *doc.findObject(id);

    REQUIRE(doc.execute(std::make_unique<SetObjectContoursCommand>(
        std::vector<ContourReplacement>{{id, {triangle(7)}}})));
    REQUIRE(doc.undoStack().undoAndDiscard(doc));
    CHECK(*doc.findObject(id) == before);
    CHECK_FALSE(doc.undoStack().canRedo());
    CHECK(doc.undoStack().isClean());

    // Not possible with a redo tail pending, nor when empty of steps.
    REQUIRE(doc.execute(std::make_unique<SetObjectContoursCommand>(
        std::vector<ContourReplacement>{{id, {triangle(7)}}})));
    REQUIRE(doc.undoStack().undo(doc));
    CHECK_FALSE(doc.undoStack().undoAndDiscard(doc));
    CHECK(doc.undoStack().canRedo());

    Document empty;
    CHECK_FALSE(empty.undoStack().undoAndDiscard(empty));
}

TEST_CASE("UndoStack::undoAndDiscard clears the clean point of a dropped step", "[document][undo]") {
    Document doc;
    const ObjectId id = addRect(doc);
    REQUIRE(doc.execute(std::make_unique<SetObjectContoursCommand>(
        std::vector<ContourReplacement>{{id, {triangle(7)}}})));
    doc.undoStack().setClean();
    REQUIRE(doc.undoStack().undoAndDiscard(doc));
    CHECK_FALSE(doc.undoStack().isClean());
}

namespace {
bool pushEdit(Document& doc, ObjectId id, double apex) {
    return doc.execute(std::make_unique<SetObjectContoursCommand>(
        std::vector<ContourReplacement>{{id, {triangle(apex)}}}));
}
}  // namespace

TEST_CASE("undoAndDiscard bumps the generation, which stays unique", "[document][undo]") {
    Document doc;
    const ObjectId id = addRect(doc);
    std::set<std::uint64_t> seen{doc.undoStack().generation()};
    auto fresh = [&] { return seen.insert(doc.undoStack().generation()).second; };
    REQUIRE(pushEdit(doc, id, 7));
    CHECK(fresh());
    doc.undoStack().breakCoalescing();
    REQUIRE(pushEdit(doc, id, 8));  // a coalesced step also changes the generation
    CHECK(fresh());
    doc.undoStack().beginMacro("m");
    REQUIRE(pushEdit(doc, id, 9));
    CHECK(fresh());  // pushes inside a macro too
    doc.undoStack().endMacro();
    REQUIRE(doc.undoStack().undoAndDiscard(doc));
    CHECK(fresh());
}

TEST_CASE("undoAndDiscard: clean point, refusals, no coalescing into a cancelled step", "[document][undo]") {
    Document doc;
    const ObjectId id = addRect(doc);
    doc.undoStack().setClean();
    REQUIRE(pushEdit(doc, id, 7));
    REQUIRE(doc.undoStack().undoAndDiscard(doc));
    CHECK(doc.undoStack().isClean());
    // A different step pushed afterwards is not clean.
    REQUIRE(pushEdit(doc, id, 9));
    CHECK_FALSE(doc.undoStack().isClean());
    REQUIRE(doc.undoStack().undoAndDiscard(doc));
    CHECK(doc.undoStack().isClean());

    // The step after a cancel starts a new run: it does not merge into whatever came before.
    REQUIRE(pushEdit(doc, id, 5));
    doc.undoStack().breakCoalescing();
    REQUIRE(pushEdit(doc, id, 6));
    const std::size_t count = doc.undoStack().count();
    REQUIRE(doc.undoStack().undoAndDiscard(doc));
    CHECK(doc.undoStack().count() == count - 1);
    REQUIRE(pushEdit(doc, id, 11));
    CHECK(doc.undoStack().count() == count);  // not merged into the step before the cancelled one

    // Refusals: inside a macro, with a redo tail.
    doc.undoStack().beginMacro("m");
    CHECK_FALSE(doc.undoStack().undoAndDiscard(doc));
    doc.undoStack().endMacro();
    REQUIRE(doc.undoStack().undo(doc));
    CHECK_FALSE(doc.undoStack().undoAndDiscard(doc));
}
