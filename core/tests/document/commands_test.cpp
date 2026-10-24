#include "carvetto/document/commands.hpp"

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cstddef>
#include <ctime>
#include <limits>
#include <memory>
#include <set>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour.hpp"

using namespace carvetto::document;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::Transform2;
using carvetto::geometry::Vec2;
using Catch::Matchers::WithinAbs;

namespace {

Object square(const Document& doc, double x, double y, double size = 10.0) {
    return Object::vector(Contour::rectangle(Box2{Vec2{x, y}, Vec2{x + size, y + size}}), doc.activeLayer(),
                          doc.activeSheet());
}

ObjectId addSquare(Document& doc, double x, double y) {
    auto cmd = std::make_unique<AddObjectCommand>(square(doc, x, y));
    AddObjectCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    return raw->id();
}

LayerId addLayer(Document& doc, const std::string& name, bool makeActive = false) {
    auto cmd = std::make_unique<AddLayerCommand>(Layer{.name = name}, makeActive);
    AddLayerCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    return raw->id();
}

}  // namespace

// ------------------------------------------------------------------------------------------------
// Acceptance criteria
// ------------------------------------------------------------------------------------------------

TEST_CASE("AC2: add object, undo, redo round-trips identity and layer membership", "[document][commands]") {
    Document doc;
    const LayerId layer = addLayer(doc, "Vectors", /*makeActive=*/true);
    REQUIRE(doc.activeLayer() == layer);

    const ObjectId id = addSquare(doc, 0, 0);
    REQUIRE(doc.hasObject(id));
    CHECK(doc.findObject(id)->layer == layer);
    CHECK(doc.objectsOnLayer(layer) == std::vector<ObjectId>{id});
    const Object snapshot = *doc.findObject(id);

    REQUIRE(doc.undo());
    CHECK_FALSE(doc.hasObject(id));
    CHECK(doc.layerIsEmpty(layer));
    CHECK(doc.undoStack().redoText() == "Add object");

    REQUIRE(doc.redo());
    REQUIRE(doc.hasObject(id));
    CHECK(*doc.findObject(id) == snapshot);
    CHECK(doc.findObject(id)->layer == layer);
    CHECK(doc.objectsOnLayer(layer) == std::vector<ObjectId>{id});

    // Undo twice removes the layer too, redo twice brings both back with the same ids.
    REQUIRE(doc.undo());
    REQUIRE(doc.undo());
    CHECK(doc.findLayer(layer) == nullptr);
    CHECK(doc.activeLayer() == doc.defaultLayer());
    REQUIRE(doc.redo());
    REQUIRE(doc.redo());
    CHECK(doc.findLayer(layer)->name == "Vectors");
    CHECK(doc.activeLayer() == layer);
    CHECK(doc.findObject(id)->layer == layer);
}

TEST_CASE("AC3: deleting a layer moves its objects to the default layer unless deleteContents",
          "[document][commands]") {
    Document doc;
    const LayerId base = doc.defaultLayer();
    const LayerId work = addLayer(doc, "Work", /*makeActive=*/true);
    const ObjectId a = addSquare(doc, 0, 0);
    const ObjectId b = addSquare(doc, 20, 0);
    const std::size_t stepsBefore = doc.undoStack().index();

    SECTION("move contents to the default layer") {
        REQUIRE(doc.execute(std::make_unique<DeleteLayerCommand>(work, /*deleteContents=*/false)));
        CHECK(doc.findLayer(work) == nullptr);
        CHECK(doc.layers().size() == 1);
        CHECK(doc.objectCount() == 2);
        CHECK(doc.findObject(a)->layer == base);
        CHECK(doc.findObject(b)->layer == base);
        CHECK(doc.activeLayer() == base);

        REQUIRE(doc.undo());
        REQUIRE(doc.findLayer(work) != nullptr);
        CHECK(doc.layerIndex(work) == std::size_t{1});
        CHECK(doc.findObject(a)->layer == work);
        CHECK(doc.findObject(b)->layer == work);
        CHECK(doc.activeLayer() == work);
        CHECK(doc.undoStack().index() == stepsBefore);

        REQUIRE(doc.redo());
        CHECK(doc.findLayer(work) == nullptr);
        CHECK(doc.findObject(a)->layer == base);
    }

    SECTION("delete contents") {
        REQUIRE(doc.execute(std::make_unique<DeleteLayerCommand>(work, /*deleteContents=*/true)));
        CHECK(doc.findLayer(work) == nullptr);
        CHECK(doc.objectCount() == 0);
        CHECK(doc.layerIsEmpty(base));

        REQUIRE(doc.undo());
        REQUIRE(doc.findLayer(work) != nullptr);
        CHECK(doc.objectCount() == 2);
        CHECK(doc.findObject(a)->layer == work);
        CHECK(doc.findObject(b)->layer == work);

        REQUIRE(doc.redo());
        CHECK(doc.objectCount() == 0);
        REQUIRE(doc.undo());
        CHECK(doc.objectCount() == 2);
    }

    SECTION("deleting the default layer itself falls back to the next layer") {
        const ObjectId onBase =
            doc.insertObject(Object::vector(Contour::circle({0, 0}, 1), base, doc.activeSheet()));
        REQUIRE(doc.execute(std::make_unique<DeleteLayerCommand>(base, false)));
        CHECK(doc.defaultLayer() == work);
        CHECK(doc.findObject(onBase)->layer == work);
        REQUIRE(doc.undo());
        CHECK(doc.defaultLayer() == base);
        CHECK(doc.findObject(onBase)->layer == base);
    }

    SECTION("the last layer cannot be deleted") {
        REQUIRE(doc.execute(std::make_unique<DeleteLayerCommand>(work, true)));
        CHECK_FALSE(doc.execute(std::make_unique<DeleteLayerCommand>(base, true)));
        CHECK(doc.layers().size() == 1);
        CHECK_FALSE(doc.execute(std::make_unique<DeleteLayerCommand>(LayerId{999}, true)));
    }
}

// Timing method (T-0014). The original wall-clock bound failed under parallel `ctest -j8` on a
// loaded machine, where this process may only get a fraction of a core. So:
//  - Time is process CPU time from `std::clock()`. On POSIX that excludes time spent waiting for
//    the CPU while other tests run; this test is single-threaded, so process CPU time is the test's
//    own work. (MSVC's `std::clock()` is wall time since process start; Windows CI runs ctest
//    serially, so the bound is still meaningful there.)
//  - The one-second bound (the T-0010 criterion) applies to the best of three 10 000-op runs, which
//    filters out one-off stalls (page faults, a descheduled timer tick).
//  - An O(n^2) regression is caught independently of machine speed by a scaling check measured the
//    same way: best(20 000) < 3 x best(10 000). Linear or n log n work gives a ratio of about 2;
//    quadratic work gives about 4. Runs of both sizes are interleaved so a slow phase of the machine
//    affects both.
namespace {

/// Inserts `count` objects, undoes all and redoes all; returns the process CPU seconds it took.
double insertUndoRedoSeconds(int count) {
    Document doc;
    const std::clock_t start = std::clock();
    for (int i = 0; i < count; ++i) {
        const double x = static_cast<double>(i % 100) * 12.0;
        const double y = static_cast<double>(i / 100) * 12.0;
        doc.execute(std::make_unique<AddObjectCommand>(square(doc, x, y)));
    }
    REQUIRE(doc.objectCount() == static_cast<std::size_t>(count));
    while (doc.undo()) {
    }
    REQUIRE(doc.objectCount() == 0);
    while (doc.redo()) {
    }
    REQUIRE(doc.objectCount() == static_cast<std::size_t>(count));
    REQUIRE(doc.undoStack().count() == static_cast<std::size_t>(count));
    const std::clock_t stop = std::clock();
    return static_cast<double>(stop - start) / static_cast<double>(CLOCKS_PER_SEC);
}

}  // namespace

TEST_CASE("AC4: 10 000 inserts, undo all and redo all complete under one second",
          "[document][commands][perf]") {
    constexpr int kCount = 10000;
    constexpr int kRuns = 3;
    double best10k = std::numeric_limits<double>::infinity();
    double best20k = std::numeric_limits<double>::infinity();
    for (int run = 0; run < kRuns; ++run) {
        best10k = std::min(best10k, insertUndoRedoSeconds(kCount));
        best20k = std::min(best20k, insertUndoRedoSeconds(2 * kCount));
    }
    INFO("best CPU seconds: 10k = " << best10k << ", 20k = " << best20k);
    CHECK(best10k < 1.0);
    // Optimised builds finish 10k in a few tens of ms; `std::clock()` ticks can be ~16 ms on
    // Windows, so the ratio uses a 50 ms floor. That keeps the check robust there; the unoptimised
    // Linux CI build (10k ~ 0.2 s) is where a quadratic regression shows up.
    const double floor10k = std::max(best10k, 0.05);
    CHECK(best20k < 3.0 * floor10k);
}

TEST_CASE("AC5: object ids are unique across undo/redo and paste assigns new ids", "[document][commands]") {
    Document doc;
    std::set<ObjectId> seen;

    const ObjectId a = addSquare(doc, 0, 0);
    const ObjectId b = addSquare(doc, 20, 0);
    seen.insert(a);
    seen.insert(b);

    // Undoing and adding again never reuses the undone id.
    REQUIRE(doc.undo());
    const ObjectId c = addSquare(doc, 40, 0);
    CHECK(c != b);
    CHECK(c > b);
    CHECK(seen.insert(c).second);

    // Redo of a surviving step keeps its id, redo after a new edit is impossible.
    CHECK_FALSE(doc.redo());

    // Copy / paste: every pasted object gets a fresh id; sources are untouched.
    const auto clipboard = doc.copyObjects({a, c});
    auto paste = std::make_unique<PasteObjectsCommand>(clipboard, Vec2{5.0, 5.0});
    const PasteObjectsCommand* pasteRaw = paste.get();
    REQUIRE(doc.execute(std::move(paste)));
    const std::vector<ObjectId> pasted = pasteRaw->pastedIds();
    REQUIRE(pasted.size() == 2);
    for (ObjectId id : pasted) {
        CHECK(seen.insert(id).second);
        CHECK(doc.hasObject(id));
    }
    CHECK(doc.hasObject(a));
    CHECK(doc.hasObject(c));
    CHECK(doc.objectCount() == 4);
    CHECK(doc.objectBounds(pasted[0]) == Box2{Vec2{5, 5}, Vec2{15, 15}});

    // Undo/redo of the paste keeps the pasted ids stable; pasting again yields yet more new ids.
    REQUIRE(doc.undo());
    CHECK(doc.objectCount() == 2);
    REQUIRE(doc.redo());
    CHECK(doc.hasObject(pasted[0]));
    CHECK(doc.hasObject(pasted[1]));

    auto paste2 = std::make_unique<PasteObjectsCommand>(clipboard);
    const PasteObjectsCommand* paste2Raw = paste2.get();
    REQUIRE(doc.execute(std::move(paste2)));
    for (ObjectId id : paste2Raw->pastedIds()) {
        CHECK(seen.insert(id).second);
    }
    CHECK(seen.size() == 7);
}

// ------------------------------------------------------------------------------------------------
// Other commands
// ------------------------------------------------------------------------------------------------

TEST_CASE("remove objects is undoable and detaches children from surviving groups", "[document][commands]") {
    Document doc;
    const ObjectId a = addSquare(doc, 0, 0);
    const ObjectId b = addSquare(doc, 20, 0);
    const ObjectId loose = addSquare(doc, 40, 0);
    auto group = std::make_unique<GroupObjectsCommand>(std::vector<ObjectId>{a, b});
    const ObjectId g = (doc.execute(std::move(group)), doc.objectIds().back());
    REQUIRE(doc.findObject(g)->isGroup());
    REQUIRE(doc.findObject(a)->parent == g);

    doc.selection().add(a);
    doc.selection().add(loose);
    REQUIRE(
        doc.execute(std::make_unique<RemoveObjectsCommand>(std::vector<ObjectId>{a, loose, ObjectId{999}})));
    CHECK_FALSE(doc.hasObject(a));
    CHECK_FALSE(doc.hasObject(loose));
    CHECK(doc.selection().empty());
    CHECK(doc.findObject(g)->asGroup()->children == std::vector<ObjectId>{b});

    REQUIRE(doc.undo());
    CHECK(doc.hasObject(a));
    CHECK(doc.hasObject(loose));
    CHECK(doc.findObject(g)->asGroup()->children == std::vector<ObjectId>{a, b});
    CHECK(doc.findObject(a)->parent == g);

    // Removing a group takes its descendants with it.
    REQUIRE(doc.execute(std::make_unique<RemoveObjectsCommand>(std::vector<ObjectId>{g})));
    CHECK(doc.objectCount() == 1);
    REQUIRE(doc.undo());
    CHECK(doc.objectCount() == 4);
    CHECK_FALSE(doc.execute(std::make_unique<RemoveObjectsCommand>(std::vector<ObjectId>{ObjectId{999}})));
}

TEST_CASE("group and ungroup round-trip", "[document][commands]") {
    Document doc;
    const LayerId other = addLayer(doc, "Other");
    const ObjectId a = addSquare(doc, 0, 0);
    const ObjectId b = addSquare(doc, 20, 0);
    doc.findObject(b)->layer = other;

    auto cmd = std::make_unique<GroupObjectsCommand>(std::vector<ObjectId>{a, b, a});
    const GroupObjectsCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    const ObjectId g = raw->groupId();
    CHECK(doc.findObject(g)->asGroup()->children == std::vector<ObjectId>{a, b});
    CHECK(doc.findObject(g)->layer == doc.defaultLayer());
    CHECK(doc.objectBounds(g) == Box2{Vec2{0, 0}, Vec2{30, 10}});

    // Already-grouped objects cannot be grouped again; an empty set is a no-op.
    CHECK_FALSE(doc.execute(std::make_unique<GroupObjectsCommand>(std::vector<ObjectId>{a})));
    CHECK_FALSE(doc.execute(std::make_unique<GroupObjectsCommand>(std::vector<ObjectId>{})));

    REQUIRE(doc.execute(std::make_unique<UngroupCommand>(g, /*toGroupLayer=*/true)));
    CHECK_FALSE(doc.hasObject(g));
    CHECK_FALSE(doc.findObject(a)->parent.has_value());
    CHECK(doc.findObject(b)->layer == doc.defaultLayer());

    REQUIRE(doc.undo());
    CHECK(doc.hasObject(g));
    CHECK(doc.findObject(b)->parent == g);
    CHECK(doc.findObject(b)->layer == other);

    REQUIRE(doc.undo());
    CHECK_FALSE(doc.hasObject(g));
    CHECK_FALSE(doc.findObject(a)->parent.has_value());
    REQUIRE(doc.redo());
    CHECK(doc.findObject(g)->id == g);
    CHECK_FALSE(doc.execute(std::make_unique<UngroupCommand>(a)));  // not a group
}

TEST_CASE("move objects between layers and sheets", "[document][commands]") {
    Document doc;
    const LayerId base = doc.defaultLayer();
    const LayerId other = addLayer(doc, "Other");
    const ObjectId a = addSquare(doc, 0, 0);

    REQUIRE(doc.execute(std::make_unique<MoveObjectsToLayerCommand>(std::vector<ObjectId>{a}, other)));
    CHECK(doc.findObject(a)->layer == other);
    CHECK_FALSE(doc.execute(std::make_unique<MoveObjectsToLayerCommand>(std::vector<ObjectId>{a}, other)));
    CHECK_FALSE(
        doc.execute(std::make_unique<MoveObjectsToLayerCommand>(std::vector<ObjectId>{a}, LayerId{999})));
    REQUIRE(doc.undo());
    CHECK(doc.findObject(a)->layer == base);

    auto addSheet = std::make_unique<AddSheetCommand>(Sheet{.name = "Back"}, true);
    const AddSheetCommand* sheetRaw = addSheet.get();
    REQUIRE(doc.execute(std::move(addSheet)));
    const SheetId back = sheetRaw->id();
    CHECK(doc.activeSheet() == back);
    REQUIRE(doc.execute(std::make_unique<MoveObjectsToSheetCommand>(std::vector<ObjectId>{a}, back)));
    CHECK(doc.findObject(a)->sheet == back);
    REQUIRE(doc.undo());
    CHECK(doc.findObject(a)->sheet == doc.sheets().front().id);
    REQUIRE(doc.undo());
    CHECK(doc.sheets().size() == 1);
    CHECK(doc.activeSheet() == doc.sheets().front().id);
}

TEST_CASE("transform objects snapshots geometry and coalesces a drag", "[document][commands]") {
    Document doc;
    const ObjectId a = addSquare(doc, 0, 0);
    const ObjectId circle =
        doc.insertObject(Object::vector(Contour::circle({0, 0}, 5), doc.activeLayer(), doc.activeSheet()));
    const Object before = *doc.findObject(circle);
    doc.undoStack().breakCoalescing();

    const std::vector<ObjectId> ids{a, circle};
    REQUIRE(doc.execute(std::make_unique<TransformObjectsCommand>(ids, Transform2::translation({1, 0}))));
    REQUIRE(doc.execute(std::make_unique<TransformObjectsCommand>(ids, Transform2::translation({2, 0}))));
    // Non-uniform scale turns the circle into Beziers; undo must still restore it exactly.
    REQUIRE(doc.execute(std::make_unique<TransformObjectsCommand>(ids, Transform2::scaling(2.0, 1.0))));
    CHECK(doc.undoStack().undoText() == "Transform objects");
    CHECK(doc.undoStack().index() == 2);  // "Add object" + the three transforms merged into one step
    const Box2 moved = *doc.objectBounds(a);
    CHECK_THAT(moved.min.x, WithinAbs(6.0, 1e-9));
    CHECK_THAT(moved.max.x, WithinAbs(26.0, 1e-9));
    CHECK(doc.findObject(circle)->asVector()->contours.front().spanCount() > 1);

    REQUIRE(doc.undo());
    CHECK(*doc.findObject(circle) == before);
    CHECK(doc.objectBounds(a) == Box2{Vec2{0, 0}, Vec2{10, 10}});
    REQUIRE(doc.redo());
    CHECK(doc.objectBounds(a) == moved);

    // A different id set does not merge.
    REQUIRE(doc.execute(std::make_unique<TransformObjectsCommand>(std::vector<ObjectId>{a},
                                                                  Transform2::translation({0, 1}))));
    CHECK(doc.undoStack().index() == 3);
    CHECK_FALSE(doc.execute(std::make_unique<TransformObjectsCommand>(std::vector<ObjectId>{ObjectId{999}},
                                                                      Transform2::translation({0, 1}))));
}

TEST_CASE("layer property edits coalesce and reorder is undoable", "[document][commands]") {
    Document doc;
    const LayerId base = doc.defaultLayer();
    const LayerId top = addLayer(doc, "Top");
    doc.undoStack().breakCoalescing();

    Layer edited = *doc.findLayer(top);
    edited.name = "T";
    REQUIRE(doc.execute(std::make_unique<SetLayerPropertiesCommand>(edited)));
    edited.name = "To";
    REQUIRE(doc.execute(std::make_unique<SetLayerPropertiesCommand>(edited)));
    edited.name = "Top!";
    edited.colour = Colour{255, 0, 0};
    edited.locked = true;
    REQUIRE(doc.execute(std::make_unique<SetLayerPropertiesCommand>(edited)));
    CHECK(doc.undoStack().index() == 2);
    CHECK(doc.findLayer(top)->name == "Top!");
    CHECK_FALSE(doc.execute(std::make_unique<SetLayerPropertiesCommand>(edited)));  // unchanged
    Layer missing = edited;
    missing.id = LayerId{999};
    CHECK_FALSE(doc.execute(std::make_unique<SetLayerPropertiesCommand>(missing)));

    // Edits to another layer are a separate step.
    Layer baseEdit = *doc.findLayer(base);
    baseEdit.visible = false;
    REQUIRE(doc.execute(std::make_unique<SetLayerPropertiesCommand>(baseEdit)));
    CHECK(doc.undoStack().index() == 3);
    REQUIRE(doc.undo());
    CHECK(doc.findLayer(base)->visible);
    REQUIRE(doc.undo());
    CHECK(doc.findLayer(top)->name == "Top");
    CHECK_FALSE(doc.findLayer(top)->locked);

    REQUIRE(doc.execute(std::make_unique<MoveLayerCommand>(top, 0)));
    CHECK(doc.layers().front().id == top);
    CHECK_FALSE(doc.execute(std::make_unique<MoveLayerCommand>(top, 0)));
    CHECK_FALSE(doc.execute(std::make_unique<MoveLayerCommand>(top, 5)));
    REQUIRE(doc.undo());
    CHECK(doc.layers().front().id == base);
}

TEST_CASE("sheet commands: add, edit, delete with contents", "[document][commands]") {
    Document doc;
    const SheetId front = doc.activeSheet();
    auto add = std::make_unique<AddSheetCommand>(Sheet{.name = "Back"});
    const AddSheetCommand* addRaw = add.get();
    REQUIRE(doc.execute(std::move(add)));
    const SheetId back = addRaw->id();
    CHECK(doc.activeSheet() == front);
    doc.undoStack().breakCoalescing();

    Sheet edited = *doc.findSheet(back);
    edited.sizeMm = {100, 100, 5};
    REQUIRE(doc.execute(std::make_unique<SetSheetPropertiesCommand>(edited)));
    edited.datum = Datum::Centre;
    REQUIRE(doc.execute(std::make_unique<SetSheetPropertiesCommand>(edited)));
    CHECK(doc.undoStack().index() == 2);
    CHECK_FALSE(doc.execute(std::make_unique<SetSheetPropertiesCommand>(edited)));
    REQUIRE(doc.undo());
    CHECK(doc.findSheet(back)->sizeMm == JobSize{300, 300, 18});

    doc.setActiveSheet(back);
    const ObjectId obj = addSquare(doc, 0, 0);
    REQUIRE(doc.findObject(obj)->sheet == back);
    REQUIRE(doc.execute(std::make_unique<DeleteSheetCommand>(back)));
    CHECK(doc.sheets().size() == 1);
    CHECK_FALSE(doc.hasObject(obj));
    CHECK(doc.activeSheet() == front);
    REQUIRE(doc.undo());
    CHECK(doc.findSheet(back)->name == "Back");
    CHECK(doc.findObject(obj)->sheet == back);
    CHECK(doc.activeSheet() == back);
    CHECK_FALSE(doc.execute(std::make_unique<DeleteSheetCommand>(SheetId{999})));
    REQUIRE(doc.execute(std::make_unique<DeleteSheetCommand>(back)));
    CHECK_FALSE(doc.execute(std::make_unique<DeleteSheetCommand>(front)));  // last sheet
}

TEST_CASE("job setup edits coalesce into one step", "[document][commands]") {
    Document doc;
    const Job original = doc.job();
    Job job = Job::singleSided(10.0, 10.0, 0.5, Units::Inches);
    REQUIRE(doc.execute(std::make_unique<SetJobCommand>(job)));
    job.datum = Datum::Centre;
    REQUIRE(doc.execute(std::make_unique<SetJobCommand>(job)));
    CHECK(doc.undoStack().index() == 1);
    CHECK(doc.job() == job);
    CHECK_FALSE(doc.execute(std::make_unique<SetJobCommand>(job)));
    REQUIRE(doc.undo());
    CHECK(doc.job() == original);
    REQUIRE(doc.redo());
    CHECK(doc.job() == job);
}

TEST_CASE("document variable commands validate names, coalesce and respect references",
          "[document][commands][variables]") {
    Document doc;
    CHECK_FALSE(doc.execute(std::make_unique<SetDocumentVariableCommand>("1bad", 1.0)));
    REQUIRE(doc.execute(std::make_unique<SetDocumentVariableCommand>("Depth", 5.0)));
    REQUIRE(doc.execute(std::make_unique<SetDocumentVariableCommand>("Depth", 6.0)));
    CHECK(doc.undoStack().index() == 1);
    CHECK_FALSE(doc.execute(std::make_unique<SetDocumentVariableCommand>("Depth", 6.0)));
    REQUIRE(doc.execute(std::make_unique<SetDocumentVariableCommand>("Other", 1.0)));
    CHECK(doc.undoStack().index() == 2);

    REQUIRE(doc.undo());
    CHECK_FALSE(doc.variables().contains("Other"));
    REQUIRE(doc.undo());
    CHECK_FALSE(doc.variables().contains("Depth"));
    REQUIRE(doc.redo());
    CHECK_THAT(*doc.variables().get("Depth"), WithinAbs(6.0, 0.0));

    doc.undoStack().breakCoalescing();
    REQUIRE(doc.execute(std::make_unique<SetDocumentVariableCommand>("Depth", 7.0)));
    REQUIRE(doc.undo());
    CHECK_THAT(*doc.variables().get("Depth"), WithinAbs(6.0, 0.0));

    doc.variables().addReference("Depth");
    CHECK_FALSE(doc.execute(std::make_unique<RemoveDocumentVariableCommand>("Depth")));
    doc.variables().removeReference("Depth");
    REQUIRE(doc.execute(std::make_unique<RemoveDocumentVariableCommand>("Depth")));
    CHECK_FALSE(doc.variables().contains("Depth"));
    REQUIRE(doc.undo());
    CHECK_THAT(*doc.variables().get("Depth"), WithinAbs(6.0, 0.0));
    CHECK_FALSE(doc.execute(std::make_unique<RemoveDocumentVariableCommand>("Missing")));
}

TEST_CASE("macro on the document groups several commands", "[document][commands]") {
    Document doc;
    doc.undoStack().beginMacro("Create layer with square");
    const LayerId layer = addLayer(doc, "L", true);
    const ObjectId id = addSquare(doc, 0, 0);
    doc.undoStack().endMacro();
    CHECK(doc.undoStack().index() == 1);
    CHECK(doc.undoStack().undoText() == "Create layer with square");
    REQUIRE(doc.undo());
    CHECK_FALSE(doc.hasObject(id));
    CHECK(doc.findLayer(layer) == nullptr);
    REQUIRE(doc.redo());
    CHECK(doc.findObject(id)->layer == layer);
}
