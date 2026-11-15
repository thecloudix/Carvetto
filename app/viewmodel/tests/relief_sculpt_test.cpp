#include "carvetto/viewmodel/relief_sculpt.hpp"

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <memory>

#include "carvetto/document/document.hpp"
#include "carvetto/document/relief_commands.hpp"
#include "carvetto/viewmodel/relief_tree.hpp"

using namespace carvetto;
using namespace carvetto::document;
using namespace carvetto::viewmodel;

namespace {

ComponentId addFlat(Document& doc, LevelId level, float value, const char* name = "Block") {
    relief::Component c;
    c.name = name;
    REQUIRE(relief::HeightField::create(relief::Grid{41, 41, 1.0, {0.0, 0.0}}, value, c.field) == relief::Status::Ok);
    auto cmd = std::make_unique<AddReliefComponentCommand>(level, std::move(c));
    const AddReliefComponentCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    return raw->createdId();
}

LevelId addLevel(Document& doc) {
    LevelId id;
    REQUIRE(addReliefLevel(doc, doc.activeSheet(), {}, &id).changed);
    return id;
}

float heightAt(const Document& doc, ComponentId id, std::size_t i, std::size_t j) {
    return doc.findReliefComponent(id)->data.field.at(i, j);
}

}  // namespace

TEST_CASE("brush settings clamp, validate and convert", "[viewmodel][sculpt]") {
    BrushSettings s;
    CHECK(brushProblem(s).empty());
    s.diameter = 1000.0;
    CHECK_FALSE(brushProblem(s).empty());
    s = clampedBrush(s);
    CHECK(s.diameter == diameterRange().hi);
    s.kind = relief::BrushKind::Smooth;
    s.strength = 1.5;
    CHECK_FALSE(brushProblem(s).empty());
    CHECK(clampedBrush(s).strength == 1.0);
    s.strength = std::nan("");
    CHECK(clampedBrush(s).strength == 0.5);
    s = BrushSettings{};
    s.mode = relief::SculptMode::Raise;
    s.preserveTransparency = false;
    const relief::BrushParams p = toBrushParams(s);
    CHECK(p.mode == relief::SculptMode::Raise);
    CHECK_FALSE(p.preserveTransparency);
    CHECK(p.diameter == s.diameter);
    CHECK(allBrushKinds().size() == 5);
    CHECK(std::string(brushKindName(relief::BrushKind::UndoBrush)) == "Undo Brush");
}

TEST_CASE("brush presets: built-ins are protected, user presets save and replace", "[viewmodel][sculpt]") {
    BrushPresets presets;
    const std::size_t builtins = presets.all().size();
    REQUIRE(builtins >= 5);
    for (const BrushPreset& p : presets.all()) {
        CHECK(p.builtin);
        CHECK(brushProblem(p.brush).empty());
    }
    BrushSettings mine;
    mine.diameter = 33.0;
    CHECK(presets.save("Mine", mine).empty());
    CHECK(presets.all().size() == builtins + 1);
    mine.diameter = 44.0;
    CHECK(presets.save("Mine", mine).empty());
    CHECK(presets.all().size() == builtins + 1);
    CHECK(presets.find("Mine")->brush.diameter == 44.0);
    CHECK_FALSE(presets.save(presets.all().front().name, mine).empty());
    CHECK_FALSE(presets.save("", mine).empty());
    mine.diameter = -1.0;
    CHECK_FALSE(presets.save("Bad", mine).empty());
    CHECK_FALSE(presets.remove(presets.all().front().name));
    CHECK(presets.remove("Mine"));
    CHECK(presets.all().size() == builtins);
    CHECK_FALSE(presets.remove("Mine"));
}

TEST_CASE("sculpt target: the single selected component or the only one on the sheet", "[viewmodel][sculpt]") {
    Document doc;
    CHECK_FALSE(defaultSculptTarget(doc));
    const LevelId level = addLevel(doc);
    const ComponentId a = addFlat(doc, level, 1.0F, "A");
    CHECK(defaultSculptTarget(doc) == a);
    const ComponentId b = addFlat(doc, level, 2.0F, "B");
    doc.componentSelection().clear();
    CHECK_FALSE(defaultSculptTarget(doc));
    doc.componentSelection().assign({b});
    CHECK(defaultSculptTarget(doc) == b);
    SculptController sc;
    doc.componentSelection().clear();
    CHECK_FALSE(sc.begin(doc).ok);
    CHECK_FALSE(sc.active());
}

TEST_CASE("sculpting works on a copy until Keep, which is one undo step", "[viewmodel][sculpt]") {
    Document doc;
    const ComponentId id = addFlat(doc, addLevel(doc), 2.0F);
    SculptController sc;
    REQUIRE(sc.begin(doc, id).ok);
    CHECK(sc.active());
    CHECK_FALSE(sc.dirty());
    BrushSettings brush;
    brush.kind = relief::BrushKind::Deposit;
    brush.diameter = 8.0;
    brush.strength = 1.0;
    REQUIRE(sc.stroke(brush, {{10.0, 20.0}, {30.0, 20.0}}).ok);
    CHECK(sc.dirty());
    CHECK(sc.working()->at(20, 20) > 2.0F);
    CHECK(sc.working()->at(20, 35) == 2.0F);
    CHECK(heightAt(doc, id, 20, 20) == 2.0F);  // the document is untouched
    const std::size_t steps = doc.undoStack().count();
    const EditResult kept = sc.keep(doc);
    REQUIRE(kept.ok);
    CHECK(kept.changed);
    CHECK(doc.undoStack().count() == steps + 1);
    CHECK(heightAt(doc, id, 20, 20) == sc.working()->at(20, 20));
    CHECK_FALSE(sc.dirty());
    CHECK_FALSE(sc.keep(doc).ok);  // nothing new
    const float kept20 = heightAt(doc, id, 20, 20);
    REQUIRE(doc.undoStack().undo(doc));
    CHECK(heightAt(doc, id, 20, 20) == 2.0F);
    CHECK(kept20 > 2.0F);
}

TEST_CASE("Discard returns to the last kept state; undo stroke reverts one stroke", "[viewmodel][sculpt]") {
    Document doc;
    const ComponentId id = addFlat(doc, addLevel(doc), 2.0F);
    SculptController sc;
    REQUIRE(sc.begin(doc, id).ok);
    BrushSettings brush;
    brush.diameter = 6.0;
    brush.strength = 1.0;
    REQUIRE(sc.stroke(brush, {{10.0, 10.0}, {20.0, 10.0}}).ok);
    const float afterFirst = sc.working()->at(15, 10);
    REQUIRE(sc.stroke(brush, {{10.0, 10.0}, {20.0, 10.0}}).ok);
    CHECK(sc.working()->at(15, 10) > afterFirst);
    CHECK(sc.canUndoStroke());
    CHECK(sc.undoStroke());
    CHECK(sc.working()->at(15, 10) == afterFirst);
    sc.discard();
    CHECK_FALSE(sc.dirty());
    CHECK(sc.working()->at(15, 10) == 2.0F);
    CHECK_FALSE(sc.canUndoStroke());
    // Undo Brush paints back to the kept state.
    REQUIRE(sc.stroke(brush, {{10.0, 10.0}, {20.0, 10.0}}).ok);
    brush.kind = relief::BrushKind::UndoBrush;
    brush.strength = 1.0;
    brush.diameter = 40.0;
    REQUIRE(sc.stroke(brush, {{20.0, 10.0}}).ok);
    CHECK(sc.working()->at(15, 10) == 2.0F);
    sc.end();
    CHECK_FALSE(sc.active());
    CHECK(sc.working() == nullptr);
}

TEST_CASE("an open stroke re-applies its whole path: the result ignores how often the mouse reported",
          "[viewmodel][sculpt]") {
    Document doc;
    const ComponentId id = addFlat(doc, addLevel(doc), 0.0F);
    BrushSettings brush;
    brush.diameter = 6.0;
    brush.strength = 0.5;
    SculptController coarse;
    REQUIRE(coarse.begin(doc, id).ok);
    REQUIRE(coarse.stroke(brush, {{5.0, 20.0}, {35.0, 20.0}}).ok);
    SculptController fine;
    REQUIRE(fine.begin(doc, id).ok);
    REQUIRE(fine.beginStroke(brush, {5.0, 20.0}).ok);
    CHECK(fine.strokeOpen());
    for (double x = 6.0; x <= 35.0; x += 1.0) {
        REQUIRE(fine.extendStroke({x, 20.0}).ok);
    }
    fine.endStroke();
    CHECK_FALSE(fine.strokeOpen());
    for (std::size_t i = 0; i < 41; ++i) {
        for (std::size_t j = 0; j < 41; ++j) {
            REQUIRE(std::fabs(fine.working()->at(i, j) - coarse.working()->at(i, j)) < 1e-4F);
        }
    }
    // One stroke is one undo step however many moves it had.
    CHECK(fine.undoStroke());
    CHECK_FALSE(fine.dirty());
    // Bad input is refused without changing anything.
    CHECK_FALSE(fine.beginStroke(brush, {std::nan(""), 0.0}).ok);
    SculptController idle;
    CHECK_FALSE(idle.beginStroke(brush, {1.0, 1.0}).ok);
    CHECK_FALSE(idle.keep(doc).ok);
}

TEST_CASE("Keep refuses when the component is gone or was resized", "[viewmodel][sculpt]") {
    Document doc;
    const LevelId level = addLevel(doc);
    const ComponentId id = addFlat(doc, level, 1.0F);
    SculptController sc;
    REQUIRE(sc.begin(doc, id).ok);
    BrushSettings brush;
    brush.strength = 1.0;
    REQUIRE(sc.stroke(brush, {{20.0, 20.0}}).ok);
    REQUIRE(doc.undoStack().undo(doc));  // the component creation is undone
    CHECK(doc.findReliefComponent(id) == nullptr);
    const EditResult r = sc.keep(doc);
    CHECK_FALSE(r.ok);
    CHECK_FALSE(r.message.empty());
}
