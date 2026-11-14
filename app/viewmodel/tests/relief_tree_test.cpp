#include "carvetto/viewmodel/relief_tree.hpp"

#include <catch2/catch_test_macros.hpp>
#include <memory>

#include "carvetto/document/document.hpp"
#include "carvetto/document/relief_commands.hpp"

using namespace carvetto;
using namespace carvetto::document;
using namespace carvetto::viewmodel;

namespace {

relief::Component block(const std::string& name, float height, std::size_t n = 4) {
    relief::Component c;
    c.name = name;
    REQUIRE(relief::HeightField::create(relief::Grid{n, n, 1.0, {0.0, 0.0}}, height, c.field) == relief::Status::Ok);
    return c;
}

struct Fixture {
    Document doc;
    LevelId l1, l2;
    ComponentId a, b, c;
    Fixture() {
        REQUIRE(addReliefLevel(doc, doc.activeSheet(), "Low", &l1).changed);
        REQUIRE(addReliefLevel(doc, doc.activeSheet(), {}, &l2).changed);
        auto add = [&](LevelId l, const char* name, float h) {
            auto cmd = std::make_unique<AddReliefComponentCommand>(l, block(name, h));
            const AddReliefComponentCommand* raw = cmd.get();
            REQUIRE(doc.execute(std::move(cmd)));
            return raw->createdId();
        };
        a = add(l1, "A", 1.0f);
        b = add(l1, "B", 2.0f);
        c = add(l2, "C", 3.0f);
    }
};

}  // namespace

TEST_CASE("tree lists levels and components top first", "[viewmodel][relief_tree]") {
    Fixture f;
    const auto rows = buildReliefTree(f.doc, f.doc.activeSheet());
    REQUIRE(rows.size() == 5);
    CHECK(rows[0].isLevel);
    CHECK(rows[0].level == f.l2);
    CHECK(rows[0].name == "Level 2");
    CHECK(rows[1].component == f.c);
    CHECK(rows[2].level == f.l1);
    CHECK(rows[3].component == f.b);  // top of Low first
    CHECK(rows[4].component == f.a);
    CHECK(rows[3].index == 1);
    CHECK(rows[4].index == 0);
}

TEST_CASE("every edit is one undoable step", "[viewmodel][relief_tree]") {
    Fixture f;
    UndoStack& undo = f.doc.undoStack();
    const std::size_t base = undo.count();
    SECTION("rename") {
        CHECK(renameReliefComponent(f.doc, f.a, "Renamed").changed);
        CHECK(f.doc.findReliefComponent(f.a)->data.name == "Renamed");
        CHECK(undo.count() == base + 1);
        CHECK(undo.undo(f.doc));
        CHECK(f.doc.findReliefComponent(f.a)->data.name == "A");
        CHECK_FALSE(renameReliefComponent(f.doc, f.a, "").ok);
        CHECK_FALSE(renameReliefLevel(f.doc, LevelId{}, "x").ok);
    }
    SECTION("mode and visibility") {
        CHECK(setReliefComponentMode(f.doc, f.a, relief::CombineMode::Subtract).changed);
        CHECK(f.doc.findReliefComponent(f.a)->data.mode == relief::CombineMode::Subtract);
        CHECK_FALSE(setReliefComponentMode(f.doc, f.a, relief::CombineMode::Subtract).changed);
        CHECK(setReliefComponentVisible(f.doc, f.a, false).changed);
        CHECK_FALSE(f.doc.findReliefComponent(f.a)->data.visible);
        CHECK(setReliefLevelVisible(f.doc, f.l1, false).changed);
        CHECK(setReliefLevelMode(f.doc, f.l1, relief::CombineMode::MergeHigh).changed);
        CHECK(undo.count() == base + 4);
    }
    SECTION("show only and hide all are one step each") {
        CHECK(showOnlyReliefComponent(f.doc, f.b).changed);
        CHECK_FALSE(f.doc.findReliefComponent(f.a)->data.visible);
        CHECK(f.doc.findReliefComponent(f.b)->data.visible);
        CHECK_FALSE(f.doc.findReliefComponent(f.c)->data.visible);
        CHECK(undo.count() == base + 1);
        CHECK(setAllReliefComponentsVisible(f.doc, f.doc.activeSheet(), true).changed);
        CHECK(f.doc.findReliefComponent(f.a)->data.visible);
        CHECK(undo.count() == base + 2);
        CHECK(undo.undo(f.doc));
        CHECK_FALSE(f.doc.findReliefComponent(f.c)->data.visible);
    }
    SECTION("delete, duplicate") {
        ComponentId copy;
        CHECK(duplicateReliefComponent(f.doc, f.a, &copy).changed);
        CHECK(f.doc.findReliefComponent(copy)->data.name == "A copy");
        CHECK(deleteReliefComponents(f.doc, {f.a, f.a, ComponentId{999}}).changed);
        CHECK(f.doc.findReliefComponent(f.a) == nullptr);
        CHECK(undo.undo(f.doc));
        CHECK(f.doc.findReliefComponent(f.a) != nullptr);
        CHECK_FALSE(deleteReliefComponents(f.doc, {ComponentId{999}}).ok);
        CHECK(deleteReliefLevel(f.doc, f.l2).changed);
        CHECK(f.doc.findReliefComponent(f.c) == nullptr);
        CHECK(undo.undo(f.doc));
        CHECK(f.doc.findReliefComponent(f.c) != nullptr);
    }
}

TEST_CASE("reorder, move and drop", "[viewmodel][relief_tree]") {
    Fixture f;
    CHECK(moveReliefComponent(f.doc, f.a, true).changed);
    CHECK(f.doc.locateReliefComponent(f.a)->index == 1);
    CHECK_FALSE(moveReliefComponent(f.doc, f.a, true).changed);  // already on top
    CHECK(moveReliefComponent(f.doc, f.a, false).changed);
    CHECK_FALSE(moveReliefComponent(f.doc, f.a, false).changed);
    CHECK(moveReliefLevel(f.doc, f.l1, true).changed);
    CHECK(*f.doc.reliefLevelIndex(f.l1) == 1);
    CHECK(moveReliefComponentToLevel(f.doc, f.a, f.l2).changed);
    CHECK(f.doc.locateReliefComponent(f.a)->level == f.l2);
    CHECK(dropReliefComponent(f.doc, f.a, f.l1, 0).changed);
    CHECK(f.doc.locateReliefComponent(f.a)->level == f.l1);
    CHECK(f.doc.locateReliefComponent(f.a)->index == 0);
    CHECK(dropReliefComponent(f.doc, f.a, f.l1, 99).changed);  // clamped to the top
    CHECK(f.doc.locateReliefComponent(f.a)->index == 1);
    CHECK_FALSE(dropReliefComponent(f.doc, ComponentId{}, f.l1, 0).ok);
    CHECK_FALSE(dropReliefComponent(f.doc, f.a, LevelId{}, 0).ok);
}

TEST_CASE("selection keeps only existing components", "[viewmodel][relief_tree]") {
    Fixture f;
    selectReliefComponents(f.doc, {f.a, ComponentId{999}, f.c});
    CHECK(f.doc.componentSelection().size() == 2);
    const auto rows = buildReliefTree(f.doc, f.doc.activeSheet());
    CHECK(rows[1].selected);
    CHECK_FALSE(rows[3].selected);
}

TEST_CASE("properties form round trip and refusals", "[viewmodel][relief_tree]") {
    Fixture f;
    ReliefPropertiesForm form;
    REQUIRE(describeReliefComponent(f.doc, f.a, form));
    CHECK(form.name == "A");
    CHECK(form.shapeHeightPercent == "100");
    CHECK(form.fieldInfo == "4 x 4 cells, 1 mm");
    CHECK_FALSE(describeReliefComponent(f.doc, ComponentId{999}, form));

    const std::size_t base = f.doc.undoStack().count();
    form.name = "Hill";
    form.mode = relief::CombineMode::MergeHigh;
    form.shapeHeightPercent = "50";
    form.baseHeight = "2.5";
    form.fadeEnabled = true;
    form.fadePercent = "40";
    form.fadeSecondX = "3";
    form.tiltEnabled = true;
    form.tiltAngleDeg = "10";
    form.tiltRaiseY = "1";
    const EditResult r = applyReliefProperties(f.doc, f.a, form);
    REQUIRE(r.ok);
    CHECK(f.doc.undoStack().count() == base + 1);  // one step for name + mode + properties
    const relief::Component& d = f.doc.findReliefComponent(f.a)->data;
    CHECK(d.name == "Hill");
    CHECK(d.mode == relief::CombineMode::MergeHigh);
    CHECK(d.shapeHeightPercent == 50.0);
    CHECK(d.baseHeight == 2.5);
    CHECK(d.fade.enabled);
    CHECK(d.fade.percent == 40.0);
    CHECK(d.tilt.angleRad > 0.17);
    CHECK(d.tilt.angleRad < 0.18);
    CHECK(f.doc.undoStack().undo(f.doc));
    CHECK(f.doc.findReliefComponent(f.a)->data.name == "A");
    CHECK(f.doc.findReliefComponent(f.a)->data.baseHeight == 0.0);

    const std::size_t steps = f.doc.undoStack().count();
    auto refuses = [&](auto edit) {
        ReliefPropertiesForm bad = form;
        edit(bad);
        const EditResult e = applyReliefProperties(f.doc, f.a, bad);
        return !e.ok && !e.message.empty() && f.doc.undoStack().count() == steps &&
               f.doc.findReliefComponent(f.a)->data.name == "A";
    };
    CHECK(refuses([](ReliefPropertiesForm& x) { x.baseHeight = "abc"; }));
    CHECK(refuses([](ReliefPropertiesForm& x) { x.baseHeight = "nan"; }));
    CHECK(refuses([](ReliefPropertiesForm& x) { x.baseHeight = "1 2"; }));
    CHECK(refuses([](ReliefPropertiesForm& x) { x.shapeHeightPercent = "-1"; }));
    CHECK(refuses([](ReliefPropertiesForm& x) { x.shapeHeightPercent = "5000"; }));
    CHECK(refuses([](ReliefPropertiesForm& x) { x.tiltAngleDeg = "90"; }));
    CHECK(refuses([](ReliefPropertiesForm& x) { x.name = ""; }));
    CHECK(refuses([](ReliefPropertiesForm& x) { x.fadeSecondX = "0"; x.fadeSecondY = "0"; }));  // anchor == second
}

TEST_CASE("level properties form", "[viewmodel][relief_tree]") {
    Fixture f;
    const std::size_t base = f.doc.undoStack().count();
    CHECK(applyReliefLevelProperties(f.doc, f.l1, "Base", relief::CombineMode::MergeLow, "4").changed);
    CHECK(f.doc.undoStack().count() == base + 1);
    CHECK(f.doc.findReliefLevel(f.l1)->name == "Base");
    CHECK(f.doc.findReliefLevel(f.l1)->baseHeight == 4.0);
    CHECK_FALSE(applyReliefLevelProperties(f.doc, f.l1, "Base", relief::CombineMode::MergeLow, "x").ok);
    CHECK_FALSE(applyReliefLevelProperties(f.doc, LevelId{}, "Base", relief::CombineMode::Add, "1").ok);
}
