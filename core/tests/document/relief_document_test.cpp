// T-0282: relief levels and components in the Document, and their undoable commands (ADR-0061).

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/relief_commands.hpp"

using namespace carvetto::document;
namespace relief = carvetto::relief;

namespace {

relief::HeightField field(std::size_t w, std::size_t h, float base) {
    std::vector<float> d(w * h);
    for (std::size_t i = 0; i < d.size(); ++i) {
        d[i] = base + static_cast<float>(i);
    }
    relief::HeightField f;
    REQUIRE(relief::HeightField::fromData(relief::Grid{w, h, 1.0, {0.0, 0.0}}, std::move(d), f) == relief::Status::Ok);
    return f;
}

relief::Component comp(const std::string& name, float base = 0.0F, std::size_t w = 2, std::size_t h = 2) {
    relief::Component c;
    c.name = name;
    c.field = field(w, h, base);
    return c;
}

/// Compact description of the relief state, to compare before/after undo.
std::string dump(const Document& doc) {
    std::string s;
    for (const ReliefLevel& l : doc.reliefLevels()) {
        s += "L" + std::to_string(l.id.value) + "@" + std::to_string(l.sheet.value) + ":" + l.name + ":" +
             std::to_string(static_cast<int>(l.mode)) + ":" + std::to_string(l.baseHeight) + ":" +
             (l.visible ? "v" : "h") + "[";
        for (const ReliefComponent& c : l.components) {
            s += "C" + std::to_string(c.id.value) + ":" + c.data.name + ":" +
                 std::to_string(static_cast<int>(c.data.mode)) + ":" + std::to_string(c.data.shapeHeightPercent) + ":" +
                 std::to_string(c.data.baseHeight) + ":" + (c.data.visible ? "v" : "h") + ":" +
                 (c.data.fade.enabled ? "F" : "f") + (c.data.tilt.enabled ? "T" : "t") + ":" +
                 std::to_string(c.data.field.width()) + "x" + std::to_string(c.data.field.height()) + ":" +
                 std::to_string(c.data.field.at(0, 0)) + ",";
        }
        s += "]";
    }
    s += "|sel";
    for (const ComponentId id : doc.componentSelection().ids()) {
        s += std::to_string(id.value) + ",";
    }
    return s;
}

/// Applies `make()`'s command, checks undo returns the exact prior state and redo the exact
/// later one (twice), and returns the state after.
template <class F>
std::string roundTrip(Document& doc, F make) {
    const std::string before = dump(doc);
    REQUIRE(doc.execute(make()));
    const std::string after = dump(doc);
    CHECK(after != before);
    REQUIRE(doc.checkIntegrity().empty());
    for (int i = 0; i < 2; ++i) {
        REQUIRE(doc.undo());
        CHECK(dump(doc) == before);
        REQUIRE(doc.checkIntegrity().empty());
        REQUIRE(doc.redo());
        CHECK(dump(doc) == after);
        REQUIRE(doc.checkIntegrity().empty());
    }
    return after;
}

struct Fixture {
    Document doc;
    LevelId l1, l2;
    ComponentId a, b, c;
    Fixture() {
        auto add = std::make_unique<AddReliefLevelCommand>(doc.activeSheet(), "Base");
        auto* p = add.get();
        REQUIRE(doc.execute(std::move(add)));
        l1 = p->createdId();
        auto add2 = std::make_unique<AddReliefLevelCommand>(doc.activeSheet(), "Top");
        auto* p2 = add2.get();
        REQUIRE(doc.execute(std::move(add2)));
        l2 = p2->createdId();
        a = addComp(l1, "A", 1.0F);
        b = addComp(l1, "B", 2.0F);
        c = addComp(l2, "C", 3.0F);
    }
    ComponentId addComp(LevelId l, const std::string& name, float base) {
        auto cmd = std::make_unique<AddReliefComponentCommand>(l, comp(name, base));
        auto* p = cmd.get();
        REQUIRE(doc.execute(std::move(cmd)));
        return p->createdId();
    }
};

}  // namespace

TEST_CASE("relief: adding levels and components allocates distinct ids from the shared generator",
          "[document][relief]") {
    Fixture f;
    CHECK(f.doc.reliefLevels().size() == 2);
    CHECK(f.doc.reliefComponentCount() == 3);
    CHECK(f.l1 != f.l2);
    CHECK(f.a != f.b);
    CHECK(f.doc.idGenerator().peek() > f.c.value);
    CHECK(f.doc.findReliefComponent(f.b)->data.name == "B");
    CHECK(f.doc.locateReliefComponent(f.b)->index == 1);
    CHECK(f.doc.locateReliefComponent(f.c)->level == f.l2);
    CHECK(f.doc.reliefCellCount() == 12);
    CHECK(f.doc.checkIntegrity().empty());
    // Undo of the last add frees the slot but ids are never reused.
    const std::uint64_t next = f.doc.idGenerator().peek();
    REQUIRE(f.doc.undo());
    f.addComp(f.l2, "D", 0.0F);
    CHECK(f.doc.idGenerator().peek() == next + 1);
}

TEST_CASE("relief: add commands undo and redo exactly and keep ids", "[document][relief]") {
    Document doc;
    ComponentId created;
    LevelId level;
    roundTrip(doc, [&] {
        auto c = std::make_unique<AddReliefLevelCommand>(doc.activeSheet(), "L");
        level = LevelId{doc.idGenerator().peek()};
        return c;
    });
    CHECK(doc.reliefLevels().front().id == level);
    roundTrip(doc, [&] {
        created = ComponentId{doc.idGenerator().peek()};
        return std::make_unique<AddReliefComponentCommand>(level, comp("X", 5.0F, 3, 2));
    });
    CHECK(doc.findReliefComponent(created)->data.field.width() == 3);
    CHECK(doc.findReliefComponent(created)->data.field.at(2, 1) == 10.0F);
}

TEST_CASE("relief: add refuses bad input", "[document][relief]") {
    Fixture f;
    const std::string before = dump(f.doc);
    CHECK_FALSE(f.doc.execute(std::make_unique<AddReliefLevelCommand>(SheetId{9999}, "x")));
    CHECK_FALSE(f.doc.execute(std::make_unique<AddReliefLevelCommand>(f.doc.activeSheet(), "")));
    CHECK_FALSE(f.doc.execute(
        std::make_unique<AddReliefLevelCommand>(f.doc.activeSheet(), std::string(kMaxReliefNameBytes + 1, 'n'))));
    CHECK_FALSE(f.doc.execute(std::make_unique<AddReliefComponentCommand>(LevelId{9999}, comp("x"))));
    relief::Component empty;
    empty.name = "e";
    CHECK_FALSE(f.doc.execute(std::make_unique<AddReliefComponentCommand>(f.l1, empty)));
    relief::Component nan = comp("n");
    nan.baseHeight = std::nan("");
    CHECK_FALSE(f.doc.execute(std::make_unique<AddReliefComponentCommand>(f.l1, nan)));
    relief::Component disabledNan = comp("n");
    disabledNan.fade.anchor.x = INFINITY;  // finite is required even while the fade is off
    CHECK_FALSE(f.doc.execute(std::make_unique<AddReliefComponentCommand>(f.l1, disabledNan)));
    relief::Component degenerate = comp("d");
    degenerate.tilt.enabled = true;  // pivot == raise
    CHECK_FALSE(f.doc.execute(std::make_unique<AddReliefComponentCommand>(f.l1, degenerate)));
    CHECK(dump(f.doc) == before);
}

TEST_CASE("relief: caps on components, levels and total cells", "[document][relief]") {
    Document doc;
    auto lvl = std::make_unique<AddReliefLevelCommand>(doc.activeSheet(), "L");
    auto* lp = lvl.get();
    REQUIRE(doc.execute(std::move(lvl)));
    const LevelId level = lp->createdId();
    // Total cells: two fields of the per-field maximum (2^25 cells) fill the document cap.
    const auto big = [] {
        relief::Component c;
        c.name = "big";
        REQUIRE(relief::HeightField::create(relief::Grid{8192, 4096, 1.0, {0, 0}}, 0.0F, c.field) ==
                relief::Status::Ok);
        return c;
    };
    for (int i = 0; i < 2; ++i) {
        REQUIRE(doc.execute(std::make_unique<AddReliefComponentCommand>(level, big())));
    }
    CHECK(doc.reliefCellCount() == kMaxReliefTotalCells);
    CHECK_FALSE(doc.execute(std::make_unique<AddReliefComponentCommand>(level, comp("one more"))));
    // saveCarv refuses an over-cap document (forced in with the raw API) before encoding any field.
    const ComponentId extra = doc.insertReliefComponent(level, ReliefComponent{ComponentId{}, comp("extra")});
    const auto saved = saveCarv(doc);
    REQUIRE_FALSE(saved.ok());
    CHECK(saved.error().code == CarvError::Code::ResourceLimit);
    doc.takeReliefComponent(extra);
    CHECK_FALSE(doc.execute(std::make_unique<DuplicateReliefComponentCommand>(doc.reliefLevels()[0].components[0].id)));
    CHECK(doc.reliefComponentCount() == 2);
}

TEST_CASE("relief: component count cap", "[document][relief]") {
    Document doc;
    auto lvl = std::make_unique<AddReliefLevelCommand>(doc.activeSheet(), "L");
    auto* lp = lvl.get();
    REQUIRE(doc.execute(std::move(lvl)));
    const LevelId level = lp->createdId();
    for (std::size_t i = 0; i < kMaxReliefComponents; ++i) {
        REQUIRE(doc.execute(std::make_unique<AddReliefComponentCommand>(level, comp("c", 0.0F, 1, 1))));
    }
    CHECK_FALSE(doc.execute(std::make_unique<AddReliefComponentCommand>(level, comp("c", 0.0F, 1, 1))));
    CHECK(doc.reliefComponentCount() == kMaxReliefComponents);
}

TEST_CASE("relief: level count cap", "[document][relief]") {
    Document doc;
    for (std::size_t i = 0; i < kMaxReliefLevels; ++i) {
        REQUIRE(doc.execute(std::make_unique<AddReliefLevelCommand>(doc.activeSheet(), "L")));
    }
    CHECK_FALSE(doc.execute(std::make_unique<AddReliefLevelCommand>(doc.activeSheet(), "L")));
}

TEST_CASE("relief: delete components restores order and position", "[document][relief]") {
    Fixture f;
    roundTrip(f.doc, [&] {
        return std::make_unique<DeleteReliefComponentsCommand>(std::vector<ComponentId>{f.a, f.c, f.a});
    });
    CHECK(f.doc.reliefComponentCount() == 1);
    CHECK_FALSE(f.doc.execute(std::make_unique<DeleteReliefComponentsCommand>(std::vector<ComponentId>{f.a})));
    CHECK_FALSE(f.doc.execute(std::make_unique<DeleteReliefComponentsCommand>(std::vector<ComponentId>{})));
}

TEST_CASE("relief: delete level restores it with its components and ids", "[document][relief]") {
    Fixture f;
    roundTrip(f.doc, [&] { return std::make_unique<DeleteReliefLevelCommand>(f.l1); });
    CHECK(f.doc.reliefLevels().size() == 1);
    CHECK(f.doc.findReliefComponent(f.a) == nullptr);
    CHECK_FALSE(f.doc.execute(std::make_unique<DeleteReliefLevelCommand>(LevelId{12345})));
    // A level in the middle comes back in the middle.
    REQUIRE(f.doc.undo());
    const LevelId l3 = [&] {
        auto c = std::make_unique<AddReliefLevelCommand>(f.doc.activeSheet(), "Third");
        auto* p = c.get();
        REQUIRE(f.doc.execute(std::move(c)));
        return p->createdId();
    }();
    roundTrip(f.doc, [&] { return std::make_unique<DeleteReliefLevelCommand>(f.l2); });
    REQUIRE(f.doc.undo());
    CHECK(f.doc.reliefLevelIndex(f.l2) == 1);
    CHECK(f.doc.reliefLevelIndex(l3) == 2);
    REQUIRE(f.doc.redo());
    REQUIRE(f.doc.undo());
    // Deleting the bottom level and undoing puts it back at the bottom.
    REQUIRE(f.doc.undo());
    REQUIRE(f.doc.execute(std::make_unique<DeleteReliefLevelCommand>(f.l1)));
    REQUIRE(f.doc.undo());
    CHECK(f.doc.reliefLevels().front().id == f.l1);
}

TEST_CASE("relief: renames", "[document][relief]") {
    Fixture f;
    roundTrip(f.doc, [&] { return std::make_unique<RenameReliefComponentCommand>(f.a, "Renamed"); });
    roundTrip(f.doc, [&] { return std::make_unique<RenameReliefLevelCommand>(f.l1, "Floor"); });
    CHECK_FALSE(f.doc.execute(std::make_unique<RenameReliefComponentCommand>(f.a, "Renamed")));
    CHECK_FALSE(f.doc.execute(std::make_unique<RenameReliefComponentCommand>(f.a, "")));
    CHECK_FALSE(f.doc.execute(std::make_unique<RenameReliefComponentCommand>(ComponentId{777}, "x")));
    CHECK_FALSE(f.doc.execute(
        std::make_unique<RenameReliefLevelCommand>(f.l1, std::string(kMaxReliefNameBytes + 1, 'x'))));
    CHECK_FALSE(f.doc.execute(std::make_unique<RenameReliefLevelCommand>(LevelId{777}, "x")));
}

TEST_CASE("relief: combine mode and property edits", "[document][relief]") {
    Fixture f;
    roundTrip(f.doc, [&] { return std::make_unique<SetReliefComponentModeCommand>(f.a, relief::CombineMode::Subtract); });
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefComponentModeCommand>(f.a, relief::CombineMode::Subtract)));

    ReliefComponentProperties p;
    p.shapeHeightPercent = 50.0;
    p.baseHeight = 1.5;
    p.fade = {true, {0, 0}, {10, 0}, 40.0};
    p.tilt = {true, {0, 0}, {0, 5}, 0.1};
    p.visible = false;
    roundTrip(f.doc, [&] { return std::make_unique<SetReliefComponentPropertiesCommand>(f.b, p); });
    const relief::Component& b = f.doc.findReliefComponent(f.b)->data;
    CHECK(b.shapeHeightPercent == 50.0);
    CHECK(b.baseHeight == 1.5);
    CHECK(b.fade.enabled);
    CHECK(b.fade.second.x == 10.0);
    CHECK(b.tilt.angleRad == 0.1);
    CHECK_FALSE(b.visible);
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefComponentPropertiesCommand>(f.b, p)));  // unchanged

    ReliefComponentProperties bad = p;
    bad.shapeHeightPercent = std::nan("");
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefComponentPropertiesCommand>(f.b, bad)));
    bad = p;
    bad.baseHeight = 7.0;
    bad.fade.second = bad.fade.anchor;  // degenerate direction
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefComponentPropertiesCommand>(f.b, bad)));
    bad = p;
    bad.baseHeight = 7.0;
    bad.tilt.raise = bad.tilt.pivot;
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefComponentPropertiesCommand>(f.b, bad)));
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefComponentPropertiesCommand>(ComponentId{777}, p)));
    CHECK(f.doc.findReliefComponent(f.b)->data.baseHeight == 1.5);
}

TEST_CASE("relief: consecutive property edits of one component coalesce into one undo step",
          "[document][relief]") {
    Fixture f;
    ReliefComponentProperties p;
    p.baseHeight = 1.0;
    REQUIRE(f.doc.execute(std::make_unique<SetReliefComponentPropertiesCommand>(f.a, p)));
    p.baseHeight = 2.0;
    REQUIRE(f.doc.execute(std::make_unique<SetReliefComponentPropertiesCommand>(f.a, p)));
    p.baseHeight = 3.0;
    REQUIRE(f.doc.execute(std::make_unique<SetReliefComponentPropertiesCommand>(f.b, p)));  // other component
    REQUIRE(f.doc.undo());
    CHECK(f.doc.findReliefComponent(f.a)->data.baseHeight == 2.0);
    CHECK(f.doc.findReliefComponent(f.b)->data.baseHeight == 0.0);
    REQUIRE(f.doc.undo());
    CHECK(f.doc.findReliefComponent(f.a)->data.baseHeight == 0.0);
}

TEST_CASE("relief: level properties", "[document][relief]") {
    Fixture f;
    roundTrip(f.doc, [&] {
        return std::make_unique<SetReliefLevelPropertiesCommand>(f.l2, relief::CombineMode::MergeHigh, 2.5, false);
    });
    CHECK(f.doc.findReliefLevel(f.l2)->mode == relief::CombineMode::MergeHigh);
    CHECK(f.doc.findReliefLevel(f.l2)->baseHeight == 2.5);
    CHECK_FALSE(f.doc.findReliefLevel(f.l2)->visible);
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefLevelPropertiesCommand>(
        f.l2, relief::CombineMode::MergeHigh, 2.5, false)));
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefLevelPropertiesCommand>(
        f.l2, relief::CombineMode::Add, std::nan(""), true)));
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefLevelPropertiesCommand>(
        LevelId{777}, relief::CombineMode::Add, 1.0, true)));
}

TEST_CASE("relief: consecutive edits of one level coalesce into one undo step", "[document][relief]") {
    Fixture f;
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelPropertiesCommand>(f.l1, relief::CombineMode::Add, 1.0, true)));
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelPropertiesCommand>(f.l1, relief::CombineMode::Add, 2.0, true)));
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelPropertiesCommand>(f.l2, relief::CombineMode::Add, 3.0, true)));
    REQUIRE(f.doc.undo());
    CHECK(f.doc.findReliefLevel(f.l1)->baseHeight == 2.0);
    CHECK(f.doc.findReliefLevel(f.l2)->baseHeight == 0.0);
    REQUIRE(f.doc.undo());
    CHECK(f.doc.findReliefLevel(f.l1)->baseHeight == 0.0);
}

TEST_CASE("relief: raw inserts refuse a missing sheet or level", "[document][relief]") {
    Fixture f;
    ReliefLevel level;
    level.sheet = SheetId{31337};
    CHECK(f.doc.insertReliefLevel(level).isNull());
    CHECK(f.doc.insertReliefComponent(LevelId{31337}, ReliefComponent{ComponentId{}, comp("x")}).isNull());
    CHECK(f.doc.reliefLevels().size() == 2);
    // A preset id reserves the generator through it.
    ReliefLevel preset;
    preset.id = LevelId{5000};
    preset.sheet = f.doc.activeSheet();
    CHECK(f.doc.insertReliefLevel(preset) == LevelId{5000});
    CHECK(f.doc.idGenerator().peek() == 5001);
    CHECK(f.doc.checkIntegrity().empty());
}

TEST_CASE("relief: reorder components within a level", "[document][relief]") {
    Fixture f;
    const ComponentId d = f.addComp(f.l1, "D", 4.0F);  // level l1: A B D
    roundTrip(f.doc, [&] { return std::make_unique<ReorderReliefComponentCommand>(f.a, 2); });
    const auto& comps = f.doc.findReliefLevel(f.l1)->components;
    CHECK(comps[0].id == f.b);
    CHECK(comps[1].id == d);
    CHECK(comps[2].id == f.a);
    roundTrip(f.doc, [&] { return std::make_unique<ReorderReliefComponentCommand>(f.a, 0); });
    CHECK(f.doc.findReliefLevel(f.l1)->components[0].id == f.a);
    CHECK_FALSE(f.doc.execute(std::make_unique<ReorderReliefComponentCommand>(f.a, 0)));
    CHECK_FALSE(f.doc.execute(std::make_unique<ReorderReliefComponentCommand>(f.a, 3)));
    CHECK_FALSE(f.doc.execute(std::make_unique<ReorderReliefComponentCommand>(ComponentId{777}, 0)));
}

TEST_CASE("relief: reorder levels among their sheet's levels", "[document][relief]") {
    Fixture f;
    roundTrip(f.doc, [&] { return std::make_unique<ReorderReliefLevelCommand>(f.l1, 1); });
    CHECK(f.doc.reliefLevels()[0].id == f.l2);
    CHECK(f.doc.reliefLevels()[1].id == f.l1);
    CHECK_FALSE(f.doc.execute(std::make_unique<ReorderReliefLevelCommand>(f.l1, 1)));
    CHECK_FALSE(f.doc.execute(std::make_unique<ReorderReliefLevelCommand>(f.l1, 2)));
    CHECK_FALSE(f.doc.execute(std::make_unique<ReorderReliefLevelCommand>(LevelId{777}, 0)));
}

TEST_CASE("relief: level order is per sheet", "[document][relief]") {
    Document doc;
    const SheetId s1 = doc.activeSheet();
    const SheetId s2 = doc.insertSheet(Sheet::fromJob(doc.job(), "Two"));
    auto lv = [&](SheetId s, const char* n) {
        auto c = std::make_unique<AddReliefLevelCommand>(s, n);
        auto* p = c.get();
        REQUIRE(doc.execute(std::move(c)));
        return p->createdId();
    };
    const LevelId a1 = lv(s1, "a1");
    const LevelId b1 = lv(s2, "b1");
    const LevelId a2 = lv(s1, "a2");
    CHECK(doc.reliefLevelsOnSheet(s1) == std::vector<LevelId>{a1, a2});
    CHECK(doc.reliefLevelsOnSheet(s2) == std::vector<LevelId>{b1});
    CHECK(doc.reliefLevelIndex(a2) == 1);
    CHECK(doc.reliefLevelIndex(b1) == 0);
    roundTrip(doc, [&] { return std::make_unique<ReorderReliefLevelCommand>(a1, 1); });
    CHECK(doc.reliefLevelsOnSheet(s1) == std::vector<LevelId>{a2, a1});
    CHECK(doc.reliefLevelsOnSheet(s2) == std::vector<LevelId>{b1});
    // A sheet with relief cannot be removed; the model of a sheet holds only its levels.
    CHECK_FALSE(doc.takeSheet(s2).has_value());
    CHECK(doc.reliefModel(s2).size() == 1);
    CHECK(doc.reliefModel(s1).size() == 2);
    CHECK(doc.reliefModel(s1)[0].name == "a2");
    // Insert at a given index among its sheet's levels.
    REQUIRE(doc.execute(std::make_unique<AddReliefLevelCommand>(s1, "mid", 1)));
    CHECK(doc.reliefModel(s1)[1].name == "mid");
}

TEST_CASE("relief: move a component to another level", "[document][relief]") {
    Fixture f;
    roundTrip(f.doc, [&] { return std::make_unique<MoveReliefComponentToLevelCommand>(f.a, f.l2, 0); });
    CHECK(f.doc.locateReliefComponent(f.a)->level == f.l2);
    CHECK(f.doc.locateReliefComponent(f.a)->index == 0);
    CHECK(f.doc.locateReliefComponent(f.c)->index == 1);
    roundTrip(f.doc, [&] { return std::make_unique<MoveReliefComponentToLevelCommand>(f.b, f.l2); });
    CHECK(f.doc.locateReliefComponent(f.b)->index == 2);  // on top by default
    CHECK(f.doc.findReliefLevel(f.l1)->components.empty());
    // Same level, unknown ids and a level on another sheet are refused.
    CHECK_FALSE(f.doc.execute(std::make_unique<MoveReliefComponentToLevelCommand>(f.a, f.l2)));
    CHECK_FALSE(f.doc.execute(std::make_unique<MoveReliefComponentToLevelCommand>(f.a, LevelId{777})));
    CHECK_FALSE(f.doc.execute(std::make_unique<MoveReliefComponentToLevelCommand>(ComponentId{777}, f.l1)));
    const SheetId other = f.doc.insertSheet(Sheet::fromJob(f.doc.job(), "Two"));
    auto lv = std::make_unique<AddReliefLevelCommand>(other, "Other");
    auto* lp = lv.get();
    REQUIRE(f.doc.execute(std::move(lv)));
    CHECK_FALSE(f.doc.execute(std::make_unique<MoveReliefComponentToLevelCommand>(f.a, lp->createdId())));
}

TEST_CASE("relief: duplicate", "[document][relief]") {
    Fixture f;
    ReliefComponentProperties p;
    p.baseHeight = 2.0;
    REQUIRE(f.doc.execute(std::make_unique<SetReliefComponentPropertiesCommand>(f.a, p)));
    ComponentId copy;
    roundTrip(f.doc, [&] {
        auto c = std::make_unique<DuplicateReliefComponentCommand>(f.a);
        copy = ComponentId{f.doc.idGenerator().peek()};
        return c;
    });
    const auto where = f.doc.locateReliefComponent(copy);
    REQUIRE(where.has_value());
    CHECK(where->level == f.l1);
    CHECK(where->index == 1);  // directly above the original
    const relief::Component& c = f.doc.findReliefComponent(copy)->data;
    CHECK(c.name == "A copy");
    CHECK(c.baseHeight == 2.0);
    CHECK(c.field.at(1, 1) == f.doc.findReliefComponent(f.a)->data.field.at(1, 1));
    CHECK(f.doc.findReliefComponent(f.a)->data.name == "A");  // the original is untouched
    CHECK_FALSE(f.doc.execute(std::make_unique<DuplicateReliefComponentCommand>(ComponentId{777})));
    // A name at the cap is truncated, not refused.
    REQUIRE(f.doc.execute(std::make_unique<RenameReliefComponentCommand>(f.b, std::string(kMaxReliefNameBytes, 'n'))));
    auto dup = std::make_unique<DuplicateReliefComponentCommand>(f.b);
    auto* dp = dup.get();
    REQUIRE(f.doc.execute(std::move(dup)));
    CHECK(f.doc.findReliefComponent(dp->createdId())->data.name.size() == kMaxReliefNameBytes);
}

TEST_CASE("relief: component selection follows the id conventions", "[document][relief]") {
    Fixture f;
    ComponentSelection& sel = f.doc.componentSelection();
    CHECK(sel.add(f.a));
    CHECK_FALSE(sel.add(f.a));
    CHECK(sel.add(f.c));
    CHECK(sel.size() == 2);
    CHECK(sel.ids() == std::vector<ComponentId>{f.a, f.c});  // selection order
    CHECK(sel.single().isNull());
    CHECK_FALSE(sel.toggle(f.a));
    CHECK(sel.single() == f.c);
    sel.assign({f.b, f.a, f.b});
    CHECK(sel.ids() == std::vector<ComponentId>{f.b, f.a});
    CHECK(sel.remove(f.b));
    CHECK_FALSE(sel.remove(f.b));
    // Deleting deselects (as for objects); moving to another level and reordering keep it.
    sel.assign({f.a, f.b});
    REQUIRE(f.doc.execute(std::make_unique<MoveReliefComponentToLevelCommand>(f.a, f.l2)));
    CHECK(sel.ids() == std::vector<ComponentId>{f.a, f.b});
    REQUIRE(f.doc.execute(std::make_unique<ReorderReliefLevelCommand>(f.l1, 1)));
    CHECK(sel.ids() == std::vector<ComponentId>{f.a, f.b});
    REQUIRE(f.doc.execute(std::make_unique<DeleteReliefComponentsCommand>(std::vector<ComponentId>{f.a})));
    CHECK(sel.ids() == std::vector<ComponentId>{f.b});
    REQUIRE(f.doc.execute(std::make_unique<DeleteReliefLevelCommand>(f.l1)));
    CHECK(sel.empty());
    CHECK(f.doc.checkIntegrity().empty());
    // A stale selection id is reported.
    sel.add(ComponentId{424242});
    const auto issues = f.doc.checkIntegrity();
    REQUIRE(issues.size() == 1);
    CHECK(issues[0].kind == IntegrityIssue::Kind::StaleComponentSelection);
}

TEST_CASE("relief: integrity reports shared ids, missing sheets and bad components", "[document][relief]") {
    Fixture f;
    // A level whose id collides with a layer.
    ReliefLevel clash;
    clash.id = LevelId{f.doc.defaultLayer().value};
    clash.sheet = f.doc.activeSheet();
    f.doc.insertReliefLevel(clash);
    bool duplicate = false;
    for (const auto& i : f.doc.checkIntegrity()) {
        duplicate = duplicate || i.kind == IntegrityIssue::Kind::DuplicateId;
    }
    CHECK(duplicate);
    f.doc.takeReliefLevel(clash.id);
    CHECK(f.doc.checkIntegrity().empty());
    // A stored component whose field was emptied.
    f.doc.findReliefComponent(f.a)->data.field = relief::HeightField{};
    const auto issues = f.doc.checkIntegrity();
    REQUIRE(issues.size() == 1);
    CHECK(issues[0].kind == IntegrityIssue::Kind::BadReliefComponent);
    // A level on a missing sheet.
    f.doc.findReliefComponent(f.a)->data = comp("A");
    f.doc.findReliefLevel(f.l1)->sheet = SheetId{31337};
    const auto missing = f.doc.checkIntegrity();
    REQUIRE(missing.size() == 1);
    CHECK(missing[0].kind == IntegrityIssue::Kind::MissingReliefSheet);
}

TEST_CASE("relief: a toolpath selection cannot name a relief id", "[document][relief]") {
    Fixture f;
    ToolpathRecord tp;
    tp.selection.objects.push_back(ObjectId{f.a.value});
    f.doc.insertToolpath(tp);
    const auto issues = f.doc.checkIntegrity();
    REQUIRE(issues.size() == 1);
    CHECK(issues[0].kind == IntegrityIssue::Kind::BadToolpathSelection);
}

TEST_CASE("relief: reliefModel feeds relief::compose", "[document][relief]") {
    Fixture f;  // l1: A(base 1: 1,2,3,4), B(2: 2,3,4,5) both Add; l2: C(3: 3,4,5,6) Add
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelPropertiesCommand>(f.l2, relief::CombineMode::Subtract, 0.0, true)));
    const auto model = f.doc.reliefModel(f.doc.activeSheet());
    relief::HeightField out;
    REQUIRE(relief::compose(model, relief::Grid{2, 2, 1.0, {0, 0}}, out) == relief::Status::Ok);
    // (A + B) - C = (1+2-3, 2+3-4, 3+4-5, 4+5-6)
    CHECK(out.at(0, 0) == 0.0F);
    CHECK(out.at(1, 0) == 1.0F);
    CHECK(out.at(0, 1) == 2.0F);
    CHECK(out.at(1, 1) == 3.0F);
}
