#include "carvetto/viewmodel/sculpt_merged_preview.hpp"

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>
#include <memory>

#include "carvetto/document/document.hpp"
#include "carvetto/document/relief_commands.hpp"
#include "carvetto/viewmodel/model_view.hpp"
#include "carvetto/viewmodel/relief_sculpt.hpp"
#include "carvetto/viewmodel/relief_tree.hpp"

using namespace carvetto;
using namespace carvetto::document;
using namespace carvetto::viewmodel;

namespace {

ComponentId addBlock(Document& doc, LevelId level, std::size_t n, double origin, float value, relief::CombineMode mode,
                     const char* name) {
    relief::Component c;
    c.name = name;
    c.mode = mode;
    REQUIRE(relief::HeightField::create(relief::Grid{n, n, 1.0, {origin, origin}}, value, c.field) ==
            relief::Status::Ok);
    // A slope so cells differ.
    for (std::size_t j = 0; j < n; ++j) {
        for (std::size_t i = 0; i < n; ++i) {
            c.field.set(i, j, value + 0.01f * static_cast<float>(i + 2 * j));
        }
    }
    auto cmd = std::make_unique<AddReliefComponentCommand>(level, std::move(c));
    const AddReliefComponentCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    return raw->createdId();
}

LevelId addLevel(Document& doc, relief::CombineMode mode) {
    LevelId id;
    REQUIRE(addReliefLevel(doc, doc.activeSheet(), {}, &id).changed);
    REQUIRE(setReliefLevelMode(doc, id, mode).ok);
    return id;
}

bool sameCells(const relief::HeightField& a, const relief::HeightField& b) {
    if (a.width() != b.width() || a.height() != b.height()) {
        return false;
    }
    for (std::size_t k = 0; k < a.data().size(); ++k) {
        const float x = a.data()[k], y = b.data()[k];
        if (std::isnan(x) != std::isnan(y) || (!std::isnan(x) && x != y)) {
            return false;
        }
    }
    return true;
}

// Strokes with moves, preview updated per move; after Keep the surface must equal the real model.
void strokeAndCompare(Document& doc, ComponentId target, std::size_t maxDim = kMaxSurfaceDimension) {
    SculptController ctl;
    REQUIRE(ctl.begin(doc, target).ok);
    SculptMergedPreview pv;
    REQUIRE(pv.begin(doc, target, *ctl.working(), maxDim));
    BrushSettings b;
    b.diameter = 8.0;
    b.strength = 1.0;
    REQUIRE(ctl.beginStroke(b, {10.0, 12.0}).ok);
    pv.update(*ctl.working(), ctl.takeDirty());
    for (int i = 1; i <= 12; ++i) {
        REQUIRE(ctl.extendStroke({10.0 + 2.0 * i, 12.0 + 1.5 * i}).ok);
        pv.update(*ctl.working(), ctl.takeDirty());
    }
    ctl.endStroke();
    REQUIRE(ctl.keep(doc).ok);
    const ModelSurface real = composeModelSurface(doc, doc.activeSheet(), maxDim);
    REQUIRE(real.ok());
    CHECK(pv.surface().stride == real.stride);
    CHECK(sameCells(pv.surface().field, real.field));
}

}  // namespace

TEST_CASE("merged sculpt preview equals the composed model", "[viewmodel][sculpt]") {
    Document doc;
    const LevelId l0 = addLevel(doc, relief::CombineMode::Add);
    addBlock(doc, l0, 60, -5.0, 1.0f, relief::CombineMode::Add, "Base");
    const ComponentId target = addBlock(doc, l0, 41, 0.0, 2.0f, relief::CombineMode::Add, "Target");
    addBlock(doc, l0, 30, 8.0, 0.5f, relief::CombineMode::MergeHigh, "Above");
    const LevelId l1 = addLevel(doc, relief::CombineMode::Subtract);
    addBlock(doc, l1, 25, 10.0, 0.3f, relief::CombineMode::Add, "Cut");
    const LevelId l2 = addLevel(doc, relief::CombineMode::MergeHigh);
    addBlock(doc, l2, 20, 2.0, 1.5f, relief::CombineMode::Add, "Top");
    strokeAndCompare(doc, target);
}

TEST_CASE("merged sculpt preview with a coarse display grid", "[viewmodel][sculpt]") {
    Document doc;
    const LevelId l0 = addLevel(doc, relief::CombineMode::Add);
    addBlock(doc, l0, 80, -10.0, 1.0f, relief::CombineMode::Add, "Base");
    const ComponentId target = addBlock(doc, l0, 41, 0.0, 2.0f, relief::CombineMode::Add, "Target");
    strokeAndCompare(doc, target, 32);  // stride 3
}

TEST_CASE("merged sculpt preview honours clipping and mirror", "[viewmodel][sculpt]") {
    {
        Document doc;
        const LevelId l0 = addLevel(doc, relief::CombineMode::Add);
        addBlock(doc, l0, 60, -5.0, 1.0f, relief::CombineMode::Add, "Base");
        const ComponentId target = addBlock(doc, l0, 41, 0.0, 2.0f, relief::CombineMode::Add, "Target");
        relief::Clip clip;
        clip.polygons.push_back({{0.0, 0.0}, {30.0, 5.0}, {35.0, 35.0}, {3.0, 30.0}});
        REQUIRE(doc.execute(std::make_unique<SetReliefLevelClipCommand>(l0, clip)));
        strokeAndCompare(doc, target);
    }
    {
        Document doc;
        const LevelId l0 = addLevel(doc, relief::CombineMode::Add);
        const ComponentId target = addBlock(doc, l0, 41, 0.0, 2.0f, relief::CombineMode::Add, "Target");
        REQUIRE(doc.execute(std::make_unique<SetReliefLevelMirrorCommand>(l0, relief::MirrorMode::LeftToRight,
                                                                          relief::Point{20.0, 20.0})));
        strokeAndCompare(doc, target);
    }
}

TEST_CASE("merged sculpt preview follows Undo Stroke and Discard", "[viewmodel][sculpt]") {
    Document doc;
    const LevelId l0 = addLevel(doc, relief::CombineMode::Add);
    addBlock(doc, l0, 60, -5.0, 1.0f, relief::CombineMode::Add, "Base");
    const ComponentId target = addBlock(doc, l0, 41, 0.0, 2.0f, relief::CombineMode::Add, "Target");
    const ModelSurface before = composeModelSurface(doc, doc.activeSheet());
    SculptController ctl;
    REQUIRE(ctl.begin(doc, target).ok);
    SculptMergedPreview pv;
    REQUIRE(pv.begin(doc, target, *ctl.working()));
    BrushSettings b;
    b.diameter = 8.0;
    REQUIRE(ctl.stroke(b, {{10.0, 10.0}, {20.0, 20.0}}).ok);
    pv.update(*ctl.working(), ctl.takeDirty());
    CHECK_FALSE(sameCells(pv.surface().field, before.field));
    REQUIRE(ctl.undoStroke());
    pv.update(*ctl.working(), ctl.takeDirty());
    CHECK(sameCells(pv.surface().field, before.field));
    REQUIRE(ctl.stroke(b, {{10.0, 10.0}, {20.0, 20.0}}).ok);
    pv.update(*ctl.working(), ctl.takeDirty());
    ctl.discard();
    pv.update(*ctl.working(), ctl.takeDirty());
    CHECK(sameCells(pv.surface().field, before.field));
}

TEST_CASE("merged sculpt preview shows a hidden target and refuses a missing one", "[viewmodel][sculpt]") {
    Document doc;
    const LevelId l0 = addLevel(doc, relief::CombineMode::Add);
    const ComponentId target = addBlock(doc, l0, 41, 0.0, 2.0f, relief::CombineMode::Add, "Target");
    REQUIRE(setReliefComponentVisible(doc, target, false).ok);
    SculptController ctl;
    REQUIRE(ctl.begin(doc, target).ok);
    SculptMergedPreview pv;
    REQUIRE(pv.begin(doc, target, *ctl.working()));
    float lo = 0.0f, hi = 0.0f;
    CHECK(pv.surface().field.minMax(lo, hi));
    CHECK_FALSE(pv.begin(doc, ComponentId{}, *ctl.working()));
    CHECK_FALSE(pv.problem().empty());
    CHECK_FALSE(pv.active());
}

TEST_CASE("a sculpt move costs the footprint, not the model", "[viewmodel][sculpt]") {
    auto perMove = [](std::size_t otherSize) {
        Document doc;
        const LevelId l0 = addLevel(doc, relief::CombineMode::Add);
        addBlock(doc, l0, otherSize, -static_cast<double>(otherSize) / 2.0, 1.0f, relief::CombineMode::Add, "Base");
        const ComponentId target = addBlock(doc, l0, 41, 0.0, 2.0f, relief::CombineMode::Add, "Target");
        const LevelId l1 = addLevel(doc, relief::CombineMode::Add);
        addBlock(doc, l1, otherSize, -static_cast<double>(otherSize) / 2.0, 0.5f, relief::CombineMode::Add, "Top");
        SculptController ctl;
        REQUIRE(ctl.begin(doc, target).ok);
        SculptMergedPreview pv;
        REQUIRE(pv.begin(doc, target, *ctl.working(), 1000));
        const std::uint64_t atStart = pv.cellEvaluations();
        BrushSettings b;
        b.diameter = 6.0;
        REQUIRE(ctl.beginStroke(b, {20.0, 20.0}).ok);
        pv.update(*ctl.working(), ctl.takeDirty());
        const std::uint64_t first = pv.cellEvaluations();
        REQUIRE(ctl.extendStroke({21.0, 20.0}).ok);
        pv.update(*ctl.working(), ctl.takeDirty());
        return std::pair<std::uint64_t, std::uint64_t>{atStart, pv.cellEvaluations() - first};
    };
    const auto small = perMove(100);
    const auto big = perMove(800);
    CHECK(big.first > 10 * small.first);  // composing the rest happened at the start ...
    CHECK(small.second == big.second);    // ... and a move does not depend on the model size
    CHECK(big.second < 6000);             // ~ (6+6+2)^2 cells x a few layers
    CHECK(big.second * 100 < big.first);
}
