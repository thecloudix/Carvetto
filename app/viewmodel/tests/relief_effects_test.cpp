#include "carvetto/viewmodel/relief_effects.hpp"

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <memory>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/relief_commands.hpp"
#include "carvetto/viewmodel/model_view.hpp"
#include "carvetto/viewmodel/relief_tree.hpp"

using namespace carvetto;
using namespace carvetto::document;
using namespace carvetto::viewmodel;

namespace {

LevelId addLevel(Document& doc) {
    LevelId id;
    REQUIRE(addReliefLevel(doc, doc.activeSheet(), {}, &id).changed);
    return id;
}

ComponentId addRamp(Document& doc, LevelId level) {
    relief::Component c;
    c.name = "Ramp";
    REQUIRE(relief::HeightField::create(relief::Grid{41, 41, 1.0, {0.0, 0.0}}, 0.0F, c.field) == relief::Status::Ok);
    for (std::size_t j = 0; j < 41; ++j) {
        for (std::size_t i = 0; i < 41; ++i) {
            c.field.set(i, j, static_cast<float>(i));  // height rises along X
        }
    }
    auto cmd = std::make_unique<AddReliefComponentCommand>(level, std::move(c));
    const AddReliefComponentCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    return raw->createdId();
}

ObjectId addVector(Document& doc, geometry::Contour contour) {
    Object o;
    o.layer = doc.defaultLayer();
    o.sheet = doc.activeSheet();
    VectorObject v;
    v.contours.push_back(std::move(contour));
    o.data = std::move(v);
    auto cmd = std::make_unique<AddObjectCommand>(std::move(o));
    const AddObjectCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    return raw->id();
}

}  // namespace

TEST_CASE("mirror mode names and the model centre", "[viewmodel][effects]") {
    CHECK(allMirrorModes().size() == 9);
    CHECK(std::string(mirrorModeName(relief::MirrorMode::QuadTopLeft)) == "Top left quadrant");
    Document doc;
    CHECK_FALSE(modelCentre(doc, doc.activeSheet()).has_value());
    const LevelId level = addLevel(doc);
    addRamp(doc, level);
    const auto c = modelCentre(doc, doc.activeSheet());
    REQUIRE(c.has_value());
    CHECK(c->x == 20.0);
    CHECK(c->y == 20.0);
}

TEST_CASE("the effects target level: picked, selected component's, else the top level", "[viewmodel][effects]") {
    Document doc;
    CHECK_FALSE(effectsTargetLevel(doc));
    const LevelId low = addLevel(doc);
    const LevelId top = addLevel(doc);
    const ComponentId inLow = addRamp(doc, low);
    doc.componentSelection().clear();
    CHECK(effectsTargetLevel(doc) == top);
    doc.componentSelection().assign({inLow});
    CHECK(effectsTargetLevel(doc) == low);
    CHECK(effectsTargetLevel(doc, top) == top);
}

TEST_CASE("clipping from selected closed vectors: apply, update, remove, each one undo step",
          "[viewmodel][effects]") {
    Document doc;
    const LevelId level = addLevel(doc);
    addRamp(doc, level);
    CHECK_FALSE(applyLevelClip(doc, level).ok);  // nothing selected
    CHECK_FALSE(removeLevelClip(doc, level).ok);
    const ObjectId r = addVector(doc, geometry::Contour::rectangle(geometry::Box2({10.0, 10.0}, {30.0, 30.0})));
    doc.selection().assign({r});
    REQUIRE(summarizeEffects(doc, level)->selectedClosedVectors == 1);
    const std::size_t steps = doc.undoStack().count();
    const EditResult a = applyLevelClip(doc, level);
    REQUIRE(a.ok);
    CHECK(a.changed);
    CHECK(doc.undoStack().count() == steps + 1);
    CHECK(doc.findReliefLevel(level)->effects.clip.polygons.size() == 1);
    CHECK(summarizeEffects(doc, level)->text().find("clipped by 1 polygon") != std::string::npos);
    // The composed model is transparent outside the outline and keeps the ramp inside it.
    const ModelSurface s = composeModelSurface(doc, doc.activeSheet());
    REQUIRE(s.ok());
    CHECK(std::isnan(s.field.sample(5.0, 20.0)));
    CHECK(s.field.sample(20.0, 20.0) == 20.0F);
    CHECK_FALSE(applyLevelClip(doc, level).ok);  // unchanged: refused, no extra step
    CHECK(doc.undoStack().count() == steps + 1);
    // Update with a circle (curves are flattened).
    const ObjectId c = addVector(doc, geometry::Contour::circle({20.0, 20.0}, 8.0));
    doc.selection().assign({c});
    REQUIRE(applyLevelClip(doc, level).ok);
    const relief::Clip& clip = doc.findReliefLevel(level)->effects.clip;
    REQUIRE(clip.polygons.size() == 1);
    CHECK(clip.polygons[0].size() > 8);
    REQUIRE(removeLevelClip(doc, level).ok);
    CHECK(doc.findReliefLevel(level)->effects.clip.polygons.empty());
    REQUIRE(doc.undoStack().undo(doc));
    CHECK(doc.findReliefLevel(level)->effects.clip.polygons.size() == 1);
}

TEST_CASE("open vectors are not used for clipping", "[viewmodel][effects]") {
    Document doc;
    const LevelId level = addLevel(doc);
    addRamp(doc, level);
    geometry::Polyline pl;
    pl.points = {{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};
    pl.closed = false;
    const geometry::Contour open = geometry::Contour::fromPolyline(pl);
    doc.selection().assign({addVector(doc, open)});
    const EditResult r = applyLevelClip(doc, level);
    CHECK_FALSE(r.ok);
    CHECK_FALSE(r.message.empty());
}

TEST_CASE("mirror mode uses the model centre, undoes, and bakes", "[viewmodel][effects]") {
    Document doc;
    const LevelId level = addLevel(doc);
    addRamp(doc, level);
    const std::size_t steps = doc.undoStack().count();
    REQUIRE(setLevelMirror(doc, level, relief::MirrorMode::LeftToRight).ok);
    CHECK(doc.undoStack().count() == steps + 1);
    const document::ReliefLevel* l = doc.findReliefLevel(level);
    CHECK(l->effects.mirror == relief::MirrorMode::LeftToRight);
    CHECK(l->effects.mirrorCenter.x == 20.0);
    CHECK(l->effects.mirrorCenter.y == 20.0);
    const ModelSurface s = composeModelSurface(doc, doc.activeSheet());
    REQUIRE(s.ok());
    // The left half (low) is copied onto the right half: x = 35 shows what is at x = 5.
    CHECK(std::fabs(s.field.sample(35.0, 20.0) - 5.0F) < 1e-3F);
    CHECK_FALSE(setLevelMirror(doc, level, relief::MirrorMode::LeftToRight).ok);  // already set
    REQUIRE(bakeLevelEffects(doc, level).ok);
    CHECK_FALSE(doc.findReliefLevel(level)->effects.any());
    REQUIRE(doc.undoStack().undo(doc));
    CHECK(doc.findReliefLevel(level)->effects.mirror == relief::MirrorMode::LeftToRight);
    REQUIRE(setLevelMirror(doc, level, relief::MirrorMode::None).ok);
    CHECK(doc.findReliefLevel(level)->effects.mirror == relief::MirrorMode::None);
    CHECK_FALSE(bakeLevelEffects(doc, level).ok);  // no effects any more
    CHECK_FALSE(setLevelMirror(doc, LevelId{}, relief::MirrorMode::LeftToRight).ok);
}
