#include "carvetto/viewmodel/relief_sculpt.hpp"

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>
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

TEST_CASE("a custom brush stamp comes from an image's luminance and alpha", "[viewmodel][sculpt]") {
    RgbaImage img;
    img.width = 4;
    img.height = 2;
    img.rgba.assign(4 * 2 * 4, 255);
    // Left half black, right half white; the last column transparent.
    for (std::size_t y = 0; y < 2; ++y) {
        for (std::size_t x = 0; x < 4; ++x) {
            std::uint8_t* px = &img.rgba[(y * 4 + x) * 4];
            px[0] = px[1] = px[2] = x < 2 ? 0 : 255;
            px[3] = x == 3 ? 0 : 255;
        }
    }
    std::string problem;
    const auto cb = customBrushFromImage(img, false, 0.5, true, problem);
    REQUIRE(cb != nullptr);
    CHECK(problem.empty());
    CHECK(cb->stamp.width() == 4);
    CHECK(cb->stamp.at(0, 0) == 0.0F);
    CHECK(cb->stamp.at(2, 1) == 1.0F);
    CHECK(std::isnan(cb->stamp.at(3, 0)));
    CHECK(cb->angleRad == 0.5);
    CHECK(cb->dynamicRotation);
    const auto inv = customBrushFromImage(img, true, 0.0, false, problem);
    REQUIRE(inv != nullptr);
    CHECK(inv->stamp.at(0, 0) == 1.0F);
    CHECK(std::fabs(inv->stamp.at(2, 0)) < 1e-6F);

    // Large images are averaged down; bad ones refused with a reason.
    RgbaImage big;
    big.width = 1000;
    big.height = 500;
    big.rgba.assign(big.width * big.height * 4, 200);
    const auto small = customBrushFromImage(big, false, 0.0, false, problem);
    REQUIRE(small != nullptr);
    CHECK(small->stamp.width() <= kMaxStampDimension);
    CHECK(small->stamp.height() <= kMaxStampDimension);
    RgbaImage bad = img;
    bad.rgba.pop_back();
    CHECK(customBrushFromImage(bad, false, 0.0, false, problem) == nullptr);
    CHECK_FALSE(problem.empty());
    RgbaImage clear = img;
    for (std::size_t i = 3; i < clear.rgba.size(); i += 4) {
        clear.rgba[i] = 0;
    }
    CHECK(customBrushFromImage(clear, false, 0.0, false, problem) == nullptr);
    CHECK(customBrushFromImage(RgbaImage{}, false, 0.0, false, problem) == nullptr);
    CHECK(customBrushFromImage(img, false, std::nan(""), false, problem) == nullptr);
}

TEST_CASE("a custom stamp paints in the shape of the image", "[viewmodel][sculpt]") {
    Document doc;
    const ComponentId id = addFlat(doc, addLevel(doc), 0.0F);
    RgbaImage img;  // an 8 x 8 stamp: left half black (no effect), right half white (full)
    img.width = 8;
    img.height = 8;
    img.rgba.assign(8 * 8 * 4, 255);
    for (std::size_t y = 0; y < 8; ++y) {
        for (std::size_t x = 0; x < 4; ++x) {
            img.rgba[(y * 8 + x) * 4] = img.rgba[(y * 8 + x) * 4 + 1] = img.rgba[(y * 8 + x) * 4 + 2] = 0;
        }
    }
    std::string problem;
    BrushSettings brush;
    brush.diameter = 20.0;
    brush.strength = 1.0;
    brush.smoothness = 0.0;
    brush.custom = customBrushFromImage(img, false, 0.0, false, problem);
    REQUIRE(brush.custom != nullptr);
    SculptController sc;
    REQUIRE(sc.begin(doc, id).ok);
    REQUIRE(sc.stroke(brush, {{20.0, 20.0}}).ok);
    // The white half (right of the centre) is raised, the black half (left) is not.
    CHECK(sc.working()->at(26, 20) > sc.working()->at(14, 20));
    CHECK(sc.working()->at(14, 20) == 0.0F);
}

TEST_CASE("user presets round-trip through text; hostile text is skipped", "[viewmodel][sculpt]") {
    BrushPresets a;
    BrushSettings b;
    b.kind = relief::BrushKind::Smooth;
    b.diameter = 33.25;
    b.strength = 0.125;
    b.smoothness = 0.75;
    b.mode = relief::SculptMode::Lower;
    b.preserveTransparency = false;
    REQUIRE(a.save("Mine", b).empty());
    REQUIRE(a.save("Other", BrushSettings{}).empty());
    CHECK(a.save("bad\tname", b).size() > 0);
    CHECK(a.save("bad\nname", b).size() > 0);
    const std::string text = a.serialiseUser();
    BrushPresets c;
    CHECK(c.loadUser(text) == 2);
    REQUIRE(c.find("Mine") != nullptr);
    CHECK(c.find("Mine")->brush == b);
    CHECK_FALSE(c.find("Mine")->builtin);
    CHECK(c.all().size() == a.all().size());
    // Loading again replaces rather than duplicates.
    CHECK(c.loadUser(text) == 2);
    CHECK(c.all().size() == a.all().size());
    // Garbage: wrong header, short lines, bad numbers, out of range, built-in name, huge counts.
    BrushPresets d;
    const std::size_t builtins = d.all().size();
    CHECK(d.loadUser("") == 0);
    CHECK(d.loadUser("nonsense\nA\t1\t2\n") == 0);
    CHECK(d.loadUser("carvetto-brush-presets 1\nA\t1\t2\n") == 0);
    CHECK(d.loadUser("carvetto-brush-presets 1\nA\t9\t10\t0.5\t0.5\t0\t1\n") == 0);
    CHECK(d.loadUser("carvetto-brush-presets 1\nA\t1\tnan\t0.5\t0.5\t0\t1\n") == 0);
    CHECK(d.loadUser("carvetto-brush-presets 1\nA\t1\t500\t0.5\t0.5\t0\t1\n") == 0);
    CHECK(d.loadUser("carvetto-brush-presets 1\nA\t1\t10x\t0.5\t0.5\t0\t1\n") == 0);
    CHECK(d.loadUser("carvetto-brush-presets 1\nSmudge\t1\t10\t0.5\t0.5\t0\t1\n") == 0);  // built-in name
    CHECK(d.all().size() == builtins);
    std::string many = "carvetto-brush-presets 1\n";
    for (int i = 0; i < 400; ++i) {
        many += "P" + std::to_string(i) + "\t2\t10\t0.5\t0.5\t0\t1\n";
    }
    CHECK(d.loadUser(many) == 256);
    CHECK(d.all().size() == builtins + 256);
}

TEST_CASE("a long stroke is chunked: bounded work per move, one undo step, same total", "[viewmodel][sculpt]") {
    Document doc;
    const ComponentId id = addFlat(doc, addLevel(doc), 0.0F);
    BrushSettings brush;
    brush.diameter = 0.5;  // the path below is 80 diameters long: several 30-diameter chunks
    brush.strength = 0.1;
    SculptController sc;
    REQUIRE(sc.begin(doc, id).ok);
    REQUIRE(sc.beginStroke(brush, {0.0, 20.0}).ok);
    for (int k = 1; k <= 400; ++k) {
        REQUIRE(sc.extendStroke({static_cast<double>(k) * 0.1, 20.0}).ok);
    }
    CHECK(sc.openStrokeSteps() >= 3);       // chunked into several session steps
    sc.endStroke();
    const float mid = sc.working()->at(20, 20);
    CHECK(mid > 0.0F);
    // Same stroke as one piece: the deposit per cell agrees to within the dab phase at chunk joins.
    SculptController whole;
    REQUIRE(whole.begin(doc, id).ok);
    REQUIRE(whole.stroke(brush, {{0.0, 20.0}, {40.0, 20.0}}).ok);
    for (std::size_t i = 2; i < 38; ++i) {
        CHECK(std::fabs(sc.working()->at(i, 20) - whole.working()->at(i, 20)) < 0.1F * mid + 1e-4F);
    }
    // The whole chunked stroke is ONE undo step.
    CHECK(sc.canUndoStroke());
    CHECK(sc.undoStroke());
    CHECK_FALSE(sc.dirty());
    CHECK_FALSE(sc.canUndoStroke());
    CHECK(sc.working()->at(20, 20) == 0.0F);
    // Two strokes undo one at a time.
    REQUIRE(sc.stroke(brush, {{0.0, 10.0}, {40.0, 10.0}}).ok);
    REQUIRE(sc.stroke(brush, {{0.0, 30.0}, {40.0, 30.0}}).ok);
    CHECK(sc.undoStroke());
    CHECK(sc.working()->at(20, 30) == 0.0F);
    CHECK(sc.working()->at(20, 10) > 0.0F);
    CHECK(sc.undoStroke());
    CHECK(sc.working()->at(20, 10) == 0.0F);
}
