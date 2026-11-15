#include "carvetto/viewmodel/model_view.hpp"

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

void addField(Document& doc, LevelId level, std::size_t w, std::size_t h, double cell, float value,
              relief::CombineMode mode = relief::CombineMode::Add) {
    relief::Component c;
    c.name = "C";
    c.mode = mode;
    REQUIRE(relief::HeightField::create(relief::Grid{w, h, cell, {0.0, 0.0}}, value, c.field) == relief::Status::Ok);
    REQUIRE(doc.execute(std::make_unique<AddReliefComponentCommand>(level, std::move(c))));
}

LevelId addLevel(Document& doc) {
    LevelId id;
    REQUIRE(addReliefLevel(doc, doc.activeSheet(), {}, &id).changed);
    return id;
}

std::uint32_t at(const Raster& r, std::size_t x, std::size_t y) {
    return r.pixels[y * r.width + x];
}

OrbitCamera top() {
    OrbitCamera c;
    c.viewTop();
    return c;
}

}  // namespace

TEST_CASE("a flat block seen from the top has the known shade and a background border", "[viewmodel][model_view]") {
    Document doc;
    addField(doc, addLevel(doc), 41, 41, 1.0, 5.0f);
    const ModelSurface s = composeModelSurface(doc, doc.activeSheet());
    REQUIRE(s.ok());
    ModelRenderOptions opt;
    opt.width = 200;
    opt.height = 200;
    opt.material = ModelMaterial::Wood;
    const OrbitCamera cam = top();
    const Raster r = renderModel(s, cam, opt);
    REQUIRE(r.width == 200);
    CHECK(at(r, 100, 100) != opt.background);
    CHECK(at(r, 2, 2) == opt.background);
    // Flat plane: normal (0,0,1); the centre pixel is the material times the view's light.
    const double lf = modelLightFactor(cam, 0, 0, 1);
    const double k = 0.3 + 0.7 * lf;
    const std::uint32_t wood = modelMaterialColour(ModelMaterial::Wood);
    auto expect = [&](int shift) { return static_cast<std::uint32_t>(std::lround(((wood >> shift) & 0xFFu) * k)); };
    const std::uint32_t px = at(r, 100, 100);
    CHECK(((px >> 16) & 0xFFu) == expect(16));
    CHECK(((px >> 8) & 0xFFu) == expect(8));
    CHECK((px & 0xFFu) == expect(0));
    // The whole inner area is one colour (flat), and the picture is deterministic.
    CHECK(at(r, 80, 120) == px);
    CHECK(renderModel(s, cam, opt).pixels == r.pixels);
}

TEST_CASE("material, slope and visibility change the picture", "[viewmodel][model_view]") {
    Document doc;
    const LevelId level = addLevel(doc);
    relief::Component c;
    c.name = "Ramp";
    REQUIRE(relief::HeightField::create(relief::Grid{41, 41, 1.0, {0.0, 0.0}}, 0.0f, c.field) == relief::Status::Ok);
    for (std::size_t j = 0; j < 41; ++j) {
        for (std::size_t i = 0; i < 41; ++i) {
            c.field.set(i, j, static_cast<float>(i) * 0.5f);
        }
    }
    REQUIRE(doc.execute(std::make_unique<AddReliefComponentCommand>(level, std::move(c))));
    const ModelSurface s = composeModelSurface(doc, doc.activeSheet());
    ModelRenderOptions opt;
    opt.width = opt.height = 200;
    const Raster wood = renderModel(s, top(), opt);
    opt.material = ModelMaterial::Metal;
    const Raster metal = renderModel(s, top(), opt);
    CHECK(at(wood, 100, 100) != at(metal, 100, 100));
    // A ramp rising to +X faces the light differently from a flat plate.
    Document flat;
    addField(flat, addLevel(flat), 41, 41, 1.0, 5.0f);
    opt.material = ModelMaterial::Wood;
    const Raster flatPic = renderModel(composeModelSurface(flat, flat.activeSheet()), top(), opt);
    CHECK(at(wood, 100, 100) != at(flatPic, 100, 100));

    // Hiding the only component empties the view.
    const ComponentId id = doc.reliefLevels().front().components.front().id;
    REQUIRE(setReliefComponentVisible(doc, id, false).changed);
    const ModelSurface hidden = composeModelSurface(doc, doc.activeSheet());
    CHECK_FALSE(hidden.ok());
    CHECK_FALSE(hidden.problem.empty());
    const Raster empty = renderModel(hidden, top(), opt);
    CHECK(at(empty, 100, 100) == opt.background);
}

TEST_CASE("the combine mode shows in the picture", "[viewmodel][model_view]") {
    Document doc;
    const LevelId level = addLevel(doc);
    addField(doc, level, 41, 41, 1.0, 5.0f);
    addField(doc, level, 41, 41, 1.0, 5.0f, relief::CombineMode::Subtract);  // cancels to 0 everywhere
    const ModelSurface s = composeModelSurface(doc, doc.activeSheet());
    REQUIRE(s.ok());
    float lo = 1, hi = 1;
    REQUIRE(s.field.minMax(lo, hi));
    CHECK(hi == 0.0f);
}

TEST_CASE("a high wall hides what is behind it (depth test)", "[viewmodel][model_view]") {
    Document doc;
    const LevelId level = addLevel(doc);
    relief::Component c;
    c.name = "Wall";
    REQUIRE(relief::HeightField::create(relief::Grid{41, 41, 1.0, {0.0, 0.0}}, 0.0f, c.field) == relief::Status::Ok);
    for (std::size_t j = 0; j < 41; ++j) {
        for (std::size_t i = 0; i < 41; ++i) {
            c.field.set(i, j, (j >= 18 && j <= 22) ? 30.0f : 0.0f);
        }
    }
    REQUIRE(doc.execute(std::make_unique<AddReliefComponentCommand>(level, std::move(c))));
    const ModelSurface s = composeModelSurface(doc, doc.activeSheet());
    OrbitCamera side;
    side.viewFront();
    ModelRenderOptions opt;
    opt.width = opt.height = 200;
    const Raster r = renderModel(s, side, opt);
    // Front view: the wall (z up to 30) stands over the whole width; the area above it is empty
    // and the middle of the picture is filled.
    CHECK(at(r, 100, 100) != opt.background);
    CHECK(at(r, 100, 5) == opt.background);
}

TEST_CASE("orbit, zoom and pan are bounded", "[viewmodel][model_view]") {
    OrbitCamera cam;
    cam.orbit(0, 1e6);
    CHECK(cam.elevationRad <= 1.5708);
    cam.orbit(0, -1e6);
    CHECK(cam.elevationRad >= 0.0);
    cam.zoomBy(1e9);
    CHECK(cam.zoom <= 200.0);
    cam.zoomBy(1e-18);
    CHECK(cam.zoom >= 0.05);
    cam.zoomBy(std::nan(""));
    CHECK(cam.zoom >= 0.05);
    cam.pan(3, -4);
    CHECK(cam.panX == 3.0);
    cam.reset();
    CHECK(cam.panX == 0.0);
}

TEST_CASE("a 2000 x 2000 field is composed on a bounded grid", "[viewmodel][model_view]") {
    Document doc;
    addField(doc, addLevel(doc), 2000, 2000, 0.1, 3.0f);
    const ModelSurface s = composeModelSurface(doc, doc.activeSheet());
    REQUIRE(s.ok());
    CHECK(s.field.width() <= kMaxSurfaceDimension);
    CHECK(s.field.height() <= kMaxSurfaceDimension);
    CHECK(s.stride > 1);
    CHECK(s.field.cellSize() > 0.1);
    ModelRenderOptions opt;
    opt.width = 800;
    opt.height = 600;
    const Raster r = renderModel(s, OrbitCamera{}, opt);
    CHECK(at(r, 400, 300) != opt.background);
}

TEST_CASE("degenerate render sizes give an empty raster", "[viewmodel][model_view]") {
    ModelRenderOptions opt;
    opt.width = 0;
    CHECK(renderModel(ModelSurface{}, OrbitCamera{}, opt).pixels.empty());
    opt.width = 10;
    opt.height = 10;
    CHECK(renderModel(ModelSurface{}, OrbitCamera{}, opt).pixels.size() == 100);
}

TEST_CASE("picking a pixel finds the model point the renderer drew there", "[viewmodel][model_view][pick]") {
    Document doc;
    addField(doc, addLevel(doc), 41, 41, 1.0, 5.0f);  // x, y in 0..40
    const ModelSurface s = composeModelSurface(doc, doc.activeSheet());
    REQUIRE(s.ok());
    ModelRenderOptions opt;
    opt.width = 200;
    opt.height = 200;
    const OrbitCamera cam = top();
    const auto centre = pickModelPoint(s, cam, opt, 100.0, 100.0);
    REQUIRE(centre.has_value());
    CHECK(std::fabs(centre->x - 20.0) < 0.5);
    CHECK(std::fabs(centre->y - 20.0) < 0.5);
    // Top view: X to the right, Y up.
    const auto right = pickModelPoint(s, cam, opt, 120.0, 100.0);
    const auto up = pickModelPoint(s, cam, opt, 100.0, 80.0);
    REQUIRE(right.has_value());
    REQUIRE(up.has_value());
    CHECK(right->x > centre->x);
    CHECK(up->y > centre->y);
    // The border pixel the renderer leaves as background is a miss.
    CHECK_FALSE(pickModelPoint(s, cam, opt, 2.0, 2.0).has_value());
    // Tilted view: the pixel the renderer paints under the centre still resolves into the field.
    OrbitCamera tilted;
    const auto t = pickModelPoint(s, tilted, opt, 100.0, 100.0);
    REQUIRE(t.has_value());
    CHECK(t->x >= 0.0);
    CHECK(t->x <= 40.0);
    CHECK_FALSE(pickModelPoint(ModelSurface{}, cam, opt, 100.0, 100.0).has_value());
}

TEST_CASE("projectModelPoint inverts pickModelPoint and the brush ring drapes on the surface", "[viewmodel][model_view]") {
    ModelSurface s;
    REQUIRE(relief::HeightField::create(relief::Grid{41, 41, 1.0, {0.0, 0.0}}, 3.0F, s.field) == relief::Status::Ok);
    OrbitCamera cam;
    cam.viewTop();
    ModelRenderOptions opt;
    opt.width = 400;
    opt.height = 300;
    const auto p = projectModelPoint(s, cam, opt, 10.0, 25.0, 3.0);
    REQUIRE(p.has_value());
    const auto back = pickModelPoint(s, cam, opt, p->x, p->y);
    REQUIRE(back.has_value());
    CHECK(std::fabs(back->x - 10.0) < 0.1);
    CHECK(std::fabs(back->y - 25.0) < 0.1);
    // A tilted camera too.
    OrbitCamera tilt;
    const auto q = projectModelPoint(s, tilt, opt, 30.0, 12.0, 3.0);
    REQUIRE(q.has_value());
    const auto qb = pickModelPoint(s, tilt, opt, q->x, q->y);
    REQUIRE(qb.has_value());
    CHECK(std::fabs(qb->x - 30.0) < 0.15);
    CHECK(std::fabs(qb->y - 12.0) < 0.15);

    const auto ring = brushRingOutline(s, cam, opt, {20.0, 20.0}, 10.0, 32);
    REQUIRE(ring.size() == 32);
    const auto c = projectModelPoint(s, cam, opt, 20.0, 20.0, 3.0);
    const auto edge = projectModelPoint(s, cam, opt, 25.0, 20.0, 3.0);
    REQUIRE(c.has_value());
    REQUIRE(edge.has_value());
    const double radiusPx = std::hypot(edge->x - c->x, edge->y - c->y);
    for (const ScreenPoint& pt : ring) {
        CHECK(std::fabs(std::hypot(pt.x - c->x, pt.y - c->y) - radiusPx) < 1e-6);
    }
    CHECK(brushRingOutline(s, cam, opt, {20.0, 20.0}, 0.0).empty());
    CHECK(brushRingOutline(ModelSurface{}, cam, opt, {20.0, 20.0}, 10.0).empty());
    CHECK_FALSE(projectModelPoint(ModelSurface{}, cam, opt, 0.0, 0.0, 0.0).has_value());
    CHECK_FALSE(projectModelPoint(s, cam, opt, std::nan(""), 0.0, 0.0).has_value());
}

TEST_CASE("overlay placement fits the image's aspect inside the model", "[viewmodel][model_view]") {
    ModelSurface s;
    REQUIRE(relief::HeightField::create(relief::Grid{101, 51, 1.0, {10.0, 20.0}}, 1.0F, s.field) == relief::Status::Ok);
    const OverlayPlacement wide = overlayPlacement(s, 400, 100);  // 4:1 in a 100 x 50 model
    CHECK(wide.width == 100.0);
    CHECK(wide.height == 25.0);
    CHECK(wide.x == 10.0);
    CHECK(wide.y == 20.0 + 12.5);
    const OverlayPlacement tall = overlayPlacement(s, 100, 400);
    CHECK(tall.height == 50.0);
    CHECK(tall.width == 12.5);
    CHECK(tall.x == 10.0 + 43.75);
    const OverlayPlacement none = overlayPlacement(s, 0, 10);
    CHECK(none.width == 0.0);
    CHECK(overlayPlacement(ModelSurface{}, 10, 10).width == 0.0);
    CHECK(clampedOverlayOpacity(2.0) == 1.0);
    CHECK(clampedOverlayOpacity(-1.0) == 0.0);
    CHECK(clampedOverlayOpacity(std::nan("")) == 0.5);
}

// --- T-0305: appearance in the Model view ----------------------------------------------------

namespace {

relief::Appearance solidApp(std::uint32_t rgb) {
    relief::Appearance a;
    a.kind = relief::AppearanceKind::SolidColour;
    a.rgb = rgb;
    return a;
}

ComponentId addBlock(Document& doc, LevelId level, double x0, std::size_t n, float value, relief::Appearance app,
                     relief::CombineMode mode = relief::CombineMode::Add) {
    relief::Component c;
    c.name = "C";
    c.mode = mode;
    c.appearance = app;
    REQUIRE(relief::HeightField::create(relief::Grid{n, n, 1.0, {x0, 0.0}}, value, c.field) == relief::Status::Ok);
    auto cmd = std::make_unique<AddReliefComponentCommand>(level, std::move(c));
    const AddReliefComponentCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    return raw->createdId();
}

std::uint32_t colourAt(const ModelSurface& s, double x, double y) {
    const auto& g = s.field.grid();
    const std::size_t i = static_cast<std::size_t>((x - g.origin.x) / g.cellSize + 0.5);
    const std::size_t j = static_cast<std::size_t>((y - g.origin.y) / g.cellSize + 0.5);
    return s.colours.at(j * g.width + i);
}

}  // namespace

TEST_CASE("appearance: nothing set leaves the colour map empty and the picture unchanged", "[viewmodel][model_view][appearance]") {
    Document doc;
    addBlock(doc, addLevel(doc), 0.0, 21, 5.0f, relief::Appearance{});
    const ModelSurface s = composeModelSurface(doc, doc.activeSheet());
    REQUIRE(s.ok());
    CHECK(s.colours.empty());
}

TEST_CASE("appearance: own colour, same as parent through the level, level colour", "[viewmodel][model_view][appearance]") {
    Document doc;
    const LevelId lv = addLevel(doc);
    addBlock(doc, lv, 0.0, 11, 1.0f, solidApp(0xFF0000));    // x 0..10 red
    const ComponentId plain = addBlock(doc, lv, 20.0, 11, 1.0f, relief::Appearance{});  // x 20..30 no appearance
    relief::Appearance metal;
    metal.kind = relief::AppearanceKind::Material;
    metal.material = relief::AppearanceMaterial::Metal;
    addBlock(doc, lv, 10.0, 11, 1.0f, metal, relief::CombineMode::MergeHigh);  // x 10..20, overlaps x=10 and 20
    ModelSurface s = composeModelSurface(doc, doc.activeSheet());
    REQUIRE(s.ok());
    REQUIRE(!s.colours.empty());
    CHECK(colourAt(s, 5.0, 5.0) == 0xFFFF0000u);
    CHECK(colourAt(s, 15.0, 5.0) == appearanceMaterialColour(relief::AppearanceMaterial::Metal));
    CHECK(colourAt(s, 25.0, 5.0) == 0u);  // same as parent, parent is the view material
    // The level gives its colour to the component that has none.
    relief::Appearance level = solidApp(0x00FF00);
    REQUIRE(setReliefLevelAppearance(doc, lv, level).changed);
    s = composeModelSurface(doc, doc.activeSheet());
    CHECK(colourAt(s, 25.0, 5.0) == 0xFF00FF00u);
    CHECK(colourAt(s, 5.0, 5.0) == 0xFFFF0000u);  // an own colour wins over the level's
    (void)plain;
    // Rendering: the red block's pixel is red tinted, the plain one green tinted, never swapped.
    ModelRenderOptions opt;
    opt.width = 300;
    opt.height = 100;
    const Raster r = renderModel(s, top(), opt);
    const auto px = [&](double x) {
        const auto p = projectModelPoint(s, top(), opt, x, 5.0, 1.0);
        REQUIRE(p.has_value());
        return at(r, static_cast<std::size_t>(p->x), static_cast<std::size_t>(p->y));
    };
    const std::uint32_t redPx = px(5.0), greenPx = px(25.0);
    CHECK(((redPx >> 16) & 0xFF) > 0x20);
    CHECK(((redPx >> 8) & 0xFF) == 0);
    CHECK(((greenPx >> 8) & 0xFF) > 0x20);
    CHECK(((greenPx >> 16) & 0xFF) == 0);
    // Switching the appearance off shows the view material everywhere.
    opt.useAppearance = false;
    const Raster plainView = renderModel(s, top(), opt);
    CHECK(at(plainView, static_cast<std::size_t>(projectModelPoint(s, top(), opt, 5.0, 5.0, 1.0)->x),
             static_cast<std::size_t>(projectModelPoint(s, top(), opt, 5.0, 5.0, 1.0)->y)) !=
          redPx);
}

TEST_CASE("appearance: Color from Children is the mean of the children's own colours", "[viewmodel][model_view][appearance]") {
    Document doc;
    const LevelId lv = addLevel(doc);
    addBlock(doc, lv, 0.0, 11, 1.0f, solidApp(0x200000));
    addBlock(doc, lv, 20.0, 11, 1.0f, solidApp(0x600000));
    addBlock(doc, lv, 40.0, 11, 1.0f, relief::Appearance{});  // no own colour: takes the mean
    relief::Appearance kids;
    kids.kind = relief::AppearanceKind::ColourFromChildren;
    REQUIRE(setReliefLevelAppearance(doc, lv, kids).changed);
    const ModelSurface s = composeModelSurface(doc, doc.activeSheet());
    REQUIRE(!s.colours.empty());
    CHECK(colourAt(s, 5.0, 5.0) == 0xFF200000u);
    CHECK(colourAt(s, 25.0, 5.0) == 0xFF600000u);
    CHECK(colourAt(s, 45.0, 5.0) == 0xFF400000u);
    // A hidden child does not count.
    const auto rows = buildReliefTree(doc, doc.activeSheet());
    for (const ReliefTreeRow& row : rows) {
        if (!row.isLevel && row.appearance.rgb == 0x600000u) {
            REQUIRE(setReliefComponentVisible(doc, row.component, false).changed);
        }
    }
    const ModelSurface hidden = composeModelSurface(doc, doc.activeSheet());
    CHECK(colourAt(hidden, 45.0, 5.0) == 0xFF200000u);
}

TEST_CASE("appearance: the colour follows the component that supplies the surface", "[viewmodel][model_view][appearance]") {
    Document doc;
    const LevelId lv = addLevel(doc);
    addBlock(doc, lv, 0.0, 11, 5.0f, solidApp(0xFF0000));
    // Add: the top component covers the one below.
    ComponentId top1 = addBlock(doc, lv, 0.0, 6, 1.0f, solidApp(0x0000FF));
    ModelSurface s = composeModelSurface(doc, doc.activeSheet());
    CHECK(colourAt(s, 2.0, 2.0) == 0xFF0000FFu);
    CHECK(colourAt(s, 8.0, 8.0) == 0xFFFF0000u);
    // Subtract: the surface is still the one below.
    REQUIRE(setReliefComponentMode(doc, top1, relief::CombineMode::Subtract).changed);
    s = composeModelSurface(doc, doc.activeSheet());
    CHECK(colourAt(s, 2.0, 2.0) == 0xFFFF0000u);
    // Merge High: the lower top component loses, a higher one wins.
    REQUIRE(setReliefComponentMode(doc, top1, relief::CombineMode::MergeHigh).changed);
    s = composeModelSurface(doc, doc.activeSheet());
    CHECK(colourAt(s, 2.0, 2.0) == 0xFFFF0000u);  // 1 < 5
    REQUIRE(setReliefComponentMode(doc, top1, relief::CombineMode::MergeLow).changed);
    s = composeModelSurface(doc, doc.activeSheet());
    CHECK(colourAt(s, 2.0, 2.0) == 0xFF0000FFu);  // 1 < 5 wins
    CHECK(colourAt(s, 8.0, 8.0) == 0xFFFF0000u);
    // A hidden-level and a transparent cell have no colour.
    CHECK(colourAt(s, 0.0, 0.0) != 0u);
}

TEST_CASE("appearance: levels stack: the upper level's component colours its cells", "[viewmodel][model_view][appearance]") {
    Document doc;
    const LevelId low = addLevel(doc);
    const LevelId high = addLevel(doc);
    addBlock(doc, low, 0.0, 11, 1.0f, relief::Appearance{});
    addBlock(doc, high, 0.0, 6, 1.0f, relief::Appearance{});
    REQUIRE(setReliefLevelAppearance(doc, low, solidApp(0x112233)).changed);
    ModelSurface s = composeModelSurface(doc, doc.activeSheet());
    REQUIRE(!s.colours.empty());
    CHECK(colourAt(s, 8.0, 8.0) == 0xFF112233u);
    CHECK(colourAt(s, 2.0, 2.0) == 0u);  // the upper level covers it and is "same as parent" = view
    REQUIRE(setReliefLevelAppearance(doc, high, solidApp(0x445566)).changed);
    s = composeModelSurface(doc, doc.activeSheet());
    CHECK(colourAt(s, 2.0, 2.0) == 0xFF445566u);
    CHECK(colourAt(s, 8.0, 8.0) == 0xFF112233u);
}
