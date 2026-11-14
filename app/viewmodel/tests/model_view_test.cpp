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
