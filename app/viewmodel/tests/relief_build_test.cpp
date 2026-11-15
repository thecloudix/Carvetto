#include "carvetto/viewmodel/relief_build.hpp"

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <string>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/relief_commands.hpp"

using namespace carvetto;
using namespace carvetto::document;
using namespace carvetto::viewmodel;

namespace {

ObjectId addPolyline(Document& doc, std::initializer_list<geometry::Vec2> pts, bool closed) {
    Object o;
    o.layer = doc.defaultLayer();
    o.sheet = doc.activeSheet();
    VectorObject v;
    geometry::Polyline pl;
    pl.points.assign(pts.begin(), pts.end());
    pl.closed = closed;
    v.contours.push_back(geometry::Contour::fromPolyline(pl));
    o.data = std::move(v);
    auto cmd = std::make_unique<AddObjectCommand>(std::move(o));
    const AddObjectCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    return raw->id();
}

/// Highest cell of the component with its properties applied (Exact height lives in them).
float topOf(const relief::Component& c) {
    relief::HeightField baked;
    REQUIRE(relief::bake(c, baked) == relief::Status::Ok);
    float lo = 0.0F;
    float hi = 0.0F;
    REQUIRE(baked.minMax(lo, hi));
    return hi;
}

ComponentId addSeed(Document& doc) {
    relief::Component seed;
    relief::HeightField f;
    REQUIRE(relief::HeightField::create(relief::Grid{4, 4, 1.0, {0.0, 0.0}}, 2.0F, f) == relief::Status::Ok);
    seed.field = std::move(f);
    seed.name = "seed";
    ModellingSession session;
    REQUIRE(session.apply(doc, std::move(seed)).ok);
    return session.lastCreated();
}

}  // namespace

TEST_CASE("build: point lists parse and reject garbage", "[viewmodel][relief][build]") {
    std::vector<relief::Point> pts;
    std::string error;
    CHECK(parsePointList("0,0; 5, 4; 10,0;", pts, error));
    CHECK(pts.size() == 3);
    CHECK(pts[1].y == 4.0);
    CHECK_FALSE(parsePointList("0,0", pts, error));
    CHECK_FALSE(parsePointList("0,0; a,b", pts, error));
    CHECK_FALSE(parsePointList("0,0; 1,inf", pts, error));
}

TEST_CASE("build: every tool lists fields with defaults and parses its own default form",
          "[viewmodel][relief][build]") {
    for (int i = 0; i < kBuildToolCount; ++i) {
        const BuildTool t = static_cast<BuildTool>(i);
        CHECK(std::string(buildToolName(t)).size() > 3);
        CHECK(std::string(buildHelp(t)).size() > 10);
        CHECK(buildFields(t).size() > 3);
        CHECK(defaultBuildForm(t).values.size() == buildFields(t).size());
    }
}

TEST_CASE("build: Two Rail Sweep needs two rails and sweeps the profile between them",
          "[viewmodel][relief][build]") {
    Document doc;
    BuildForm form = defaultBuildForm(BuildTool::TwoRailSweep);
    CHECK_FALSE(previewBuild(doc, form).ok);
    const ObjectId a = addPolyline(doc, {{0, 0}, {40, 0}}, false);
    const ObjectId b = addPolyline(doc, {{0, 10}, {40, 10}}, false);
    doc.selection().assign({a});
    CHECK_FALSE(previewBuild(doc, form).ok);   // one rail only
    doc.selection().assign({a, b});
    const std::size_t steps = doc.undoStack().count();
    ModellingPreview p = previewBuild(doc, form);
    REQUIRE(p.ok);
    CHECK(doc.undoStack().count() == steps);   // preview does not touch the document
    CHECK(doc.reliefComponentCount() == 0);
    CHECK(topOf(p.component) > 3.5F);
    CHECK(topOf(p.component) <= 4.01F);
    form.values["Exact height"] = "2";
    p = previewBuild(doc, form);
    REQUIRE(p.ok);
    CHECK(std::fabs(topOf(p.component) - 2.0F) < 0.01F);
    form.values["Section profile"] = "0,0; 5,x";
    CHECK_FALSE(previewBuild(doc, form).ok);
    form.values["Section profile"] = "0,0; 5,4; 10,0";
    form.values["Cell size"] = "abc";
    p = previewBuild(doc, form);
    CHECK_FALSE(p.ok);
    CHECK(p.message.find("Cell size") != std::string::npos);
    form.values["Cell size"] = "0.5";
    form.values["Section at every node"] = "true";
    CHECK(previewBuild(doc, form).ok);   // both rails have two vertices
}

TEST_CASE("build: Two Rail Sweep apply is one undo step and re-applying updates in place",
          "[viewmodel][relief][build]") {
    Document doc;
    const ObjectId a = addPolyline(doc, {{0, 0}, {40, 0}}, false);
    const ObjectId b = addPolyline(doc, {{0, 10}, {40, 10}}, false);
    doc.selection().assign({a, b});
    BuildForm form = defaultBuildForm(BuildTool::TwoRailSweep);
    ModellingSession session;
    const std::size_t index = doc.undoStack().index();
    REQUIRE(session.apply(doc, previewBuild(doc, form).component).ok);
    CHECK(doc.reliefComponentCount() == 1);
    CHECK(doc.undoStack().index() == index + 1);
    form.values["Exact height"] = "1";
    REQUIRE(session.apply(doc, previewBuild(doc, form).component).ok);
    CHECK(doc.reliefComponentCount() == 1);
    CHECK(doc.undoStack().index() == index + 2);
    CHECK(topOf(doc.findReliefComponent(session.lastCreated())->data) < 1.01F);
    REQUIRE(doc.undo());
    REQUIRE(doc.undo());
    CHECK(doc.reliefComponentCount() == 0);
}

TEST_CASE("build: Extrude builds along selected rails from a profile or a component",
          "[viewmodel][relief][build]") {
    Document doc;
    BuildForm form = defaultBuildForm(BuildTool::ExtrudeWeave);
    CHECK_FALSE(previewBuild(doc, form).ok);
    const ObjectId r1 = addPolyline(doc, {{0, 10}, {40, 10}}, false);
    const ObjectId r2 = addPolyline(doc, {{20, 0}, {20, 20}}, false);
    doc.selection().assign({r1});
    ModellingPreview p = previewBuild(doc, form);
    REQUIRE(p.ok);
    CHECK(std::fabs(topOf(p.component) - 3.0F) < 0.3F);
    const float plain = topOf(p.component);
    doc.selection().assign({r1, r2});
    form.values["Weave"] = "true";
    form.values["Weave under %"] = "50";
    p = previewBuild(doc, form);
    REQUIRE(p.ok);
    CHECK(topOf(p.component) <= plain + 0.01F);
    form.values["Weave under %"] = "-5";
    CHECK_FALSE(previewBuild(doc, form).ok);
    form.values["Weave"] = "false";
    form.values["Weave under %"] = "50";
    form.values["Source"] = "Component";
    CHECK_FALSE(previewBuild(doc, form).ok);   // no component selected
    const ComponentId seed = addSeed(doc);
    doc.componentSelection().assign({seed});
    p = previewBuild(doc, form);
    REQUIRE(p.ok);
    CHECK(std::fabs(topOf(p.component) - 2.0F) < 0.01F);
    form.values["Source"] = "Cross section";
    form.values["Section profile"] = "4,0; -4,0";
    CHECK_FALSE(previewBuild(doc, form).ok);   // across not rising
}

TEST_CASE("build: Turn and Spin use the typed profile or the selected open vector",
          "[viewmodel][relief][build]") {
    Document doc;
    BuildForm form = defaultBuildForm(BuildTool::TurnSpin);
    ModellingPreview p = previewBuild(doc, form);   // default Spin profile
    REQUIRE(p.ok);
    CHECK(std::fabs(topOf(p.component) - 5.0F) < 0.01F);
    form.values["Scale to exact height"] = "true";
    form.values["Exact height"] = "3";
    p = previewBuild(doc, form);
    REQUIRE(p.ok);
    CHECK(std::fabs(topOf(p.component) - 3.0F) < 0.01F);
    form.values["Profile"] = "";
    CHECK_FALSE(previewBuild(doc, form).ok);
    const ObjectId v = addPolyline(doc, {{0, 2}, {6, 2}, {12, 0}}, false);
    doc.selection().assign({v});
    p = previewBuild(doc, form);
    REQUIRE(p.ok);
    form.values["Kind"] = "Turn";
    form.values["Scale to exact height"] = "false";
    form.values["Profile"] = "0,0; 20,0";   // zero radius
    CHECK_FALSE(previewBuild(doc, form).ok);
    form.values["Profile"] = "0,0; 20,10; 40,0";
    CHECK(previewBuild(doc, form).ok);
}

TEST_CASE("build: Textured Area tiles the selected seed inside the selected boundary",
          "[viewmodel][relief][build]") {
    Document doc;
    BuildForm form = defaultBuildForm(BuildTool::TexturedArea);
    CHECK_FALSE(previewBuild(doc, form).ok);
    const ComponentId seed = addSeed(doc);
    doc.componentSelection().assign({seed});
    CHECK_FALSE(previewBuild(doc, form).ok);   // no boundary
    const ObjectId rect = addPolyline(doc, {{0, 0}, {20, 0}, {20, 12}, {0, 12}}, true);
    doc.selection().assign({rect});
    ModellingPreview p = previewBuild(doc, form);
    REQUIRE(p.ok);
    CHECK(p.component.field.width() >= 19);
    CHECK(std::fabs(topOf(p.component) - 2.0F) < 0.01F);
    form.values["Reflection"] = "Both";
    form.values["Spacing X %"] = "25";
    CHECK(previewBuild(doc, form).ok);
    form.values["Spacing X %"] = "-100";
    CHECK_FALSE(previewBuild(doc, form).ok);
    form.values["Spacing X %"] = "0";
    form.values["Set anchor"] = "true";
    form.values["Anchor X"] = "nope";
    CHECK_FALSE(previewBuild(doc, form).ok);
}
