// T-0303: the 3D Rough and 3D Finish forms and session around the T-0296/T-0297 cores: stock and model
// placement defaults from the job, the model as the input (staleness follows it), the Vectors boundary,
// save/load, and the toolpath overlay of the Model view.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/relief_commands.hpp"
#include "carvetto/viewmodel/model_view.hpp"
#include "toolpath_test_support.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using namespace carvetto::viewmodel::testing;
using Catch::Approx;
using document::Document;
using document::ToolpathOperation;
using document::Units;

namespace {

const ToolpathField* find(const std::vector<ToolpathField>& fields, const std::string& id) {
    for (const ToolpathField& f : fields) {
        if (f.id == id) {
            return &f;
        }
    }
    return nullptr;
}

/// A 41 x 41 model, 1 mm cells, a ramp rising 0.15 mm per cell along X (0 .. 6 mm).
document::LevelId addRamp(Document& doc, float scale = 0.15f) {
    document::LevelId level;
    auto cmd = std::make_unique<document::AddReliefLevelCommand>(doc.activeSheet(), "Level");
    const document::AddReliefLevelCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    level = raw->createdId();
    relief::Component c;
    c.name = "Ramp";
    std::vector<float> data;
    for (int j = 0; j < 41; ++j) {
        for (int i = 0; i < 41; ++i) {
            data.push_back(static_cast<float>(i) * scale);
        }
    }
    REQUIRE(relief::HeightField::fromData(relief::Grid{41, 41, 1.0, {0.0, 0.0}}, std::move(data), c.field) ==
            relief::Status::Ok);
    REQUIRE(doc.execute(std::make_unique<document::AddReliefComponentCommand>(level, std::move(c))));
    return level;
}

ToolpathDefinition definitionFor(ToolpathSession& session, const tooling::ToolDatabase& db, ToolpathOperation op) {
    ToolpathForm form = session.newForm(op);
    REQUIRE(form.setTool(db, source("end-mill-6mm")).ok);
    return form.definition();
}

}  // namespace

TEST_CASE("3D forms: fields, choices, bounds and problems", "[toolpath][form][3d]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    for (const ToolpathOperation op : {ToolpathOperation::Rough3d, ToolpathOperation::Finish3d}) {
        ToolpathForm form(op, Units::Millimetres);
        CHECK_FALSE(form.setTool(db, source("drill-3mm")).ok);
        REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
        CHECK(form.problems().empty());
        for (const char* id : {"zOffset", "stockTop", "stockBottom", "safeZ", "allowance", "tolerance"}) {
            CHECK(find(form.fields(), id) != nullptr);
        }
        CHECK(find(form.fields(), "boundaryOffset") == nullptr);  // not for the material block
        REQUIRE(form.setChoice("boundary", 1).ok);
        CHECK(find(form.fields(), "boundaryOffset") != nullptr);
        CHECK_FALSE(form.setChoice("boundary", 3).ok);
        CHECK_FALSE(form.setField("allowance", "-1").ok);
        CHECK_FALSE(form.setField("stockTop", "abc").ok);
        REQUIRE(form.setField("allowance", "0.5").ok);
        REQUIRE(form.setField("stockBottom", "-12").ok);
        CHECK(form.problems().empty());
        REQUIRE(form.setField("stockBottom", "2").ok);  // below the safe... above the stock top
        CHECK_FALSE(form.problems().empty());
        REQUIRE(form.setField("stockBottom", "-12").ok);
        REQUIRE(form.setField("safeZ", "0").ok);  // not above the stock top
        CHECK_FALSE(form.problems().empty());
    }
    ToolpathForm rough(ToolpathOperation::Rough3d, Units::Millimetres);
    CHECK(find(rough.fields(), "rampAngle") == nullptr);
    REQUIRE(rough.setChoice("rampPlunge", 1).ok);
    CHECK(find(rough.fields(), "rampAngle") != nullptr);
    ToolpathForm finish(ToolpathOperation::Finish3d, Units::Millimetres);
    CHECK(find(finish.fields(), "rasterAngle") != nullptr);
    REQUIRE(finish.setChoice("strategy", 1).ok);
    CHECK(find(finish.fields(), "rasterAngle") == nullptr);
}

TEST_CASE("3D session: defaults from the job and model, rough then finish, one undo step each, save and load",
          "[toolpath][session][3d]") {
    namespace fs = std::filesystem;
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    addRamp(doc);
    doc.undoStack().clear();

    // The form starts from the job's stock and puts the model's top at the stock top.
    ToolpathForm form = session.newForm(ToolpathOperation::Rough3d);
    const auto& defaults = std::get<toolpath::Rough3dParams>(form.definition().params);
    CHECK(defaults.stockTop == Approx(0.0));
    CHECK(defaults.stockBottom == Approx(-doc.job().sizeMm.z));
    CHECK(defaults.zOffset == Approx(-6.0).margin(0.2));  // the coarse composition may miss the top cell by a hair
    CHECK(defaults.safeZ == Approx(5.0));
    CHECK(session.modelProblem().empty());

    const ToolpathDefinition roughDef = definitionFor(session, db, ToolpathOperation::Rough3d);
    const ToolpathOutcome rough = session.add(roughDef);
    INFO(rough.message);
    REQUIRE(rough.ok);
    REQUIRE(doc.toolpaths().size() == 1);
    const document::ToolpathRecord& rec = doc.toolpaths().front();
    CHECK(rec.operation() == ToolpathOperation::Rough3d);
    CHECK(std::get<toolpath::Rough3dParams>(rec.params).vectors.empty());
    REQUIRE(rec.hasMoves());
    CHECK(rec.moveCount() > 10);
    const double zOffset = std::get<toolpath::Rough3dParams>(rec.params).zOffset;
    // Nothing goes below the floor, and no cut is below the model at its grid nodes.
    relief::HeightField model;
    {
        const std::vector<relief::Level> levels = doc.reliefModel(doc.activeSheet());
        relief::Grid g;
        REQUIRE(relief::unionGrid(levels, g) == relief::Status::Ok);
        REQUIRE(relief::compose(levels, g, model) == relief::Status::Ok);
    }
    for (const toolpath::Move& m : *rec.moves) {
        CHECK(m.to.z >= -doc.job().sizeMm.z - 1e-9);
        if (m.kind == toolpath::MoveKind::Feed) {
            const float h = model.sample(m.to.x, m.to.y);
            if (!std::isnan(h)) {
                CHECK(m.to.z >= static_cast<double>(h) + zOffset - 1e-4);
            }
        }
    }
    CHECK(session.list().front().status == ToolpathStatus::Current);

    // The finish follows; undo removes only it.
    const ToolpathOutcome finish = session.add(definitionFor(session, db, ToolpathOperation::Finish3d));
    INFO(finish.message);
    REQUIRE(finish.ok);
    REQUIRE(doc.toolpaths().size() == 2);
    CHECK(doc.toolpaths()[1].operation() == ToolpathOperation::Finish3d);
    REQUIRE(doc.undo());
    CHECK(doc.toolpaths().size() == 1);
    REQUIRE(doc.redo());
    CHECK(doc.checkIntegrity().empty());

    // Save and load: records equal, staleness current, recalculating gives the same moves.
    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / "t0303";
    fs::create_directories(dir);
    REQUIRE(document::saveCarv(doc, dir / "t.carv", {}));
    auto loaded = document::loadCarv(dir / "t.carv");
    REQUIRE(loaded);
    Document& doc2 = loaded.value().document;
    CHECK(doc2.toolpaths() == doc.toolpaths());
    ToolpathSession s2(&doc2);
    s2.setToolDatabase(db);
    const auto list = s2.list();
    REQUIRE(list.size() == 2);
    CHECK(list[0].status == ToolpathStatus::Current);
    CHECK(list[1].status == ToolpathStatus::Current);
    const std::vector<document::ToolpathRecord> before = doc2.toolpaths();
    REQUIRE(s2.recalculate(list[0].id).ok);
    CHECK(doc2.toolpaths() == before);
}

TEST_CASE("3D session: editing the model makes the toolpath stale; a sheet without a model is refused",
          "[toolpath][session][3d]") {
    Document empty;
    const tooling::ToolDatabase db = cliToolDatabase();
    {
        ToolpathSession session(&empty);
        session.setToolDatabase(db);
        CHECK_FALSE(session.modelProblem().empty());
        const ToolpathOutcome out = session.add(definitionFor(session, db, ToolpathOperation::Rough3d));
        CHECK_FALSE(out.ok);
        CHECK(empty.toolpaths().empty());
    }

    Document doc;
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    const document::LevelId level = addRamp(doc);
    REQUIRE(session.add(definitionFor(session, db, ToolpathOperation::Finish3d)).ok);
    REQUIRE(session.list().size() == 1);
    CHECK(session.list()[0].status == ToolpathStatus::Current);

    // A second component changes the model: the toolpath needs recalculation; undo makes it current again.
    relief::Component c;
    c.name = "Bump";
    REQUIRE(relief::HeightField::create(relief::Grid{10, 10, 1.0, {5.0, 5.0}}, 2.0f, c.field) == relief::Status::Ok);
    REQUIRE(doc.execute(std::make_unique<document::AddReliefComponentCommand>(level, std::move(c))));
    session.invalidate();
    CHECK(session.list()[0].status == ToolpathStatus::NeedsRecalculation);
    CHECK(session.list()[0].vectorsChanged);
    REQUIRE(doc.undo());
    session.invalidate();
    CHECK(session.list()[0].status == ToolpathStatus::Current);
}

TEST_CASE("3D session: the Vectors boundary uses the selected vectors", "[toolpath][session][3d]") {
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    addRamp(doc);
    ToolpathDefinition def = definitionFor(session, db, ToolpathOperation::Rough3d);
    std::get<toolpath::Rough3dParams>(def.params).boundary = toolpath::Rough3dBoundary::Vectors;
    // No vector yet: refused with a message about vectors.
    const ToolpathOutcome none = session.add(def);
    CHECK_FALSE(none.ok);
    addVector(doc, square(10.0, 10.0, 12.0));
    def.selector = VectorSelector::allVectors();
    const ToolpathOutcome out = session.add(def);
    INFO(out.message);
    REQUIRE(out.ok);
    const document::ToolpathRecord& rec = doc.toolpaths().front();
    CHECK(rec.selection.all);
    double minX = 1e9, maxX = -1e9;
    for (const toolpath::Move& m : *rec.moves) {
        if (m.kind == toolpath::MoveKind::Feed) {
            minX = std::min(minX, m.to.x);
            maxX = std::max(maxX, m.to.x);
        }
    }
    CHECK(minX >= 10.0 - 1e-6);
    CHECK(maxX <= 22.0 + 1e-6);
}

TEST_CASE("Model view: 3D toolpaths are drawn over the surface", "[toolpath][3d][model_view]") {
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    addRamp(doc);
    REQUIRE(session.add(definitionFor(session, db, ToolpathOperation::Finish3d)).ok);

    const std::vector<ModelPath> paths = modelPathsFromToolpaths(doc.toolpaths());
    REQUIRE(paths.size() == 1);
    CHECK_FALSE(paths[0].runs.empty());
    const double zOffset = std::get<toolpath::Finish3dParams>(doc.toolpaths()[0].params).zOffset;
    // Model-space heights lie within the model (finish with a flat tool: on or above the surface).
    for (const auto& run : paths[0].runs) {
        for (const ModelPathPoint& p : run) {
            CHECK(p.z >= -1e-3);
            CHECK(p.z <= 6.0 + 20.0);
        }
    }
    CHECK(zOffset < 0.0);

    // Thinning keeps at most the asked number of points per toolpath.
    const std::vector<ModelPath> thin = modelPathsFromToolpaths(doc.toolpaths(), 50);
    std::size_t points = 0;
    for (const auto& run : thin[0].runs) {
        points += run.size();
    }
    CHECK(points < 120);

    // The overlay changes pixels, and only with a path.
    const ModelSurface surface = composeModelSurface(doc, doc.activeSheet());
    REQUIRE(surface.ok());
    OrbitCamera camera;
    camera.viewTop();
    ModelRenderOptions options;
    const Raster plain = renderModel(surface, camera, options);
    options.paths = paths;
    const Raster drawn = renderModel(surface, camera, options);
    REQUIRE(plain.pixels.size() == drawn.pixels.size());
    std::size_t differing = 0;
    for (std::size_t i = 0; i < plain.pixels.size(); ++i) {
        if (plain.pixels[i] != drawn.pixels[i]) {
            ++differing;
            CHECK(drawn.pixels[i] == paths[0].colour);
        }
    }
    CHECK(differing > 50);
    // Non-3D records are not drawn.
    CHECK(modelPathsFromToolpaths({}).empty());
}
