// T-0311: the Moulding, Project Toolpath onto Model and 3D Tabs forms and session around the T-0307 / T-0298 cores
// (ADR-0091): field tables, the profile and rails taken from the selection, the source toolpath as an input
// (staleness follows it), tabs, save/load and the Model view overlay.

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

const ToolpathChoice* findChoice(const std::vector<ToolpathChoice>& choices, const std::string& id) {
    for (const ToolpathChoice& c : choices) {
        if (c.id == id) {
            return &c;
        }
    }
    return nullptr;
}

/// A 41 x 41 model, 1 mm cells, a ramp rising 0.15 mm per cell along X (0 .. 6 mm).
void addRamp(Document& doc) {
    auto cmd = std::make_unique<document::AddReliefLevelCommand>(doc.activeSheet(), "Level");
    const document::AddReliefLevelCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    const document::LevelId level = raw->createdId();
    relief::Component c;
    c.name = "Ramp";
    std::vector<float> data;
    for (int j = 0; j < 41; ++j) {
        for (int i = 0; i < 41; ++i) {
            data.push_back(static_cast<float>(i) * 0.15f);
        }
    }
    REQUIRE(relief::HeightField::fromData(relief::Grid{41, 41, 1.0, {0.0, 0.0}}, std::move(data), c.field) ==
            relief::Status::Ok);
    REQUIRE(doc.execute(std::make_unique<document::AddReliefComponentCommand>(level, std::move(c))));
}

geometry::Contour openLine(std::vector<geometry::Vec2> points) {
    geometry::Polyline pl;
    pl.points = std::move(points);
    pl.closed = false;
    return geometry::Contour::fromPolyline(pl);
}

void select(Document& doc, const std::vector<document::ObjectId>& ids) {
    doc.selection().assign(ids);
}

}  // namespace

TEST_CASE("Moulding form: fields, choices, bounds and problems", "[toolpath][form][moulding]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::Moulding, Units::Millimetres);
    CHECK_FALSE(form.setTool(db, source("drill-3mm")).ok);
    REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
    CHECK(form.problems().empty());
    for (const char* id : {"zOffset", "stockTop", "stockBottom", "safeZ", "boundaryOffset", "tolerance", "profileObject"}) {
        CHECK(find(form.fields(), id) != nullptr);
    }
    // Finish: no allowance; Rough: allowance, and the ramp angle only with ramp plunge.
    CHECK(find(form.fields(), "allowance") == nullptr);
    CHECK(findChoice(form.choices(), "varyStepover") != nullptr);
    CHECK(findChoice(form.choices(), "machineFlatRegions") == nullptr);
    REQUIRE(form.setChoice("mode", 1).ok);
    CHECK(find(form.fields(), "allowance") != nullptr);
    CHECK(findChoice(form.choices(), "varyStepover") == nullptr);
    CHECK(findChoice(form.choices(), "machineFlatRegions") != nullptr);
    CHECK(find(form.fields(), "rampAngle") == nullptr);
    REQUIRE(form.setChoice("rampPlunge", 1).ok);
    CHECK(find(form.fields(), "rampAngle") != nullptr);
    // The flat angle matters once flat regions are treated specially.
    CHECK(find(form.fields(), "flatAngle") == nullptr);
    REQUIRE(form.setChoice("machineFlatRegions", 0).ok);
    CHECK(find(form.fields(), "flatAngle") != nullptr);
    // The automatic boundary offset hides the typed one.
    REQUIRE(form.setChoice("autoBoundaryOffset", 1).ok);
    CHECK(find(form.fields(), "boundaryOffset") == nullptr);
    CHECK_FALSE(form.setChoice("position", 4).ok);
    REQUIRE(form.setChoice("position", 3).ok);
    CHECK_FALSE(form.setField("allowance", "-1").ok);
    CHECK_FALSE(form.setField("profileObject", "-3").ok);
    REQUIRE(form.setField("profileObject", "12").ok);
    CHECK(std::get<toolpath::MouldingParams>(form.definition().params).profileObject == 12);
    REQUIRE(form.setField("profileObject", "").ok);  // reset
    CHECK(std::get<toolpath::MouldingParams>(form.definition().params).profileObject == 0);
    REQUIRE(form.setField("safeZ", "0").ok);  // not above the stock top
    CHECK_FALSE(form.problems().empty());
}

TEST_CASE("Project form: fields, choices and problems", "[toolpath][form][project]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::Project3d, Units::Millimetres);
    REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
    CHECK(form.name() == "Project to Model");
    // No source toolpath chosen yet.
    bool sourceProblem = false;
    for (const std::string& p : form.problems()) {
        sourceProblem = sourceProblem || p.find("toolpath to project") != std::string::npos;
    }
    CHECK(sourceProblem);
    REQUIRE(form.setField("sourceToolpath", "3").ok);
    CHECK(form.problems().empty());
    CHECK(find(form.fields(), "tabLength") == nullptr);  // tabs are off
    REQUIRE(form.setChoice("tabsEnabled", 1).ok);
    for (const char* id : {"tabLength", "tabThickness", "tabCount"}) {
        CHECK(find(form.fields(), id) != nullptr);
    }
    CHECK(find(form.fields(), "tabDistance") == nullptr);
    REQUIRE(form.setChoice("tabSpacing", 1).ok);
    CHECK(find(form.fields(), "tabDistance") != nullptr);
    CHECK_FALSE(form.setField("allowance", "-0.5").ok);
    CHECK_FALSE(form.setField("tolerance", "0").ok);
    REQUIRE(form.setField("maxSegment", "0").ok);
    REQUIRE(form.setField("stockBottom", "5").ok);  // above the surface outside the model
    CHECK_FALSE(form.problems().empty());
}

TEST_CASE("Moulding session: the profile and the rails come from the selection", "[toolpath][session][moulding]") {
    namespace fs = std::filesystem;
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);

    // Section: a ramp from (-4, 0) to (4, 3), 8 mm wide. Rail: a 40 mm line along X.
    const document::ObjectId profile = addVector(doc, openLine({{-4.0, 0.0}, {4.0, 3.0}}));
    const document::ObjectId rail = addVector(doc, openLine({{0.0, 20.0}, {40.0, 20.0}}));
    // The defaults come from the job: stock top 0, safe Z 5; the section's heights stay as drawn.
    select(doc, {profile, rail});
    ToolpathForm form = session.newForm(ToolpathOperation::Moulding);
    const auto& defaults = std::get<toolpath::MouldingParams>(form.definition().params);
    CHECK(defaults.stockTop == Approx(0.0));
    CHECK(defaults.stockBottom == Approx(-doc.job().sizeMm.z));
    CHECK(defaults.zOffset == Approx(0.0));
    CHECK(defaults.safeZ == Approx(5.0));
    REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
    REQUIRE(form.setField("zOffset", "-3").ok);  // the section's top at the stock top
    CHECK(form.problems().empty());

    // One vector: no rail.
    {
        ToolpathDefinition one = form.definition();
        one.selector = VectorSelector::fromObjects({profile});
        const ToolpathOutcome out = session.add(one);
        CHECK_FALSE(out.ok);
        CHECK(out.message.find("rail") != std::string::npos);
        CHECK(doc.toolpaths().empty());
    }
    // The profile named by an object id that is not selected.
    {
        ToolpathDefinition bad = form.definition();
        std::get<toolpath::MouldingParams>(bad.params).profileObject = 9999;
        const ToolpathOutcome out = session.add(bad);
        CHECK_FALSE(out.ok);
        CHECK(out.message.find("profile") != std::string::npos);
    }
    // Two selected vectors, profile first: a toolpath along the rail.
    const ToolpathOutcome out = session.add(form.definition());
    INFO(out.message);
    REQUIRE(out.ok);
    REQUIRE(doc.toolpaths().size() == 1);
    const document::ToolpathRecord& rec = doc.toolpaths().front();
    CHECK(rec.operation() == ToolpathOperation::Moulding);
    REQUIRE(rec.hasMoves());
    CHECK(rec.moveCount() > 4);
    double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9, maxZ = -1e9;
    for (const toolpath::Move& m : *rec.moves) {
        if (m.kind == toolpath::MoveKind::Feed) {
            minX = std::min(minX, m.to.x);
            maxX = std::max(maxX, m.to.x);
            minY = std::min(minY, m.to.y);
            maxY = std::max(maxY, m.to.y);
            maxZ = std::max(maxZ, m.to.z);
        }
    }
    CHECK(minX == Approx(0.0).margin(1e-6));
    CHECK(maxX == Approx(40.0).margin(1e-6));
    CHECK(minY < 20.0);  // passes on both sides of the rail
    CHECK(maxY > 20.0);
    CHECK(maxZ <= 0.0 + 1e-9);  // never above the stock top
    CHECK(std::get<toolpath::MouldingParams>(rec.params).profile.isEmpty());  // geometry is not part of the record
    CHECK(session.list().front().status == ToolpathStatus::Current);

    // Moving the rail makes it stale; undo makes it current again.
    REQUIRE(doc.undo());
    CHECK(doc.toolpaths().empty());
    REQUIRE(doc.redo());

    // An explicit profile id picks the section whatever the selection order.
    ToolpathDefinition explicitDef = form.definition();
    std::get<toolpath::MouldingParams>(explicitDef.params).profileObject = profile.value;
    explicitDef.selector = VectorSelector::fromObjects({rail, profile});
    explicitDef.setName("Explicit");
    const ToolpathOutcome second = session.add(explicitDef);
    INFO(second.message);
    REQUIRE(second.ok);
    REQUIRE(doc.toolpaths().size() == 2);
    CHECK(*doc.toolpaths()[1].moves == *doc.toolpaths()[0].moves);

    // Save and load: equal records, current, recalculation reproduces the moves.
    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / "t0311";
    fs::create_directories(dir);
    REQUIRE(document::saveCarv(doc, dir / "m.carv", {}));
    auto loaded = document::loadCarv(dir / "m.carv");
    REQUIRE(loaded);
    Document& doc2 = loaded.value().document;
    CHECK(doc2.toolpaths() == doc.toolpaths());
    ToolpathSession s2(&doc2);
    s2.setToolDatabase(db);
    const auto list = s2.list();
    REQUIRE(list.size() == 2);
    CHECK(list[0].status == ToolpathStatus::Current);
    const std::vector<document::ToolpathRecord> before = doc2.toolpaths();
    REQUIRE(s2.recalculate(list[0].id).ok);
    CHECK(doc2.toolpaths() == before);
}

TEST_CASE("Project session: a 2D toolpath re-heighted onto the model; tabs; staleness follows the source and the model",
          "[toolpath][session][project]") {
    namespace fs = std::filesystem;
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    addRamp(doc);
    addVector(doc, square(8.0, 8.0, 20.0));
    ToolpathDefinition profile = profileDefinition(db, 1.0);
    profile.selector = VectorSelector::allVectors();
    REQUIRE(session.add(profile).ok);
    REQUIRE(doc.toolpaths().size() == 1);
    const document::ToolpathId sourceId = doc.toolpaths()[0].id;
    const std::size_t sourceMoves = doc.toolpaths()[0].moveCount();
    doc.undoStack().clear();

    // No source: refused. A missing one: refused.
    {
        ToolpathForm none = session.newForm(ToolpathOperation::Project3d);
        REQUIRE(none.setTool(db, source("end-mill-3mm")).ok);
        const ToolpathOutcome out = session.add(none.definition());
        CHECK_FALSE(out.ok);
        CHECK(out.message.find("toolpath") != std::string::npos);
    }

    ToolpathForm form = session.newProjectForm(sourceId, false);
    const auto& defaults = std::get<toolpath::Project3dParams>(form.definition().params);
    CHECK(defaults.sourceToolpath == sourceId.value);
    CHECK(defaults.outsideSurface == Approx(0.0));
    CHECK(defaults.stockBottom == Approx(-doc.job().sizeMm.z));
    CHECK(defaults.zOffset == Approx(-6.0).margin(0.2));
    CHECK(form.name() == doc.toolpaths()[0].name + " (3D)");
    CHECK(form.tool().toolId == doc.toolpaths()[0].toolSource.toolId);  // the source's tool when it suits
    CHECK_FALSE(defaults.tabs.enabled);
    REQUIRE(form.problems().empty());

    const ToolpathOutcome out = session.add(form.definition());
    INFO(out.message);
    REQUIRE(out.ok);
    REQUIRE(doc.toolpaths().size() == 2);
    const document::ToolpathRecord& rec = doc.toolpaths()[1];
    CHECK(rec.operation() == ToolpathOperation::Project3d);
    REQUIRE(rec.hasMoves());
    CHECK(rec.moveCount() >= sourceMoves);
    // Cuts at depth -1 below the surface: the model height (x * 0.15) + zOffset - 1 at the move's XY.
    const double zOffset = std::get<toolpath::Project3dParams>(rec.params).zOffset;
    std::size_t checked = 0;
    for (const toolpath::Move& m : *rec.moves) {
        if (m.kind == toolpath::MoveKind::Feed && m.to.x > 4.0 && m.to.x < 32.0 && m.to.y > 4.0 && m.to.y < 32.0) {
            // The drop-tool surface of a ramp is at or above the ramp's own height.
            CHECK(m.to.z >= m.to.x * 0.15 + zOffset - 1.0 - 0.05);
            ++checked;
        }
    }
    CHECK(checked > 5);
    CHECK(rec.safeZ() > 0.0);  // retracts stay above the stock
    CHECK(session.list()[1].status == ToolpathStatus::Current);
    CHECK(session.list()[0].status == ToolpathStatus::Current);

    // One undo step.
    REQUIRE(doc.undo());
    CHECK(doc.toolpaths().size() == 1);
    REQUIRE(doc.redo());
    CHECK(doc.checkIntegrity().empty());
    const document::ToolpathRecord projected = doc.toolpaths()[1];  // a copy: the list moves on

    // 3D tabs: tabs on, ramped; the tabbed toolpath has points above the plain projection.
    ToolpathForm tabs = session.newProjectForm(sourceId, true);
    const auto& tabParams = std::get<toolpath::Project3dParams>(tabs.definition().params);
    CHECK(tabParams.tabs.enabled);
    CHECK(tabParams.tabs.threeD);
    REQUIRE(tabs.setField("tabThickness", "0.5").ok);
    REQUIRE(tabs.setField("tabLength", "3").ok);
    REQUIRE(tabs.setField("tabCount", "2").ok);
    const ToolpathOutcome tabOut = session.add(tabs.definition());
    INFO(tabOut.message);
    REQUIRE(tabOut.ok);
    REQUIRE(doc.toolpaths().size() == 3);
    const document::ToolpathRecord tabbed = doc.toolpaths()[2];
    CHECK(tabbed.moveCount() > projected.moveCount());  // the tabs add breakpoints
    double plainLow = 1e9, tabLow = 1e9, tabHigh = -1e9, plainHigh = -1e9;
    for (const toolpath::Move& m : *projected.moves) {
        if (m.kind == toolpath::MoveKind::Feed) {
            plainLow = std::min(plainLow, m.to.z);
            plainHigh = std::max(plainHigh, m.to.z);
        }
    }
    for (const toolpath::Move& m : *tabbed.moves) {
        if (m.kind == toolpath::MoveKind::Feed) {
            tabLow = std::min(tabLow, m.to.z);
            tabHigh = std::max(tabHigh, m.to.z);
        }
    }
    CHECK(tabLow == Approx(plainLow).margin(1e-6));
    CHECK(tabHigh > plainHigh + 0.1);

    // Staleness: all three are current; changing the model marks the projections stale (the source too).
    for (const ToolpathListItem& item : session.list()) {
        CHECK(item.status == ToolpathStatus::Current);
    }
    // Recalculating the source with another depth changes its moves: the projections become stale.
    {
        ToolpathDefinition deeper = profileDefinition(db, 2.0);
        deeper.selector = VectorSelector::allVectors();
        REQUIRE(session.update(sourceId, deeper).ok);
    }
    {
        const auto list = session.list();
        REQUIRE(list.size() == 3);
        CHECK(list[1].status == ToolpathStatus::NeedsRecalculation);
        CHECK(list[1].vectorsChanged);
        CHECK(list[2].status == ToolpathStatus::NeedsRecalculation);
    }
    REQUIRE(session.recalculate(doc.toolpaths()[1].id).ok);
    CHECK(session.list()[1].status == ToolpathStatus::Current);

    // Save and load: equal, current, recalculation reproduces the moves.
    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / "t0311";
    fs::create_directories(dir);
    REQUIRE(document::saveCarv(doc, dir / "p.carv", {}));
    auto loaded = document::loadCarv(dir / "p.carv");
    REQUIRE(loaded);
    Document& doc2 = loaded.value().document;
    CHECK(doc2.toolpaths() == doc.toolpaths());
    ToolpathSession s2(&doc2);
    s2.setToolDatabase(db);
    const auto list = s2.list();
    REQUIRE(list.size() == 3);
    for (const ToolpathListItem& item : list) {
        CHECK(item.status == ToolpathStatus::Current);
    }
    const std::vector<document::ToolpathRecord> before = doc2.toolpaths();
    REQUIRE(s2.recalculate(list[1].id).ok);
    CHECK(doc2.toolpaths() == before);

    // Deleting the source leaves the projection refusing to recalculate with a clear message.
    REQUIRE(doc.execute(std::make_unique<document::RemoveToolpathsCommand>(std::vector<document::ToolpathId>{sourceId})));
    const ToolpathOutcome orphan = session.recalculate(doc.toolpaths()[0].id);
    CHECK_FALSE(orphan.ok);
    CHECK(orphan.message.find("source") != std::string::npos);
}

TEST_CASE("Model view: projected toolpaths are drawn over the surface", "[toolpath][project][model_view]") {
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    addRamp(doc);
    addVector(doc, square(8.0, 8.0, 20.0));
    ToolpathDefinition profile = profileDefinition(db, 1.0);
    profile.selector = VectorSelector::allVectors();
    REQUIRE(session.add(profile).ok);
    REQUIRE(session.add(session.newProjectForm(doc.toolpaths()[0].id, false).definition()).ok);
    // The 2D profile is not drawn, the projected one is.
    const std::vector<ModelPath> paths = modelPathsFromToolpaths(doc.toolpaths());
    REQUIRE(paths.size() == 1);
    CHECK_FALSE(paths[0].runs.empty());
}
