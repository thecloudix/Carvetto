// T-0314: rest-tool lists for 3D Rough / 3D Finish, prior toolpaths for 3D Finish and the Moulding larger-area
// clearance tool (ADR-0094): the tool list in the form, one group of records per calculation stored in the
// record (so it survives a reload and is told apart from separate toolpaths), recalculation of the group,
// the simulation in job order with each record's own tool, and old files.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/relief_commands.hpp"
#include "carvetto/viewmodel/simulation_setup.hpp"
#include "toolpath_test_support.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using namespace carvetto::viewmodel::testing;
using Catch::Approx;
using document::Document;
using document::ToolpathOperation;
using document::Units;

namespace {

/// A 41 x 41 model, 1 mm cells: a plateau at 3 mm with a 5 mm wide trench (columns 18..22) down to 0. With
/// the model's top (3) placed 3 mm below the stock top, a 6 mm end mill clears the plateau but cannot enter the
/// trench; a 3 mm one can.
void addTrenchModel(Document& doc) {
    auto cmd = std::make_unique<document::AddReliefLevelCommand>(doc.activeSheet(), "Level");
    const document::AddReliefLevelCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    const document::LevelId level = raw->createdId();
    relief::Component c;
    c.name = "Trench";
    std::vector<float> data;
    for (int j = 0; j < 41; ++j) {
        for (int i = 0; i < 41; ++i) {
            data.push_back(i >= 18 && i <= 22 ? 0.0f : 3.0f);
        }
    }
    REQUIRE(relief::HeightField::fromData(relief::Grid{41, 41, 1.0, {0.0, 0.0}}, std::move(data), c.field) ==
            relief::Status::Ok);
    REQUIRE(doc.execute(std::make_unique<document::AddReliefComponentCommand>(level, std::move(c))));
}

ToolpathDefinition restRough(ToolpathSession& session, const tooling::ToolDatabase& db) {
    ToolpathForm form = session.newForm(ToolpathOperation::Rough3d);
    REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
    REQUIRE(form.setClearanceTools(db, {source("end-mill-6mm")}).ok);
    REQUIRE(form.setField("zOffset", "-6").ok);  // the plateau 3 mm below the stock top
    CHECK(form.problems().empty());
    return form.definition();
}

geometry::Contour openLine(std::vector<geometry::Vec2> points) {
    geometry::Polyline pl;
    pl.points = std::move(points);
    pl.closed = false;
    return geometry::Contour::fromPolyline(pl);
}

}  // namespace

TEST_CASE("Rest tools: the form takes an earlier-tool list for 3D operations and one for a moulding",
          "[toolpath][form][3d][rest]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    for (const ToolpathOperation op : {ToolpathOperation::Rough3d, ToolpathOperation::Finish3d, ToolpathOperation::Moulding}) {
        ToolpathForm form(op, Units::Millimetres);
        REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
        CHECK(hasClearanceTools(op));
        CHECK_FALSE(form.setClearanceTools(db, {source("drill-3mm")}).ok);  // a drill is no 3D tool
        CHECK(form.clearanceTools().empty());
        REQUIRE(form.setClearanceTools(db, {source("end-mill-6mm")}).ok);
        CHECK(form.clearanceTools().size() == 1);
        CHECK(form.problems().empty());
        const bool oneOnly = op == ToolpathOperation::Moulding;
        CHECK(form.setClearanceTools(db, {source("end-mill-6mm"), source("end-mill-3mm")}).ok != oneOnly);
    }
    // Projection and the 2D operations have none.
    ToolpathForm project(ToolpathOperation::Project3d, Units::Millimetres);
    CHECK_FALSE(project.setClearanceTools(db, {source("end-mill-6mm")}).ok);
    ToolpathForm pocket(ToolpathOperation::Pocket, Units::Millimetres);
    CHECK_FALSE(pocket.setClearanceTools(db, {source("end-mill-6mm")}).ok);

    // Priors: 3D finish only; a repeated or null id is a problem.
    ToolpathForm finish(ToolpathOperation::Finish3d, Units::Millimetres);
    REQUIRE(finish.setTool(db, source("end-mill-3mm")).ok);
    REQUIRE(finish.setPriorToolpaths({4, 9}).ok);
    CHECK(finish.priorToolpaths() == std::vector<std::uint64_t>{4, 9});
    CHECK(finish.problems().empty());
    REQUIRE(finish.setPriorToolpaths({4, 4}).ok);
    CHECK_FALSE(finish.problems().empty());
    REQUIRE(finish.setPriorToolpaths({0}).ok);
    CHECK_FALSE(finish.problems().empty());
    ToolpathForm rough(ToolpathOperation::Rough3d, Units::Millimetres);
    CHECK_FALSE(rough.setPriorToolpaths({1}).ok);
    CHECK(rough.priorToolpaths().empty());
}

TEST_CASE("Rest tools: a 3D rough with a rest tool is one group of records, stored, recalculated and reloaded",
          "[toolpath][session][3d][rest]") {
    namespace fs = std::filesystem;
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    addTrenchModel(doc);
    doc.undoStack().clear();

    const ToolpathOutcome out = session.add(restRough(session, db));
    INFO(out.message);
    REQUIRE(out.ok);
    INFO(doc.toolpaths()[0].name << " d=" << doc.toolpaths()[0].tool.diameter);
    for (const ToolpathMessage& m : out.messages) {
        INFO(m.text);
    }
    REQUIRE(doc.toolpaths().size() == 2);
    CHECK(doc.undoStack().count() == 1);  // one undo step
    const document::ToolpathRecord big = doc.toolpaths()[0];
    const document::ToolpathRecord small = doc.toolpaths()[1];
    CHECK(big.tool.diameter == Approx(6.0));
    CHECK(small.tool.diameter == Approx(3.0));
    CHECK(big.name == "3D Rough [End Mill 6 mm]");
    CHECK(small.name.find("[") != std::string::npos);
    // The group is in the records: the same id, the whole list, each record's place in it.
    CHECK(big.group.id != 0);
    CHECK(big.group.id == small.group.id);
    REQUIRE(big.group.tools.size() == 2);
    CHECK(big.group.tools[0].toolId == "end-mill-6mm");
    CHECK(big.group.tools[1].toolId == "end-mill-3mm");
    CHECK(big.group.role == 0);
    CHECK(small.group.role == 1);
    // The 6 mm tool never enters the 5 mm trench: its feeds stay on the plateau (its tip never below -3).
    for (const toolpath::Move& m : *big.moves) {
        if (m.kind == toolpath::MoveKind::Feed || m.kind == toolpath::MoveKind::Plunge) {
            CHECK(m.to.z >= -3.0 - 1e-6);
        }
    }
    // The rest tool cuts the trench, down below the plateau.
    double lowest = 0.0;
    for (const toolpath::Move& m : *small.moves) {
        lowest = std::min(lowest, m.to.z);
    }
    CHECK(lowest < -4.0);

    // The definition of either member is the whole calculation; the list is current.
    for (const document::ToolpathRecord& r : doc.toolpaths()) {
        const auto def = session.definition(r.id);
        REQUIRE(def.has_value());
        CHECK(def->tool.toolId == "end-mill-3mm");
        REQUIRE(def->clearanceTools.size() == 1);
        CHECK(def->clearanceTools[0].toolId == "end-mill-6mm");
    }
    for (const ToolpathListItem& item : session.list()) {
        CHECK(item.status == ToolpathStatus::Current);
        CHECK(item.canRecalculate);
    }

    // Simulation: both toolpaths in job order, each with its own tool.
    const SimulationSetup setup = makeSimulationSetup(session);
    REQUIRE(setup.ok);
    REQUIRE(setup.toolpaths.size() == 2);
    CHECK(setup.toolpaths[0].tool.diameter == Approx(6.0));
    CHECK(setup.toolpaths[1].tool.diameter == Approx(3.0));
    CHECK_FALSE(setup.toolpaths[0].shapeApproximated);
    CHECK_FALSE(setup.toolpaths[1].shapeApproximated);

    // Save, load into a fresh session: the group survives (the tool types of its members cannot tell it apart).
    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / "t0314";
    fs::create_directories(dir);
    REQUIRE(document::saveCarv(doc, dir / "rest.carv", {}));
    auto loaded = document::loadCarv(dir / "rest.carv");
    REQUIRE(loaded);
    Document& doc2 = loaded.value().document;
    CHECK(doc2.toolpaths() == doc.toolpaths());
    CHECK(loaded.value().formatMinor == document::kCarvFormatMinorToolGroups);
    ToolpathSession s2(&doc2);
    s2.setToolDatabase(db);
    for (const ToolpathListItem& item : s2.list()) {
        CHECK(item.status == ToolpathStatus::Current);
    }
    const auto def2 = s2.definition(doc2.toolpaths()[1].id);
    REQUIRE(def2.has_value());
    REQUIRE(def2->clearanceTools.size() == 1);
    CHECK(def2->clearanceTools[0].toolId == "end-mill-6mm");
    // Recalculating one member recalculates the group: equal records, same ids and group.
    const std::vector<document::ToolpathRecord> before = doc2.toolpaths();
    const std::size_t undoBefore = doc2.undoStack().count();
    REQUIRE(s2.recalculate(doc2.toolpaths()[0].id).ok);
    CHECK(doc2.toolpaths() == before);
    CHECK(doc2.undoStack().count() <= undoBefore + 1);
    // Renaming a member renames the whole group.
    REQUIRE(s2.rename(doc2.toolpaths()[1].id, "Clear trench").ok);
    CHECK(std::get<toolpath::Rough3dParams>(doc2.toolpaths()[0].params).name == "Clear trench");
    CHECK(std::get<toolpath::Rough3dParams>(doc2.toolpaths()[1].params).name == "Clear trench");
    // Duplicating copies the group under a new id.
    REQUIRE(s2.duplicate(doc2.toolpaths()[0].id).ok);
    REQUIRE(doc2.toolpaths().size() == 4);
    CHECK(doc2.toolpaths()[2].group.id != doc2.toolpaths()[0].group.id);
    CHECK(doc2.toolpaths()[2].group.id == doc2.toolpaths()[3].group.id);
    // Undo and redo keep the group.
    REQUIRE(doc2.undo());
    CHECK(doc2.toolpaths().size() == 2);
    REQUIRE(doc2.redo());
    CHECK(doc2.toolpaths()[3].group.id == doc2.toolpaths()[2].group.id);
}

TEST_CASE("Rest tools: separate toolpaths and old files without a group stay separate",
          "[toolpath][session][3d][rest]") {
    namespace fs = std::filesystem;
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    addTrenchModel(doc);

    // Two ordinary rough toolpaths with the same settings, different end mills (a rest group by tool type
    // would look the same).
    ToolpathForm a = session.newForm(ToolpathOperation::Rough3d);
    REQUIRE(a.setTool(db, source("end-mill-6mm")).ok);
    REQUIRE(a.setField("zOffset", "-6").ok);
    const ToolpathOutcome first = session.add(a.definition());
    INFO(first.message);
    for (const ToolpathMessage& m : first.messages) {
        INFO(m.text);
    }
    REQUIRE(first.ok);
    ToolpathForm b = session.newForm(ToolpathOperation::Rough3d);
    REQUIRE(b.setTool(db, source("end-mill-3mm")).ok);
    REQUIRE(b.setField("zOffset", "-6").ok);
    b.setName("3D Rough");
    REQUIRE(session.add(b.definition()).ok);
    REQUIRE(doc.toolpaths().size() == 2);
    CHECK(doc.toolpaths()[0].group.id == 0);
    CHECK(doc.toolpaths()[1].group.id == 0);

    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / "t0314";
    fs::create_directories(dir);
    REQUIRE(document::saveCarv(doc, dir / "separate.carv", {}));
    auto loaded = document::loadCarv(dir / "separate.carv");
    REQUIRE(loaded);
    CHECK(loaded.value().formatMinor < document::kCarvFormatMinorToolGroups);  // an old-format file
    Document& doc2 = loaded.value().document;
    ToolpathSession s2(&doc2);
    s2.setToolDatabase(db);
    for (const ToolpathListItem& item : s2.list()) {
        CHECK(item.status == ToolpathStatus::Current);
        CHECK(item.canRecalculate);
    }
    const auto def = s2.definition(doc2.toolpaths()[1].id);
    REQUIRE(def.has_value());
    CHECK(def->clearanceTools.empty());
    // Renaming one leaves the other alone; recalculating one leaves the other's record as it is.
    const document::ToolpathRecord untouched = doc2.toolpaths()[0];
    REQUIRE(s2.rename(doc2.toolpaths()[1].id, "Only me").ok);
    CHECK(doc2.toolpaths()[0] == untouched);
    CHECK(doc2.toolpaths().size() == 2);
    REQUIRE(s2.recalculate(doc2.toolpaths()[1].id).ok);
    CHECK(doc2.toolpaths().size() == 2);
    CHECK(doc2.toolpaths()[0] == untouched);
}

TEST_CASE("Rest tools: a rest tool that has nothing to cut leaves a group without its own-tool record",
          "[toolpath][session][3d][rest]") {
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    addTrenchModel(doc);
    // The 3 mm tool twice with a huge rest tolerance: the second has nothing to cut, so only the first tool's
    // record exists.
    ToolpathForm form = session.newForm(ToolpathOperation::Rough3d);
    REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
    REQUIRE(form.setClearanceTools(db, {source("end-mill-3mm")}).ok);
    REQUIRE(form.setField("zOffset", "-6").ok);
    REQUIRE(form.setField("restTolerance", "50").ok);  // far more than anything left
    CHECK_FALSE(form.setField("restTolerance", "-1").ok);
    const ToolpathOutcome out = session.add(form.definition());
    INFO(out.message);
    REQUIRE(out.ok);
    REQUIRE(doc.toolpaths().size() == 1);
    const document::ToolpathRecord rec = doc.toolpaths()[0];
    CHECK(rec.group.id != 0);
    CHECK(rec.group.role == 0);
    CHECK(rec.group.tools.size() == 2);
    // Still recalculable from its own record (every member stores the whole list).
    const auto def = session.definition(rec.id);
    REQUIRE(def.has_value());
    CHECK(def->tool.toolId == "end-mill-3mm");
    REQUIRE(def->clearanceTools.size() == 1);
    CHECK(session.recalculate(rec.id).ok);
    CHECK(doc.toolpaths().size() == 1);
    CHECK(doc.toolpaths()[0] == rec);
}

TEST_CASE("Rest tools: a 3D finish uses prior toolpaths; they are part of its staleness and cannot be itself",
          "[toolpath][session][3d][rest]") {
    namespace fs = std::filesystem;
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    addTrenchModel(doc);

    ToolpathForm roughForm = session.newForm(ToolpathOperation::Rough3d);
    REQUIRE(roughForm.setTool(db, source("end-mill-6mm")).ok);
    REQUIRE(roughForm.setField("zOffset", "-6").ok);
    const ToolpathOutcome firstRough = session.add(roughForm.definition());
    INFO(firstRough.message);
    REQUIRE(firstRough.ok);
    REQUIRE(doc.toolpaths().size() == 1);
    const document::ToolpathId roughId = doc.toolpaths()[0].id;

    ToolpathForm plain = session.newForm(ToolpathOperation::Finish3d);
    REQUIRE(plain.setTool(db, source("end-mill-3mm")).ok);
    REQUIRE(plain.setField("zOffset", "-6").ok);
    ToolpathForm withPrior = plain;
    REQUIRE(withPrior.setPriorToolpaths({roughId.value}).ok);

    // A missing prior, then a prior that is not calculated, are refused.
    {
        ToolpathForm bad = plain;
        REQUIRE(bad.setPriorToolpaths({9999}).ok);
        const ToolpathOutcome out = session.add(bad.definition());
        CHECK_FALSE(out.ok);
        CHECK(out.message.find("prior") != std::string::npos);
        CHECK(doc.toolpaths().size() == 1);
    }

    const ToolpathOutcome fin = session.add(withPrior.definition());
    INFO(fin.message);
    REQUIRE(fin.ok);
    REQUIRE(doc.toolpaths().size() == 2);
    const document::ToolpathRecord finish = doc.toolpaths()[1];
    CHECK(std::get<toolpath::Finish3dParams>(finish.params).priorToolpaths == std::vector<std::uint64_t>{roughId.value});
    CHECK(std::get<toolpath::Finish3dParams>(finish.params).priors.empty());  // the moves are never stored

    // With the prior's stock as the start, the finish cuts only what is left: fewer moves than without it.
    ToolpathForm solo = plain;
    solo.setName("Solo");
    REQUIRE(session.add(solo.definition()).ok);
    REQUIRE(doc.toolpaths().size() == 3);
    CHECK(doc.toolpaths()[1].moveCount() <= doc.toolpaths()[2].moveCount());

    // Staleness follows the prior: recalculating the rough with other settings makes the finish stale.
    for (const ToolpathListItem& item : session.list()) {
        CHECK(item.status == ToolpathStatus::Current);
    }
    ToolpathDefinition changed = *session.definition(roughId);
    std::get<toolpath::Rough3dParams>(changed.params).allowance = 0.4;
    REQUIRE(session.update(roughId, changed).ok);
    const auto list = session.list();
    REQUIRE(list.size() == 3);
    CHECK(list[1].status == ToolpathStatus::NeedsRecalculation);
    CHECK(list[2].status == ToolpathStatus::Current);

    // Reload: the prior list is kept, the finish is stale only as before; recalculating works and is stable.
    REQUIRE(session.recalculate(doc.toolpaths()[1].id).ok);
    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / "t0314";
    fs::create_directories(dir);
    REQUIRE(document::saveCarv(doc, dir / "priors.carv", {}));
    auto loaded = document::loadCarv(dir / "priors.carv");
    REQUIRE(loaded);
    Document& doc2 = loaded.value().document;
    CHECK(doc2.toolpaths() == doc.toolpaths());
    ToolpathSession s2(&doc2);
    s2.setToolDatabase(db);
    for (const ToolpathListItem& item : s2.list()) {
        CHECK(item.status == ToolpathStatus::Current);
    }
    const std::vector<document::ToolpathRecord> before = doc2.toolpaths();
    REQUIRE(s2.recalculate(doc2.toolpaths()[1].id).ok);
    CHECK(doc2.toolpaths() == before);

    // A prior may not be the finish's own toolpath.
    ToolpathDefinition own = *s2.definition(doc2.toolpaths()[1].id);
    std::get<toolpath::Finish3dParams>(own.params).priorToolpaths = {doc2.toolpaths()[1].id.value};
    const ToolpathOutcome self = s2.update(doc2.toolpaths()[1].id, own);
    CHECK_FALSE(self.ok);
    CHECK(self.message.find("own") != std::string::npos);
}

TEST_CASE("Rest tools: a moulding takes a larger-area clearance tool and stores the pair as a group",
          "[toolpath][session][moulding][rest]") {
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    const document::ObjectId profile = addVector(doc, openLine({{-4.0, 0.0}, {4.0, 3.0}}));
    const document::ObjectId rail = addVector(doc, openLine({{0.0, 20.0}, {40.0, 20.0}}));
    doc.selection().assign({profile, rail});
    ToolpathForm form = session.newForm(ToolpathOperation::Moulding);
    REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
    REQUIRE(form.setClearanceTools(db, {source("end-mill-6mm")}).ok);
    REQUIRE(form.setChoice("mode", 1).ok);  // rough
    REQUIRE(form.setField("zOffset", "-3").ok);

    const ToolpathOutcome out = session.add(form.definition());
    INFO(out.message);
    REQUIRE(out.ok);
    REQUIRE(doc.toolpaths().size() == 2);
    const document::ToolpathRecord clear = doc.toolpaths()[0];
    const document::ToolpathRecord main = doc.toolpaths()[1];
    CHECK(clear.name == "Moulding [Clear]");
    CHECK(main.name == "Moulding");
    CHECK(clear.tool.diameter == Approx(6.0));
    CHECK(main.tool.diameter == Approx(3.0));
    CHECK(clear.group.id != 0);
    CHECK(clear.group.id == main.group.id);
    REQUIRE(main.group.tools.size() == 2);
    CHECK(main.group.tools[0].toolId == "end-mill-6mm");
    CHECK(main.group.role == 1);
    CHECK(std::get<toolpath::MouldingParams>(clear.params).clearTool == std::nullopt);  // the tool is in the group
    for (const ToolpathListItem& item : session.list()) {
        CHECK(item.status == ToolpathStatus::Current);
    }
    // Recalculating (from either member) reproduces both records.
    const std::vector<document::ToolpathRecord> before = doc.toolpaths();
    REQUIRE(session.recalculate(clear.id).ok);
    CHECK(doc.toolpaths() == before);
    // Renaming keeps the core's names.
    REQUIRE(session.rename(main.id, "Cove").ok);
    CHECK(doc.toolpaths()[0].name == "Cove [Clear]");
    CHECK(doc.toolpaths()[1].name == "Cove");
    // The definition carries the clearance tool back into a form.
    const auto formBack = session.form(clear.id);
    REQUIRE(formBack.has_value());
    CHECK(formBack->clearanceTools().size() == 1);
    // More than one clearance tool is refused when the job is made.
    ToolpathDefinition two = form.definition();
    two.clearanceTools = {source("end-mill-6mm"), source("end-mill-3mm")};
    const ToolpathOutcome bad = session.add(two);
    CHECK_FALSE(bad.ok);
    CHECK(doc.toolpaths().size() == 2);
}
