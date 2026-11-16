// T-0323: the Laser Cut / Fill form (fields shown by strategy, choices, bounds, tool restriction) and a laser
// toolpath through the session: calculation, power on the moves, undo, save/load, recalculation.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/viewmodel/save_toolpaths.hpp"
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
}  // namespace

TEST_CASE("laser form: fields follow the strategy, bounds and choices are checked", "[toolpath][form][laser]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::LaserCutFill, Units::Millimetres);
    REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
    CHECK(form.problems().empty());
    CHECK(find(form.fields(), "power") != nullptr);
    CHECK(find(form.fields(), "moveSpeed") != nullptr);
    CHECK(find(form.fields(), "repeats") != nullptr);
    CHECK(find(form.fields(), "stepover") == nullptr);  // the default strategy cuts the outside
    CHECK(find(form.fields(), "overscan") == nullptr);
    CHECK(find(form.fields(), "passDepth") == nullptr);  // no cut depth: one pass
    REQUIRE(form.setChoice("strategy", 3).ok);            // hatch fill
    CHECK(find(form.fields(), "stepover") != nullptr);
    CHECK(find(form.fields(), "hatchAngle") != nullptr);
    CHECK(find(form.fields(), "overscan") != nullptr);
    REQUIRE(form.setChoice("crossHatch", 1).ok);
    REQUIRE(form.setChoice("fillMode", 1).ok);
    REQUIRE(form.setChoice("outline", 1).ok);
    REQUIRE(form.setField("cutDepth", "3").ok);
    CHECK(find(form.fields(), "passDepth") != nullptr);
    REQUIRE(form.setField("power", "80").ok);
    REQUIRE(form.setField("repeats", "2").ok);
    CHECK_FALSE(form.setField("power", "0").ok);
    CHECK_FALSE(form.setField("power", "101").ok);
    CHECK_FALSE(form.setField("power", "abc").ok);
    CHECK_FALSE(form.setField("repeats", "0").ok);
    CHECK_FALSE(form.setField("repeats", "1001").ok);
    CHECK_FALSE(form.setField("stepover", "0").ok);
    CHECK_FALSE(form.setField("overscan", "-1").ok);
    CHECK_FALSE(form.setField("cutDepth", "-1").ok);
    CHECK_FALSE(form.setChoice("strategy", 4).ok);
    const auto& p = std::get<toolpath::LaserCutFillParams>(form.definition().params);
    CHECK(p.power == 80.0);
    CHECK(p.repeats == 2);
    CHECK(p.strategy == toolpath::LaserStrategy::HatchFill);
    CHECK(p.crossHatch);
}

TEST_CASE("laser form: only an end mill is accepted as the beam", "[toolpath][form][laser]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::LaserCutFill, Units::Millimetres);
    CHECK_FALSE(form.setTool(db, source("v-bit-90")).ok);
    CHECK_FALSE(form.setTool(db, source("drill-3mm")).ok);
    CHECK(form.setTool(db, source("end-mill-3mm")).ok);
}

TEST_CASE("laser session: add, undo, save, load and recalculate", "[toolpath][session][laser]") {
    namespace fs = std::filesystem;
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    addVector(doc, geometry::Contour::circle({20, 20}, 10.0));
    doc.undoStack().clear();
    ToolpathForm form(ToolpathOperation::LaserCutFill, Units::Millimetres);
    REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
    REQUIRE(form.setChoice("strategy", 3).ok);
    REQUIRE(form.setField("stepover", "1").ok);
    REQUIRE(form.setField("power", "45").ok);
    REQUIRE(form.setField("moveSpeed", "700").ok);
    const ToolpathOutcome out = session.add(form.definition());
    INFO(out.message);
    REQUIRE(out.ok);
    REQUIRE(doc.toolpaths().size() == 1);
    const document::ToolpathRecord& rec = doc.toolpaths().front();
    CHECK(rec.operation() == ToolpathOperation::LaserCutFill);
    CHECK(rec.tool.laser);
    CHECK(rec.tool.feedRate == 700.0);
    REQUIRE(rec.hasMoves());
    std::size_t lit = 0;
    for (const toolpath::Move& m : *rec.moves) {
        if (m.jetPower > 0.0) {
            CHECK(m.jetPower == 45.0);
            ++lit;
        }
    }
    CHECK(lit > 10);
    CHECK(doc.checkIntegrity().empty());

    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / "laser";
    fs::create_directories(dir);
    REQUIRE(document::saveCarv(doc, dir / "t.carv", {}));
    auto loaded = document::loadCarv(dir / "t.carv");
    REQUIRE(loaded);
    Document& doc2 = loaded.value().document;
    ToolpathSession s2(&doc2);
    s2.setToolDatabase(db);
    const auto list = s2.list();
    REQUIRE(list.size() == 1);
    CHECK(list[0].status == ToolpathStatus::Current);
    const std::vector<document::ToolpathRecord> before = doc2.toolpaths();
    REQUIRE(s2.recalculate(list[0].id).ok);
    CHECK(doc2.toolpaths() == before);

    REQUIRE(doc.undo());
    CHECK(doc.toolpaths().empty());
}

TEST_CASE("T-0326: saving a laser toolpath checks the machine's laser flag and the post", "[toolpath][session][laser][save]") {
    namespace fs = std::filesystem;
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    addVector(doc, geometry::Contour::circle({20, 20}, 10.0));
    ToolpathForm laser(ToolpathOperation::LaserCutFill, Units::Millimetres);
    REQUIRE(laser.setTool(db, source("end-mill-3mm")).ok);
    REQUIRE(session.add(laser.definition()).ok);
    const SaveSource laserSource = makeSaveSource(session);

    const std::vector<PostInfo> posts = discoverPosts(fs::path(CARVETTO_POSTS_DIR), {});
    const auto postNamed = [&](const char* file) {
        for (const PostInfo& p : posts) {
            if (p.path.filename() == file) {
                return p;
            }
        }
        FAIL("no post " << file);
        return posts.front();
    };
    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / "laser_save";
    fs::create_directories(dir);
    SaveOptions opt;
    opt.directory = dir;
    opt.checkLaserSetup = true;
    tooling::Machine machine;
    machine.id = "m";
    machine.name = "Diode";

    // A post without laser support is refused with the setup message, a laser post is accepted.
    CHECK(prepareSave(laserSource, postNamed("grbl_mm.pp"), opt).error.find("LASER_SUPPORT") != std::string::npos);
    CHECK(prepareSave(laserSource, postNamed("grbl_laser_mm.pp"), opt).error.empty());
    // A machine without the flag is refused even with the laser post; flagged, it passes.
    opt.machine = machine;
    const SaveJob unflagged = prepareSave(laserSource, postNamed("grbl_laser_mm.pp"), opt);
    CHECK(unflagged.error.find("this job has a laser toolpath") == 0);
    CHECK(unflagged.error.find("\"Diode\" is not marked as a laser machine") != std::string::npos);
    CHECK_FALSE(generateOutput(unflagged).ok);
    opt.machine->laser = true;
    const SaveJob flagged = prepareSave(laserSource, postNamed("grbl_laser_mm.pp"), opt);
    CHECK(flagged.error.empty());
    CHECK(generateOutput(flagged).ok);
    // The check is off unless asked for (the command line is unchanged), and a job without a laser toolpath is
    // not checked at all.
    opt.machine->laser = false;
    opt.checkLaserSetup = false;
    CHECK(prepareSave(laserSource, postNamed("grbl_laser_mm.pp"), opt).error.empty());
    opt.checkLaserSetup = true;
    Document plain;
    ToolpathSession plainSession(&plain);
    plainSession.setToolDatabase(db);
    addVector(plain, geometry::Contour::circle({20, 20}, 10.0));
    ToolpathForm profile(ToolpathOperation::Profile, Units::Millimetres);
    REQUIRE(profile.setTool(db, source("end-mill-3mm")).ok);
    REQUIRE(profile.setField("cutDepth", "2").ok);
    REQUIRE(plainSession.add(profile.definition()).ok);
    CHECK(prepareSave(makeSaveSource(plainSession), postNamed("grbl_mm.pp"), opt).error.empty());
}

TEST_CASE("T-0326: which laser settings make the selected vectors an input", "[toolpath][laser]") {
    toolpath::LaserPictureParams picture;
    CHECK_FALSE(boundaryUsesVectors(picture));
    picture.strategy = toolpath::PhotoStrategy::SelectedVectors;
    CHECK(boundaryUsesVectors(picture));
    toolpath::LaserSketchParams sketch;
    CHECK_FALSE(boundaryUsesVectors(sketch));
    sketch.boundary = toolpath::SketchBoundary::Vectors;
    CHECK(boundaryUsesVectors(sketch));
    sketch.source = toolpath::SketchSource::Model;  // a model sketch inside the vectors reads them too
    CHECK(boundaryUsesVectors(sketch));
    CHECK(readsModel(sketch));
    CHECK_FALSE(usesBitmap(sketch));
    CHECK_FALSE(boundaryUsesVectors(toolpath::LaserCutFillParams{}));
}
