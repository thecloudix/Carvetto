// T-0237: the thread mill form (pitch preset hides the pitch, cutter-dependent fields, choices) and
// a thread mill through the session: calculation, undo, save/load, recalculation.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <string>

#include "carvetto/document/carv_io.hpp"
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

TEST_CASE("thread mill form: fields, visibility and bounds", "[toolpath][form][thread_mill]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::ThreadMill, Units::Millimetres);
    REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
    CHECK(form.problems().empty());
    CHECK(find(form.fields(), "pitch") != nullptr);
    CHECK(find(form.fields(), "threadedLength") == nullptr);  // single point
    REQUIRE(form.setChoice("cutter", 1).ok);
    CHECK(find(form.fields(), "threadedLength") != nullptr);
    REQUIRE(form.setChoice("pitchPreset", 1).ok);  // ISO metric
    CHECK(find(form.fields(), "pitch") == nullptr);
    REQUIRE(form.setChoice("pitchPreset", 0).ok);
    REQUIRE(form.setField("diameter", "8").ok);
    REQUIRE(form.setField("pitch", "1.25").ok);
    REQUIRE(form.setField("fitTolerance", "-0.1").ok);
    CHECK(find(form.fields(), "fitTolerance")->text == "-0.1");
    CHECK_FALSE(form.setField("pitch", "0").ok);
    CHECK_FALSE(form.setField("diameter", "-1").ok);
    CHECK_FALSE(form.setField("threadLength", "0").ok);
    CHECK_FALSE(form.setField("startDepth", "-1").ok);
    CHECK_FALSE(form.setField("pitch", "abc").ok);
    CHECK_FALSE(form.setField("diameter", "1e9").ok);
    REQUIRE(form.setChoice("type", 1).ok);
    REQUIRE(form.setChoice("hand", 1).ok);
    REQUIRE(form.setChoice("cutDirection", 1).ok);
    CHECK_FALSE(form.setChoice("hand", 2).ok);
    CHECK_FALSE(form.setChoice("nothing", 0).ok);
}

TEST_CASE("thread mill form: only an end mill is accepted", "[toolpath][form][thread_mill]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::ThreadMill, Units::Millimetres);
    CHECK_FALSE(form.setTool(db, source("v-bit-90")).ok);
    CHECK_FALSE(form.setTool(db, source("drill-3mm")).ok);
    CHECK(form.setTool(db, source("end-mill-3mm")).ok);
}

TEST_CASE("thread mill session: add, undo, save, load and recalculate", "[toolpath][session][thread_mill]") {
    namespace fs = std::filesystem;
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    addVector(doc, geometry::Contour::circle({20, 20}, 2.45));
    doc.undoStack().clear();
    ToolpathForm form(ToolpathOperation::ThreadMill, Units::Millimetres);
    REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
    REQUIRE(form.setChoice("pitchPreset", 1).ok);
    REQUIRE(form.setField("threadLength", "6").ok);
    const ToolpathOutcome out = session.add(form.definition());
    INFO(out.message);
    REQUIRE(out.ok);
    REQUIRE(doc.toolpaths().size() == 1);
    const document::ToolpathRecord& rec = doc.toolpaths().front();
    CHECK(rec.operation() == ToolpathOperation::ThreadMill);
    REQUIRE(rec.hasMoves());
    int turns = 0;
    double minZ = 0.0;
    toolpath::Vec3 at{0, 0, 0};
    for (const toolpath::Move& m : *rec.moves) {
        if (toolpath::isArc(m.kind) && std::fabs(m.to.x - at.x) < 1e-12 && std::fabs(m.to.y - at.y) < 1e-12 &&
            std::fabs(m.to.z - at.z) > 1e-12) {
            ++turns;
            CHECK(std::fabs(m.to.z - at.z) == Approx(1.0).margin(1e-6));
        }
        minZ = std::min(minZ, m.to.z);
        at = m.to;
    }
    CHECK(turns == 6);
    CHECK(minZ == Approx(-6.0).margin(1e-9));
    CHECK(doc.checkIntegrity().empty());

    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / "thread_mill";
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

// T-0262: the thread mill tool type in the form, and "create circles for internal threads".
namespace {
tooling::ToolDatabase dbWithThreadMill() {
    tooling::ToolDatabase db = cliToolDatabase();
    tooling::Tool t;
    t.id = "tm-4";
    t.name = "Thread mill 4";
    t.type = tooling::ToolType::ThreadMill;
    t.diameter = 4.0;
    t.includedAngle = 3.14159265358979 / 3.0;
    t.toothHeight = 0.6;
    t.threadedLength = 0.0;
    t.defaultCuttingData.toolNumber = 7;
    t.defaultCuttingData.feedRate = 500.0;
    t.defaultCuttingData.plungeRate = 200.0;
    t.defaultCuttingData.spindleSpeed = 12000.0;
    t.defaultCuttingData.passDepth = 1.0;
    t.defaultCuttingData.stepover = 1.0;
    REQUIRE(db.addTool(t));
    return db;
}

std::size_t vectorCount(const Document& doc) {
    std::size_t n = 0;
    for (const auto& kv : doc.objects()) {
        n += kv.second.isVector() ? 1u : 0u;
    }
    return n;
}
}  // namespace

TEST_CASE("thread mill form accepts thread mills and end mills only", "[toolpath][form][thread_mill][T-0262]") {
    const tooling::ToolDatabase db = dbWithThreadMill();
    ToolpathForm form(ToolpathOperation::ThreadMill, Units::Millimetres);
    CHECK(form.setTool(db, source("tm-4")).ok);
    CHECK(form.setTool(db, source("end-mill-3mm")).ok);
    CHECK_FALSE(form.setTool(db, source("drill-3mm")).ok);
    CHECK_FALSE(form.setTool(db, source("v-bit-90")).ok);
    // Other operations do not offer a thread mill.
    ToolpathForm drill(ToolpathOperation::Drill, Units::Millimetres);
    CHECK_FALSE(drill.setTool(db, source("tm-4")).ok);
}

TEST_CASE("thread mill: create circles inserts the bores in the same undo step", "[toolpath][session][thread_mill][T-0262]") {
    Document doc;
    const tooling::ToolDatabase db = dbWithThreadMill();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    addVector(doc, geometry::Contour::circle({20, 20}, 2.45));
    addVector(doc, geometry::Contour::circle({40, 20}, 2.45));
    doc.undoStack().clear();
    ToolpathForm form(ToolpathOperation::ThreadMill, Units::Millimetres);
    REQUIRE(form.setTool(db, source("tm-4")).ok);
    REQUIRE(form.setChoice("pitchPreset", 1).ok);  // M6 x 1
    REQUIRE(form.setChoice("createCircles", 1).ok);
    REQUIRE(vectorCount(doc) == 2);
    const std::size_t before = doc.undoStack().count();
    const ToolpathOutcome out = session.add(form.definition());
    INFO(out.message);
    REQUIRE(out.ok);
    CHECK(doc.toolpaths().size() == 1);
    CHECK(vectorCount(doc) == 4);
    CHECK(doc.undoStack().count() == before + 1);  // one step: toolpath and circles
    // The circles are the bore (minor) diameter of M6x1: 6 - 1.0825.
    int bores = 0;
    for (const auto& kv : doc.objects()) {
        const document::VectorObject* v = kv.second.asVector();
        if (v != nullptr && kv.second.name == "Thread bore") {
            ++bores;
            REQUIRE(v->contours.size() == 1);
            const geometry::Box2 b = v->contours.front().bounds();
            CHECK(b.max.x - b.min.x == Approx(6.0 - 1.0825).margin(1e-6));
        }
    }
    CHECK(bores == 2);
    CHECK(doc.checkIntegrity().empty());
    // The toolpath is not stale because of the circles it made.
    const auto list = session.list();
    REQUIRE(list.size() == 1);
    CHECK(list[0].status == ToolpathStatus::Current);
    // Recalculating does not add them again.
    REQUIRE(session.recalculate(list[0].id).ok);
    CHECK(vectorCount(doc) == 4);

    REQUIRE(doc.undo());  // the add (the recalculation changed nothing)
    CHECK(doc.toolpaths().empty());
    CHECK(vectorCount(doc) == 2);
    REQUIRE(doc.redo());
    CHECK(vectorCount(doc) == 4);
    CHECK(doc.toolpaths().size() == 1);
}

TEST_CASE("thread mill: no circles unless asked for, internal, and calculated", "[toolpath][session][thread_mill][T-0262]") {
    Document doc;
    const tooling::ToolDatabase db = dbWithThreadMill();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    addVector(doc, geometry::Contour::circle({20, 20}, 2.45));
    ToolpathForm form(ToolpathOperation::ThreadMill, Units::Millimetres);
    REQUIRE(form.setTool(db, source("tm-4")).ok);
    REQUIRE(form.setChoice("pitchPreset", 1).ok);
    REQUIRE(session.add(form.definition()).ok);  // flag off
    CHECK(vectorCount(doc) == 1);
    REQUIRE(form.setChoice("createCircles", 1).ok);
    REQUIRE(form.setChoice("type", 1).ok);  // external
    REQUIRE(session.add(form.definition()).ok);
    CHECK(vectorCount(doc) == 1);
    // A refused calculation (tool too large for the bore) adds nothing.
    REQUIRE(form.setChoice("type", 0).ok);
    REQUIRE(form.setField("diameter", "4").ok);
    REQUIRE(form.setChoice("pitchPreset", 0).ok);
    const std::size_t toolpaths = doc.toolpaths().size();
    const ToolpathOutcome bad = session.add(form.definition());
    CHECK_FALSE(bad.ok);
    CHECK(doc.toolpaths().size() == toolpaths);
    CHECK(vectorCount(doc) == 1);
}
