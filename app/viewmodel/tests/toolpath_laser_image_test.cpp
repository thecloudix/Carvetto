// T-0324: the Laser Picture and Laser Sketch Engraving forms and sessions (ADR-0104, ADR-0105): fields and
// bounds, the bitmap or relief model as the input, power on the moves, machine limits, staleness, save and load.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/relief_commands.hpp"
#include "carvetto/geometry/transform2.hpp"
#include "carvetto/viewmodel/bitmap_import.hpp"
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

template <class F>
std::vector<std::uint8_t> pgm(int w, int h, F value) {
    const std::string head = "P5\n" + std::to_string(w) + " " + std::to_string(h) + "\n255\n";
    std::vector<std::uint8_t> out(head.begin(), head.end());
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            out.push_back(static_cast<std::uint8_t>(value(x, y)));
        }
    }
    return out;
}

/// Left half black, right half white.
document::ObjectId importHalf(Document& doc) {
    const BitmapImportOutcome out =
        importBitmapIntoDocument(doc, pgm(8, 4, [](int x, int) { return x < 4 ? 0 : 255; }), "half.pgm", {});
    INFO(out.error);
    REQUIRE(out.ok);
    return out.object;
}

/// A 41 x 41 model, 1 mm cells, a ramp rising along X.
document::LevelId addRamp(Document& doc) {
    auto cmd = std::make_unique<document::AddReliefLevelCommand>(doc.activeSheet(), "Level");
    const document::AddReliefLevelCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    const document::LevelId level = raw->createdId();
    relief::Component c;
    c.name = "Ramp";
    std::vector<float> data;
    for (int j = 0; j < 41; ++j) {
        for (int i = 0; i < 41; ++i) {
            data.push_back(static_cast<float>(i) * 0.15F);
        }
    }
    REQUIRE(relief::HeightField::fromData(relief::Grid{41, 41, 1.0, {0.0, 0.0}}, std::move(data), c.field) ==
            relief::Status::Ok);
    REQUIRE(doc.execute(std::make_unique<document::AddReliefComponentCommand>(level, std::move(c))));
    return level;
}

ToolpathDefinition pictureFor(const tooling::ToolDatabase& db, const Document& doc) {
    ToolpathForm form(ToolpathOperation::LaserPicture, Units::Millimetres);
    REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
    REQUIRE(form.setField("lineSpacing", "5").ok);
    form.setSelector(VectorSelector::fromSelection(doc));
    return form.definition();
}

ToolpathDefinition sketchFor(const tooling::ToolDatabase& db, const Document& doc, bool model) {
    ToolpathForm form(ToolpathOperation::LaserSketch, Units::Millimetres);
    REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
    REQUIRE(form.setField("stepover", "4").ok);
    if (model) {
        REQUIRE(form.setChoice("source", 1).ok);
    } else {
        form.setSelector(VectorSelector::fromSelection(doc));
    }
    return form.definition();
}

struct Burn {
    double minX{1e9};
    double maxX{-1e9};
    double minY{1e9};
    double maxY{-1e9};
    std::size_t moves{0};
};

Burn burnOf(const document::ToolpathRecord& rec) {
    Burn b;
    for (const toolpath::Move& m : *rec.moves) {
        if (m.jetPower > 0.0) {
            b.minX = std::min(b.minX, m.to.x);
            b.maxX = std::max(b.maxX, m.to.x);
            b.minY = std::min(b.minY, m.to.y);
            b.maxY = std::max(b.maxY, m.to.y);
            ++b.moves;
        }
    }
    return b;
}

}  // namespace

TEST_CASE("laser picture form: fields follow dither and strategy, bounds and tool are checked",
          "[toolpath][form][laser][picture]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::LaserPicture, Units::Millimetres);
    CHECK_FALSE(form.setTool(db, source("v-bit-90")).ok);
    CHECK_FALSE(form.setTool(db, source("drill-3mm")).ok);
    REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
    CHECK(toolsFor(db, ToolpathOperation::LaserPicture).size() >= 1);
    CHECK(form.problems().empty());
    for (const char* id : {"power", "minPower", "moveSpeed", "passes", "overscan", "lineSpacing", "lineAngle", "safeZ"}) {
        CHECK(find(form.fields(), id) != nullptr);
    }
    CHECK(find(form.fields(), "chordal") == nullptr);  // Selected Vectors only
    const auto choices = form.choices();
    for (const char* id : {"strategy", "modulatePower", "dither", "invert"}) {
        CHECK(std::any_of(choices.begin(), choices.end(), [&](const ToolpathChoice& c) { return c.id == id; }));
    }
    REQUIRE(form.setChoice("dither", 1).ok);
    CHECK(find(form.fields(), "minPower") == nullptr);  // a dithered picture burns at full power
    const auto dithered = form.choices();
    CHECK_FALSE(std::any_of(dithered.begin(), dithered.end(), [](const ToolpathChoice& c) { return c.id == "modulatePower"; }));
    REQUIRE(form.setChoice("dither", 0).ok);
    REQUIRE(form.setChoice("modulatePower", 0).ok);
    CHECK(find(form.fields(), "minPower") == nullptr);
    REQUIRE(form.setChoice("modulatePower", 1).ok);
    REQUIRE(form.setChoice("strategy", 2).ok);  // selected vectors
    CHECK(find(form.fields(), "lineAngle") == nullptr);
    CHECK(find(form.fields(), "overscan") == nullptr);
    CHECK(find(form.fields(), "chordal") != nullptr);
    REQUIRE(form.setChoice("strategy", 1).ok);
    REQUIRE(form.setField("power", "70").ok);
    REQUIRE(form.setField("minPower", "10").ok);
    REQUIRE(form.setField("passes", "2").ok);
    REQUIRE(form.setField("lineSpacing", "0.2").ok);
    REQUIRE(form.setField("lineAngle", "30").ok);
    CHECK_FALSE(form.setField("power", "0").ok);
    CHECK_FALSE(form.setField("power", "101").ok);
    CHECK_FALSE(form.setField("minPower", "-1").ok);
    CHECK_FALSE(form.setField("minPower", "101").ok);
    CHECK_FALSE(form.setField("passes", "0").ok);
    CHECK_FALSE(form.setField("passes", "1001").ok);
    CHECK_FALSE(form.setField("lineSpacing", "0").ok);
    CHECK_FALSE(form.setField("overscan", "-1").ok);
    CHECK_FALSE(form.setChoice("strategy", 3).ok);
    const auto& p = std::get<toolpath::LaserPictureParams>(form.definition().params);
    CHECK(p.power == 70.0);
    CHECK(p.minPower == 10.0);
    CHECK(p.passes == 2);
    CHECK(p.lineSpacing == Approx(0.2));
    CHECK(p.lineAngle == Approx(0.5235987755982988));
    CHECK(p.strategy == toolpath::PhotoStrategy::Hatch);
}

TEST_CASE("laser sketch form: fields follow the source and boundary, bounds and tool are checked",
          "[toolpath][form][laser][sketch]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::LaserSketch, Units::Millimetres);
    CHECK_FALSE(form.setTool(db, source("v-bit-90")).ok);
    REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
    CHECK(form.problems().empty());
    for (const char* id : {"power", "moveSpeed", "overscan", "lineThickness", "stepover", "hatchAngle", "safeZ"}) {
        CHECK(find(form.fields(), id) != nullptr);
    }
    CHECK(find(form.fields(), "startX") == nullptr);  // vectors boundary only
    CHECK(find(form.fields(), "chordal") == nullptr);
    const auto has = [&](const char* id) {
        const auto cs = form.choices();
        return std::any_of(cs.begin(), cs.end(), [&](const ToolpathChoice& c) { return c.id == id; });
    };
    for (const char* id : {"source", "boundary", "invert", "machineLimitBoundary", "crossHatch"}) {
        CHECK(has(id));
    }
    CHECK_FALSE(has("modelTone"));  // 3D model source only
    CHECK_FALSE(has("fill"));
    REQUIRE(form.setChoice("source", 1).ok);
    CHECK(has("modelTone"));
    REQUIRE(form.setChoice("boundary", 1).ok);
    CHECK(has("fill"));
    CHECK(find(form.fields(), "startX") != nullptr);
    CHECK(find(form.fields(), "chordal") != nullptr);
    REQUIRE(form.setChoice("fill", 2).ok);
    REQUIRE(form.setChoice("crossHatch", 1).ok);
    REQUIRE(form.setChoice("machineLimitBoundary", 1).ok);
    REQUIRE(form.setField("lineThickness", "0.5").ok);
    REQUIRE(form.setField("stepover", "0.8").ok);
    REQUIRE(form.setField("hatchAngle", "45").ok);
    CHECK_FALSE(form.setField("lineThickness", "0").ok);
    CHECK_FALSE(form.setField("stepover", "0").ok);
    CHECK_FALSE(form.setField("power", "101").ok);
    CHECK_FALSE(form.setField("overscan", "-1").ok);
    CHECK_FALSE(form.setChoice("fill", 3).ok);
    CHECK_FALSE(form.setChoice("source", 2).ok);
    const auto& p = std::get<toolpath::LaserSketchParams>(form.definition().params);
    CHECK(p.source == toolpath::SketchSource::Model);
    CHECK(p.boundary == toolpath::SketchBoundary::Vectors);
    CHECK(p.fill == toolpath::SketchFill::BetweenVectors);
    CHECK(p.crossHatch);
    CHECK(p.machineLimitBoundary);
    CHECK(p.lineThickness == Approx(0.5));
    CHECK(p.hatchAngle == Approx(0.7853981633974483));
}

TEST_CASE("laser picture session: burn the selected bitmap, one undo step, save, load, recalculate",
          "[toolpath][session][laser][picture]") {
    namespace fs = std::filesystem;
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    const document::ObjectId bitmap = importHalf(doc);
    doc.undoStack().clear();

    ToolpathDefinition def = pictureFor(db, doc);
    std::get<toolpath::LaserPictureParams>(def.params).power = 55.0;
    const ToolpathOutcome out = session.add(def);
    INFO(out.message);
    REQUIRE(out.ok);
    REQUIRE(doc.toolpaths().size() == 1);
    const document::ToolpathRecord& rec = doc.toolpaths().front();
    CHECK(rec.operation() == ToolpathOperation::LaserPicture);
    CHECK(rec.selection.objects == std::vector<document::ObjectId>{bitmap});
    CHECK(rec.tool.laser);
    REQUIRE(rec.hasMoves());
    const geometry::Box2 box = doc.findObject(bitmap)->asBitmap()->placementMm;
    const double splitX = 0.5 * (box.min.x + box.max.x);
    const Burn burn = burnOf(rec);
    CHECK(burn.moves > 3);
    CHECK(burn.minX == Approx(box.min.x).margin(1e-6));
    CHECK(burn.maxX == Approx(splitX).margin(1e-6));  // the dark left half only
    for (const toolpath::Move& m : *rec.moves) {
        if (m.jetPower > 0.0) {
            CHECK(m.jetPower == Approx(55.0));
            CHECK(m.to.z == 0.0);
        }
    }

    // Invert swaps the halves.
    {
        ToolpathDefinition inv = def;
        std::get<toolpath::LaserPictureParams>(inv.params).invert = true;
        const ToolpathCalculation calc = session.prepare(inv).run();
        REQUIRE(calc.ok);
        const Burn right = burnOf(calc.items.front().record);
        CHECK(right.minX == Approx(splitX).margin(1e-6));
        CHECK(right.maxX == Approx(box.max.x).margin(1e-6));
    }

    CHECK(doc.checkIntegrity().empty());
    REQUIRE(session.list().size() == 1);
    CHECK(session.list().front().status == ToolpathStatus::Current);

    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / "laser_picture";
    fs::create_directories(dir);
    REQUIRE(document::saveCarv(doc, dir / "t.carv", {}));
    auto loaded = document::loadCarv(dir / "t.carv");
    REQUIRE(loaded);
    Document& doc2 = loaded.value().document;
    CHECK(doc2.toolpaths() == doc.toolpaths());
    CHECK(loaded.value().formatMinor == document::kCarvFormatMinorLaserImages);
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
    CHECK(doc.findObject(bitmap) != nullptr);
}

TEST_CASE("laser picture session: moving the bitmap makes it stale; refusals say what is missing",
          "[toolpath][session][laser][picture]") {
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    const document::ObjectId bitmap = importHalf(doc);
    doc.undoStack().clear();
    REQUIRE(session.add(pictureFor(db, doc)).ok);
    CHECK(session.list().front().status == ToolpathStatus::Current);
    REQUIRE(doc.execute(std::make_unique<document::TransformObjectsCommand>(
        std::vector<document::ObjectId>{bitmap}, geometry::Transform2::translation({5.0, 0.0}))));
    CHECK(session.list().front().status == ToolpathStatus::NeedsRecalculation);
    CHECK(session.list().front().vectorsChanged);
    REQUIRE(doc.undo());
    CHECK(session.list().front().status == ToolpathStatus::Current);

    // No bitmap selected.
    Document empty;
    ToolpathSession s2(&empty);
    s2.setToolDatabase(db);
    addVector(empty, square(10, 10, 20));
    ToolpathDefinition def = pictureFor(db, empty);
    def.selector = VectorSelector::allVectors();
    const ToolpathOutcome out = s2.add(def);
    CHECK_FALSE(out.ok);
    CHECK(out.message.find("bitmap") != std::string::npos);
    CHECK(empty.toolpaths().empty());

    // Selected Vectors needs vectors besides the bitmap.
    ToolpathDefinition sel = pictureFor(db, doc);
    std::get<toolpath::LaserPictureParams>(sel.params).strategy = toolpath::PhotoStrategy::SelectedVectors;
    const ToolpathOutcome none = session.add(sel);
    CHECK_FALSE(none.ok);
    CHECK(none.message.find("vectors") != std::string::npos);
}

TEST_CASE("laser sketch session: bitmap source, one undo step, save, load, recalculate",
          "[toolpath][session][laser][sketch]") {
    namespace fs = std::filesystem;
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    const document::ObjectId bitmap = importHalf(doc);
    doc.undoStack().clear();
    ToolpathDefinition def = sketchFor(db, doc, false);
    std::get<toolpath::LaserSketchParams>(def.params).power = 35.0;
    const ToolpathOutcome out = session.add(def);
    INFO(out.message);
    REQUIRE(out.ok);
    const document::ToolpathRecord& rec = doc.toolpaths().front();
    CHECK(rec.operation() == ToolpathOperation::LaserSketch);
    CHECK(rec.selection.objects == std::vector<document::ObjectId>{bitmap});
    CHECK(rec.tool.laser);
    const geometry::Box2 box = doc.findObject(bitmap)->asBitmap()->placementMm;
    const Burn burn = burnOf(rec);
    CHECK(burn.moves > 2);
    CHECK(burn.minX == Approx(box.min.x).margin(1e-6));
    CHECK(burn.maxX == Approx(0.5 * (box.min.x + box.max.x)).margin(1e-6));
    for (const toolpath::Move& m : *rec.moves) {
        if (m.jetPower > 0.0) {
            CHECK(m.jetPower == 35.0);
        }
    }
    CHECK(doc.checkIntegrity().empty());

    const fs::path dir = fs::path(CARVETTO_VM_WORK_DIR) / "laser_sketch";
    fs::create_directories(dir);
    REQUIRE(document::saveCarv(doc, dir / "t.carv", {}));
    auto loaded = document::loadCarv(dir / "t.carv");
    REQUIRE(loaded);
    Document& doc2 = loaded.value().document;
    CHECK(doc2.toolpaths() == doc.toolpaths());
    ToolpathSession s2(&doc2);
    s2.setToolDatabase(db);
    REQUIRE(s2.list().size() == 1);
    CHECK(s2.list()[0].status == ToolpathStatus::Current);
    const std::vector<document::ToolpathRecord> before = doc2.toolpaths();
    REQUIRE(s2.recalculate(s2.list()[0].id).ok);
    CHECK(doc2.toolpaths() == before);
    REQUIRE(doc.undo());
    CHECK(doc.toolpaths().empty());
}

TEST_CASE("laser sketch session: the Vectors boundary needs closed vectors and bounds the sketch",
          "[toolpath][session][laser][sketch]") {
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    const document::ObjectId bitmap = importHalf(doc);
    (void)bitmap;
    ToolpathDefinition def = sketchFor(db, doc, false);
    std::get<toolpath::LaserSketchParams>(def.params).boundary = toolpath::SketchBoundary::Vectors;
    const ToolpathOutcome none = session.add(def);
    CHECK_FALSE(none.ok);
    CHECK(none.message.find("closed vectors") != std::string::npos);
    CHECK(doc.toolpaths().empty());
    // A vector over the black half, selected with the bitmap.
    const geometry::Box2 box = doc.findObject(bitmap)->asBitmap()->placementMm;
    const double x0 = box.min.x + 0.1 * box.width();
    const double y0 = box.min.y + 0.2 * box.height();
    const document::ObjectId vec = addVector(doc, square(x0, y0, 0.3 * box.height()));
    doc.selection().assign({bitmap, vec});
    def.selector = VectorSelector::fromSelection(doc);
    const ToolpathOutcome out = session.add(def);
    INFO(out.message);
    REQUIRE(out.ok);
    const Burn burn = burnOf(doc.toolpaths().front());
    CHECK(burn.moves > 0);
    CHECK(burn.minX >= x0 - 1e-6);
    CHECK(burn.maxX <= x0 + 0.3 * box.height() + 1e-6);
    CHECK(burn.minY >= y0 - 1e-6);
    CHECK(burn.maxY <= y0 + 0.3 * box.height() + 1e-6);
}

TEST_CASE("laser sketch session: 3D model source reads the relief model; editing it makes the toolpath stale",
          "[toolpath][session][laser][sketch][model]") {
    Document empty;
    const tooling::ToolDatabase db = cliToolDatabase();
    {
        ToolpathSession session(&empty);
        session.setToolDatabase(db);
        const ToolpathOutcome out = session.add(sketchFor(db, empty, true));
        CHECK_FALSE(out.ok);
        CHECK(out.message.find("relief model") != std::string::npos);
        CHECK(empty.toolpaths().empty());
    }
    Document doc;
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    const document::LevelId level = addRamp(doc);
    doc.undoStack().clear();
    ToolpathDefinition def = sketchFor(db, doc, true);
    const ToolpathOutcome out = session.add(def);
    INFO(out.message);
    REQUIRE(out.ok);
    const document::ToolpathRecord& rec = doc.toolpaths().front();
    CHECK(rec.operation() == ToolpathOperation::LaserSketch);
    REQUIRE(rec.hasMoves());
    // The ramp is low on the left (dark in the depth tone): the burn stays over about the left half of 41 mm.
    const Burn burn = burnOf(rec);
    CHECK(burn.moves > 3);
    CHECK(burn.minX == Approx(-0.5).margin(1e-6));  // the first cell starts half a cell before its centre
    CHECK(burn.maxX > 17.0);
    CHECK(burn.maxX < 22.0);
    CHECK(session.list().front().status == ToolpathStatus::Current);

    relief::Component c;
    c.name = "Bump";
    REQUIRE(relief::HeightField::create(relief::Grid{10, 10, 1.0, {5.0, 5.0}}, 2.0F, c.field) == relief::Status::Ok);
    REQUIRE(doc.execute(std::make_unique<document::AddReliefComponentCommand>(level, std::move(c))));
    session.invalidate();
    CHECK(session.list()[0].status == ToolpathStatus::NeedsRecalculation);
    REQUIRE(doc.undo());
    session.invalidate();
    CHECK(session.list()[0].status == ToolpathStatus::Current);

    // The shaded tone is a different input to the same operation: a ramp seen from the light is white.
    ToolpathDefinition shaded = def;
    std::get<toolpath::LaserSketchParams>(shaded.params).modelTone = toolpath::SketchModelTone::Shaded;
    std::get<toolpath::LaserSketchParams>(shaded.params).invert = true;
    const ToolpathCalculation calc = session.prepare(shaded).run();
    INFO(calc.errorText());
    REQUIRE(calc.ok);
    CHECK(burnOf(calc.items.front().record).moves > 0);
}

TEST_CASE("laser sketch session: the machine limit boundary clips to the material and the machine's work area",
          "[toolpath][session][laser][sketch][model]") {
    Document doc;
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathSession session(&doc);
    session.setToolDatabase(db);
    addRamp(doc);  // covers -0.5 .. 40.5 mm
    doc.undoStack().clear();
    ToolpathDefinition def = sketchFor(db, doc, true);
    std::get<toolpath::LaserSketchParams>(def.params).stepover = 1.0;
    {
        const ToolpathCalculation calc = session.prepare(def).run();
        REQUIRE(calc.ok);
        CHECK(burnOf(calc.items.front().record).minX == Approx(-0.5).margin(1e-6));
    }
    std::get<toolpath::LaserSketchParams>(def.params).machineLimitBoundary = true;
    const ToolpathCalculation clipped = session.prepare(def).run();
    INFO(clipped.errorText());
    REQUIRE(clipped.ok);
    const Burn burn = burnOf(clipped.items.front().record);
    CHECK(burn.minX == Approx(0.0).margin(1e-6));
    CHECK(burn.minY >= -1e-6);

    // The limits themselves: the sheet's material rectangle, cut down by a machine's work area.
    const document::Sheet* sheet = doc.findSheet(doc.activeSheet());
    REQUIRE(sheet != nullptr);
    const std::optional<geometry::Box2> plain = machineLimitsOf(doc, nullptr);
    REQUIRE(plain.has_value());
    CHECK(plain->min.x == sheet->materialBoundsMm().min.x);
    CHECK(plain->max.x == sheet->materialBoundsMm().max.x);
    tooling::Machine machine;
    machine.workAreaX = 20.0;
    machine.workAreaY = 10.0;
    const std::optional<geometry::Box2> cut = machineLimitsOf(doc, &machine);
    REQUIRE(cut.has_value());
    CHECK(cut->min.x == plain->min.x);
    CHECK(cut->max.x == Approx(plain->min.x + 20.0));
    CHECK(cut->max.y == Approx(plain->min.y + 10.0));
    // A centred datum puts the rectangle's corner at a negative X and Y that differ: the area is measured from it.
    {
        Document centred(document::Job::singleSided(100.0, 60.0, 5.0, Units::Millimetres, document::Datum::Centre));
        const std::optional<geometry::Box2> c = machineLimitsOf(centred, &machine);
        REQUIRE(c.has_value());
        CHECK(c->min.x == Approx(-50.0));
        CHECK(c->min.y == Approx(-30.0));
        CHECK(c->max.x == Approx(-30.0));
        CHECK(c->max.y == Approx(-20.0));
    }
    machine.workAreaX = 1.0e6;  // larger than the material: the material is the limit
    CHECK(machineLimitsOf(doc, &machine)->max.x == plain->max.x);
    machine.workAreaX = 0.0;  // not stated
    CHECK(machineLimitsOf(doc, &machine)->max.x == plain->max.x);
}

TEST_CASE("laser image operations are named and restricted to the beam tool", "[toolpath][laser]") {
    CHECK(std::string(operationName(ToolpathOperation::LaserPicture)) == "laser_picture");
    CHECK(std::string(operationName(ToolpathOperation::LaserSketch)) == "laser_sketch");
    for (const ToolpathOperation op : {ToolpathOperation::LaserPicture, ToolpathOperation::LaserSketch}) {
        CHECK(toolAllowedFor(op, tooling::ToolType::EndMill));
        CHECK_FALSE(toolAllowedFor(op, tooling::ToolType::VBit));
        CHECK(defaultDefinition(op).operation() == op);
        CHECK(usesBitmap(defaultDefinition(op).params) == true);
        CHECK_FALSE(readsModel(defaultDefinition(op).params));
    }
    toolpath::LaserSketchParams model;
    model.source = toolpath::SketchSource::Model;
    CHECK(readsModel(model));
    CHECK_FALSE(usesBitmap(model));
    CHECK_FALSE(usesBitmap(toolpath::LaserCutFillParams{}));
    CHECK(usesBitmap(toolpath::ImageEngraveParams{}));
    CHECK(readsModel(toolpath::Rough3dParams{}));
}
