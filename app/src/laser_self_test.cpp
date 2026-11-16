// Self-test of the laser toolpaths in the real app (T-0326, ADR-0113), offscreen: Laser Cut / Fill, Laser Picture and
// Laser Sketch Engraving through the Add menu and the form dialog, calculation, edit, stale and recalculate, the burn
// marks of the simulation, the laser setup check of Save Toolpaths and the machine laser flag.

#include <QAction>
#include <QComboBox>
#include <QColor>
#include <QCoreApplication>
#include <QImage>
#include <QLabel>
#include <QPushButton>
#include <QString>
#include <QTemporaryDir>
#include <QTreeWidget>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/relief_commands.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/transform2.hpp"
#include "carvetto/viewmodel/file_session.hpp"
#include "carvetto/viewmodel/save_toolpaths.hpp"
#include "main_window.hpp"
#include "preview_panel.hpp"
#include "relief_view.hpp"
#include "save_toolpaths_dialog.hpp"
#include "toolpath_form_dialog.hpp"
#include "toolpaths_panel.hpp"

namespace carvetto::app {

namespace {

#define LS_REQUIRE(cond, text)                          \
    do {                                                \
        if (!(cond)) {                                  \
            return std::string("laser: ") + (text);     \
        }                                               \
    } while (false)

bool accept(ToolpathsPanel* panel) {
    ToolpathFormDialog* dlg = panel->formDialog();
    if (dlg == nullptr || !dlg->okButton()->isEnabled()) {
        return false;
    }
    dlg->accept();
    return true;
}

struct Burn {
    std::size_t lit{0};
    double minX{1e9};
    double maxX{-1e9};
    double minPower{1e9};
    double maxPower{0.0};
};

Burn burnOf(const document::ToolpathRecord& rec) {
    Burn b;
    if (rec.moves != nullptr) {
        for (const toolpath::Move& m : *rec.moves) {
            if (m.kind == toolpath::MoveKind::Feed && m.jetPower > 0.0) {
                ++b.lit;
                b.minX = std::min(b.minX, m.to.x);
                b.maxX = std::max(b.maxX, m.to.x);
                b.minPower = std::min(b.minPower, m.jetPower);
                b.maxPower = std::max(b.maxPower, m.jetPower);
            }
        }
    }
    return b;
}

/// Runs the simulation to its end; the number of cells the laser marked.
std::size_t simulateBurn(MainWindow& window, std::string& problem) {
    PreviewPanel* preview = window.preview();
    preview->qualityCombo()->setCurrentIndex(0);
    if (!preview->startSimulation() || !preview->waitSimulation(120000)) {
        problem = "the simulation did not run: " + preview->statusLabel()->text().toStdString();
        return 0;
    }
    preview->scrubTo(preview->playback().moveCount());
    if (!preview->waitSimulation(120000)) {
        problem = "the final frame never arrived";
        return 0;
    }
    const auto frame = preview->simulation().frame();
    return frame != nullptr ? frame->burnedCells : 0;
}

}  // namespace

std::string laserSelfTest(MainWindow& window) {
    window.setQuiet(true);
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(100.0, 100.0, 18.0, document::Units::Millimetres)));
    document::Document& doc = window.currentDocument();
    ToolpathsPanel* panel = window.toolpaths();
    LS_REQUIRE(panel != nullptr, "no Toolpaths panel");
    for (const document::ToolpathOperation op : {document::ToolpathOperation::LaserCutFill,
                                                 document::ToolpathOperation::LaserPicture,
                                                 document::ToolpathOperation::LaserSketch}) {
        LS_REQUIRE(panel->addAction(op) != nullptr, "the Add menu lacks a laser entry");
    }

    // The beam: a 1 mm end mill with a maximum burn rate (the simulation needs it).
    const tooling::ToolDatabase original = panel->toolDatabase();
    tooling::ToolDatabase db = original;
    {
        const tooling::Tool* base = db.findTool("end-mill-3mm");
        LS_REQUIRE(base != nullptr, "the starter tools lack the 3 mm end mill");
        tooling::Tool beam = *base;
        beam.diameter = 1.0;
        beam.defaultCuttingData.feedRate = 600.0;
        beam.defaultCuttingData.maxBurnRate = 3000.0;
        LS_REQUIRE(db.updateTool(beam), "could not set up the beam tool");
        panel->applySetup(db, panel->activeMachineId());
    }

    // 1. Laser Cut / Fill: a rectangle, hatch fill, through the form.
    const bool added = doc.execute(std::make_unique<document::AddObjectCommand>(document::Object::vector(
        geometry::Contour::rectangle(geometry::Box2{{10.0, 10.0}, {40.0, 40.0}}), doc.activeLayer(), doc.activeSheet())));
    LS_REQUIRE(added, "could not add the rectangle");
    const document::ObjectId rect = doc.objectIds().back();
    doc.selection().assign({rect});
    window.documentChanged();
    std::size_t steps = doc.undoStack().index();
    panel->addAction(document::ToolpathOperation::LaserCutFill)->trigger();
    ToolpathFormDialog* dlg = panel->formDialog();
    LS_REQUIRE(dlg != nullptr, "Add Laser Cut / Fill opened no form");
    LS_REQUIRE(!dlg->okButton()->isEnabled(), "OK is enabled before a tool is chosen");
    LS_REQUIRE(dlg->pickTool("end-mill-3mm") && dlg->fieldEdit("power") != nullptr && dlg->fieldEdit("moveSpeed") != nullptr,
               "the laser form lacks the tool or a Power / Move Speed field");
    LS_REQUIRE(!dlg->typeField("power", "0") && !dlg->typeField("power", "150"), "a power outside (0, 100] was accepted");
    LS_REQUIRE(dlg->chooseOption("strategy", 3) && dlg->fieldEdit("stepover") != nullptr, "Hatch fill shows no Stepover field");
    LS_REQUIRE(dlg->typeField("power", "60") && dlg->typeField("stepover", "1") && dlg->okButton()->isEnabled(),
               "the Cut / Fill form cannot be completed");
    LS_REQUIRE(accept(panel) && panel->waitIdle(120000), "the Cut / Fill calculation did not finish");
    LS_REQUIRE(doc.toolpaths().size() == 1 && doc.undoStack().index() == steps + 1,
               "adding the Cut / Fill was not one undo step: " + panel->statusText().toStdString());
    {
        const document::ToolpathRecord& rec = doc.toolpaths().front();
        LS_REQUIRE(rec.operation() == document::ToolpathOperation::LaserCutFill && rec.hasMoves() && rec.tool.laser,
                   "the record is not a laser Cut / Fill");
        const Burn b = burnOf(rec);
        LS_REQUIRE(b.lit > 20 && b.minPower == 60.0 && b.maxPower == 60.0, "the beam power is not the form's 60 %");
        LS_REQUIRE(b.minX >= 10.0 - 1e-6 && b.maxX <= 40.0 + 1e-6, "the hatch leaves the rectangle");
    }
    // Edit the power: recalculated in one step.
    panel->selectToolpath(doc.toolpaths().front().id);
    panel->editAction()->trigger();
    dlg = panel->formDialog();
    LS_REQUIRE(dlg != nullptr && dlg->typeField("power", "40"), "the edit form rejected a power of 40");
    LS_REQUIRE(accept(panel) && panel->waitIdle(120000) && doc.undoStack().index() == steps + 2 &&
                   burnOf(doc.toolpaths().front()).maxPower == 40.0,
               "editing the power did not recalculate in one step");
    // Moving the rectangle makes it stale; recalculate.
    LS_REQUIRE(doc.execute(std::make_unique<document::TransformObjectsCommand>(
                   std::vector<document::ObjectId>{rect}, geometry::Transform2::translation({5.0, 0.0}))),
               "could not move the rectangle");
    window.documentChanged();
    LS_REQUIRE(panel->list()->topLevelItem(0)->text(3) == QStringLiteral("Stale"), "moving the vector did not mark it stale");
    panel->selectToolpath(doc.toolpaths().front().id);
    panel->recalculateAction()->trigger();
    LS_REQUIRE(panel->waitIdle(120000) && panel->list()->topLevelItem(0)->text(3) == QStringLiteral("Valid"),
               "recalculation did not clear the stale state");
    LS_REQUIRE(burnOf(doc.toolpaths().front()).minX >= 15.0 - 1e-6, "the recalculated hatch did not follow the moved vector");

    // The simulation marks the surface inside the rectangle (now 15 .. 45) and not elsewhere.
    {
        std::string problem;
        const std::size_t marked = simulateBurn(window, problem);
        LS_REQUIRE(problem.empty(), problem);
        LS_REQUIRE(marked > 100, "the simulation marked no cells for the hatch");
        ReliefView* relief = window.reliefView();
        const QColor burnt = relief->pixelAtStock({30.0, 25.0});
        const QColor bare = relief->pixelAtStock({80.0, 80.0});
        LS_REQUIRE(burnt.isValid() && bare.isValid() && burnt != bare, "the burn is not drawn in the simulation");
    }

    // The save check: a non-laser post and an unflagged machine are refused with the setup message.
    {
        QTemporaryDir outDir;
        LS_REQUIRE(outDir.isValid(), "no temporary directory");
        const std::filesystem::path outPath = viewmodel::pathFromUtf8(outDir.path().toStdString());
        viewmodel::ToolpathSession& session = panel->session();
        std::vector<tooling::Machine> machines;
        for (const auto& entry : session.toolDatabase().machines()) {
            machines.push_back(entry.second);
        }
        LS_REQUIRE(!machines.empty(), "no machine in the tool database");
        machines.front().laser = false;
        machines.front().associatedPosts.clear();  // so that picking the machine leaves the post alone
        machines.front().defaultPost.clear();
        SaveToolpathsDialog dialog(viewmodel::makeSaveSource(session), "job", machines, outPath);
        dialog.show();
        LS_REQUIRE(dialog.waitIdle() && dialog.postCount() >= 3, "the Save Toolpaths dialog did not start");
        LS_REQUIRE(dialog.selectPost("grbl_mm.pp") && dialog.waitIdle() && !dialog.output().ok &&
                       dialog.output().error.find("LASER_SUPPORT") != std::string::npos && !dialog.canSave(),
                   "a laser job was not refused for a post without laser support: " + dialog.output().error);
        LS_REQUIRE(dialog.selectPost("grbl_laser_mm.pp") && dialog.waitIdle() && dialog.output().ok && dialog.canSave(),
                   "a laser job was refused for the laser post without a machine: " + dialog.output().error);
        dialog.setMachineIndex(1);
        LS_REQUIRE(dialog.waitIdle(), "the dialog did not finish for the machine");
        LS_REQUIRE(!dialog.output().ok && dialog.output().error.find("not marked as a laser machine") != std::string::npos &&
                       dialog.messageShown().find("laser machine") != std::string::npos,
                   "a laser job was not refused for a machine without the laser flag: " + dialog.output().error);
        machines.front().laser = true;
        SaveToolpathsDialog flagged(viewmodel::makeSaveSource(session), "job", machines, outPath);
        flagged.show();
        LS_REQUIRE(flagged.waitIdle() && flagged.selectPost("grbl_laser_mm.pp") && flagged.waitIdle(),
                   "the second dialog did not start");
        flagged.setMachineIndex(1);
        LS_REQUIRE(flagged.waitIdle() && flagged.output().ok &&
                       flagged.previewShown().find("M4") != std::string::npos,
                   "the flagged laser machine with the laser post did not give laser output: " + flagged.output().error);
    }

    // 2. The bitmap: a 64 x 32 PNG, left half black, imported and selected.
    QTemporaryDir dir;
    LS_REQUIRE(dir.isValid(), "no temporary directory for the PNG");
    QImage png(64, 32, QImage::Format_RGB32);
    png.fill(Qt::white);
    for (int y = 0; y < png.height(); ++y) {
        for (int x = 0; x < png.width() / 2; ++x) {
            png.setPixelColor(x, y, Qt::black);
        }
    }
    const QString pngPath = dir.filePath(QStringLiteral("half.png"));
    LS_REQUIRE(png.save(pngPath), "could not write the test PNG");
    LS_REQUIRE(window.importBitmapFile(std::filesystem::path(pngPath.toStdString())) && doc.selection().size() == 1,
               "importing the PNG failed");
    const document::ObjectId bitmap = doc.selection().ids().front();
    LS_REQUIRE(doc.findObject(bitmap) != nullptr && doc.findObject(bitmap)->asBitmap() != nullptr, "no bitmap object");
    const geometry::Box2 placed = doc.findObject(bitmap)->asBitmap()->placementMm;
    const double mid = 0.5 * (placed.min.x + placed.max.x);

    // 3. Laser Picture: the beam burns the dark half only.
    steps = doc.undoStack().index();
    panel->addAction(document::ToolpathOperation::LaserPicture)->trigger();
    dlg = panel->formDialog();
    LS_REQUIRE(dlg != nullptr && dlg->pickTool("end-mill-3mm"), "Add Laser Picture opened no form or the beam cannot be picked");
    LS_REQUIRE(dlg->fieldEdit("lineSpacing") != nullptr && dlg->fieldEdit("power") != nullptr, "the picture form lacks a field");
    LS_REQUIRE(!dlg->typeField("lineSpacing", "0"), "a zero line spacing was accepted");
    LS_REQUIRE(dlg->typeField("lineSpacing", "1") && dlg->okButton()->isEnabled(), "the picture form cannot be completed");
    LS_REQUIRE(dlg->chooseOption("strategy", 2) && !dlg->okButton()->isEnabled(),
               "Selected Vectors without vectors can be accepted");
    LS_REQUIRE(dlg->chooseOption("strategy", 0) && dlg->okButton()->isEnabled(), "Raster cannot be chosen again");
    LS_REQUIRE(accept(panel) && panel->waitIdle(120000), "the Laser Picture calculation did not finish");
    LS_REQUIRE(doc.toolpaths().size() == 2 && doc.undoStack().index() == steps + 1,
               "adding the Laser Picture was not one undo step: " + panel->statusText().toStdString());
    {
        const document::ToolpathRecord& rec = doc.toolpaths()[1];
        LS_REQUIRE(rec.operation() == document::ToolpathOperation::LaserPicture && rec.hasMoves() && rec.tool.laser,
                   "the record is not a laser picture");
        const Burn b = burnOf(rec);
        LS_REQUIRE(b.lit > 5 && b.minX >= placed.min.x - 1e-6 && b.maxX <= mid + 1e-6,
                   "the picture is not burned over the dark half only");
        LS_REQUIRE(b.maxPower == 100.0, "a black pixel does not burn at full power");
    }
    // Edit: dither on, a different record, still one step.
    panel->selectToolpath(doc.toolpaths()[1].id);
    panel->editAction()->trigger();
    dlg = panel->formDialog();
    LS_REQUIRE(dlg != nullptr && dlg->chooseOption("dither", 1) && dlg->fieldEdit("minPower") == nullptr,
               "the dither choice is missing or still shows the minimum power");
    LS_REQUIRE(accept(panel) && panel->waitIdle(120000) && doc.undoStack().index() == steps + 2 &&
                   std::get<toolpath::LaserPictureParams>(doc.toolpaths()[1].params).dither,
               "editing the dither did not recalculate in one step");

    // 4. Laser Sketch Engraving, bitmap source.
    doc.selection().assign({bitmap});
    window.documentChanged();
    steps = doc.undoStack().index();
    panel->addAction(document::ToolpathOperation::LaserSketch)->trigger();
    dlg = panel->formDialog();
    LS_REQUIRE(dlg != nullptr && dlg->pickTool("end-mill-3mm"), "Add Laser Sketch Engraving opened no form");
    LS_REQUIRE(dlg->fieldEdit("stepover") != nullptr && dlg->fieldEdit("lineThickness") != nullptr &&
                   dlg->fieldEdit("hatchAngle") != nullptr && dlg->fieldEdit("overscan") != nullptr && !dlg->chooseOption("modelTone", 1),
               "the sketch form lacks a field or offers the model tone for a bitmap");
    LS_REQUIRE(dlg->chooseOption("machineLimitBoundary", 1) && dlg->typeField("overscan", "3"),
               "the machine limit boundary or the overscan cannot be set");
    LS_REQUIRE(dlg->typeField("stepover", "2") && dlg->typeField("lineThickness", "1") && dlg->okButton()->isEnabled(),
               "the sketch form cannot be completed");
    // The Vectors boundary needs vectors besides the bitmap.
    LS_REQUIRE(dlg->chooseOption("boundary", 1) && !dlg->okButton()->isEnabled(),
               "the Vectors boundary without vectors can be accepted");
    LS_REQUIRE(dlg->chooseOption("boundary", 0) && dlg->okButton()->isEnabled(), "the whole source cannot be chosen again");
    // The 3D model source needs a model; the choice shows the tone and the bitmap needs no longer be there.
    LS_REQUIRE(dlg->chooseOption("source", 1) && dlg->chooseOption("modelTone", 0), "the source choice did not show the model tone");
    LS_REQUIRE(!dlg->okButton()->isEnabled(), "a model source without a model can be accepted");
    LS_REQUIRE(dlg->chooseOption("source", 0) && dlg->okButton()->isEnabled(), "the bitmap source cannot be chosen again");
    LS_REQUIRE(accept(panel) && panel->waitIdle(120000), "the Laser Sketch calculation did not finish");
    LS_REQUIRE(doc.toolpaths().size() == 3 && doc.undoStack().index() == steps + 1,
               "adding the Laser Sketch was not one undo step: " + panel->statusText().toStdString());
    {
        const document::ToolpathRecord& rec = doc.toolpaths()[2];
        LS_REQUIRE(rec.operation() == document::ToolpathOperation::LaserSketch && rec.hasMoves() && rec.tool.laser,
                   "the record is not a laser sketch");
        const Burn b = burnOf(rec);
        LS_REQUIRE(b.lit > 3 && b.minX >= placed.min.x - 1e-6 && b.maxX <= mid + 1e-6,
                   "the sketch is not burned over the dark half only");
        // The machine limit boundary keeps even the beam-off overscan inside the sheet's 100 x 100 limits.
        for (const toolpath::Move& m : *rec.moves) {
            LS_REQUIRE(m.to.x >= -1e-9 && m.to.x <= 100.0 + 1e-9 && m.to.y >= -1e-9 && m.to.y <= 100.0 + 1e-9,
                       "a move leaves the machine limits");
        }
    }
    // Moving the bitmap makes both bitmap toolpaths stale.
    LS_REQUIRE(doc.execute(std::make_unique<document::TransformObjectsCommand>(
                   std::vector<document::ObjectId>{bitmap}, geometry::Transform2::translation({2.0, 0.0}))),
               "could not move the bitmap");
    window.documentChanged();
    LS_REQUIRE(panel->list()->topLevelItem(1)->text(3) == QStringLiteral("Stale") &&
                   panel->list()->topLevelItem(2)->text(3) == QStringLiteral("Stale"),
               "moving the bitmap did not mark the picture and the sketch stale");
    panel->recalculateAllAction()->trigger();
    LS_REQUIRE(panel->waitIdle(240000) && panel->list()->topLevelItem(1)->text(3) == QStringLiteral("Valid") &&
                   panel->list()->topLevelItem(2)->text(3) == QStringLiteral("Valid"),
               "Recalculate all did not clear the stale states");

    // 5. Laser Sketch Engraving, 3D model source: a wavy model on the sheet.
    auto level = std::make_unique<document::AddReliefLevelCommand>(doc.activeSheet(), "Level");
    const document::AddReliefLevelCommand* rawLevel = level.get();
    LS_REQUIRE(doc.execute(std::move(level)), "could not add a level");
    relief::Component c;
    c.name = "Waves";
    std::vector<float> data;
    for (int j = 0; j < 41; ++j) {
        for (int i = 0; i < 41; ++i) {
            data.push_back(static_cast<float>(3.0 + 3.0 * std::sin(i * 0.3) * std::cos(j * 0.25)));
        }
    }
    LS_REQUIRE(relief::HeightField::fromData(relief::Grid{41, 41, 1.0, {50.0, 50.0}}, std::move(data), c.field) ==
                   relief::Status::Ok,
               "could not build the model");
    LS_REQUIRE(doc.execute(std::make_unique<document::AddReliefComponentCommand>(rawLevel->createdId(), std::move(c))),
               "could not add the model");
    doc.selection().clear();
    window.documentChanged();
    steps = doc.undoStack().index();
    panel->addAction(document::ToolpathOperation::LaserSketch)->trigger();
    dlg = panel->formDialog();
    LS_REQUIRE(dlg != nullptr && dlg->pickTool("end-mill-3mm"), "the second sketch form did not open");
    LS_REQUIRE(!dlg->okButton()->isEnabled(), "a bitmap sketch without a bitmap can be accepted");
    LS_REQUIRE(dlg->chooseOption("source", 1) && dlg->chooseOption("modelTone", 1),
               "the model source offers no tone choice");
    LS_REQUIRE(dlg->typeField("stepover", "2") && dlg->typeField("lineThickness", "1") && dlg->okButton()->isEnabled(),
               "the model sketch cannot be completed");
    LS_REQUIRE(accept(panel) && panel->waitIdle(240000), "the model sketch calculation did not finish");
    LS_REQUIRE(doc.toolpaths().size() == 4 && doc.undoStack().index() == steps + 1,
               "adding the model sketch was not one undo step: " + panel->statusText().toStdString());
    {
        const document::ToolpathRecord& rec = doc.toolpaths()[3];
        const auto* p = std::get_if<toolpath::LaserSketchParams>(&rec.params);
        LS_REQUIRE(p != nullptr && p->source == toolpath::SketchSource::Model && rec.hasMoves() && burnOf(rec).lit > 3,
                   "the model sketch record is wrong");
        const Burn b = burnOf(rec);
        // The model's cells are 1 mm wide about 50 .. 90: its extent is half a cell wider.
        LS_REQUIRE(b.minX >= 49.5 - 1e-6 && b.maxX <= 90.5 + 1e-6 && b.maxX > 60.0,
                   "the model sketch is not within the model: " + std::to_string(b.minX) + " .. " + std::to_string(b.maxX));
    }
    // Only the model sketch is simulated here, with the picture's and the cut's marks too: all three kinds burn.
    {
        std::string problem;
        const std::size_t marked = simulateBurn(window, problem);
        LS_REQUIRE(problem.empty(), problem);
        LS_REQUIRE(marked > 100, "the simulation marked no cells for the picture and the sketches");
    }

    // 6. Save and reload: the records come back unchanged.
    const std::filesystem::path file(dir.filePath(QStringLiteral("laser.carv")).toStdString());
    LS_REQUIRE(window.saveFileAs(file) && window.openFile(file), "saving and reloading failed");
    document::Document& loaded = window.currentDocument();
    LS_REQUIRE(loaded.toolpaths().size() == 4 &&
                   loaded.toolpaths()[0].operation() == document::ToolpathOperation::LaserCutFill &&
                   loaded.toolpaths()[1].operation() == document::ToolpathOperation::LaserPicture &&
                   loaded.toolpaths()[2].operation() == document::ToolpathOperation::LaserSketch &&
                   loaded.toolpaths()[3].operation() == document::ToolpathOperation::LaserSketch &&
                   loaded.toolpaths()[3].hasMoves() && loaded.toolpaths()[3].tool.laser,
               "the reloaded file lost a laser toolpath");
    window.toolpaths()->applySetup(original, window.toolpaths()->activeMachineId());  // later self-tests use the starter tools
    return std::string();
}

}  // namespace carvetto::app
