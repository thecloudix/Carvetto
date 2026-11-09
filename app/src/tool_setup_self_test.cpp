// Self-test of the tool / machine / material setup (T-0235): drives the real dialog widgets
// offscreen, applies the result to the Toolpaths dock and the Preview readout, and checks the
// setup file round trip and a hostile file.

#include <QAction>
#include <QComboBox>
#include <QCoreApplication>
#include <QDir>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QTemporaryDir>
#include <QTreeWidget>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include "carvetto/document/document.hpp"
#include "carvetto/viewmodel/file_session.hpp"
#include "carvetto/viewmodel/tool_setup.hpp"
#include "main_window.hpp"
#include "preview_panel.hpp"
#include "tool_setup_dialog.hpp"
#include "toolpath_form_dialog.hpp"
#include "toolpaths_panel.hpp"

namespace carvetto::app {

namespace {

#define TS_REQUIRE(cond, text)                                \
    do {                                                      \
        if (!(cond)) {                                        \
            return std::string("tool setup: ") + (text);      \
        }                                                     \
    } while (false)

template <typename T>
T* child(ToolSetupDialog& dialog, const char* name) {
    return dialog.findChild<T*>(QString::fromLatin1(name));
}

void typeText(QLineEdit* edit, const QString& text) {
    edit->setText(text);
    emit edit->textEdited(text);
}

}  // namespace

std::string toolSetupSelfTest(MainWindow& window) {
    window.setQuiet(true);
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(300.0, 300.0, 18.0, document::Units::Millimetres)));
    ToolpathsPanel* panel = window.toolpaths();
    TS_REQUIRE(window.toolSetupAction() != nullptr && window.jobSetupAction() != nullptr, "the Setup menu actions are missing");
    const tooling::ToolDatabase original = panel->toolDatabase();
    const std::size_t originalTools = original.tools().size();
    TS_REQUIRE(originalTools > 0 && !original.machines().empty(), "the starting tool database is empty");

    QTemporaryDir dir;
    TS_REQUIRE(dir.isValid(), "no temporary directory");
    const std::filesystem::path base = viewmodel::pathFromUtf8(dir.path().toStdString());

    ToolSetupDialog dialog(original, panel->activeMachineId(), document::Units::Millimetres, {}, nullptr);
    dialog.setQuiet(true);
    dialog.show();
    auto* tools = child<QTreeWidget>(dialog, "toolList");
    auto* machines = child<QTreeWidget>(dialog, "machineList");
    auto* materials = child<QTreeWidget>(dialog, "materialList");
    auto* errors = child<QLabel>(dialog, "setupErrors");
    TS_REQUIRE(tools != nullptr && machines != nullptr && materials != nullptr && errors != nullptr,
               "the dialog lacks its lists");
    TS_REQUIRE(static_cast<std::size_t>(tools->topLevelItemCount()) == originalTools, "the tool list is incomplete");
    TS_REQUIRE(machines->topLevelItemCount() == static_cast<int>(original.machines().size()), "the machine list is incomplete");

    // 1. A new end mill through the Add menu, edited through the form fields.
    QAction* addEndMill = dialog.findChild<QAction*>(QStringLiteral("toolAdd_end_mill"));
    TS_REQUIRE(addEndMill != nullptr, "the Add menu has no end mill entry");
    addEndMill->trigger();
    TS_REQUIRE(static_cast<std::size_t>(tools->topLevelItemCount()) == originalTools + 1, "Add did not add a tool");
    const std::string toolId = viewmodel::newToolId(original);
    const tooling::Tool* added = dialog.database().findTool(toolId);
    TS_REQUIRE(added != nullptr && added->type == tooling::ToolType::EndMill, "the new tool is not in the database");
    typeText(child<QLineEdit>(dialog, "toolName"), QStringLiteral("Setup test cutter"));
    child<QDoubleSpinBox>(dialog, "toolDiameter")->setValue(4.5);
    child<QSpinBox>(dialog, "toolFlutes")->setValue(3);
    added = dialog.database().findTool(toolId);
    TS_REQUIRE(added->name == "Setup test cutter" && added->diameter == 4.5 && added->flutes == 3,
               "edits did not reach the database");
    TS_REQUIRE(errors->text().isEmpty(), "a valid edit shows an error");

    // 2. An invalid value is reported and not stored.
    child<QDoubleSpinBox>(dialog, "toolDiameter")->setValue(0.0);
    TS_REQUIRE(!errors->text().isEmpty(), "a zero diameter shows no error");
    TS_REQUIRE(dialog.database().findTool(toolId)->diameter == 4.5, "an invalid diameter was stored");
    child<QDoubleSpinBox>(dialog, "toolDiameter")->setValue(4.5);
    TS_REQUIRE(errors->text().isEmpty(), "the error stays after the value is fixed");

    // 3. A machine with a rapid rate, made the active one; a material; a feeds-and-speeds entry.
    child<QPushButton>(dialog, "machineAdd")->click();
    const std::string machineId = viewmodel::newMachineId(original);
    TS_REQUIRE(dialog.database().findMachine(machineId) != nullptr, "Add machine added nothing");
    typeText(child<QLineEdit>(dialog, "machineName"), QStringLiteral("Test router"));
    child<QDoubleSpinBox>(dialog, "machineRapid")->setValue(7000.0);
    child<QDoubleSpinBox>(dialog, "machineWorkX")->setValue(900.0);
    typeText(child<QLineEdit>(dialog, "machinePosts"), QStringLiteral("grbl, mach3"));
    child<QComboBox>(dialog, "machineDefaultPost")->setCurrentIndex(1);
    emit child<QComboBox>(dialog, "machineDefaultPost")->activated(1);
    child<QPushButton>(dialog, "machineUse")->click();
    const tooling::Machine* machine = dialog.database().findMachine(machineId);
    TS_REQUIRE(machine != nullptr && machine->name == "Test router" && machine->rapidRate == 7000.0 &&
                   machine->workAreaX == 900.0 && machine->defaultPost == "grbl" && machine->associatedPosts.size() == 2,
               "the machine form did not reach the database");
    TS_REQUIRE(dialog.activeMachineId() == machineId, "Use this machine did not select the machine");

    child<QPushButton>(dialog, "materialAdd")->click();
    const std::string materialId = viewmodel::newMaterialId(original);
    TS_REQUIRE(dialog.database().findMaterial(materialId) != nullptr, "Add material added nothing");

    dialog.selectTool(toolId);
    auto* entryMachine = child<QComboBox>(dialog, "entryMachine");
    auto* entryMaterial = child<QComboBox>(dialog, "entryMaterial");
    entryMachine->setCurrentIndex(entryMachine->findData(QString::fromStdString(machineId)));
    entryMaterial->setCurrentIndex(entryMaterial->findData(QString::fromStdString(materialId)));
    child<QPushButton>(dialog, "entryAdd")->click();
    auto* table = child<QTableWidget>(dialog, "cuttingTable");
    TS_REQUIRE(table != nullptr && table->rowCount() == 2, "the feeds table has no entry row");
    table->item(1, 5)->setText(QStringLiteral("1234"));
    TS_REQUIRE(dialog.database().findTool(toolId)->findCuttingData(machineId, materialId) != nullptr &&
                   dialog.database().findTool(toolId)->findCuttingData(machineId, materialId)->feedRate == 1234.0,
               "the feed rate typed in the table did not reach the database");
    table->item(1, 5)->setText(QStringLiteral("fast"));
    TS_REQUIRE(!errors->text().isEmpty(), "a non-numeric feed shows no error");
    table->item(1, 5)->setText(QStringLiteral("1234"));

    // 4. Duplicate, then delete the copy.
    child<QPushButton>(dialog, "toolDuplicate")->click();
    TS_REQUIRE(static_cast<std::size_t>(tools->topLevelItemCount()) == originalTools + 2, "Duplicate did not add a tool");
    child<QPushButton>(dialog, "toolDelete")->click();
    TS_REQUIRE(static_cast<std::size_t>(tools->topLevelItemCount()) == originalTools + 1, "Delete did not remove the tool");

    // 5. Export, and import into a dialog that starts with only the original tools.
    const std::filesystem::path exported = base / "export.json";
    TS_REQUIRE(dialog.exportFile(exported), "export failed");
    {
        ToolSetupDialog other(original, std::string(), document::Units::Inches, {}, nullptr);
        other.setQuiet(true);
        TS_REQUIRE(other.importFile(exported), "import of the exported file failed");
        TS_REQUIRE(other.database().findTool(toolId) != nullptr && other.database().findMachine(machineId) != nullptr,
                   "the imported database lacks the new items");
        TS_REQUIRE(!other.importFile(base / "missing.json") && !other.errorText().isEmpty(),
                   "importing a missing file did not report an error");
    }

    // 6. Accept: the dock, the picker, the readout and the Save dialog's machine list read the result.
    dialog.accept();
    const std::filesystem::path setupFile = base / "config" / viewmodel::kSetupFileName;
    TS_REQUIRE(panel->useSetupFile(setupFile, std::string()).empty(), "a missing setup file produced a message");
    panel->applySetup(dialog.database(), dialog.activeMachineId());
    TS_REQUIRE(panel->setupSaveError().empty() && std::filesystem::exists(setupFile), "the setup file was not written");
    TS_REQUIRE(panel->toolDatabase().findTool(toolId) != nullptr, "the dock's tool database lacks the new tool");
    TS_REQUIRE(panel->session().machineSetup().rapidRateMmPerMin == 7000.0, "the rapid rate did not reach the session");
    TS_REQUIRE(window.preview()->rapidRate()->value() == 7000.0, "the Preview rapid rate is not the machine's");
    TS_REQUIRE(panel->session().toolDatabase().findMachine(machineId) != nullptr, "the session lacks the machine");
    panel->addAction(document::ToolpathOperation::Profile)->trigger();
    ToolpathFormDialog* form = panel->formDialog();
    TS_REQUIRE(form != nullptr && form->pickTool(toolId), "the toolpath form does not offer the new tool");
    form->reject();
    QCoreApplication::processEvents();

    // 7. The file round-trips, and a hostile file is refused with the database kept.
    TS_REQUIRE(panel->useSetupFile(setupFile, panel->activeMachineId()).empty() &&
                   panel->toolDatabase() == dialog.database(),
               "the setup file did not round-trip");
    TS_REQUIRE(panel->activeMachineId() == machineId, "the active machine was lost on reload");
    {
        std::ofstream bad(base / "bad.json", std::ios::binary);
        bad << "{\"format\":\"carvetto-tool-database\",\"version\":1,\"machines\":[{\"id\":\"m\",\"name\":\"M\","
               "\"rapid_mm_min\":1e999}],\"groups\":[],\"materials\":[],\"tools\":[]}";
    }
    const std::string refused = panel->useSetupFile(base / "bad.json", panel->activeMachineId());
    TS_REQUIRE(!refused.empty(), "a file with a non-finite number was accepted");
    TS_REQUIRE(panel->toolDatabase().findMachine("m") == nullptr, "a refused file changed the database");

    // Restore the starting database for the tests that follow; no file is written any more.
    panel->useSetupFile(std::filesystem::path(), std::string());
    panel->applySetup(original, std::string());
    TS_REQUIRE(panel->toolDatabase() == original, "the starting database was not restored");
    return {};
}

}  // namespace carvetto::app
