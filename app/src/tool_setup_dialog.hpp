#pragma once

#include <QDialog>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

#include "carvetto/document/units.hpp"
#include "carvetto/tooling/tool_database.hpp"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QSpinBox;
class QTableWidget;
class QTabWidget;
class QTreeWidget;

namespace carvetto::app {

/// Editor for the tool database, machines and materials (T-0235). Works on a copy of the database;
/// the caller reads `database()` and `activeMachineId()` after the dialog was accepted. All rules
/// (validation, ids, defaults) live in `viewmodel/tool_setup.hpp`; this class is widgets. Widgets
/// have object names so the self-test can drive them: toolList, toolName, toolType, toolDiameter,
/// toolAngle, toolTip, toolCorner, toolFlutes, toolProfile, toolNotes, cuttingTable, machineList,
/// machineName, machineRapid, ..., materialList, setupErrors.
class ToolSetupDialog : public QDialog {
   public:
    /// `toolpathUses(toolId)` returns the names of the toolpaths that use a tool (for the delete
    /// warning); may be empty.
    ToolSetupDialog(tooling::ToolDatabase db, std::string activeMachineId, document::Units units,
                    std::function<std::vector<std::string>(const std::string&)> toolpathUses, QWidget* parent = nullptr);

    const tooling::ToolDatabase& database() const { return db_; }
    const std::string& activeMachineId() const { return activeMachineId_; }
    /// Suppresses confirmation and error boxes (self-test): confirmations count as "yes".
    void setQuiet(bool quiet) { quiet_ = quiet; }

    // Operations behind the buttons (the self-test clicks the buttons; these are shared).
    void addTool(tooling::ToolType type);
    void duplicateTool();
    void deleteTool();
    void addMachine();
    void duplicateMachine();
    void deleteMachine();
    void addMaterial();
    void duplicateMaterial();
    void deleteMaterial();
    /// Merges a tool-database file into the working copy (existing ids are kept). Returns false and
    /// shows the reason when the file is refused.
    bool importFile(const std::filesystem::path& path);
    bool exportFile(const std::filesystem::path& path);

    void selectTool(const std::string& id);
    void selectMachine(const std::string& id);
    /// Last problem found in an edit (empty when the form is valid).
    QString errorText() const;

    void accept() override;

   private:
    void buildTools(QWidget* page);
    void buildMachines(QWidget* page);
    void buildMaterials(QWidget* page);
    void refreshAll();
    void refreshTools();
    void refreshMachines();
    void refreshMaterials();
    void loadTool();
    void commitTool();
    void loadMachine();
    void commitMachine();
    void setUnits(document::Units units);
    void updateFieldVisibility();
    void fillCuttingTable(const tooling::Tool& tool);
    bool readCuttingTable(tooling::Tool& tool, QString& error);
    void addCuttingEntry();
    void removeCuttingEntry();
    void setError(const QString& text);
    bool confirm(const QString& title, const QString& text);
    std::string currentToolId() const;
    std::string currentMachineId() const;
    std::string currentMaterialId() const;
    double lengthToUi(double mm) const;
    double lengthFromUi(double value) const;

    tooling::ToolDatabase db_;
    std::string activeMachineId_;
    document::Units units_;
    std::function<std::vector<std::string>(const std::string&)> uses_;
    bool quiet_{false};
    bool loading_{false};

    QTabWidget* tabs_{nullptr};
    QComboBox* unitsBox_{nullptr};
    QLabel* errors_{nullptr};

    // Tools tab.
    QTreeWidget* toolList_{nullptr};
    QPushButton* toolDuplicate_{nullptr};
    QPushButton* toolDelete_{nullptr};
    QWidget* toolForm_{nullptr};
    QLineEdit* toolName_{nullptr};
    QComboBox* toolType_{nullptr};
    QDoubleSpinBox* toolDiameter_{nullptr};
    QDoubleSpinBox* toolAngle_{nullptr};
    QDoubleSpinBox* toolTip_{nullptr};
    QDoubleSpinBox* toolCorner_{nullptr};
    QSpinBox* toolFlutes_{nullptr};
    QPlainTextEdit* toolProfile_{nullptr};
    QLineEdit* toolNotes_{nullptr};
    QTableWidget* cuttingTable_{nullptr};
    QComboBox* entryMachine_{nullptr};
    QComboBox* entryMaterial_{nullptr};
    std::vector<QWidget*> angleWidgets_, tipWidgets_, cornerWidgets_, profileWidgets_, diameterWidgets_;

    // Machines tab.
    QTreeWidget* machineList_{nullptr};
    QPushButton* machineDuplicate_{nullptr};
    QPushButton* machineDelete_{nullptr};
    QPushButton* machineUse_{nullptr};
    QWidget* machineForm_{nullptr};
    QLineEdit* machineName_{nullptr};
    QDoubleSpinBox* workX_{nullptr};
    QDoubleSpinBox* workY_{nullptr};
    QDoubleSpinBox* workZ_{nullptr};
    QDoubleSpinBox* rapid_{nullptr};
    QDoubleSpinBox* accelX_{nullptr};
    QDoubleSpinBox* accelY_{nullptr};
    QDoubleSpinBox* accelZ_{nullptr};
    QDoubleSpinBox* accelAngular_{nullptr};
    QDoubleSpinBox* maxSpindle_{nullptr};
    QDoubleSpinBox* maxFeed_{nullptr};
    QLineEdit* posts_{nullptr};
    QComboBox* defaultPost_{nullptr};

    // Materials tab.
    QTreeWidget* materialList_{nullptr};
    QPushButton* materialDuplicate_{nullptr};
    QPushButton* materialDelete_{nullptr};
};

class MainWindow;
/// Self-test of the setup dialogs through the real widgets; returns an empty string on success.
std::string toolSetupSelfTest(MainWindow& window);

}  // namespace carvetto::app
