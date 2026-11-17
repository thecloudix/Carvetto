#pragma once

#include <QDialog>
#include <QString>
#include <string>

#include "carvetto/document/toolpath_record.hpp"
#include "carvetto/tooling/tool_database.hpp"
#include "carvetto/viewmodel/parts_import_form.hpp"

class QCheckBox;
class QComboBox;
class QDialogButtonBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPlainTextEdit;
class QPushButton;

namespace carvetto::app {

class MainWindow;

/// The File > Import Parts List dialog (T-0336): a mapping file and a parts list (each chosen from a file or pasted),
/// a tool per Profile / Drilling / Pocket, "toolpaths" and "allow partial import" switches, and a live preview
/// (counts and line-numbered diagnostics). All parsing and validation is `viewmodel::PartsImportForm`; OK is
/// enabled only when the form says it can apply. The caller applies the form in one undo step.
class PartsImportDialog : public QDialog {
   public:
    PartsImportDialog(tooling::ToolDatabase db, QWidget* parent = nullptr);

    /// The configured form (texts, tools, switches); the caller calls `apply` on it.
    viewmodel::PartsImportForm& form() { return form_; }
    const viewmodel::PartsImportForm& form() const { return form_; }

    // --- Scripted access (the self-test drives the real widgets) --------------------------------
    void typeMapping(const QString& text);
    void typeParts(const QString& text);
    /// Reads the file into the mapping / parts box as the Browse buttons do; "" on success, else why not.
    std::string chooseMappingFile(const QString& path);
    std::string choosePartsFile(const QString& path);
    /// Picks a tool for an operation (Profile, Drill, Pocket) by tool id; "" = none. False when not offered.
    bool pickTool(document::ToolpathOperation operation, const std::string& toolId);
    void setAllowPartial(bool on);
    void setCreateToolpaths(bool on);

    QListWidget* diagnosticsList() const { return diagnostics_; }
    QLabel* summaryLabel() const { return summary_; }
    QPushButton* okButton() const { return ok_; }
    bool canApply() const;

   private:
    void refresh();
    QComboBox* toolCombo(document::ToolpathOperation operation) const;
    void browse(bool mapping);
    QString loadInto(const QString& path, bool mapping);

    tooling::ToolDatabase db_;
    viewmodel::PartsImportForm form_;
    QPlainTextEdit* mappingEdit_{nullptr};
    QPlainTextEdit* partsEdit_{nullptr};
    QLabel* fileNote_{nullptr};
    QComboBox* profileCombo_{nullptr};
    QComboBox* drillCombo_{nullptr};
    QComboBox* pocketCombo_{nullptr};
    QCheckBox* toolpathsCheck_{nullptr};
    QCheckBox* partialCheck_{nullptr};
    QListWidget* diagnostics_{nullptr};
    QLabel* summary_{nullptr};
    QPushButton* ok_{nullptr};
    bool loading_{false};
};

/// Self-test of the Import Parts List dialog (T-0336): File menu entry, mapping and parts from files and pasted
/// text, diagnostics, partial import, tools, and Apply as one undo step through the real window. Empty on success.
std::string partsImportSelfTest(MainWindow& window);

}  // namespace carvetto::app
