#pragma once

#include <QDialog>
#include <QString>
#include <filesystem>
#include <functional>
#include <string>

#include "carvetto/viewmodel/variables_model.hpp"

class QLabel;
class QLineEdit;
class QPushButton;
class QTableWidget;

namespace carvetto::app {

/// Edit > Document Variables (T-0257): lists the document's variables (name, value, uses) and
/// creates, changes, renames, deletes, imports and exports them. Every action is applied at once as
/// one undo step by `viewmodel::VariablesModel` (the dialog is non-modal in spirit: Close just
/// closes it); a refusal (invalid name or value, a variable in use) is shown inline and changes
/// nothing. `onEdited` runs after a successful edit so the window can refresh the views.
class VariablesDialog : public QDialog {
   public:
    VariablesDialog(viewmodel::VariablesModel& model, QWidget* parent = nullptr);

    std::function<void()> onEdited;

    /// The inline message of the last action.
    QString message() const;
    bool lastActionOk() const { return lastOk_; }
    /// Rows currently shown.
    int rowCount() const;

    // --- Scripted access (the self-test drives the real widgets) --------------------------------
    /// Types `name` and `value` into the entry fields and presses Set (create or change).
    bool setVariable(const std::string& name, const std::string& value);
    /// Selects `name`'s row and presses Delete.
    bool deleteVariable(const std::string& name);
    /// Edits the value cell of `name`'s row as the user would.
    bool editValueCell(const std::string& name, const std::string& value);
    bool importFrom(const std::filesystem::path& path);
    bool exportTo(const std::filesystem::path& path);

   private:
    void reload();
    void report(const viewmodel::VariableEdit& result);
    void importDialog();
    void exportDialog();
    void deleteSelected();
    void renameSelected();
    QString selectedName() const;

    viewmodel::VariablesModel& model_;
    QTableWidget* table_{nullptr};
    QLineEdit* nameEdit_{nullptr};
    QLineEdit* valueEdit_{nullptr};
    QPushButton* set_{nullptr};
    QPushButton* delete_{nullptr};
    QPushButton* rename_{nullptr};
    QLabel* status_{nullptr};
    bool lastOk_{true};
    bool loading_{false};
};

}  // namespace carvetto::app

namespace carvetto::app {

class MainWindow;

/// Drives the Document Variables dialog and the toolpath form's calculation boxes through the real
/// widgets (self-test, offscreen). Returns an empty string on success, else the first failure.
/// Replaces the window's document.
std::string variablesSelfTest(MainWindow& window);

}  // namespace carvetto::app
