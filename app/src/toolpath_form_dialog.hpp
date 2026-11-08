#pragma once

#include <QDialog>
#include <QString>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "carvetto/document/ids.hpp"
#include "carvetto/document/toolpath_record.hpp"
#include "carvetto/tooling/tool_database.hpp"
#include "carvetto/viewmodel/toolpath_form.hpp"

class QCheckBox;
class QComboBox;
class QDialogButtonBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;
class QVBoxLayout;
class QWidget;

namespace carvetto::app {

/// Human name of an operation ("Profile", "V-Carve", ...).
const char* operationLabel(document::ToolpathOperation operation);

/// What the form needs to know about the document, copied so the dialog never holds a reference to
/// it (the document can be replaced while the dialog is open; the panel then closes the dialog).
struct FormDocumentInfo {
    struct LayerInfo {
        document::LayerId id;
        std::string name;
    };
    std::vector<LayerInfo> layers;
    /// The current selection when the dialog opened (the "Selected objects" choice).
    std::vector<document::ObjectId> selection;
    /// Number of vectors a selector selects now (the readout). Optional.
    std::function<std::size_t(const viewmodel::VectorSelector&)> countVectors;
};

/// The operation form generated from a `viewmodel::ToolpathForm`: name, tool picker (tool database),
/// vectors (selector), and the numeric fields and options of the operation grouped as the viewmodel
/// lists them. All validation is the viewmodel's: a bad entry shows its message inline and the
/// form keeps its old value; OK is disabled while anything is invalid or incomplete.
class ToolpathFormDialog : public QDialog {
   public:
    ToolpathFormDialog(viewmodel::ToolpathForm form, tooling::ToolDatabase db, FormDocumentInfo info, bool editing,
                       QWidget* parent = nullptr);

    const viewmodel::ToolpathForm& form() const { return form_; }
    /// The inline message (last rejection, else what is still missing); empty when complete.
    QString message() const { return message_; }
    bool canAccept() const;

    // --- Scripted access (the self-test drives the real widgets) --------------------------------
    /// Types `text` into the field's edit (as the user would). False when there is no such field
    /// or the viewmodel rejected it.
    bool typeField(const std::string& id, const std::string& text);
    bool chooseOption(const std::string& id, int option);
    bool pickTool(const std::string& toolId);
    QLineEdit* fieldEdit(const std::string& id) const;
    QPushButton* okButton() const { return ok_; }

   private:
    void buildStatic();
    void rebuildParameters();
    void refreshPlaceholders();
    void refreshStatus();
    void refreshCuttingData();
    void refreshVectors();
    void applySelectorFromWidgets();
    void applyTool();
    void applyClearance();
    void setMessage(const std::string& text, bool error);

    viewmodel::ToolpathForm form_;
    tooling::ToolDatabase db_;
    FormDocumentInfo info_;
    bool editing_{false};
    bool building_{false};
    QString message_;
    std::set<std::string> badFields_;

    QLineEdit* nameEdit_{nullptr};
    QComboBox* toolCombo_{nullptr};
    QComboBox* dataCombo_{nullptr};
    QLabel* toolInfo_{nullptr};
    QListWidget* clearanceList_{nullptr};
    QComboBox* clearanceCombo_{nullptr};
    QWidget* clearanceBox_{nullptr};
    QComboBox* baseCombo_{nullptr};
    QListWidget* layerList_{nullptr};
    QCheckBox* openCheck_{nullptr};
    QCheckBox* closedCheck_{nullptr};
    QCheckBox* circlesCheck_{nullptr};
    QLineEdit* diameterEdit_{nullptr};
    QLabel* countLabel_{nullptr};
    QWidget* paramHost_{nullptr};
    QVBoxLayout* paramLayout_{nullptr};
    QLabel* status_{nullptr};
    QPushButton* ok_{nullptr};
    std::map<std::string, QLineEdit*> edits_;
    std::map<std::string, QWidget*> choiceWidgets_;
};

}  // namespace carvetto::app
