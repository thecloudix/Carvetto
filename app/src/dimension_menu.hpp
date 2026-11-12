#pragma once

#include <QDialog>
#include <string>

#include "carvetto/document/document.hpp"
#include "carvetto/viewmodel/dimension_tool.hpp"

class QAction;
class QCheckBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QMenu;
class QSpinBox;

namespace carvetto::app {

class MainWindow;

/// Options form of the dimension tool and of an existing dimension (T-0255): decimal places,
/// angle decimal places, text height, offset (edit only), reflex angle (edit, angular), custom text,
/// font (the starter font or one embedded in the document; a font file can be added, which embeds
/// it) and the layer (new dimensions). Lengths are in display units; entries are parsed and
/// range-checked by the view model; OK stays open (with the reason in the dialog) while anything is
/// refused.
class DimensionDialog : public QDialog {
   public:
    DimensionDialog(document::Document& doc, const viewmodel::DimensionOptions& initial, bool editing,
                    QWidget* parent = nullptr);

    /// Options as entered; meaningful when `problem()` is empty.
    viewmodel::DimensionOptions options() const;
    /// Why the entries cannot be used (empty when they can).
    std::string problem() const;
    /// Reads, checks and embeds a font file into the document (own undo step) and selects it.
    /// Returns an empty string, or the reason it was refused.
    std::string addFontFile(const QString& path);

    void accept() override;

   private:
    document::Document& doc_;
    bool editing_;
    document::Units units_;
    viewmodel::DimensionOptions initial_;
    QComboBox* kind_{nullptr};
    QCheckBox* vertical_{nullptr};
    QCheckBox* diameter_{nullptr};
    QSpinBox* decimals_{nullptr};
    QSpinBox* angleDecimals_{nullptr};
    QLineEdit* textHeight_{nullptr};
    QLineEdit* offset_{nullptr};
    QCheckBox* reflex_{nullptr};
    QCheckBox* useCustom_{nullptr};
    QLineEdit* customText_{nullptr};
    QComboBox* font_{nullptr};
    QComboBox* layer_{nullptr};
    QLabel* error_{nullptr};
};

/// Edit > Dimensions: the dimension tool, its options, Edit Selected Dimension and Delete Selected
/// Dimension. Thin glue over `viewmodel::DimensionTool`.
class DimensionMenu {
   public:
    DimensionMenu(MainWindow& window, QMenu* editMenu);

    /// Enables the entries from the selection.
    void refresh();
    QAction* toolAction() const { return tool_; }
    QAction* optionsAction() const { return options_; }
    QAction* editAction() const { return edit_; }
    QAction* deleteAction() const { return delete_; }

    /// The dialogs run modally from the menu entries; these do the work after OK (and are what the
    /// self-test calls with options read from a dialog it drove).
    void applyToolOptions(const viewmodel::DimensionOptions& options);
    bool applyEdit(document::ObjectId id, const viewmodel::DimensionOptions& options);
    bool deleteSelected();

   private:
    void openOptions();
    void openEdit();

    MainWindow& window_;
    QAction* tool_{nullptr};
    QAction* options_{nullptr};
    QAction* edit_{nullptr};
    QAction* delete_{nullptr};
};

/// Drives place / edit / delete of each dimension kind through the real window.
std::string dimensionSelfTest(MainWindow& window);

}  // namespace carvetto::app
