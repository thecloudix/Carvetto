#pragma once

#include <QDialog>
#include <memory>
#include <optional>
#include <string>

#include "carvetto/document/units.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/viewmodel/node_edit.hpp"

class QCheckBox;
class QLineEdit;
class QMenu;
class QPoint;
class QWidget;

namespace carvetto::app {

class MainWindow;

/// Properties of the selected node(s): exact X / Y in display units, typed absolute or relative
/// (T-0206). Parsing and range rules are `viewmodel::parseNumber`; OK stays open while an entry is
/// invalid.
class NodePropertiesDialog : public QDialog {
   public:
    struct Values {
        std::optional<double> xMm;
        std::optional<double> yMm;
        bool relative{false};
    };

    NodePropertiesDialog(document::Units units, std::optional<geometry::Vec2> current, QWidget* parent = nullptr);

    /// The entered values (millimetres); false with `error` set when an entry is invalid or none
    /// was given.
    bool values(Values* out, std::string* error) const;
    /// Test hook: fills the fields.
    void setTexts(const QString& x, const QString& y, bool relative);

    void accept() override;

   private:
    document::Units units_;
    QLineEdit* x_{nullptr};
    QLineEdit* y_{nullptr};
    QCheckBox* relative_{nullptr};
};

/// The node-editing menus of the NodeEdit tool: the span and node context menus of the feature
/// inventory (built from `NodeEditor::canRun`) and the Properties dialog. Thin glue; every
/// operation and its message come from `viewmodel::NodeEditor`.
class NodeEditMenu {
   public:
    explicit NodeEditMenu(MainWindow& window);

    /// Right click at widget pixel `px`: selects the span / node under it (unless it already is
    /// selected) and shows the matching menu at `globalPos`. Returns false when nothing editable is
    /// there (no menu).
    bool showContextMenu(const QPoint& globalPos, geometry::Vec2 px);
    /// The menu `showContextMenu` would show for a handle of this kind (not executed; caller owns).
    /// Entries are enabled from the current selection. `posMm` is where Insert Point acts.
    QMenu* buildMenu(viewmodel::HandleType kind, geometry::Vec2 posMm);

    /// Runs an operation on the window's document, refreshes the window and reports a refusal in
    /// the status bar (or a message box with `showErrors`).
    viewmodel::NodeEditResult run(viewmodel::NodeOp op, const viewmodel::NodeOpArgs& args = {},
                                  bool showErrors = false);
    /// Shows the Properties dialog for the selected nodes and applies it.
    void showProperties();
    /// Applies dialog values without a dialog.
    viewmodel::NodeEditResult applyProperties(const NodePropertiesDialog::Values& values);
    /// Switches to the Select tool (the menu's Exit).
    void exit();

   private:
    void report(const viewmodel::NodeEditResult& result, bool showErrors);

    MainWindow& window_;
};

}  // namespace carvetto::app
