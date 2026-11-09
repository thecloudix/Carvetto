#pragma once

#include <QDialog>
#include <vector>

#include "carvetto/viewmodel/vector_ops.hpp"

class QAction;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QMenu;

namespace carvetto::app {

class MainWindow;

/// Modal form for Offset Vectors: distance (display units), direction, join type, delete
/// originals. Validation is `viewmodel::validateOffsetOptions`; OK stays open while it fails.
class OffsetDialog : public QDialog {
   public:
    OffsetDialog(document::Units units, const viewmodel::OffsetOptions& initial, QWidget* parent = nullptr);

    /// Options as currently entered (distance converted to millimetres).
    viewmodel::OffsetOptions options() const;
    void setOptions(const viewmodel::OffsetOptions& options);

    void accept() override;

   private:
    document::Units units_;
    QDoubleSpinBox* distance_{nullptr};
    QComboBox* direction_{nullptr};
    QComboBox* join_{nullptr};
    QCheckBox* deleteOriginals_{nullptr};
};

/// The Edit > Vectors submenu: Weld, Subtract, Intersect, Exclude and Offset. Entries are enabled
/// from the selection each time the menu opens (`viewmodel::canWeld` etc.). Thin glue: the work
/// and its error messages come from `viewmodel/vector_ops.hpp`.
class VectorOpsMenu {
   public:
    VectorOpsMenu(MainWindow& window, QMenu* editMenu);

    /// Re-evaluates which entries are enabled for the current selection.
    void refresh();
    QAction* booleanAction(viewmodel::BooleanKind kind) const;
    QAction* offsetAction() const { return offset_; }

    /// Runs an operation on the window's document and refreshes the window on success. With
    /// `showErrors` a failure is reported in a message box; the result is returned either way.
    viewmodel::VectorOpResult runBoolean(viewmodel::BooleanKind kind, bool showErrors = true);
    viewmodel::VectorOpResult runOffset(const viewmodel::OffsetOptions& options, bool showErrors = true);

   private:
    /// Vector Validator dialog: Search / Fix / Select Offenders on the selection, else all visible vectors.
    void runValidator();
    void report(const viewmodel::VectorOpResult& result, bool showErrors);

    MainWindow& window_;
    QAction* weld_{nullptr};
    QAction* subtract_{nullptr};
    QAction* intersect_{nullptr};
    QAction* exclude_{nullptr};
    QAction* offset_{nullptr};
    viewmodel::OffsetOptions lastOffset_;
    bool lastVCarvingMode_{false};
};

}  // namespace carvetto::app
