#pragma once

#include <QDialog>
#include <functional>
#include <map>
#include <string>

#include "carvetto/viewmodel/transform_form.hpp"
#include "carvetto/viewmodel/transform_ops.hpp"

class QAction;
class QCheckBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QMenu;

namespace carvetto::app {

class MainWindow;

/// Modal numeric form for Move, Scale, Rotate or Mirror (T-0205). Only hosts widgets: labels, live
/// values and validation come from `viewmodel::TransformForm`; OK stays open while an entry is
/// refused. The options are written back to the form.
class TransformDialog : public QDialog {
   public:
    TransformDialog(viewmodel::TransformForm& form, std::function<const document::Document&()> doc, QWidget* parent = nullptr);

    void accept() override;

   private:
    void rebuild();
    void readOptions();
    QString titleFor() const;

    viewmodel::TransformForm& form_;
    std::function<const document::Document&()> doc_;
    QWidget* body_{nullptr};
    std::map<std::string, QLineEdit*> edits_;
    QLabel* message_{nullptr};
    QComboBox* anchor_{nullptr};
    QComboBox* mode_{nullptr};
    QCheckBox* absolute_{nullptr};
    QCheckBox* link_{nullptr};
    QCheckBox* coordinates_{nullptr};
    QComboBox* axis_{nullptr};
    QCheckBox* copy_{nullptr};
    QCheckBox* rotated_{nullptr};
    QCheckBox* material_{nullptr};
    QCheckBox* line_{nullptr};
};

/// The Transform menu: Move (M), Scale (T), Rotate (R), Mirror (H, V; Ctrl = copy, Shift = about
/// the material centre) and the Align submenu (F10 pops it up; F9 centres in the material).
/// Entries are enabled from the selection each time the menu opens. Thin glue over
/// `viewmodel/transform_form.hpp` and `viewmodel/transform_ops.hpp`.
class TransformMenu {
   public:
    explicit TransformMenu(MainWindow& window);

    /// Re-evaluates which entries are enabled for the current selection.
    void refresh();

    QAction* moveAction() const { return move_; }
    QAction* scaleAction() const { return scale_; }
    QAction* rotateAction() const { return rotate_; }
    QAction* mirrorAction(viewmodel::MirrorAxis axis, bool copy, bool aboutMaterial) const;
    QAction* alignAction(viewmodel::AlignKind kind) const;
    QAction* centreAction(viewmodel::CentreAxes axes) const;
    QAction* spaceAction(viewmodel::SpaceDirection direction) const;
    QAction* spaceSelectionOrderAction() const { return spaceOrder_; }
    QAction* spaceInsideAction() const { return spaceInside_; }

    /// Persistent form of a kind (its options survive between uses; typed values do not).
    viewmodel::TransformForm& form(viewmodel::TransformFormKind kind);
    /// Applies a form to the window's document and refreshes the window on success. With
    /// `showErrors` a failure is reported in a message box; the result is returned either way.
    viewmodel::TransformResult run(viewmodel::TransformForm& form, bool showErrors = true);
    viewmodel::TransformResult runMirror(const viewmodel::MirrorOptions& options, bool showErrors = true);
    viewmodel::TransformResult runAlign(viewmodel::AlignKind kind, bool showErrors = true);
    viewmodel::TransformResult runCentre(viewmodel::CentreAxes axes, bool showErrors = true);
    viewmodel::TransformResult runSpace(viewmodel::SpaceDirection direction, bool showErrors = true);

   private:
    void openDialog(viewmodel::TransformFormKind kind);
    void report(const viewmodel::TransformResult& result, bool showErrors);
    QAction* add(QMenu* menu, const QString& text, const QString& shortcut);

    MainWindow& window_;
    QMenu* menu_{nullptr};
    QMenu* alignMenu_{nullptr};
    QAction* move_{nullptr};
    QAction* scale_{nullptr};
    QAction* rotate_{nullptr};
    std::map<int, QAction*> mirror_;
    std::map<int, QAction*> align_;
    std::map<int, QAction*> centre_;
    std::map<int, QAction*> space_;
    QAction* spaceOrder_{nullptr};
    QAction* spaceInside_{nullptr};
    std::map<int, viewmodel::TransformForm> forms_;
};

/// Drives the real window through a handle drag, a numeric move, an alignment and undo (self-test
/// step). Returns an empty string on success, else the failure reason. Leaves the document as it
/// found it.
std::string transformSelfTest(MainWindow& window);

}  // namespace carvetto::app
