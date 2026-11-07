#pragma once

#include <QDialog>

#include "carvetto/viewmodel/array_copies.hpp"

class QAction;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QMenu;
class QSpinBox;

namespace carvetto::app {

class MainWindow;

/// Modal forms for the three copy operations (T-0209). Lengths are entered in display units and
/// stored in millimetres, angles in degrees and stored in radians. OK stays open while the
/// viewmodel's validation fails.
class ArrayCopyDialog : public QDialog {
   public:
    ArrayCopyDialog(document::Units units, const viewmodel::ArrayOptions& initial, QWidget* parent = nullptr);
    viewmodel::ArrayOptions options() const;
    void setOptions(const viewmodel::ArrayOptions& options);
    void accept() override;

   private:
    document::Units units_;
    QSpinBox* rows_{nullptr};
    QSpinBox* columns_{nullptr};
    QComboBox* spacing_{nullptr};
    QDoubleSpinBox* x_{nullptr};
    QDoubleSpinBox* y_{nullptr};
    QDoubleSpinBox* rowShift_{nullptr};
    QDoubleSpinBox* columnShift_{nullptr};
    QComboBox* symmetry_{nullptr};
    QCheckBox* group_{nullptr};
};

class CircularCopyDialog : public QDialog {
   public:
    CircularCopyDialog(document::Units units, const viewmodel::CircularOptions& initial, QWidget* parent = nullptr);
    viewmodel::CircularOptions options() const;
    void setOptions(const viewmodel::CircularOptions& options);
    void accept() override;

   private:
    document::Units units_;
    QDoubleSpinBox* centreX_{nullptr};
    QDoubleSpinBox* centreY_{nullptr};
    QSpinBox* count_{nullptr};
    QComboBox* mode_{nullptr};
    QDoubleSpinBox* angle_{nullptr};
    QCheckBox* rotate_{nullptr};
};

class AlongVectorsDialog : public QDialog {
   public:
    AlongVectorsDialog(document::Units units, const viewmodel::AlongVectorOptions& initial,
                       QWidget* parent = nullptr);
    viewmodel::AlongVectorOptions options() const;
    void setOptions(const viewmodel::AlongVectorOptions& options);
    void accept() override;

   private:
    document::Units units_;
    document::ObjectId guide_;
    QComboBox* kind_{nullptr};
    QDoubleSpinBox* diameter_{nullptr};
    QComboBox* spacing_{nullptr};
    QDoubleSpinBox* distance_{nullptr};
    QSpinBox* count_{nullptr};
    QCheckBox* even_{nullptr};
    QCheckBox* align_{nullptr};
    QCheckBox* newLayer_{nullptr};
    QCheckBox* reverse_{nullptr};
};

/// The Edit > Copies submenu: Array Copy, Circular Copy and Copy Along Vectors. Entries are
/// enabled from the selection when the menu opens. Thin glue over `viewmodel/array_copies.hpp`.
class CopiesMenu {
   public:
    CopiesMenu(MainWindow& window, QMenu* editMenu);

    void refresh();
    QAction* arrayAction() const { return array_; }
    QAction* circularAction() const { return circular_; }
    QAction* alongAction() const { return along_; }

    viewmodel::CopyResult runArray(const viewmodel::ArrayOptions& options, bool showErrors = true);
    viewmodel::CopyResult runCircular(const viewmodel::CircularOptions& options, bool showErrors = true);
    viewmodel::CopyResult runAlong(const viewmodel::AlongVectorOptions& options, bool showErrors = true);

   private:
    void report(const viewmodel::CopyResult& result, bool showErrors);

    MainWindow& window_;
    QAction* array_{nullptr};
    QAction* circular_{nullptr};
    QAction* along_{nullptr};
    viewmodel::ArrayOptions lastArray_;
    viewmodel::CircularOptions lastCircular_;
    viewmodel::AlongVectorOptions lastAlong_;
};

}  // namespace carvetto::app
