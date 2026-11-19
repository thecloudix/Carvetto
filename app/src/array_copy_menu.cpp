#include "array_copy_menu.hpp"

#include <QAction>
#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QMenu>
#include <QMessageBox>
#include <QSpinBox>
#include <QString>
#include <QVBoxLayout>
#include <QVariant>
#include <cmath>
#include <string>

#include "carvetto/document/units.hpp"
#include "main_window.hpp"
#include "nest_dialog.hpp"
#include "plate_production_dialog.hpp"

namespace carvetto::app {

using viewmodel::AlongVectorOptions;
using viewmodel::ArrayOptions;
using viewmodel::CircularOptions;
using viewmodel::CopyResult;

namespace {

constexpr double kPi = 3.14159265358979323846;

QDoubleSpinBox* lengthBox(QWidget* parent, document::Units units, double minMm, double maxMm) {
    auto* box = new QDoubleSpinBox(parent);
    box->setDecimals(4);
    box->setRange(document::toDisplay(minMm, units), document::toDisplay(maxMm, units));
    const std::string_view suffix = document::unitSuffix(units);
    box->setSuffix(QStringLiteral(" ") + QString::fromUtf8(suffix.data(), static_cast<int>(suffix.size())));
    return box;
}

void finishDialog(QDialog* dialog, QFormLayout* form) {
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, dialog);
    QObject::connect(buttons, &QDialogButtonBox::accepted, dialog, [dialog]() { dialog->accept(); });
    QObject::connect(buttons, &QDialogButtonBox::rejected, dialog, [dialog]() { dialog->reject(); });
    auto* layout = new QVBoxLayout(dialog);
    layout->addLayout(form);
    layout->addWidget(buttons);
}

bool acceptIfValid(QDialog* dialog, const std::string& problem) {
    if (!problem.empty()) {
        QMessageBox::warning(dialog, dialog->windowTitle(), QString::fromStdString(problem));
        return false;
    }
    return true;
}

}  // namespace

// --- Array --------------------------------------------------------------------------------------

ArrayCopyDialog::ArrayCopyDialog(document::Units units, const ArrayOptions& initial, QWidget* parent)
    : QDialog(parent), units_(units) {
    setWindowTitle(QCoreApplication::translate("ArrayCopyMenu", "Array Copy"));
    rows_ = new QSpinBox(this);
    rows_->setRange(1, viewmodel::kMaxArrayLines);
    columns_ = new QSpinBox(this);
    columns_->setRange(1, viewmodel::kMaxArrayLines);
    spacing_ = new QComboBox(this);
    spacing_->addItem(QCoreApplication::translate("ArrayCopyMenu", "Offset (step between copies)"), static_cast<int>(viewmodel::ArraySpacing::Offset));
    spacing_->addItem(QCoreApplication::translate("ArrayCopyMenu", "Gap (between copies)"), static_cast<int>(viewmodel::ArraySpacing::Gap));
    x_ = lengthBox(this, units, -1.0e6, 1.0e6);
    y_ = lengthBox(this, units, -1.0e6, 1.0e6);
    rowShift_ = lengthBox(this, units, -1.0e6, 1.0e6);
    columnShift_ = lengthBox(this, units, -1.0e6, 1.0e6);
    symmetry_ = new QComboBox(this);
    symmetry_->addItem(QCoreApplication::translate("ArrayCopyMenu", "None"), static_cast<int>(viewmodel::ArraySymmetry::None));
    symmetry_->addItem(QCoreApplication::translate("ArrayCopyMenu", "Mirror alternate columns"), static_cast<int>(viewmodel::ArraySymmetry::MirrorColumns));
    symmetry_->addItem(QCoreApplication::translate("ArrayCopyMenu", "Mirror alternate rows"), static_cast<int>(viewmodel::ArraySymmetry::MirrorRows));
    symmetry_->addItem(QCoreApplication::translate("ArrayCopyMenu", "Mirror both"), static_cast<int>(viewmodel::ArraySymmetry::MirrorBoth));
    symmetry_->addItem(QCoreApplication::translate("ArrayCopyMenu", "Rotate alternate copies 180"), static_cast<int>(viewmodel::ArraySymmetry::Rotate180));
    group_ = new QCheckBox(QCoreApplication::translate("ArrayCopyMenu", "Group copies"), this);
    auto* form = new QFormLayout;
    form->addRow(QCoreApplication::translate("ArrayCopyMenu", "Rows"), rows_);
    form->addRow(QCoreApplication::translate("ArrayCopyMenu", "Columns"), columns_);
    form->addRow(QCoreApplication::translate("ArrayCopyMenu", "Spacing"), spacing_);
    form->addRow(QCoreApplication::translate("ArrayCopyMenu", "X"), x_);
    form->addRow(QCoreApplication::translate("ArrayCopyMenu", "Y"), y_);
    form->addRow(QCoreApplication::translate("ArrayCopyMenu", "Row displacement"), rowShift_);
    form->addRow(QCoreApplication::translate("ArrayCopyMenu", "Column displacement"), columnShift_);
    form->addRow(QCoreApplication::translate("ArrayCopyMenu", "Symmetry"), symmetry_);
    form->addRow(group_);
    finishDialog(this, form);
    setOptions(initial);
}

ArrayOptions ArrayCopyDialog::options() const {
    ArrayOptions o;
    o.rows = rows_->value();
    o.columns = columns_->value();
    o.spacing = static_cast<viewmodel::ArraySpacing>(spacing_->currentData().toInt());
    o.xMm = document::fromDisplay(x_->value(), units_);
    o.yMm = document::fromDisplay(y_->value(), units_);
    o.rowDisplacementMm = document::fromDisplay(rowShift_->value(), units_);
    o.columnDisplacementMm = document::fromDisplay(columnShift_->value(), units_);
    o.symmetry = static_cast<viewmodel::ArraySymmetry>(symmetry_->currentData().toInt());
    o.groupCopies = group_->isChecked();
    return o;
}

void ArrayCopyDialog::setOptions(const ArrayOptions& o) {
    rows_->setValue(o.rows);
    columns_->setValue(o.columns);
    spacing_->setCurrentIndex(spacing_->findData(static_cast<int>(o.spacing)));
    x_->setValue(document::toDisplay(o.xMm, units_));
    y_->setValue(document::toDisplay(o.yMm, units_));
    rowShift_->setValue(document::toDisplay(o.rowDisplacementMm, units_));
    columnShift_->setValue(document::toDisplay(o.columnDisplacementMm, units_));
    symmetry_->setCurrentIndex(symmetry_->findData(static_cast<int>(o.symmetry)));
    group_->setChecked(o.groupCopies);
}

void ArrayCopyDialog::accept() {
    if (acceptIfValid(this, viewmodel::validateArrayOptions(options()))) {
        QDialog::accept();
    }
}

// --- Circular -----------------------------------------------------------------------------------

CircularCopyDialog::CircularCopyDialog(document::Units units, const CircularOptions& initial, QWidget* parent)
    : QDialog(parent), units_(units) {
    setWindowTitle(QCoreApplication::translate("ArrayCopyMenu", "Circular Copy"));
    centreX_ = lengthBox(this, units, -1.0e6, 1.0e6);
    centreY_ = lengthBox(this, units, -1.0e6, 1.0e6);
    count_ = new QSpinBox(this);
    count_->setRange(2, 100000);
    mode_ = new QComboBox(this);
    mode_->addItem(QCoreApplication::translate("ArrayCopyMenu", "Total angle"), static_cast<int>(viewmodel::CircularAngleMode::TotalAngle));
    mode_->addItem(QCoreApplication::translate("ArrayCopyMenu", "Step angle (negative = counter-clockwise)"),
                   static_cast<int>(viewmodel::CircularAngleMode::StepAngle));
    angle_ = new QDoubleSpinBox(this);
    angle_->setDecimals(3);
    angle_->setRange(-360.0, 360.0);
    angle_->setSuffix(QCoreApplication::translate("CircularCopyDialog", " deg"));
    rotate_ = new QCheckBox(QCoreApplication::translate("ArrayCopyMenu", "Rotate copies"), this);
    auto* form = new QFormLayout;
    form->addRow(QCoreApplication::translate("ArrayCopyMenu", "Centre X"), centreX_);
    form->addRow(QCoreApplication::translate("ArrayCopyMenu", "Centre Y"), centreY_);
    form->addRow(QCoreApplication::translate("ArrayCopyMenu", "Copies (including the original)"), count_);
    form->addRow(QCoreApplication::translate("ArrayCopyMenu", "Angle mode"), mode_);
    form->addRow(QCoreApplication::translate("ArrayCopyMenu", "Angle"), angle_);
    form->addRow(rotate_);
    finishDialog(this, form);
    setOptions(initial);
}

CircularOptions CircularCopyDialog::options() const {
    CircularOptions o;
    o.centreMm = {document::fromDisplay(centreX_->value(), units_), document::fromDisplay(centreY_->value(), units_)};
    o.count = count_->value();
    o.mode = static_cast<viewmodel::CircularAngleMode>(mode_->currentData().toInt());
    o.angleRad = angle_->value() * kPi / 180.0;
    o.rotateCopies = rotate_->isChecked();
    return o;
}

void CircularCopyDialog::setOptions(const CircularOptions& o) {
    centreX_->setValue(document::toDisplay(o.centreMm.x, units_));
    centreY_->setValue(document::toDisplay(o.centreMm.y, units_));
    count_->setValue(o.count);
    mode_->setCurrentIndex(mode_->findData(static_cast<int>(o.mode)));
    angle_->setValue(o.angleRad * 180.0 / kPi);
    rotate_->setChecked(o.rotateCopies);
}

void CircularCopyDialog::accept() {
    if (acceptIfValid(this, viewmodel::validateCircularOptions(options()))) {
        QDialog::accept();
    }
}

// --- Along vectors ------------------------------------------------------------------------------

AlongVectorsDialog::AlongVectorsDialog(document::Units units, const AlongVectorOptions& initial, QWidget* parent)
    : QDialog(parent), units_(units), guide_(initial.guide) {
    setWindowTitle(QCoreApplication::translate("ArrayCopyMenu", "Copy Along Vectors"));
    kind_ = new QComboBox(this);
    kind_->addItem(QCoreApplication::translate("ArrayCopyMenu", "Copy objects"), static_cast<int>(viewmodel::AlongCopyKind::Objects));
    kind_->addItem(QCoreApplication::translate("ArrayCopyMenu", "Copy circles"), static_cast<int>(viewmodel::AlongCopyKind::Circles));
    diameter_ = lengthBox(this, units, 0.0001, 1.0e6);
    spacing_ = new QComboBox(this);
    spacing_->addItem(QCoreApplication::translate("ArrayCopyMenu", "Distance between copies"), static_cast<int>(viewmodel::AlongSpacing::Distance));
    spacing_->addItem(QCoreApplication::translate("ArrayCopyMenu", "Number of copies"), static_cast<int>(viewmodel::AlongSpacing::Count));
    distance_ = lengthBox(this, units, 0.0001, 1.0e6);
    count_ = new QSpinBox(this);
    count_->setRange(1, 1000000);
    even_ = new QCheckBox(QCoreApplication::translate("ArrayCopyMenu", "Force even spacing"), this);
    align_ = new QCheckBox(QCoreApplication::translate("ArrayCopyMenu", "Align objects to curve"), this);
    newLayer_ = new QCheckBox(QCoreApplication::translate("ArrayCopyMenu", "Create copies on new layer"), this);
    reverse_ = new QCheckBox(QCoreApplication::translate("ArrayCopyMenu", "Reverse direction"), this);
    auto* form = new QFormLayout;
    form->addRow(QCoreApplication::translate("ArrayCopyMenu", "Copy"), kind_);
    form->addRow(QCoreApplication::translate("ArrayCopyMenu", "Circle diameter"), diameter_);
    form->addRow(QCoreApplication::translate("ArrayCopyMenu", "Spacing"), spacing_);
    form->addRow(QCoreApplication::translate("ArrayCopyMenu", "Distance"), distance_);
    form->addRow(QCoreApplication::translate("ArrayCopyMenu", "Number of copies"), count_);
    form->addRow(even_);
    form->addRow(align_);
    form->addRow(newLayer_);
    form->addRow(reverse_);
    finishDialog(this, form);
    setOptions(initial);
}

AlongVectorOptions AlongVectorsDialog::options() const {
    AlongVectorOptions o;
    o.guide = guide_;
    o.kind = static_cast<viewmodel::AlongCopyKind>(kind_->currentData().toInt());
    o.circleDiameterMm = document::fromDisplay(diameter_->value(), units_);
    o.spacing = static_cast<viewmodel::AlongSpacing>(spacing_->currentData().toInt());
    o.distanceMm = document::fromDisplay(distance_->value(), units_);
    o.count = count_->value();
    o.forceEvenSpacing = even_->isChecked();
    o.alignToCurve = align_->isChecked();
    o.newLayer = newLayer_->isChecked();
    o.reverse = reverse_->isChecked();
    return o;
}

void AlongVectorsDialog::setOptions(const AlongVectorOptions& o) {
    guide_ = o.guide;
    kind_->setCurrentIndex(kind_->findData(static_cast<int>(o.kind)));
    diameter_->setValue(document::toDisplay(o.circleDiameterMm, units_));
    spacing_->setCurrentIndex(spacing_->findData(static_cast<int>(o.spacing)));
    distance_->setValue(document::toDisplay(o.distanceMm, units_));
    count_->setValue(o.count);
    even_->setChecked(o.forceEvenSpacing);
    align_->setChecked(o.alignToCurve);
    newLayer_->setChecked(o.newLayer);
    reverse_->setChecked(o.reverse);
}

void AlongVectorsDialog::accept() {
    if (acceptIfValid(this, viewmodel::validateAlongVectorOptions(options()))) {
        QDialog::accept();
    }
}

// --- Menu ---------------------------------------------------------------------------------------

CopiesMenu::CopiesMenu(MainWindow& window, QMenu* editMenu) : window_(window) {
    QMenu* menu = editMenu->addMenu(QCoreApplication::translate("ArrayCopyMenu", "&Copies"));
    array_ = menu->addAction(QCoreApplication::translate("ArrayCopyMenu", "&Array Copy..."));
    circular_ = menu->addAction(QCoreApplication::translate("ArrayCopyMenu", "&Circular Copy..."));
    along_ = menu->addAction(QCoreApplication::translate("ArrayCopyMenu", "Copy Along &Vectors..."));
    QObject::connect(array_, &QAction::triggered, menu, [this]() {
        ArrayCopyDialog dialog(window_.currentDocument().job().displayUnits, lastArray_, &window_);
        if (dialog.exec() == QDialog::Accepted) {
            lastArray_ = dialog.options();
            runArray(lastArray_);
        }
    });
    QObject::connect(circular_, &QAction::triggered, menu, [this]() {
        CircularCopyDialog dialog(window_.currentDocument().job().displayUnits, lastCircular_, &window_);
        if (dialog.exec() == QDialog::Accepted) {
            lastCircular_ = dialog.options();
            runCircular(lastCircular_);
        }
    });
    QObject::connect(along_, &QAction::triggered, menu, [this]() {
        lastAlong_.guide = viewmodel::defaultGuideObject(window_.currentDocument());
        AlongVectorsDialog dialog(window_.currentDocument().job().displayUnits, lastAlong_, &window_);
        if (dialog.exec() == QDialog::Accepted) {
            lastAlong_ = dialog.options();
            runAlong(lastAlong_);
        }
    });
    menu->addSeparator();
    plate_ = menu->addAction(QCoreApplication::translate("ArrayCopyMenu", "&Plate Production (Data Merge)..."));
    QObject::connect(plate_, &QAction::triggered, menu, [this]() { showPlateProduction(window_); });
    nest_ = editMenu->addAction(QCoreApplication::translate("ArrayCopyMenu", "&Nest Selected Vectors..."));
    QObject::connect(nest_, &QAction::triggered, menu, [this]() {
        NestDialog dialog(window_.currentDocument(), lastNest_, &window_);
        if (dialog.exec() == QDialog::Accepted) {
            lastNest_ = dialog.options();
            runNest(lastNest_);
        }
    });
    QObject::connect(menu, &QMenu::aboutToShow, menu, [this]() { refresh(); });
    QObject::connect(editMenu, &QMenu::aboutToShow, menu, [this]() { refresh(); });
    for (QAction* action : {array_, circular_, along_, plate_, nest_}) {
        action->setEnabled(false);  // until the menu opens (the window may not have a document yet)
    }
}

void CopiesMenu::refresh() {
    const document::Document& doc = window_.currentDocument();
    array_->setEnabled(viewmodel::canArrayCopy(doc));
    circular_->setEnabled(viewmodel::canCircularCopy(doc));
    along_->setEnabled(viewmodel::canCopyAlongVectors(doc));
    plate_->setEnabled(viewmodel::canDataMerge(doc));
    nest_->setEnabled(viewmodel::canNestSelection(doc));
}

CopyResult CopiesMenu::runArray(const ArrayOptions& options, bool showErrors) {
    CopyResult result = viewmodel::applyArrayCopy(window_.currentDocument(), options);
    report(result, showErrors);
    return result;
}

CopyResult CopiesMenu::runCircular(const CircularOptions& options, bool showErrors) {
    CopyResult result = viewmodel::applyCircularCopy(window_.currentDocument(), options);
    report(result, showErrors);
    return result;
}

CopyResult CopiesMenu::runAlong(const AlongVectorOptions& options, bool showErrors) {
    CopyResult result = viewmodel::applyCopyAlongVectors(window_.currentDocument(), options);
    report(result, showErrors);
    return result;
}

viewmodel::NestSelectionResult CopiesMenu::runNest(const viewmodel::NestSelectionOptions& options, bool showErrors) {
    viewmodel::NestSelectionResult result = viewmodel::nestSelection(window_.currentDocument(), options);
    if (result.ok) {
        window_.documentChanged();
        if (!result.message.empty() && showErrors) {
            QMessageBox::information(&window_, QCoreApplication::translate("ArrayCopyMenu", "Nest Selected Vectors"),
                                     QString::fromStdString(result.message));
        }
    } else if (showErrors) {
        QMessageBox::warning(&window_, QCoreApplication::translate("ArrayCopyMenu", "Nest Selected Vectors"),
                             QString::fromStdString(result.message));
    }
    return result;
}

void CopiesMenu::report(const CopyResult& result, bool showErrors) {
    if (result.ok) {
        window_.documentChanged();
    } else if (showErrors) {
        QMessageBox::warning(&window_, QCoreApplication::translate("ArrayCopyMenu", "Copies"), QString::fromStdString(result.message));
    }
}

}  // namespace carvetto::app
