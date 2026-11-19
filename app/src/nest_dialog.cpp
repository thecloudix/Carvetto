#include "nest_dialog.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QMessageBox>
#include <QSpinBox>
#include <QString>
#include <QVBoxLayout>
#include <QVariant>
#include <string>

namespace carvetto::app {

using viewmodel::NestArea;
using viewmodel::NestSelectionOptions;

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

}  // namespace

NestDialog::NestDialog(const document::Document& doc, const NestSelectionOptions& initial, QWidget* parent)
    : QDialog(parent), units_(doc.job().displayUnits) {
    setWindowTitle(QCoreApplication::translate("NestDialog", "Nest Selected Vectors"));
    area_ = new QComboBox(this);
    area_->addItem(QCoreApplication::translate("NestDialog", "New sheets"), static_cast<int>(NestArea::NewSheets));
    area_->addItem(QCoreApplication::translate("NestDialog", "Active sheet"), static_cast<int>(NestArea::ActiveSheet));
    area_->addItem(QCoreApplication::translate("NestDialog", "Nest boundary layer"), static_cast<int>(NestArea::Boundary));
    sheetLength_ = lengthBox(this, units_, 0.001, 1.0e6);
    sheetWidth_ = lengthBox(this, units_, 0.001, 1.0e6);
    boundaryLayer_ = new QComboBox(this);
    for (const document::Layer& layer : doc.layers()) {
        boundaryLayer_->addItem(QString::fromStdString(layer.name), QVariant::fromValue(static_cast<qulonglong>(layer.id.value)));
    }
    toolDia_ = lengthBox(this, units_, 0.0, 1.0e6);
    clearance_ = lengthBox(this, units_, 0.0, 1.0e6);
    border_ = lengthBox(this, units_, 0.0, 1.0e6);
    rotation_ = new QDoubleSpinBox(this);
    rotation_->setDecimals(2);
    rotation_->setRange(0.0, 360.0);
    rotation_->setSuffix(QCoreApplication::translate("NestDialog", " deg"));
    rotation_->setSpecialValueText(QCoreApplication::translate("NestDialog", "Off"));
    mirror_ = new QCheckBox(QCoreApplication::translate("NestDialog", "Mirror parts to find best fit"), this);
    inside_ = new QCheckBox(QCoreApplication::translate("NestDialog", "Allow parts inside other parts"), this);
    corner_ = new QComboBox(this);
    corner_->addItem(QCoreApplication::translate("NestDialog", "Bottom left"), static_cast<int>(geometry::NestCorner::BottomLeft));
    corner_->addItem(QCoreApplication::translate("NestDialog", "Bottom right"), static_cast<int>(geometry::NestCorner::BottomRight));
    corner_->addItem(QCoreApplication::translate("NestDialog", "Top left"), static_cast<int>(geometry::NestCorner::TopLeft));
    corner_->addItem(QCoreApplication::translate("NestDialog", "Top right"), static_cast<int>(geometry::NestCorner::TopRight));
    direction_ = new QComboBox(this);
    direction_->addItem(QCoreApplication::translate("NestDialog", "Along X"), static_cast<int>(geometry::NestDirection::AlongX));
    direction_->addItem(QCoreApplication::translate("NestDialog", "Along Y"), static_cast<int>(geometry::NestDirection::AlongY));
    copies_ = new QSpinBox(this);
    copies_->setRange(1, viewmodel::kMaxNestCopies);
    remove_ = new QCheckBox(QCoreApplication::translate("NestDialog", "Remove original parts"), this);

    auto* form = new QFormLayout;
    form->addRow(QCoreApplication::translate("NestDialog", "Nest onto"), area_);
    form->addRow(QCoreApplication::translate("NestDialog", "Sheet length"), sheetLength_);
    form->addRow(QCoreApplication::translate("NestDialog", "Sheet width"), sheetWidth_);
    form->addRow(QCoreApplication::translate("NestDialog", "Boundary layer"), boundaryLayer_);
    form->addRow(QCoreApplication::translate("NestDialog", "Tool diameter"), toolDia_);
    form->addRow(QCoreApplication::translate("NestDialog", "Clearance"), clearance_);
    form->addRow(QCoreApplication::translate("NestDialog", "Border gap"), border_);
    form->addRow(QCoreApplication::translate("NestDialog", "Rotate parts in steps of"), rotation_);
    form->addRow(mirror_);
    form->addRow(inside_);
    form->addRow(QCoreApplication::translate("NestDialog", "Nest from"), corner_);
    form->addRow(QCoreApplication::translate("NestDialog", "Nest direction"), direction_);
    form->addRow(QCoreApplication::translate("NestDialog", "Copies of each part"), copies_);
    form->addRow(remove_);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, [this]() { accept(); });
    connect(buttons, &QDialogButtonBox::rejected, this, [this]() { reject(); });
    auto* layout = new QVBoxLayout(this);
    layout->addLayout(form);
    layout->addWidget(buttons);
    connect(area_, &QComboBox::currentIndexChanged, this, [this]() { updateEnabled(); });
    setOptions(initial);
}

void NestDialog::updateEnabled() {
    const auto area = static_cast<NestArea>(area_->currentData().toInt());
    sheetLength_->setEnabled(area == NestArea::NewSheets);
    sheetWidth_->setEnabled(area == NestArea::NewSheets);
    boundaryLayer_->setEnabled(area == NestArea::Boundary);
}

NestSelectionOptions NestDialog::options() const {
    NestSelectionOptions o;
    o.area = static_cast<NestArea>(area_->currentData().toInt());
    o.sheets = {{document::fromDisplay(sheetLength_->value(), units_), document::fromDisplay(sheetWidth_->value(), units_)}};
    if (boundaryLayer_->currentIndex() >= 0) {
        o.boundaryLayer = document::LayerId{static_cast<std::uint64_t>(boundaryLayer_->currentData().toULongLong())};
    }
    o.toolDiameterMm = document::fromDisplay(toolDia_->value(), units_);
    o.clearanceMm = document::fromDisplay(clearance_->value(), units_);
    o.borderGapMm = document::fromDisplay(border_->value(), units_);
    o.rotationStepRad = rotation_->value() * kPi / 180.0;
    o.mirror = mirror_->isChecked();
    o.allowInside = inside_->isChecked();
    o.corner = static_cast<geometry::NestCorner>(corner_->currentData().toInt());
    o.direction = static_cast<geometry::NestDirection>(direction_->currentData().toInt());
    o.copies = copies_->value();
    o.removeOriginals = remove_->isChecked();
    return o;
}

void NestDialog::setOptions(const NestSelectionOptions& o) {
    area_->setCurrentIndex(area_->findData(static_cast<int>(o.area)));
    const geometry::NestSheet sheet = o.sheets.empty() ? geometry::NestSheet{} : o.sheets.front();
    sheetLength_->setValue(document::toDisplay(sheet.lengthMm, units_));
    sheetWidth_->setValue(document::toDisplay(sheet.widthMm, units_));
    const int layerIndex = boundaryLayer_->findData(QVariant::fromValue(static_cast<qulonglong>(o.boundaryLayer.value)));
    if (layerIndex >= 0) boundaryLayer_->setCurrentIndex(layerIndex);
    toolDia_->setValue(document::toDisplay(o.toolDiameterMm, units_));
    clearance_->setValue(document::toDisplay(o.clearanceMm, units_));
    border_->setValue(document::toDisplay(o.borderGapMm, units_));
    rotation_->setValue(o.rotationStepRad * 180.0 / kPi);
    mirror_->setChecked(o.mirror);
    inside_->setChecked(o.allowInside);
    corner_->setCurrentIndex(corner_->findData(static_cast<int>(o.corner)));
    direction_->setCurrentIndex(direction_->findData(static_cast<int>(o.direction)));
    copies_->setValue(o.copies);
    remove_->setChecked(o.removeOriginals);
    updateEnabled();
}

void NestDialog::accept() {
    const std::string problem = viewmodel::validateNestOptions(options());
    if (!problem.empty()) {
        QMessageBox::warning(this, windowTitle(), QString::fromStdString(problem));
        return;
    }
    QDialog::accept();
}

}  // namespace carvetto::app
