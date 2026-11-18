#include "rotary_material_dialog.hpp"

#include <QCoreApplication>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QStringList>
#include <QVBoxLayout>
#include <QVariant>

namespace carvetto::app {

using document::ZZero;

namespace {

QDoubleSpinBox* makeBox(QWidget* parent, const char* name, double minimum) {
    auto* box = new QDoubleSpinBox(parent);
    box->setObjectName(QString::fromLatin1(name));
    box->setDecimals(4);
    box->setRange(minimum, 100000.0);
    box->setKeyboardTracking(false);
    return box;
}

}  // namespace

RotaryMaterialDialog::RotaryMaterialDialog(const viewmodel::RotaryMaterialForm& initial, QWidget* parent)
    : QDialog(parent) {
    setWindowTitle(QCoreApplication::translate("RotaryMaterialDialog", "Material Setup (rotary)"));
    diameter_ = makeBox(this, "materialDiameter", 0.0);
    gapOutside_ = makeBox(this, "gapOutside", 0.0);
    gapInside_ = makeBox(this, "gapInside", 0.0);
    zZero_ = new QComboBox(this);
    zZero_->setObjectName(QStringLiteral("materialZZero"));
    zZero_->addItem(QCoreApplication::translate("RotaryMaterialDialog", "Cylinder surface"), QVariant(static_cast<int>(ZZero::MaterialSurface)));
    zZero_->addItem(QCoreApplication::translate("RotaryMaterialDialog", "Cylinder axis (centre)"), QVariant(static_cast<int>(ZZero::CylinderAxis)));
    thickness_ = new QLabel(this);
    thickness_->setObjectName(QStringLiteral("modelThickness"));
    errors_ = new QLabel(this);
    errors_->setWordWrap(true);
    errors_->setStyleSheet(QStringLiteral("color: #b00020;"));
    errors_->hide();

    auto* form = new QFormLayout;
    form->addRow(QCoreApplication::translate("RotaryMaterialDialog", "Diameter:"), diameter_);
    form->addRow(QCoreApplication::translate("RotaryMaterialDialog", "Side Z zero:"), zZero_);
    form->addRow(QCoreApplication::translate("RotaryMaterialDialog", "Gap outside model:"), gapOutside_);
    form->addRow(QCoreApplication::translate("RotaryMaterialDialog", "Gap inside model:"), gapInside_);
    form->addRow(QCoreApplication::translate("RotaryMaterialDialog", "Model thickness:"), thickness_);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, [this]() { accept(); });
    connect(buttons, &QDialogButtonBox::rejected, this, [this]() { reject(); });
    auto* layout = new QVBoxLayout(this);
    layout->addLayout(form);
    layout->addWidget(errors_);
    layout->addWidget(buttons);

    setForm(initial);
    for (QDoubleSpinBox* box : {diameter_, gapOutside_, gapInside_}) {
        connect(box, &QDoubleSpinBox::valueChanged, this, [this](double) { updateThickness(); });
    }
}

viewmodel::RotaryMaterialForm RotaryMaterialDialog::form() const {
    viewmodel::RotaryMaterialForm f;
    f.units = units_;
    f.diameter = diameter_->value();
    f.gapOutside = gapOutside_->value();
    f.gapInside = gapInside_->value();
    f.zZero = static_cast<ZZero>(zZero_->currentData().toInt());
    return f;
}

void RotaryMaterialDialog::setForm(const viewmodel::RotaryMaterialForm& f) {
    units_ = f.units;
    const QString suffix = QStringLiteral(" ") + QString::fromUtf8(document::unitSuffix(f.units).data(),
                                                                   static_cast<qsizetype>(document::unitSuffix(f.units).size()));
    for (QDoubleSpinBox* box : {diameter_, gapOutside_, gapInside_}) {
        box->setSuffix(suffix);
    }
    diameter_->setValue(f.diameter);
    gapOutside_->setValue(f.gapOutside);
    gapInside_->setValue(f.gapInside);
    const int index = zZero_->findData(QVariant(static_cast<int>(f.zZero)));
    if (index >= 0) {
        zZero_->setCurrentIndex(index);
    }
    updateThickness();
}

void RotaryMaterialDialog::updateThickness() {
    const QString suffix = QStringLiteral(" ") + QString::fromUtf8(document::unitSuffix(units_).data(),
                                                                   static_cast<qsizetype>(document::unitSuffix(units_).size()));
    thickness_->setText(QString::number(viewmodel::rotaryModelThickness(form()), 'f', 4) + suffix);
}

void RotaryMaterialDialog::accept() {
    const viewmodel::RotaryMaterialForm f = form();
    const auto errors = viewmodel::validateRotaryMaterial(f);
    if (!errors.empty()) {
        QStringList lines;
        for (const viewmodel::RotaryMaterialError& e : errors) {
            lines << QString::fromStdString(e.message);
        }
        errors_->setText(lines.join(QStringLiteral("\n")));
        errors_->show();
        return;
    }
    result_ = f;
    errors_->hide();
    QDialog::accept();
}

}  // namespace carvetto::app
