#include "new_job_dialog.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QString>
#include <QStringList>
#include <QVBoxLayout>
#include <QVariant>
#include <string>

namespace carvetto::app {

using document::Datum;
using document::FlipDirection;
using document::JobType;
using document::RotaryAxis;
using document::Units;
using document::ZZero;

namespace {

QDoubleSpinBox* makeLengthBox(QWidget* parent, double minimum) {
    auto* box = new QDoubleSpinBox(parent);
    box->setDecimals(4);
    box->setRange(minimum, 100000.0);
    box->setKeyboardTracking(false);
    return box;
}

template <class Enum>
void addChoice(QComboBox* combo, const char* text, Enum value) {
    combo->addItem(QString::fromUtf8(text), QVariant(static_cast<int>(value)));
}

template <class Enum>
Enum choice(const QComboBox* combo) {
    return static_cast<Enum>(combo->currentData().toInt());
}

template <class Enum>
void select(QComboBox* combo, Enum value) {
    const int index = combo->findData(QVariant(static_cast<int>(value)));
    if (index >= 0) {
        combo->setCurrentIndex(index);
    }
}

}  // namespace

NewJobDialog::NewJobDialog(const viewmodel::NewJobForm& initial, QWidget* parent) : QDialog(parent) {
    setWindowTitle(QStringLiteral("New Job"));

    type_ = new QComboBox(this);
    addChoice(type_, "Single sided", JobType::SingleSided);
    addChoice(type_, "Double sided", JobType::DoubleSided);
    addChoice(type_, "Rotary", JobType::Rotary);

    units_ = new QComboBox(this);
    addChoice(units_, "Millimetres", Units::Millimetres);
    addChoice(units_, "Inches", Units::Inches);

    width_ = makeLengthBox(this, 0.0);
    height_ = makeLengthBox(this, 0.0);
    thickness_ = makeLengthBox(this, 0.0);
    length_ = makeLengthBox(this, 0.0);
    diameter_ = makeLengthBox(this, 0.0);

    rotaryAxis_ = new QComboBox(this);
    addChoice(rotaryAxis_, "Along X", RotaryAxis::X);
    addChoice(rotaryAxis_, "Along Y", RotaryAxis::Y);

    flip_ = new QComboBox(this);
    addChoice(flip_, "Flip left-right", FlipDirection::Horizontal);
    addChoice(flip_, "Flip top-bottom", FlipDirection::Vertical);

    zZero_ = new QComboBox(this);
    addChoice(zZero_, "Top of material", ZZero::MaterialSurface);
    addChoice(zZero_, "Machine bed", ZZero::MachineBed);
    addChoice(zZero_, "Cylinder axis (rotary)", ZZero::CylinderAxis);

    datum_ = new QComboBox(this);
    addChoice(datum_, "Bottom left", Datum::BottomLeft);
    addChoice(datum_, "Bottom right", Datum::BottomRight);
    addChoice(datum_, "Top left", Datum::TopLeft);
    addChoice(datum_, "Top right", Datum::TopRight);
    addChoice(datum_, "Centre", Datum::Centre);

    useOffset_ = new QCheckBox(QStringLiteral("Place datum at an offset"), this);
    offsetX_ = makeLengthBox(this, -100000.0);
    offsetY_ = makeLengthBox(this, -100000.0);

    errors_ = new QLabel(this);
    errors_->setWordWrap(true);
    errors_->setStyleSheet(QStringLiteral("color: #b00020;"));
    errors_->hide();

    auto* form = new QFormLayout;
    form->addRow(QStringLiteral("Job type:"), type_);
    form->addRow(QStringLiteral("Units:"), units_);
    form->addRow(QStringLiteral("Width (X):"), width_);
    form->addRow(QStringLiteral("Height (Y):"), height_);
    form->addRow(QStringLiteral("Thickness (Z):"), thickness_);
    form->addRow(QStringLiteral("Rotary length:"), length_);
    form->addRow(QStringLiteral("Rotary diameter:"), diameter_);
    form->addRow(QStringLiteral("Rotary axis:"), rotaryAxis_);
    form->addRow(QStringLiteral("Second side:"), flip_);
    form->addRow(QStringLiteral("Z zero:"), zZero_);
    form->addRow(QStringLiteral("XY datum:"), datum_);
    form->addRow(QString(), useOffset_);
    form->addRow(QStringLiteral("Datum X:"), offsetX_);
    form->addRow(QStringLiteral("Datum Y:"), offsetY_);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, [this]() { accept(); });
    connect(buttons, &QDialogButtonBox::rejected, this, [this]() { reject(); });

    auto* layout = new QVBoxLayout(this);
    layout->addLayout(form);
    layout->addWidget(errors_);
    layout->addWidget(buttons);

    setForm(initial);

    connect(type_, &QComboBox::currentIndexChanged, this, [this](int) { updateEnabledFields(); });
    connect(useOffset_, &QCheckBox::toggled, this, [this](bool) { updateEnabledFields(); });
    connect(units_, &QComboBox::currentIndexChanged, this, [this](int) { unitsChanged(); });
}

viewmodel::NewJobForm NewJobDialog::form() const {
    viewmodel::NewJobForm f;
    f.type = choice<JobType>(type_);
    f.units = choice<Units>(units_);
    f.width = width_->value();
    f.height = height_->value();
    f.thickness = thickness_->value();
    f.length = length_->value();
    f.diameter = diameter_->value();
    f.rotaryAxis = choice<RotaryAxis>(rotaryAxis_);
    f.flip = choice<FlipDirection>(flip_);
    f.zZero = choice<ZZero>(zZero_);
    f.datum = choice<Datum>(datum_);
    f.useOffset = useOffset_->isChecked();
    f.offsetX = offsetX_->value();
    f.offsetY = offsetY_->value();
    return f;
}

void NewJobDialog::setForm(const viewmodel::NewJobForm& f) {
    updating_ = true;
    select(type_, f.type);
    select(units_, f.units);
    width_->setValue(f.width);
    height_->setValue(f.height);
    thickness_->setValue(f.thickness);
    length_->setValue(f.length);
    diameter_->setValue(f.diameter);
    select(rotaryAxis_, f.rotaryAxis);
    select(flip_, f.flip);
    select(zZero_, f.zZero);
    select(datum_, f.datum);
    useOffset_->setChecked(f.useOffset);
    offsetX_->setValue(f.offsetX);
    offsetY_->setValue(f.offsetY);
    shownUnits_ = f.units;
    const QString suffix =
        QStringLiteral(" ") + QString::fromUtf8(document::unitSuffix(f.units).data(),
                                                static_cast<int>(document::unitSuffix(f.units).size()));
    for (QDoubleSpinBox* box : {width_, height_, thickness_, length_, diameter_, offsetX_, offsetY_}) {
        box->setSuffix(suffix);
    }
    updating_ = false;
    updateEnabledFields();
}

void NewJobDialog::unitsChanged() {
    if (updating_) {
        return;
    }
    // Read the values in the unit they were typed in, then convert them to the new unit.
    viewmodel::NewJobForm f = form();
    const Units target = f.units;
    f.units = shownUnits_;
    setForm(viewmodel::convertFormUnits(f, target));
}

void NewJobDialog::updateEnabledFields() {
    const auto type = choice<JobType>(type_);
    const bool rotary = type == JobType::Rotary;
    width_->setEnabled(!rotary);
    height_->setEnabled(!rotary);
    thickness_->setEnabled(!rotary);
    length_->setEnabled(rotary);
    diameter_->setEnabled(rotary);
    rotaryAxis_->setEnabled(rotary);
    flip_->setEnabled(type == JobType::DoubleSided);
    datum_->setEnabled(!rotary);
    useOffset_->setEnabled(!rotary);
    offsetX_->setEnabled(!rotary && useOffset_->isChecked());
    offsetY_->setEnabled(!rotary && useOffset_->isChecked());
}

void NewJobDialog::accept() {
    const viewmodel::NewJobResult result = viewmodel::validateNewJob(form());
    if (!result.ok()) {
        QStringList lines;
        for (const viewmodel::NewJobError& e : result.errors) {
            lines << QString::fromStdString(e.message);
        }
        errors_->setText(lines.join(QStringLiteral("\n")));
        errors_->show();
        return;
    }
    job_ = result.job;
    errors_->hide();
    QDialog::accept();
}

}  // namespace carvetto::app
