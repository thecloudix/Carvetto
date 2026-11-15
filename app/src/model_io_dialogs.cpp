#include "model_io_dialogs.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFileInfo>
#include <QFormLayout>
#include <QImage>
#include <QLabel>
#include <QPixmap>
#include <QPushButton>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>

namespace carvetto::app {

namespace {

constexpr int kPreviewSide = 300;

QDoubleSpinBox* makeSpin(double lo, double hi, double value, int decimals, QWidget* parent) {
    auto* s = new QDoubleSpinBox(parent);
    s->setDecimals(decimals);
    s->setRange(lo, hi);
    s->setValue(value);
    s->setKeyboardTracking(false);
    return s;
}

}  // namespace

MeshImportDialog::MeshImportDialog(io::TriangleMesh mesh, const QString& fileName, QWidget* parent)
    : QDialog(parent), mesh_(std::move(mesh)) {
    setWindowTitle(QStringLiteral("Orientate Model"));
    name_ = QFileInfo(fileName).completeBaseName().toStdString();

    units_ = new QComboBox(this);
    units_->addItems({QStringLiteral("Millimetres"), QStringLiteral("Inches")});
    top_ = new QComboBox(this);
    top_->addItems({QStringLiteral("Top (+Z)"), QStringLiteral("Bottom (-Z)"), QStringLiteral("Right (+X)"),
                    QStringLiteral("Left (-X)"), QStringLiteral("Back (+Y)"), QStringLiteral("Front (-Y)")});
    rotation_ = new QComboBox(this);
    rotation_->addItems({QStringLiteral("0 degrees"), QStringLiteral("90 degrees"), QStringLiteral("180 degrees"),
                         QStringLiteral("270 degrees")});
    resize_ = new QCheckBox(QStringLiteral("Resize to"), this);
    lock_ = new QCheckBox(QStringLiteral("Lock aspect ratio"), this);
    lock_->setChecked(true);
    driver_ = new QComboBox(this);
    driver_->addItems({QStringLiteral("driven by X"), QStringLiteral("driven by Y"), QStringLiteral("driven by Z")});
    sizeX_ = makeSpin(0.001, 100000.0, 100.0, 3, this);
    sizeY_ = makeSpin(0.001, 100000.0, 100.0, 3, this);
    sizeZ_ = makeSpin(0.001, 100000.0, 10.0, 3, this);
    centre_ = new QCheckBox(QStringLiteral("Centre model on the origin"), this);
    centre_->setChecked(true);
    zeroBottom_ = new QCheckBox(QStringLiteral("Zero plane at the bottom of the model"), this);
    zeroBottom_->setChecked(true);
    zero_ = makeSpin(-100000.0, 100000.0, 0.0, 3, this);
    discard_ = new QCheckBox(QStringLiteral("Discard model below the zero plane"), this);
    discard_->setChecked(true);
    cell_ = makeSpin(0.001, 100.0, 0.5, 3, this);

    picture_ = new QLabel(this);
    picture_->setFixedSize(kPreviewSide, kPreviewSide);
    picture_->setAlignment(Qt::AlignCenter);
    undercuts_ = new QLabel(this);
    undercuts_->setWordWrap(true);
    info_ = new QLabel(this);
    info_->setWordWrap(true);
    buttons_ = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons_, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons_, &QDialogButtonBox::rejected, this, &QDialog::reject);

    auto* form = new QFormLayout;
    form->addRow(QStringLiteral("Model units"), units_);
    form->addRow(QStringLiteral("Face that becomes the top"), top_);
    form->addRow(QStringLiteral("Rotate about Z"), rotation_);
    form->addRow(resize_, driver_);
    form->addRow(QStringLiteral("Width (X)"), sizeX_);
    form->addRow(QStringLiteral("Height (Y)"), sizeY_);
    form->addRow(QStringLiteral("Thickness (Z)"), sizeZ_);
    form->addRow(lock_);
    form->addRow(centre_);
    form->addRow(zeroBottom_);
    form->addRow(QStringLiteral("Zero plane Z"), zero_);
    form->addRow(discard_);
    form->addRow(QStringLiteral("Cell size (mm)"), cell_);
    auto* side = new QVBoxLayout;
    side->addWidget(picture_);
    side->addWidget(undercuts_);
    side->addWidget(info_);
    side->addStretch(1);
    auto* row = new QHBoxLayout;
    row->addLayout(form);
    row->addLayout(side);
    auto* outer = new QVBoxLayout(this);
    outer->addLayout(row);
    outer->addWidget(buttons_);

    // Start from the model's own size so the size boxes show something true.
    viewmodel::MeshImportSettings none;
    if (auto prepared = viewmodel::prepareMesh(mesh_, none); prepared.ok()) {
        const io::Vec3 size = io::meshBounds(prepared.value()).size();
        sizeX_->setValue(std::clamp(size.x, 0.001, 100000.0));
        sizeY_->setValue(std::clamp(size.y, 0.001, 100000.0));
        sizeZ_->setValue(std::clamp(size.z, 0.001, 100000.0));
        const double longer = std::max(size.x, size.y);
        cell_->setValue(std::clamp(std::round(longer / 200.0 * 100.0) / 100.0, 0.01, 100.0));
    }

    const auto changed = [this]() { update(); };
    for (QComboBox* c : {units_, top_, rotation_, driver_}) {
        connect(c, &QComboBox::currentIndexChanged, this, changed);
    }
    for (QCheckBox* c : {resize_, lock_, centre_, zeroBottom_, discard_}) {
        connect(c, &QCheckBox::toggled, this, changed);
    }
    for (QDoubleSpinBox* s : {sizeX_, sizeY_, sizeZ_, zero_, cell_}) {
        connect(s, &QDoubleSpinBox::valueChanged, this, changed);
    }
    update();
}

viewmodel::MeshImportSettings MeshImportDialog::settings() const {
    viewmodel::MeshImportSettings s;
    s.units = units_->currentIndex() == 1 ? io::MeshUnits::Inches : io::MeshUnits::Millimetres;
    static constexpr io::TopFace faces[] = {io::TopFace::PlusZ, io::TopFace::MinusZ, io::TopFace::PlusX,
                                            io::TopFace::MinusX, io::TopFace::PlusY, io::TopFace::MinusY};
    s.orientation.top = faces[std::clamp(top_->currentIndex(), 0, 5)];
    s.orientation.zQuarterTurns = std::clamp(rotation_->currentIndex(), 0, 3);
    s.resize = resize_->isChecked();
    s.lockRatio = lock_->isChecked();
    s.driver = std::clamp(driver_->currentIndex(), 0, 2);
    s.targetSize = {sizeX_->value(), sizeY_->value(), sizeZ_->value()};
    s.centre = centre_->isChecked();
    s.zeroAtBottom = zeroBottom_->isChecked();
    s.zeroPlane = zero_->value();
    s.discardBelowZero = discard_->isChecked();
    s.cellSize = cell_->value();
    return s;
}

void MeshImportDialog::setSettings(const viewmodel::MeshImportSettings& s) {
    updating_ = true;
    units_->setCurrentIndex(s.units == io::MeshUnits::Inches ? 1 : 0);
    int face = 0;
    switch (s.orientation.top) {
        case io::TopFace::PlusZ: face = 0; break;
        case io::TopFace::MinusZ: face = 1; break;
        case io::TopFace::PlusX: face = 2; break;
        case io::TopFace::MinusX: face = 3; break;
        case io::TopFace::PlusY: face = 4; break;
        case io::TopFace::MinusY: face = 5; break;
    }
    top_->setCurrentIndex(face);
    rotation_->setCurrentIndex(std::clamp(s.orientation.zQuarterTurns, 0, 3));
    resize_->setChecked(s.resize);
    lock_->setChecked(s.lockRatio);
    driver_->setCurrentIndex(std::clamp(s.driver, 0, 2));
    sizeX_->setValue(s.targetSize.x);
    sizeY_->setValue(s.targetSize.y);
    sizeZ_->setValue(s.targetSize.z);
    centre_->setChecked(s.centre);
    zeroBottom_->setChecked(s.zeroAtBottom);
    zero_->setValue(s.zeroPlane);
    discard_->setChecked(s.discardBelowZero);
    cell_->setValue(s.cellSize);
    updating_ = false;
    update();
}

void MeshImportDialog::update() {
    if (updating_) {
        return;
    }
    const viewmodel::MeshImportSettings s = settings();
    zero_->setEnabled(!s.zeroAtBottom);
    const bool free = resize_->isChecked() && !lock_->isChecked();
    driver_->setEnabled(resize_->isChecked() && lock_->isChecked());
    sizeX_->setEnabled(resize_->isChecked() && (free || s.driver == 0));
    sizeY_->setEnabled(resize_->isChecked() && (free || s.driver == 1));
    sizeZ_->setEnabled(resize_->isChecked() && (free || s.driver == 2));
    lock_->setEnabled(resize_->isChecked());

    preview_ = viewmodel::previewMeshImport(mesh_, s);
    buttons_->button(QDialogButtonBox::Ok)->setEnabled(preview_.ok);
    if (!preview_.ok) {
        picture_->setPixmap(QPixmap());
        picture_->setText(QString());
        undercuts_->setText(QString::fromStdString(preview_.error));
        info_->clear();
        return;
    }
    QImage img(static_cast<int>(preview_.imageWidth), static_cast<int>(preview_.imageHeight),
               QImage::Format_ARGB32);
    for (std::size_t y = 0; y < preview_.imageHeight; ++y) {
        for (std::size_t x = 0; x < preview_.imageWidth; ++x) {
            img.setPixel(static_cast<int>(x), static_cast<int>(y), preview_.image[y * preview_.imageWidth + x]);
        }
    }
    picture_->setPixmap(QPixmap::fromImage(
        img.scaled(kPreviewSide, kPreviewSide, Qt::KeepAspectRatio, Qt::FastTransformation)));
    if (preview_.undercutCells > 0) {
        undercuts_->setText(QStringLiteral("Undercuts: %1 cells (red) are hidden below the top surface and are "
                                           "lost in a height field.")
                                .arg(preview_.undercutCells));
    } else {
        undercuts_->setText(QStringLiteral("No undercuts."));
    }
    info_->setText(QStringLiteral("Size %1 x %2 x %3 mm; material needed above the zero plane: %4 mm.")
                       .arg(preview_.size.x, 0, 'f', 2)
                       .arg(preview_.size.y, 0, 'f', 2)
                       .arg(preview_.size.z, 0, 'f', 2)
                       .arg(preview_.maxHeight, 0, 'f', 2));
}

ModelExportDialog::ModelExportDialog(viewmodel::ModelExportKind kind, double autoBackZ, QWidget* parent)
    : QDialog(parent), kind_(kind), autoZ_(autoBackZ) {
    setWindowTitle(QStringLiteral("Export Model"));
    back_ = new QComboBox(this);
    back_->addItems({QStringLiteral("Closed solid with flat back"), QStringLiteral("Top surface only")});
    auto_ = new QCheckBox(QStringLiteral("Back Z chosen automatically"), this);
    auto_->setChecked(true);
    backZ_ = makeSpin(-100000.0, 100000.0, autoBackZ, 3, this);
    format_ = new QComboBox(this);
    format_->addItems({QStringLiteral("PNG, 16 bit"), QStringLiteral("PGM, 16 bit")});
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    auto* form = new QFormLayout(this);
    if (kind == viewmodel::ModelExportKind::Heightmap) {
        form->addRow(QStringLiteral("Image format"), format_);
        form->addRow(new QLabel(QStringLiteral("Lowest point is black, highest is white."), this));
    } else {
        form->addRow(QStringLiteral("Back"), back_);
        form->addRow(auto_);
        form->addRow(QStringLiteral("Back Z (mm)"), backZ_);
    }
    form->addRow(buttons);
    connect(back_, &QComboBox::currentIndexChanged, this, [this]() { sync(); });
    connect(auto_, &QCheckBox::toggled, this, [this]() { sync(); });
    sync();
}

void ModelExportDialog::sync() {
    const bool flat = back_->currentIndex() == 0;
    auto_->setEnabled(flat);
    backZ_->setEnabled(flat && !auto_->isChecked());
    if (auto_->isChecked()) {
        backZ_->setValue(autoZ_);
    }
}

viewmodel::ModelExportSettings ModelExportDialog::settings() const {
    viewmodel::ModelExportSettings s;
    s.kind = kind_;
    s.back = back_->currentIndex() == 0 ? io::BackFace::Flat : io::BackFace::Open;
    s.backAuto = auto_->isChecked();
    s.backZ = backZ_->value();
    s.heightmap = format_->currentIndex() == 0 ? io::HeightmapFormat::Png16 : io::HeightmapFormat::Pgm16;
    return s;
}

void ModelExportDialog::setSettings(const viewmodel::ModelExportSettings& s) {
    back_->setCurrentIndex(s.back == io::BackFace::Flat ? 0 : 1);
    auto_->setChecked(s.backAuto);
    backZ_->setValue(s.backZ);
    format_->setCurrentIndex(s.heightmap == io::HeightmapFormat::Png16 ? 0 : 1);
    sync();
}

}  // namespace carvetto::app
