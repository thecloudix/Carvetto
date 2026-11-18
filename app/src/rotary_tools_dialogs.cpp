#include "rotary_tools_dialogs.hpp"

#include <QAction>
#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QMenu>
#include <QMessageBox>
#include <QVBoxLayout>
#include <algorithm>

#include "carvetto/document/job.hpp"
#include "carvetto/io/mesh.hpp"
#include "carvetto/viewmodel/file_session.hpp"
#include "main_window.hpp"

namespace carvetto::app {

namespace {

QDoubleSpinBox* spin(double lo, double hi, double value, int decimals, const char* name, QWidget* parent) {
    auto* s = new QDoubleSpinBox(parent);
    s->setDecimals(decimals);
    s->setRange(lo, hi);
    s->setValue(value);
    s->setKeyboardTracking(false);
    s->setObjectName(QString::fromUtf8(name));
    return s;
}

constexpr io::TopFace kFaces[] = {io::TopFace::PlusZ,  io::TopFace::MinusZ, io::TopFace::PlusX,
                                  io::TopFace::MinusX, io::TopFace::PlusY,  io::TopFace::MinusY};

int faceIndex(io::TopFace f) {
    for (int i = 0; i < 6; ++i) {
        if (kFaces[i] == f) {
            return i;
        }
    }
    return 0;
}

void report(MainWindow& window, const QString& title, const std::string& why, std::string* error) {
    if (error != nullptr) {
        *error = why;
    }
    if (!window.quiet()) {
        QMessageBox::warning(&window, title, QString::fromStdString(why));
    }
}

}  // namespace

// --- Vector Unwrapper ----------------------------------------------------------------------------------

UnwrapperDialog::UnwrapperDialog(QWidget* parent) : QDialog(parent) {
    setWindowTitle(QStringLiteral("Vector Unwrapper"));
    setObjectName(QStringLiteral("unwrapperDialog"));
    auto* form = new QFormLayout();
    centre_ = new QComboBox(this);
    centre_->setObjectName(QStringLiteral("unwrapCentre"));
    centre_->addItems({QStringLiteral("Centre of the contour"), QStringLiteral("Centre of the workspace"),
                       QStringLiteral("Custom point")});
    centreX_ = spin(-100000.0, 100000.0, 0.0, 3, "unwrapCentreX", this);
    centreY_ = spin(-100000.0, 100000.0, 0.0, 3, "unwrapCentreY", this);
    refine_ = new QCheckBox(QStringLiteral("Refine unwrapped vectors"), this);
    refine_->setObjectName(QStringLiteral("unwrapRefine"));
    refine_->setChecked(true);
    tolerance_ = spin(0.0001, 10.0, 0.01, 4, "unwrapTolerance", this);
    form->addRow(QStringLiteral("Unwrap round"), centre_);
    form->addRow(QStringLiteral("Custom X (mm)"), centreX_);
    form->addRow(QStringLiteral("Custom Y (mm)"), centreY_);
    form->addRow(refine_);
    form->addRow(QStringLiteral("Tolerance (mm)"), tolerance_);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    auto* outer = new QVBoxLayout(this);
    outer->addLayout(form);
    outer->addWidget(buttons);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(centre_, &QComboBox::currentIndexChanged, this, [this]() { sync(); });
    connect(refine_, &QCheckBox::toggled, this, [this]() { sync(); });
    sync();
}

void UnwrapperDialog::sync() {
    const bool custom = centre_->currentIndex() == 2;
    centreX_->setEnabled(custom);
    centreY_->setEnabled(custom);
    tolerance_->setEnabled(refine_->isChecked());
}

viewmodel::UnwrapperSettings UnwrapperDialog::settings() const {
    viewmodel::UnwrapperSettings s;
    switch (centre_->currentIndex()) {
        case 1: s.centre = geometry::UnwrapCentre::Workspace; break;
        case 2: s.centre = geometry::UnwrapCentre::Custom; break;
        default: s.centre = geometry::UnwrapCentre::Contour; break;
    }
    s.customCentre = {centreX_->value(), centreY_->value()};
    s.refine = refine_->isChecked();
    s.tolerance = tolerance_->value();
    return s;
}

void UnwrapperDialog::setSettings(const viewmodel::UnwrapperSettings& s) {
    int idx = 0;
    if (s.centre == geometry::UnwrapCentre::Workspace) {
        idx = 1;
    } else if (s.centre == geometry::UnwrapCentre::Custom) {
        idx = 2;
    }
    centre_->setCurrentIndex(idx);
    centreX_->setValue(s.customCentre.x);
    centreY_->setValue(s.customCentre.y);
    refine_->setChecked(s.refine);
    tolerance_->setValue(s.tolerance);
    sync();
}

// --- Rotary import -------------------------------------------------------------------------------------

RotaryImportDialog::RotaryImportDialog(QWidget* parent) : QDialog(parent) {
    setWindowTitle(QStringLiteral("Import 3D Model into Rotary Job"));
    setObjectName(QStringLiteral("rotaryImportDialog"));
    auto* form = new QFormLayout();
    units_ = new QComboBox(this);
    units_->setObjectName(QStringLiteral("rotImportUnits"));
    units_->addItems({QStringLiteral("Millimetres"), QStringLiteral("Inches")});
    type_ = new QComboBox(this);
    type_->setObjectName(QStringLiteral("rotImportType"));
    type_->addItems({QStringLiteral("Full 3D (wrapped round the cylinder)"), QStringLiteral("Flat (top-down)")});
    top_ = new QComboBox(this);
    top_->setObjectName(QStringLiteral("rotImportTop"));
    top_->addItems({QStringLiteral("+Z"), QStringLiteral("-Z"), QStringLiteral("+X"), QStringLiteral("-X"),
                    QStringLiteral("+Y"), QStringLiteral("-Y")});
    rotation_ = new QComboBox(this);
    rotation_->setObjectName(QStringLiteral("rotImportRotation"));
    rotation_->addItems({QStringLiteral("0"), QStringLiteral("90"), QStringLiteral("180"), QStringLiteral("270")});
    fit_ = new QCheckBox(QStringLiteral("Fit to material"), this);
    fit_->setObjectName(QStringLiteral("rotImportFit"));
    moveAxis_ = new QCheckBox(QStringLiteral("Rotation axis movement"), this);
    moveAxis_->setObjectName(QStringLiteral("rotImportMoveAxis"));
    shiftA_ = spin(-100000.0, 100000.0, 0.0, 3, "rotImportShiftA", this);
    shiftB_ = spin(-100000.0, 100000.0, 0.0, 3, "rotImportShiftB", this);
    centreAlong_ = new QCheckBox(QStringLiteral("Centre along the axis"), this);
    centreAlong_->setObjectName(QStringLiteral("rotImportCentre"));
    centreAlong_->setChecked(true);
    along_ = spin(-100000.0, 100000.0, 0.0, 3, "rotImportAlong", this);
    cell_ = spin(0.01, 100.0, 0.5, 3, "rotImportCell", this);
    form->addRow(QStringLiteral("Model type"), type_);
    form->addRow(QStringLiteral("File units"), units_);
    form->addRow(QStringLiteral("Top face"), top_);
    form->addRow(QStringLiteral("Rotate about Z (degrees)"), rotation_);
    form->addRow(fit_);
    form->addRow(moveAxis_);
    form->addRow(QStringLiteral("Axis shift across the axis, first (mm)"), shiftA_);
    form->addRow(QStringLiteral("Axis shift across the axis, second (mm)"), shiftB_);
    form->addRow(centreAlong_);
    form->addRow(QStringLiteral("Start along the axis (mm)"), along_);
    form->addRow(QStringLiteral("Cell size, flat model (mm)"), cell_);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    auto* outer = new QVBoxLayout(this);
    outer->addLayout(form);
    outer->addWidget(buttons);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(type_, &QComboBox::currentIndexChanged, this, [this]() { sync(); });
    connect(moveAxis_, &QCheckBox::toggled, this, [this]() { sync(); });
    connect(centreAlong_, &QCheckBox::toggled, this, [this]() { sync(); });
    sync();
}

void RotaryImportDialog::sync() {
    const bool full = type_->currentIndex() == 0;
    fit_->setEnabled(full);
    moveAxis_->setEnabled(full);
    shiftA_->setEnabled(full && moveAxis_->isChecked());
    shiftB_->setEnabled(full && moveAxis_->isChecked());
    centreAlong_->setEnabled(full);
    along_->setEnabled(full && !centreAlong_->isChecked());
    cell_->setEnabled(!full);
}

viewmodel::RotaryImportSettings RotaryImportDialog::settings() const {
    viewmodel::RotaryImportSettings s;
    s.mesh.units = units_->currentIndex() == 1 ? io::MeshUnits::Inches : io::MeshUnits::Millimetres;
    s.mesh.orientation.top = kFaces[std::clamp(top_->currentIndex(), 0, 5)];
    s.mesh.orientation.zQuarterTurns = std::clamp(rotation_->currentIndex(), 0, 3);
    s.mesh.cellSize = cell_->value();
    s.type = type_->currentIndex() == 1 ? viewmodel::RotaryModelType::Flat : viewmodel::RotaryModelType::Full3D;
    s.fitToMaterial = fit_->isChecked();
    s.moveAxis = moveAxis_->isChecked();
    s.axisShift = {shiftA_->value(), shiftB_->value()};
    s.centreAlong = centreAlong_->isChecked();
    s.alongOffset = along_->value();
    return s;
}

void RotaryImportDialog::setSettings(const viewmodel::RotaryImportSettings& s) {
    units_->setCurrentIndex(s.mesh.units == io::MeshUnits::Inches ? 1 : 0);
    top_->setCurrentIndex(faceIndex(s.mesh.orientation.top));
    rotation_->setCurrentIndex(std::clamp(s.mesh.orientation.zQuarterTurns, 0, 3));
    cell_->setValue(s.mesh.cellSize);
    type_->setCurrentIndex(s.type == viewmodel::RotaryModelType::Flat ? 1 : 0);
    fit_->setChecked(s.fitToMaterial);
    moveAxis_->setChecked(s.moveAxis);
    shiftA_->setValue(s.axisShift.x);
    shiftB_->setValue(s.axisShift.y);
    centreAlong_->setChecked(s.centreAlong);
    along_->setValue(s.alongOffset);
    sync();
}

// --- Actions -------------------------------------------------------------------------------------------

bool unwrapSelection(MainWindow& window, const std::optional<viewmodel::UnwrapperSettings>& settings,
                     std::string* error) {
    viewmodel::UnwrapperSettings use;
    if (settings) {
        use = *settings;
    } else {
        UnwrapperDialog dialog(&window);
        if (dialog.exec() != QDialog::Accepted) {
            return false;
        }
        use = dialog.settings();
    }
    const viewmodel::VectorOpResult r = viewmodel::unwrapSelectedVectors(window.currentDocument(), use);
    if (!r.ok) {
        report(window, QStringLiteral("Vector Unwrapper"), r.message, error);
        return false;
    }
    window.documentChanged();
    return true;
}

bool importRotaryMeshFile(MainWindow& window, const std::filesystem::path& path,
                          const std::optional<viewmodel::RotaryImportSettings>& settings, std::string* error) {
    const QString title = QStringLiteral("Import 3D Model into Rotary Job");
    auto mesh = io::readMeshFile(viewmodel::pathToUtf8(path));
    if (!mesh.ok()) {
        report(window, title, mesh.error(), error);
        return false;
    }
    viewmodel::RotaryImportSettings use;
    if (settings) {
        use = *settings;
    } else {
        RotaryImportDialog dialog(&window);
        if (dialog.exec() != QDialog::Accepted) {
            return false;
        }
        use = dialog.settings();
    }
    document::Document& doc = window.currentDocument();
    const viewmodel::MeshImportOutcome r =
        viewmodel::importMeshIntoRotaryJob(doc, doc.activeSheet(), mesh.value(), use, path.stem().string());
    if (!r.ok) {
        report(window, title, r.error, error);
        return false;
    }
    window.documentChanged();
    return true;
}

void addRotaryToolsActions(MainWindow& window, QMenu* menu) {
    QAction* unwrap = menu->addAction(QStringLiteral("&Vector Unwrapper (rotary)..."));
    unwrap->setObjectName(QStringLiteral("vectorUnwrapperAction"));
    QObject::connect(unwrap, &QAction::triggered, &window, [&window]() { unwrapSelection(window, std::nullopt); });
    QAction* import = menu->addAction(QStringLiteral("&Import 3D Model into Rotary Job..."));
    import->setObjectName(QStringLiteral("rotaryImportAction"));
    QObject::connect(import, &QAction::triggered, &window, [&window]() {
        const QString name = QFileDialog::getOpenFileName(&window, QStringLiteral("Import 3D Model into Rotary Job"),
                                                          QString(),
                                                          QStringLiteral("3D models (*.stl *.obj);;All files (*)"));
        if (!name.isEmpty()) {
            importRotaryMeshFile(window, viewmodel::pathFromUtf8(name.toStdString()), std::nullopt);
        }
    });
    QObject::connect(menu, &QMenu::aboutToShow, &window, [&window, unwrap, import]() {
        const bool rotary = window.currentDocument().job().type == document::JobType::Rotary;
        unwrap->setEnabled(rotary);
        import->setEnabled(rotary);
    });
}

}  // namespace carvetto::app
