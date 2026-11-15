#include "modelling_panel.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QStackedWidget>
#include <QVBoxLayout>

namespace carvetto::app {

namespace {

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

std::string str(const QLineEdit* e) {
    return e->text().toStdString();
}

QPushButton* button(const char* text, const char* tip, QWidget* parent) {
    auto* b = new QPushButton(QString::fromUtf8(text), parent);
    b->setToolTip(QString::fromUtf8(tip));
    return b;
}

void fillModeBox(QComboBox* box) {
    for (const relief::CombineMode m : viewmodel::allCombineModes()) {
        box->addItem(QString::fromUtf8(viewmodel::combineModeName(m)), static_cast<int>(m));
    }
}

relief::CombineMode modeOf(const QComboBox* box) {
    return static_cast<relief::CombineMode>(box->currentData().toInt());
}

void setMode(QComboBox* box, relief::CombineMode m) {
    box->setCurrentIndex(box->findData(static_cast<int>(m)));
}

}  // namespace

QLineEdit* ModellingPanel::addEdit(QFormLayout* form, const QString& prefix, const QString& label, const char* tip) {
    auto* e = new QLineEdit(this);
    e->setObjectName(prefix + QStringLiteral(": ") + label);
    e->setToolTip(QString::fromUtf8(tip));
    form->addRow(label, e);
    return e;
}

QLineEdit* ModellingPanel::field(const QString& label) const {
    return findChild<QLineEdit*>(label);
}
QComboBox* ModellingPanel::combo(const QString& label) const {
    return findChild<QComboBox*>(label);
}
QCheckBox* ModellingPanel::check(const QString& label) const {
    return findChild<QCheckBox*>(label);
}

ModellingPanel::ModellingPanel(DocumentGetter getter, QWidget* parent) : PanelBase(std::move(getter), parent) {
    auto* outer = new QVBoxLayout(this);
    toolBox_ = new QComboBox(this);
    toolBox_->setObjectName(QStringLiteral("Modelling tool"));
    toolBox_->addItem(QStringLiteral("Create Shape"));
    toolBox_->addItem(QStringLiteral("Bitmap to Relief"));
    toolBox_->addItem(QStringLiteral("Zero Plane"));
    toolBox_->addItem(QStringLiteral("Scale Model Z"));
    for (int i = 0; i < viewmodel::kBuildToolCount; ++i) {
        toolBox_->addItem(QString::fromUtf8(viewmodel::buildToolName(static_cast<viewmodel::BuildTool>(i))));
    }
    for (int i = 0; i < viewmodel::kModifyToolCount; ++i) {
        toolBox_->addItem(QString::fromUtf8(viewmodel::modifyToolName(static_cast<viewmodel::ModifyTool>(i))));
    }
    outer->addWidget(toolBox_);
    stack_ = new QStackedWidget(this);
    stack_->addWidget(buildShapePage());
    stack_->addWidget(buildBitmapPage());
    stack_->addWidget(buildZeroPlanePage());
    stack_->addWidget(buildScaleZPage());
    for (int i = 0; i < viewmodel::kBuildToolCount; ++i) {
        stack_->addWidget(buildBuildPage(static_cast<viewmodel::BuildTool>(i)));
    }
    for (int i = 0; i < viewmodel::kModifyToolCount; ++i) {
        stack_->addWidget(buildModifyPage(static_cast<viewmodel::ModifyTool>(i)));
    }
    outer->addWidget(stack_);
    status_ = new QLabel(this);
    status_->setObjectName(QStringLiteral("Modelling status"));
    status_->setWordWrap(true);
    outer->addWidget(status_);
    auto* row = new QHBoxLayout();
    previewButton_ = button("Preview", "Show the result alone in the Model view", this);
    applyButton_ = button("Apply", "Add the result to the model (one undo step); applying again updates it", this);
    resetButton_ = button("Reset", "Return the form to its defaults and clear the preview", this);
    newButton_ = button("Start New Component", "The next Apply adds another component instead of updating", this);
    row->addWidget(previewButton_);
    row->addWidget(applyButton_);
    outer->addLayout(row);
    auto* row2 = new QHBoxLayout();
    row2->addWidget(resetButton_);
    row2->addWidget(newButton_);
    outer->addLayout(row2);
    outer->addStretch(1);
    connect(toolBox_, &QComboBox::currentIndexChanged, this, [this](int i) {
        stack_->setCurrentIndex(i);
        clearPreview();
        message({}, false);
        const bool modify = i >= static_cast<int>(Tool::Modify);
        // Scale Z and the modify tools change existing components; there is nothing to "start new".
        newButton_->setEnabled(i != static_cast<int>(Tool::ScaleZ) && !modify);
        if (modify && doc() != nullptr) {
            writeModify(viewmodel::defaultModifyForm(
                *doc(), static_cast<viewmodel::ModifyTool>(i - static_cast<int>(Tool::Modify))));
        }
    });
    connect(previewButton_, &QPushButton::clicked, this, [this]() { preview(); });
    connect(applyButton_, &QPushButton::clicked, this, [this]() { apply(); });
    connect(resetButton_, &QPushButton::clicked, this, [this]() { reset(); });
    connect(newButton_, &QPushButton::clicked, this, [this]() { startNew(); });
    reset();
}

QWidget* ModellingPanel::buildShapePage() {
    auto* page = new QWidget(this);
    auto* form = new QFormLayout(page);
    const QString p = QStringLiteral("Shape");
    addEdit(form, p, QStringLiteral("Name"), "Name of the new component");
    auto* kind = new QComboBox(page);
    kind->setObjectName(QStringLiteral("Shape: Kind"));
    for (const char* k : {"Plane", "Round", "Angled", "Smooth", "Custom"}) {
        kind->addItem(QString::fromUtf8(k));
    }
    form->addRow(QStringLiteral("Kind"), kind);
    addEdit(form, p, QStringLiteral("Angle"), "Edge angle in degrees (Angled: 0 to 90, Round: up to 90)");
    addEdit(form, p, QStringLiteral("Profile"), "Custom kind: \"depth,height; depth,height; ...\", depth 0 to 1 rising");
    auto* limitOn = new QCheckBox(QStringLiteral("Limit height"), page);
    limitOn->setObjectName(QStringLiteral("Shape: Limit height"));
    form->addRow(limitOn);
    addEdit(form, p, QStringLiteral("Limit"), "Height cap in mm");
    auto* corners = new QCheckBox(QStringLiteral("Preserve internal corners"), page);
    corners->setObjectName(QStringLiteral("Shape: Preserve internal corners"));
    form->addRow(corners);
    auto* blend = new QCheckBox(QStringLiteral("Blend to inner vectors"), page);
    blend->setObjectName(QStringLiteral("Shape: Blend to inner vectors"));
    form->addRow(blend);
    addEdit(form, p, QStringLiteral("Smoothing"), "-1 concave, 0 linear, +1 convex (Smooth)");
    addEdit(form, p, QStringLiteral("Scale %"), "Shape height in percent");
    addEdit(form, p, QStringLiteral("Base height"), "Z offset in mm");
    auto* mode = new QComboBox(page);
    mode->setObjectName(QStringLiteral("Shape: Combine mode"));
    fillModeBox(mode);
    form->addRow(QStringLiteral("Combine mode"), mode);
    addEdit(form, p, QStringLiteral("Cell size"), "Resolution of the surface in mm");
    return page;
}

QWidget* ModellingPanel::buildBitmapPage() {
    auto* page = new QWidget(this);
    auto* form = new QFormLayout(page);
    const QString p = QStringLiteral("Bitmap");
    addEdit(form, p, QStringLiteral("Name"), "Name of the new component");
    auto* file = addEdit(form, p, QStringLiteral("File"), "Image file; empty uses the selected bitmap");
    auto* browse = button("Browse...", "Choose an image file", page);
    form->addRow(browse);
    auto* use = button("Use selected bitmap", "Take size and position from the selected bitmap", page);
    form->addRow(use);
    addEdit(form, p, QStringLiteral("Width"), "Model width in mm");
    addEdit(form, p, QStringLiteral("Z height"), "Height of white in mm");
    addEdit(form, p, QStringLiteral("Corner X"), "Lower-left corner, mm");
    addEdit(form, p, QStringLiteral("Corner Y"), "Lower-left corner, mm");
    auto* invert = new QCheckBox(QStringLiteral("Invert"), page);
    invert->setObjectName(QStringLiteral("Bitmap: Invert"));
    form->addRow(invert);
    auto* autoScale = new QCheckBox(QStringLiteral("Auto-scale"), page);
    autoScale->setObjectName(QStringLiteral("Bitmap: Auto-scale"));
    form->addRow(autoScale);
    auto* mode = new QComboBox(page);
    mode->setObjectName(QStringLiteral("Bitmap: Combine mode"));
    fillModeBox(mode);
    form->addRow(QStringLiteral("Combine mode"), mode);
    connect(browse, &QPushButton::clicked, this, [this, file]() {
        const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("Image"), {},
                                                          QStringLiteral("Images (*.png *.jpg *.jpeg *.bmp *.gif *.pgm *.ppm *.pbm)"));
        if (!path.isEmpty()) {
            file->setText(path);
        }
    });
    connect(use, &QPushButton::clicked, this, [this]() { useSelectedBitmap(); });
    return page;
}

QWidget* ModellingPanel::buildZeroPlanePage() {
    auto* page = new QWidget(this);
    auto* form = new QFormLayout(page);
    const QString p = QStringLiteral("Zero Plane");
    addEdit(form, p, QStringLiteral("Name"), "Name of the new component");
    addEdit(form, p, QStringLiteral("Cell size"), "Resolution in mm; the plane covers the job");
    addEdit(form, p, QStringLiteral("Corner X"), "Lower-left corner, mm");
    addEdit(form, p, QStringLiteral("Corner Y"), "Lower-left corner, mm");
    return page;
}

QWidget* ModellingPanel::buildScaleZPage() {
    auto* page = new QWidget(this);
    auto* form = new QFormLayout(page);
    const QString p = QStringLiteral("Scale Z");
    auto* kind = new QComboBox(page);
    kind->setObjectName(QStringLiteral("Scale Z: Kind"));
    kind->addItem(QStringLiteral("Percent"));
    kind->addItem(QStringLiteral("Exact height"));
    form->addRow(QStringLiteral("Scale by"), kind);
    addEdit(form, p, QStringLiteral("Value"), "Percent, or the exact height in mm");
    auto* both = new QCheckBox(QStringLiteral("Both sides of the zero plane"), page);
    both->setObjectName(QStringLiteral("Scale Z: Both sides"));
    form->addRow(both);
    form->addRow(new QLabel(QStringLiteral("Scales the selected components, or the whole model when none is selected."), page));
    return page;
}

QWidget* ModellingPanel::buildFieldsPage(const QString& p, const std::vector<viewmodel::ModifyField>& fields,
                                         const char* helpText) {
    auto* page = new QWidget(this);
    auto* form = new QFormLayout(page);
    for (const viewmodel::ModifyField& spec : fields) {
        const QString label = qs(spec.label);
        const QString name = p + QStringLiteral(": ") + label;
        switch (spec.kind) {
            case viewmodel::ModifyField::Kind::Number: addEdit(form, p, label, spec.tip.c_str()); break;
            case viewmodel::ModifyField::Kind::Check: {
                auto* c = new QCheckBox(label, page);
                c->setObjectName(name);
                c->setToolTip(qs(spec.tip));
                form->addRow(c);
                break;
            }
            case viewmodel::ModifyField::Kind::Choice: {
                auto* c = new QComboBox(page);
                c->setObjectName(name);
                c->setToolTip(qs(spec.tip));
                for (const std::string& ch : spec.choices) {
                    c->addItem(qs(ch));
                }
                form->addRow(label, c);
                break;
            }
        }
    }
    auto* help = new QLabel(QString::fromUtf8(helpText), page);
    help->setWordWrap(true);
    form->addRow(help);
    return page;
}

QWidget* ModellingPanel::buildModifyPage(viewmodel::ModifyTool tool) {
    return buildFieldsPage(QString::fromUtf8(viewmodel::modifyToolName(tool)), viewmodel::modifyFields(tool),
                           viewmodel::modifyHelp(tool));
}

QWidget* ModellingPanel::buildBuildPage(viewmodel::BuildTool tool) {
    return buildFieldsPage(QString::fromUtf8(viewmodel::buildToolName(tool)), viewmodel::buildFields(tool),
                           viewmodel::buildHelp(tool));
}

// --- form <-> widgets ---------------------------------------------------------------------------------

void ModellingPanel::writeShape(const viewmodel::CreateShapeForm& f) {
    field(QStringLiteral("Shape: Name"))->setText(qs(f.name));
    combo(QStringLiteral("Shape: Kind"))->setCurrentIndex(static_cast<int>(f.kind));
    field(QStringLiteral("Shape: Angle"))->setText(qs(f.angleDeg));
    check(QStringLiteral("Shape: Limit height"))->setChecked(f.limitEnabled);
    field(QStringLiteral("Shape: Limit"))->setText(qs(f.limit));
    check(QStringLiteral("Shape: Preserve internal corners"))->setChecked(f.preserveInternalCorners);
    check(QStringLiteral("Shape: Blend to inner vectors"))->setChecked(f.blendToInnerVectors);
    field(QStringLiteral("Shape: Smoothing"))->setText(qs(f.smoothing));
    field(QStringLiteral("Shape: Scale %"))->setText(qs(f.scalePercent));
    field(QStringLiteral("Shape: Base height"))->setText(qs(f.baseHeight));
    setMode(combo(QStringLiteral("Shape: Combine mode")), f.mode);
    field(QStringLiteral("Shape: Cell size"))->setText(qs(f.cellSize));
    field(QStringLiteral("Shape: Profile"))->setText(qs(f.profile));
}

viewmodel::CreateShapeForm ModellingPanel::readShape() const {
    viewmodel::CreateShapeForm f;
    f.name = str(field(QStringLiteral("Shape: Name")));
    f.kind = static_cast<relief::ShapeKind>(combo(QStringLiteral("Shape: Kind"))->currentIndex());
    f.angleDeg = str(field(QStringLiteral("Shape: Angle")));
    f.limitEnabled = check(QStringLiteral("Shape: Limit height"))->isChecked();
    f.limit = str(field(QStringLiteral("Shape: Limit")));
    f.preserveInternalCorners = check(QStringLiteral("Shape: Preserve internal corners"))->isChecked();
    f.blendToInnerVectors = check(QStringLiteral("Shape: Blend to inner vectors"))->isChecked();
    f.smoothing = str(field(QStringLiteral("Shape: Smoothing")));
    f.scalePercent = str(field(QStringLiteral("Shape: Scale %")));
    f.baseHeight = str(field(QStringLiteral("Shape: Base height")));
    f.mode = modeOf(combo(QStringLiteral("Shape: Combine mode")));
    f.cellSize = str(field(QStringLiteral("Shape: Cell size")));
    f.profile = str(field(QStringLiteral("Shape: Profile")));
    return f;
}

void ModellingPanel::writeBitmap(const viewmodel::BitmapReliefForm& f) {
    field(QStringLiteral("Bitmap: Name"))->setText(qs(f.name));
    field(QStringLiteral("Bitmap: File"))->setText(qs(f.filePath));
    field(QStringLiteral("Bitmap: Width"))->setText(qs(f.widthMm));
    field(QStringLiteral("Bitmap: Z height"))->setText(qs(f.zHeightMm));
    field(QStringLiteral("Bitmap: Corner X"))->setText(qs(f.cornerX));
    field(QStringLiteral("Bitmap: Corner Y"))->setText(qs(f.cornerY));
    check(QStringLiteral("Bitmap: Invert"))->setChecked(f.invert);
    check(QStringLiteral("Bitmap: Auto-scale"))->setChecked(f.autoScale);
    setMode(combo(QStringLiteral("Bitmap: Combine mode")), f.mode);
}

viewmodel::BitmapReliefForm ModellingPanel::readBitmap() const {
    viewmodel::BitmapReliefForm f;
    f.name = str(field(QStringLiteral("Bitmap: Name")));
    f.filePath = str(field(QStringLiteral("Bitmap: File")));
    f.widthMm = str(field(QStringLiteral("Bitmap: Width")));
    f.zHeightMm = str(field(QStringLiteral("Bitmap: Z height")));
    f.cornerX = str(field(QStringLiteral("Bitmap: Corner X")));
    f.cornerY = str(field(QStringLiteral("Bitmap: Corner Y")));
    f.invert = check(QStringLiteral("Bitmap: Invert"))->isChecked();
    f.autoScale = check(QStringLiteral("Bitmap: Auto-scale"))->isChecked();
    f.mode = modeOf(combo(QStringLiteral("Bitmap: Combine mode")));
    return f;
}

void ModellingPanel::writeZeroPlane(const viewmodel::ZeroPlaneForm& f) {
    field(QStringLiteral("Zero Plane: Name"))->setText(qs(f.name));
    field(QStringLiteral("Zero Plane: Cell size"))->setText(qs(f.cellSize));
    field(QStringLiteral("Zero Plane: Corner X"))->setText(qs(f.cornerX));
    field(QStringLiteral("Zero Plane: Corner Y"))->setText(qs(f.cornerY));
}

viewmodel::ZeroPlaneForm ModellingPanel::readZeroPlane() const {
    viewmodel::ZeroPlaneForm f;
    f.name = str(field(QStringLiteral("Zero Plane: Name")));
    f.cellSize = str(field(QStringLiteral("Zero Plane: Cell size")));
    f.cornerX = str(field(QStringLiteral("Zero Plane: Corner X")));
    f.cornerY = str(field(QStringLiteral("Zero Plane: Corner Y")));
    return f;
}

void ModellingPanel::writeScaleZ(const viewmodel::ScaleZForm& f) {
    combo(QStringLiteral("Scale Z: Kind"))->setCurrentIndex(f.kind == relief::ScaleZ::Kind::Percent ? 0 : 1);
    field(QStringLiteral("Scale Z: Value"))->setText(qs(f.value));
    check(QStringLiteral("Scale Z: Both sides"))->setChecked(f.bothSides);
}

viewmodel::ScaleZForm ModellingPanel::readScaleZ() const {
    viewmodel::ScaleZForm f;
    f.kind = combo(QStringLiteral("Scale Z: Kind"))->currentIndex() == 0 ? relief::ScaleZ::Kind::Percent
                                                                          : relief::ScaleZ::Kind::ExactHeight;
    f.value = str(field(QStringLiteral("Scale Z: Value")));
    f.bothSides = check(QStringLiteral("Scale Z: Both sides"))->isChecked();
    return f;
}

void ModellingPanel::writeModify(const viewmodel::ModifyForm& f) {
    const QString p = QString::fromUtf8(viewmodel::modifyToolName(f.tool));
    for (const viewmodel::ModifyField& spec : viewmodel::modifyFields(f.tool)) {
        const QString name = p + QStringLiteral(": ") + qs(spec.label);
        const auto it = f.values.find(spec.label);
        const std::string& v = it != f.values.end() ? it->second : spec.defaultValue;
        switch (spec.kind) {
            case viewmodel::ModifyField::Kind::Number: field(name)->setText(qs(v)); break;
            case viewmodel::ModifyField::Kind::Check: check(name)->setChecked(v == "true"); break;
            case viewmodel::ModifyField::Kind::Choice: combo(name)->setCurrentText(qs(v)); break;
        }
    }
}

void ModellingPanel::writeBuild(const viewmodel::BuildForm& f) {
    const QString p = QString::fromUtf8(viewmodel::buildToolName(f.tool));
    for (const viewmodel::ModifyField& spec : viewmodel::buildFields(f.tool)) {
        const QString name = p + QStringLiteral(": ") + qs(spec.label);
        const auto it = f.values.find(spec.label);
        const std::string& v = it != f.values.end() ? it->second : spec.defaultValue;
        switch (spec.kind) {
            case viewmodel::ModifyField::Kind::Number: field(name)->setText(qs(v)); break;
            case viewmodel::ModifyField::Kind::Check: check(name)->setChecked(v == "true"); break;
            case viewmodel::ModifyField::Kind::Choice: combo(name)->setCurrentText(qs(v)); break;
        }
    }
}

viewmodel::BuildForm ModellingPanel::readBuild() const {
    viewmodel::BuildForm f;
    f.tool = buildTool();
    const QString p = QString::fromUtf8(viewmodel::buildToolName(f.tool));
    for (const viewmodel::ModifyField& spec : viewmodel::buildFields(f.tool)) {
        const QString name = p + QStringLiteral(": ") + qs(spec.label);
        switch (spec.kind) {
            case viewmodel::ModifyField::Kind::Number: f.values[spec.label] = str(field(name)); break;
            case viewmodel::ModifyField::Kind::Check:
                f.values[spec.label] = check(name)->isChecked() ? "true" : "false";
                break;
            case viewmodel::ModifyField::Kind::Choice:
                f.values[spec.label] = combo(name)->currentText().toStdString();
                break;
        }
    }
    return f;
}

viewmodel::ModifyForm ModellingPanel::readModify() const {
    viewmodel::ModifyForm f;
    f.tool = modifyTool();
    const QString p = QString::fromUtf8(viewmodel::modifyToolName(f.tool));
    for (const viewmodel::ModifyField& spec : viewmodel::modifyFields(f.tool)) {
        const QString name = p + QStringLiteral(": ") + qs(spec.label);
        switch (spec.kind) {
            case viewmodel::ModifyField::Kind::Number: f.values[spec.label] = str(field(name)); break;
            case viewmodel::ModifyField::Kind::Check:
                f.values[spec.label] = check(name)->isChecked() ? "true" : "false";
                break;
            case viewmodel::ModifyField::Kind::Choice:
                f.values[spec.label] = combo(name)->currentText().toStdString();
                break;
        }
    }
    return f;
}

// --- actions --------------------------------------------------------------------------------------------

void ModellingPanel::setTool(Tool t) {
    toolBox_->setCurrentIndex(static_cast<int>(t));
}

ModellingPanel::Tool ModellingPanel::tool() const {
    const int i = toolBox_->currentIndex();
    if (i >= static_cast<int>(Tool::Modify)) {
        return Tool::Modify;
    }
    return i >= static_cast<int>(Tool::Build) ? Tool::Build : static_cast<Tool>(i);
}

void ModellingPanel::setModifyTool(viewmodel::ModifyTool t) {
    toolBox_->setCurrentIndex(static_cast<int>(Tool::Modify) + static_cast<int>(t));
}

void ModellingPanel::setBuildTool(viewmodel::BuildTool t) {
    toolBox_->setCurrentIndex(static_cast<int>(Tool::Build) + static_cast<int>(t));
}

viewmodel::BuildTool ModellingPanel::buildTool() const {
    return static_cast<viewmodel::BuildTool>(toolBox_->currentIndex() - static_cast<int>(Tool::Build));
}

viewmodel::ModifyTool ModellingPanel::modifyTool() const {
    return static_cast<viewmodel::ModifyTool>(toolBox_->currentIndex() - static_cast<int>(Tool::Modify));
}

void ModellingPanel::message(const std::string& text, bool error) {
    status_->setText(qs(text));
    status_->setStyleSheet(error ? QStringLiteral("color: #b00020;") : QString());
}

void ModellingPanel::clearPreview() {
    if (onPreview) {
        onPreview(nullptr);
    }
}

void ModellingPanel::useSelectedBitmap() {
    if (const document::Document* d = doc()) {
        const viewmodel::BitmapReliefForm f = viewmodel::defaultBitmapForm(*d);
        field(QStringLiteral("Bitmap: Width"))->setText(qs(f.widthMm));
        field(QStringLiteral("Bitmap: Corner X"))->setText(qs(f.cornerX));
        field(QStringLiteral("Bitmap: Corner Y"))->setText(qs(f.cornerY));
        field(QStringLiteral("Bitmap: File"))->clear();
    }
}

bool ModellingPanel::preview() {
    const document::Document* d = doc();
    if (d == nullptr) {
        return false;
    }
    const relief::HeightField* shown = nullptr;
    viewmodel::ModellingPreview p;
    viewmodel::ScaleZPreview sp;
    switch (tool()) {
        case Tool::Modify: {
            lastModify_ = viewmodel::previewModify(*d, readModify());
            p.ok = lastModify_.ok;
            p.message = lastModify_.message;
            p.info = lastModify_.info;
            shown = lastModify_.previewField();
            if (!p.ok) {
                clearPreview();
                message(p.message, true);
                return false;
            }
            if (onPreview) {
                onPreview(shown);   // null for vector-only results clears the preview
            }
            message(p.info, false);
            return true;
        }
        case Tool::Build: p = viewmodel::previewBuild(*d, readBuild()); break;
        case Tool::CreateShape: p = viewmodel::previewCreateShape(*d, readShape()); break;
        case Tool::Bitmap: p = viewmodel::previewBitmapRelief(*d, readBitmap(), decoder); break;
        case Tool::ZeroPlane: p = viewmodel::previewZeroPlane(*d, readZeroPlane()); break;
        case Tool::ScaleZ:
            sp = viewmodel::previewScaleZ(*d, readScaleZ());
            p.ok = sp.ok;
            p.message = sp.message;
            if (sp.ok) {
                p.component = sp.components.front();
                p.info = std::to_string(sp.components.size()) + " component(s) scaled by " +
                         QString::number(sp.factor, 'g', 6).toStdString();
            }
            break;
    }
    if (!p.ok) {
        clearPreview();
        message(p.message, true);
        return false;
    }
    shown = &p.component.field;
    if (tool() == Tool::Build && relief::bake(p.component, previewBaked_) == relief::Status::Ok) {
        shown = &previewBaked_;   // Exact height of Turn/Spin lives in the shape height percent
    }
    if (onPreview) {
        onPreview(shown);
    }
    message(p.info, false);
    return true;
}

bool ModellingPanel::apply() {
    document::Document* d = doc();
    if (d == nullptr) {
        return false;
    }
    viewmodel::EditResult r;
    switch (tool()) {
        case Tool::ScaleZ: r = viewmodel::applyScaleZ(*d, readScaleZ()); break;
        case Tool::Modify: r = viewmodel::applyModify(*d, readModify()); break;
        default: {
            viewmodel::ModellingPreview p;
            if (tool() == Tool::Build) {
                p = viewmodel::previewBuild(*d, readBuild());
            } else if (tool() == Tool::CreateShape) {
                p = viewmodel::previewCreateShape(*d, readShape());
            } else if (tool() == Tool::Bitmap) {
                p = viewmodel::previewBitmapRelief(*d, readBitmap(), decoder);
            } else {
                p = viewmodel::previewZeroPlane(*d, readZeroPlane());
            }
            if (!p.ok) {
                r.ok = false;
                r.message = p.message;
            } else {
                // The seed of Textured Area / Extrude is the selected component: keep it selected so
                // applying again re-reads the same seed rather than the component just made.
                const std::vector<document::ComponentId> before = d->componentSelection().ids();
                r = session_.apply(*d, std::move(p.component));
                if (tool() == Tool::Build && r.ok && !before.empty()) {
                    d->componentSelection().assign(before);
                }
            }
            break;
        }
    }
    clearPreview();
    if (!r.ok) {
        message(r.message, true);
    } else {
        message(tool() == Tool::ScaleZ ? "Scaled." : tool() == Tool::Modify ? "Done." : "Applied.", false);
    }
    return finish(r);
}

void ModellingPanel::reset() {
    writeShape(viewmodel::CreateShapeForm{});
    viewmodel::BitmapReliefForm bf;
    if (const document::Document* d = doc()) {
        bf = viewmodel::defaultBitmapForm(*d);
    }
    writeBitmap(bf);
    writeZeroPlane(viewmodel::ZeroPlaneForm{});
    writeScaleZ(viewmodel::ScaleZForm{});
    const document::Document none;
    const document::Document* md = doc() != nullptr ? doc() : &none;
    for (int i = 0; i < viewmodel::kModifyToolCount; ++i) {
        writeModify(viewmodel::defaultModifyForm(*md, static_cast<viewmodel::ModifyTool>(i)));
    }
    for (int i = 0; i < viewmodel::kBuildToolCount; ++i) {
        writeBuild(viewmodel::defaultBuildForm(static_cast<viewmodel::BuildTool>(i)));
    }
    clearPreview();
    message({}, false);
}

void ModellingPanel::startNew() {
    session_.startNew();
    clearPreview();
    message("The next Apply adds a new component.", false);
}

}  // namespace carvetto::app
