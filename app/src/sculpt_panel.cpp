#include "sculpt_panel.hpp"

#include <QButtonGroup>
#include <QCheckBox>
#include <QFileDialog>
#include <QComboBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include "carvetto/viewmodel/model_view.hpp"

namespace carvetto::app {

namespace {

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

// Pictures far beyond any sensible reference image are refused before they are converted.
constexpr int kMaxImageSide = 8192;

QImage loadImageFile(const QString& path) {
    QImage img(path);
    return img.isNull() || img.width() > kMaxImageSide || img.height() > kMaxImageSide ? QImage() : img;
}

constexpr int kSliderSteps = 1000;

int toSlider(double v, viewmodel::Range r) {
    const double span = r.hi - r.lo;
    return span > 0.0 ? static_cast<int>(std::lround((v - r.lo) / span * kSliderSteps)) : 0;
}

double fromSlider(int v, viewmodel::Range r) {
    return r.lo + (r.hi - r.lo) * static_cast<double>(v) / kSliderSteps;
}

}  // namespace

// --- SculptPanel -------------------------------------------------------------------------------------

SculptPanel::SculptPanel(DocumentGetter getter, QWidget* parent) : PanelBase(std::move(getter), parent) {
    auto* outer = new QVBoxLayout(this);
    startButton_ = new QPushButton(QStringLiteral("Start Sculpting"), this);
    startButton_->setObjectName(QStringLiteral("Sculpt: Start"));
    startButton_->setToolTip(QStringLiteral(
        "Sculpt the selected component (or the only one) in the Model view; left-drag paints"));
    outer->addWidget(startButton_);

    auto* palette = new QGridLayout();
    auto* group = new QButtonGroup(this);
    int n = 0;
    for (const relief::BrushKind kind : viewmodel::allBrushKinds()) {
        auto* b = new QPushButton(QString::fromUtf8(viewmodel::brushKindName(kind)), this);
        b->setCheckable(true);
        b->setObjectName(QStringLiteral("Brush: ") + QString::fromUtf8(viewmodel::brushKindName(kind)));
        b->setProperty("brushKind", static_cast<int>(kind));
        group->addButton(b);
        palette->addWidget(b, n / 3, n % 3);
        connect(b, &QPushButton::clicked, this, [this, kind]() {
            brush_.kind = kind;
            brush_ = viewmodel::clampedBrush(brush_);
            writeWidgets(brush_);
        });
        ++n;
    }
    outer->addLayout(palette);

    auto addSlider = [this, outer](const char* label) {
        auto* row = new QHBoxLayout();
        row->addWidget(new QLabel(QString::fromUtf8(label), this));
        auto* s = new QSlider(Qt::Horizontal, this);
        s->setRange(0, kSliderSteps);
        s->setObjectName(QStringLiteral("Sculpt: ") + QString::fromUtf8(label));
        row->addWidget(s, 1);
        outer->addLayout(row);
        connect(s, &QSlider::valueChanged, this, [this]() { widgetsChanged(); });
        return s;
    };
    addSlider("Diameter");
    addSlider("Strength");
    addSlider("Smoothness");
    readout_ = new QLabel(this);
    readout_->setObjectName(QStringLiteral("Sculpt: Readout"));
    outer->addWidget(readout_);

    auto* modeRow = new QHBoxLayout();
    modeRow->addWidget(new QLabel(QStringLiteral("Mode"), this));
    auto* mode = new QComboBox(this);
    mode->setObjectName(QStringLiteral("Sculpt: Mode"));
    for (const relief::SculptMode m : {relief::SculptMode::Normal, relief::SculptMode::Raise, relief::SculptMode::Lower}) {
        mode->addItem(QString::fromUtf8(viewmodel::sculptModeName(m)), static_cast<int>(m));
    }
    modeRow->addWidget(mode, 1);
    outer->addLayout(modeRow);
    connect(mode, &QComboBox::currentIndexChanged, this, [this]() { widgetsChanged(); });
    auto* preserve = new QCheckBox(QStringLiteral("Preserve transparency"), this);
    preserve->setObjectName(QStringLiteral("Sculpt: Preserve transparency"));
    preserve->setToolTip(QStringLiteral("Transparent (empty) cells of the component stay empty"));
    outer->addWidget(preserve);
    connect(preserve, &QCheckBox::toggled, this, [this]() { widgetsChanged(); });

    auto* presetRow = new QHBoxLayout();
    presetBox_ = new QComboBox(this);
    presetBox_->setObjectName(QStringLiteral("Sculpt: Preset"));
    presetRow->addWidget(presetBox_, 1);
    auto* saveBtn = new QPushButton(QStringLiteral("Save Preset..."), this);
    presetRow->addWidget(saveBtn);
    outer->addLayout(presetRow);
    connect(presetBox_, &QComboBox::activated, this, [this](int i) {
        applyPreset(presetBox_->itemText(i).toStdString());
    });
    connect(saveBtn, &QPushButton::clicked, this, [this]() {
        bool ok = false;
        const QString name = QInputDialog::getText(this, QStringLiteral("Save brush preset"),
                                                   QStringLiteral("Preset name"), QLineEdit::Normal, {}, &ok);
        if (ok) {
            savePreset(name.toStdString());
        }
    });

    auto* delBtn = new QPushButton(QStringLiteral("Delete Preset"), this);
    delBtn->setObjectName(QStringLiteral("Sculpt: Delete Preset"));
    outer->addWidget(delBtn);
    connect(delBtn, &QPushButton::clicked, this, [this]() {
        if (presetBox_->currentIndex() >= 0) {
            removePreset(presetBox_->currentText().toStdString());
        }
    });

    // Custom brush from an image.
    auto* stampRow = new QHBoxLayout();
    auto* stampBtn = new QPushButton(QStringLiteral("Brush Image..."), this);
    stampBtn->setObjectName(QStringLiteral("Sculpt: Brush Image"));
    stampBtn->setToolTip(QStringLiteral("Use a picture as the brush shape: light is high, transparent is no effect"));
    auto* stampClear = new QPushButton(QStringLiteral("Round Brush"), this);
    stampClear->setObjectName(QStringLiteral("Sculpt: Round Brush"));
    stampRow->addWidget(stampBtn);
    stampRow->addWidget(stampClear);
    outer->addLayout(stampRow);
    auto* invert = new QCheckBox(QStringLiteral("Invert image"), this);
    invert->setObjectName(QStringLiteral("Sculpt: Invert image"));
    auto* rotate = new QCheckBox(QStringLiteral("Rotate with stroke"), this);
    rotate->setObjectName(QStringLiteral("Sculpt: Rotate with stroke"));
    outer->addWidget(invert);
    outer->addWidget(rotate);
    connect(stampBtn, &QPushButton::clicked, this, [this]() {
        const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("Brush image"), {},
                                                          QStringLiteral("Images (*.png *.jpg *.jpeg *.bmp *.gif)"));
        if (!path.isEmpty() && !setBrushImage(loadImageFile(path))) {
            message("That picture could not be used as a brush", true);
        }
    });
    connect(stampClear, &QPushButton::clicked, this, [this]() { clearBrushImage(); });
    connect(invert, &QCheckBox::toggled, this, [this]() {
        setBrushImageOptions(check(QStringLiteral("Sculpt: Invert image"))->isChecked(),
                             check(QStringLiteral("Sculpt: Rotate with stroke"))->isChecked());
    });
    connect(rotate, &QCheckBox::toggled, this, [this]() {
        setBrushImageOptions(check(QStringLiteral("Sculpt: Invert image"))->isChecked(),
                             check(QStringLiteral("Sculpt: Rotate with stroke"))->isChecked());
    });

    // Reference (overlay) image over the model.
    auto* overlayRow = new QHBoxLayout();
    auto* overlayBtn = new QPushButton(QStringLiteral("Overlay Image..."), this);
    overlayBtn->setObjectName(QStringLiteral("Sculpt: Overlay Image"));
    overlayBtn->setToolTip(QStringLiteral("Lay a reference picture over the model in the Model view"));
    auto* overlayClear = new QPushButton(QStringLiteral("Clear Overlay"), this);
    overlayClear->setObjectName(QStringLiteral("Sculpt: Clear Overlay"));
    overlayRow->addWidget(overlayBtn);
    overlayRow->addWidget(overlayClear);
    outer->addLayout(overlayRow);
    auto* opacityRow = new QHBoxLayout();
    opacityRow->addWidget(new QLabel(QStringLiteral("Overlay opacity"), this));
    auto* opacity = new QSlider(Qt::Horizontal, this);
    opacity->setRange(0, kSliderSteps);
    opacity->setValue(kSliderSteps / 2);
    opacity->setObjectName(QStringLiteral("Sculpt: Overlay opacity"));
    opacityRow->addWidget(opacity, 1);
    outer->addLayout(opacityRow);
    connect(overlayBtn, &QPushButton::clicked, this, [this]() {
        const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("Overlay image"), {},
                                                          QStringLiteral("Images (*.png *.jpg *.jpeg *.bmp *.gif)"));
        if (!path.isEmpty() && !setOverlayImage(loadImageFile(path))) {
            message("That picture could not be used as an overlay", true);
        }
    });
    connect(overlayClear, &QPushButton::clicked, this, [this]() { clearOverlayImage(); });
    connect(opacity, &QSlider::valueChanged, this,
            [this](int v) { setOverlayOpacity(static_cast<double>(v) / kSliderSteps); });

    auto* row = new QHBoxLayout();
    undoButton_ = new QPushButton(QStringLiteral("Undo Stroke"), this);
    undoButton_->setObjectName(QStringLiteral("Sculpt: Undo Stroke"));
    keepButton_ = new QPushButton(QStringLiteral("Keep"), this);
    keepButton_->setObjectName(QStringLiteral("Sculpt: Keep"));
    keepButton_->setToolTip(QStringLiteral("Write the sculpted result into the model (one undo step)"));
    discardButton_ = new QPushButton(QStringLiteral("Discard"), this);
    discardButton_->setObjectName(QStringLiteral("Sculpt: Discard"));
    discardButton_->setToolTip(QStringLiteral("Throw away the strokes since the last Keep"));
    row->addWidget(undoButton_);
    row->addWidget(keepButton_);
    row->addWidget(discardButton_);
    outer->addLayout(row);

    status_ = new QLabel(this);
    status_->setObjectName(QStringLiteral("Sculpt: Status"));
    status_->setWordWrap(true);
    outer->addWidget(status_);
    outer->addStretch(1);

    connect(startButton_, &QPushButton::clicked, this, [this]() {
        if (sculpting()) {
            stopSculpting();
        } else {
            startSculpting();
        }
    });
    connect(undoButton_, &QPushButton::clicked, this, [this]() { undoStroke(); });
    connect(keepButton_, &QPushButton::clicked, this, [this]() { keep(); });
    connect(discardButton_, &QPushButton::clicked, this, [this]() { discard(); });
    fillPresets();
    writeWidgets(brush_);
    updateButtons();
}

QPushButton* SculptPanel::brushButton(relief::BrushKind kind) const {
    return findChild<QPushButton*>(QStringLiteral("Brush: ") + QString::fromUtf8(viewmodel::brushKindName(kind)));
}
QSlider* SculptPanel::slider(const QString& name) const {
    return findChild<QSlider*>(name);
}
QComboBox* SculptPanel::combo(const QString& name) const {
    return findChild<QComboBox*>(name);
}
QCheckBox* SculptPanel::check(const QString& name) const {
    return findChild<QCheckBox*>(name);
}

void SculptPanel::message(const std::string& text, bool error) {
    status_->setText(qs(text));
    status_->setStyleSheet(error ? QStringLiteral("color: #b00020;") : QString());
}

void SculptPanel::fillPresets() {
    presetBox_->clear();
    for (const viewmodel::BrushPreset& p : presets_.all()) {
        presetBox_->addItem(qs(p.name));
    }
    presetBox_->setCurrentIndex(-1);
}

void SculptPanel::writeWidgets(const viewmodel::BrushSettings& s) {
    writing_ = true;
    viewmodel::BrushSettings c = viewmodel::clampedBrush(s);
    if (!c.custom) {
        c.custom = brush_.custom;  // a preset or a slider never drops the picture brush
    }
    brush_ = c;
    for (const relief::BrushKind k : viewmodel::allBrushKinds()) {
        brushButton(k)->setChecked(k == c.kind);
    }
    slider(QStringLiteral("Sculpt: Diameter"))->setValue(toSlider(c.diameter, viewmodel::diameterRange()));
    slider(QStringLiteral("Sculpt: Strength"))->setValue(toSlider(c.strength, viewmodel::strengthRange(c.kind)));
    slider(QStringLiteral("Sculpt: Smoothness"))->setValue(toSlider(c.smoothness, {0.0, 1.0}));
    QComboBox* mode = combo(QStringLiteral("Sculpt: Mode"));
    mode->setCurrentIndex(mode->findData(static_cast<int>(c.mode)));
    check(QStringLiteral("Sculpt: Preserve transparency"))->setChecked(c.preserveTransparency);
    readout_->setText(QStringLiteral("Diameter %1 mm, strength %2, smoothness %3")
                          .arg(c.diameter, 0, 'f', 1)
                          .arg(c.strength, 0, 'f', 2)
                          .arg(c.smoothness, 0, 'f', 2));
    writing_ = false;
}

void SculptPanel::widgetsChanged() {
    if (writing_) {
        return;
    }
    viewmodel::BrushSettings s = brush_;
    s.diameter = fromSlider(slider(QStringLiteral("Sculpt: Diameter"))->value(), viewmodel::diameterRange());
    s.strength = fromSlider(slider(QStringLiteral("Sculpt: Strength"))->value(), viewmodel::strengthRange(s.kind));
    s.smoothness = fromSlider(slider(QStringLiteral("Sculpt: Smoothness"))->value(), {0.0, 1.0});
    s.mode = static_cast<relief::SculptMode>(combo(QStringLiteral("Sculpt: Mode"))->currentData().toInt());
    s.preserveTransparency = check(QStringLiteral("Sculpt: Preserve transparency"))->isChecked();
    writeWidgets(s);
}

void SculptPanel::setBrush(const viewmodel::BrushSettings& s) {
    writeWidgets(s);
}

viewmodel::BrushSettings SculptPanel::brush() const {
    return brush_;
}

bool SculptPanel::applyPreset(const std::string& name) {
    const viewmodel::BrushPreset* p = presets_.find(name);
    if (p == nullptr) {
        message("Unknown preset", true);
        return false;
    }
    writeWidgets(p->brush);
    message("Preset \"" + name + "\" selected.", false);
    return true;
}

bool SculptPanel::savePreset(const std::string& name) {
    const std::string why = presets_.save(name, brush_);
    if (!why.empty()) {
        message(why, true);
        return false;
    }
    fillPresets();
    presetBox_->setCurrentIndex(presetBox_->findText(qs(name)));
    message("Preset \"" + name + "\" saved.", false);
    persistPresets();
    return true;
}

bool SculptPanel::removePreset(const std::string& name) {
    if (!presets_.remove(name)) {
        message("\"" + name + "\" is a built-in preset or does not exist", true);
        return false;
    }
    fillPresets();
    message("Preset \"" + name + "\" deleted.", false);
    persistPresets();
    return true;
}

std::size_t SculptPanel::loadPresets(const std::string& text) {
    const std::size_t n = presets_.loadUser(text);
    fillPresets();
    return n;
}

void SculptPanel::persistPresets() {
    if (onPresetsChanged) {
        onPresetsChanged(presets_.serialiseUser());
    }
}

void SculptPanel::rebuildStamp() {
    if (brushImage_.isNull()) {
        brush_.custom.reset();
        return;
    }
    const QImage rgba = brushImage_.convertToFormat(QImage::Format_RGBA8888);
    viewmodel::RgbaImage img;
    img.width = static_cast<std::size_t>(rgba.width());
    img.height = static_cast<std::size_t>(rgba.height());
    img.rgba.resize(img.width * img.height * 4);
    for (int y = 0; y < rgba.height(); ++y) {
        const uchar* line = rgba.constScanLine(y);
        std::copy(line, line + static_cast<std::size_t>(rgba.width()) * 4, img.rgba.begin() + static_cast<std::ptrdiff_t>(y) * rgba.width() * 4);
    }
    std::string problem;
    brush_.custom = viewmodel::customBrushFromImage(img, brushInvert_, 0.0, brushRotate_, problem);
    if (!brush_.custom) {
        brushImage_ = QImage();
        message(problem, true);
    }
}

bool SculptPanel::setBrushImage(const QImage& image) {
    if (image.isNull()) {
        return false;
    }
    brushImage_ = image;
    rebuildStamp();
    if (brushImage_.isNull()) {
        return false;
    }
    message("Brush shape taken from the picture (" + std::to_string(image.width()) + " x " +
                std::to_string(image.height()) + ").",
            false);
    return true;
}

void SculptPanel::clearBrushImage() {
    brushImage_ = QImage();
    brush_.custom.reset();
    message("Round brush.", false);
}

void SculptPanel::setBrushImageOptions(bool invert, bool rotateWithStroke) {
    brushInvert_ = invert;
    brushRotate_ = rotateWithStroke;
    if (check(QStringLiteral("Sculpt: Invert image"))->isChecked() != invert) {
        check(QStringLiteral("Sculpt: Invert image"))->setChecked(invert);
    }
    if (check(QStringLiteral("Sculpt: Rotate with stroke"))->isChecked() != rotateWithStroke) {
        check(QStringLiteral("Sculpt: Rotate with stroke"))->setChecked(rotateWithStroke);
    }
    rebuildStamp();
}

bool SculptPanel::setOverlayImage(const QImage& image) {
    if (image.isNull()) {
        return false;
    }
    overlayImage_ = image;
    overlayChanged();
    return true;
}

void SculptPanel::clearOverlayImage() {
    overlayImage_ = QImage();
    overlayChanged();
}

void SculptPanel::setOverlayOpacity(double opacity) {
    overlayOpacity_ = viewmodel::clampedOverlayOpacity(opacity);
    QSlider* s = slider(QStringLiteral("Sculpt: Overlay opacity"));
    const int v = static_cast<int>(std::lround(overlayOpacity_ * kSliderSteps));
    if (s->value() != v) {
        s->setValue(v);  // comes back here with the same value
        return;
    }
    overlayChanged();
}

void SculptPanel::overlayChanged() {
    if (onOverlay) {
        onOverlay(overlayImage_, overlayOpacity_);
    }
}

void SculptPanel::updateButtons() {
    const bool on = sculpting();
    startButton_->setText(on ? QStringLiteral("Stop Sculpting") : QStringLiteral("Start Sculpting"));
    undoButton_->setEnabled(on && controller_.canUndoStroke());
    keepButton_->setEnabled(on && controller_.dirty());
    discardButton_->setEnabled(on && controller_.dirty());
}

// Composes the rest of the model once; on failure the component is previewed alone.
void SculptPanel::rebuildMerged() {
    const document::Document* d = doc();
    if (d == nullptr || !controller_.active() || !merged_.begin(*d, controller_.target(), *controller_.working())) {
        merged_.end();
    }
    controller_.takeDirty();
}

void SculptPanel::showWorking() {
    const relief::HeightField* w = controller_.working();
    if (merged_.active() && w != nullptr) {
        merged_.update(*w, controller_.takeDirty());
    }
    if (merged_.active() && onPreviewSurface) {
        onPreviewSurface(merged_.surface());
    } else if (onPreview) {
        onPreview(w);
    }
    updateButtons();
}

bool SculptPanel::startSculpting() {
    document::Document* d = doc();
    if (d == nullptr) {
        return false;
    }
    const viewmodel::EditResult r = controller_.begin(*d);
    if (!r.ok) {
        message(r.message, true);
        return false;
    }
    const document::ReliefComponent* c = d->findReliefComponent(controller_.target());
    message("Sculpting \"" + (c != nullptr ? c->data.name : std::string()) +
                "\": left-drag in the Model view paints; Keep writes it into the model.",
            false);
    if (onActiveChanged) {
        onActiveChanged(true);
    }
    rebuildMerged();
    showWorking();
    return true;
}

void SculptPanel::stopSculpting() {
    if (!sculpting()) {
        return;
    }
    const bool dropped = controller_.dirty();
    controller_.end();
    merged_.end();
    if (onPreview) {
        onPreview(nullptr);
    }
    if (onActiveChanged) {
        onActiveChanged(false);
    }
    updateButtons();
    message(dropped ? "Sculpting stopped; strokes that were not kept were dropped." : "Sculpting stopped.", false);
}

bool SculptPanel::keep() {
    document::Document* d = doc();
    if (d == nullptr) {
        return false;
    }
    const viewmodel::EditResult r = controller_.keep(*d);
    message(r.ok ? "Kept." : r.message, !r.ok);
    const bool ok = finish(r);
    updateButtons();
    return ok && r.changed;
}

void SculptPanel::discard() {
    controller_.discard();
    message("Discarded the strokes since the last Keep.", false);
    showWorking();
}

bool SculptPanel::undoStroke() {
    const bool ok = controller_.undoStroke();
    showWorking();
    return ok;
}

bool SculptPanel::paintPress(relief::Point at) {
    if (!sculpting()) {
        return false;
    }
    const viewmodel::EditResult r = controller_.beginStroke(brush_, at);
    if (!r.ok) {
        message(r.message, true);
        return false;
    }
    showWorking();
    return true;
}

bool SculptPanel::paintMove(relief::Point at) {
    if (!controller_.strokeOpen()) {
        return false;
    }
    const viewmodel::EditResult r = controller_.extendStroke(at);
    if (!r.ok) {
        message(r.message, true);
        return false;
    }
    showWorking();
    return true;
}

void SculptPanel::paintRelease() {
    controller_.endStroke();
    updateButtons();
}

void SculptPanel::refresh() {
    if (!sculpting()) {
        return;
    }
    const document::Document* d = doc();
    if (d == nullptr || d->findReliefComponent(controller_.target()) == nullptr) {
        stopSculpting();
        message("The sculpted component is gone; sculpting stopped.", true);
        return;
    }
    if (!controller_.dirty() && !controller_.documentMatches(*d)) {
        // An outside edit (an Undo of a Keep, a Modify tool): follow the document.
        const document::ComponentId target = controller_.target();
        if (controller_.begin(*d, target).ok) {
            rebuildMerged();
            showWorking();
        }
        return;
    }
    // Another part of the model may have changed: compose it again (once, not per move).
    if (!controller_.strokeOpen()) {
        rebuildMerged();
        showWorking();
        return;
    }
    updateButtons();
}

// --- LevelEffectsPanel -------------------------------------------------------------------------------

LevelEffectsPanel::LevelEffectsPanel(DocumentGetter getter, QWidget* parent) : PanelBase(std::move(getter), parent) {
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(4, 4, 4, 4);
    outer->addWidget(new QLabel(QStringLiteral("Level effects"), this));
    summary_ = new QLabel(this);
    summary_->setObjectName(QStringLiteral("Effects: Summary"));
    summary_->setWordWrap(true);
    outer->addWidget(summary_);
    auto* clipRow = new QHBoxLayout();
    apply_ = new QPushButton(QStringLiteral("Apply Clipping"), this);
    apply_->setObjectName(QStringLiteral("Effects: Apply Clipping"));
    apply_->setToolTip(QStringLiteral("Clip the level by the selected closed vectors (applying again updates it)"));
    remove_ = new QPushButton(QStringLiteral("Remove Clipping"), this);
    remove_->setObjectName(QStringLiteral("Effects: Remove Clipping"));
    clipRow->addWidget(apply_);
    clipRow->addWidget(remove_);
    outer->addLayout(clipRow);
    auto* mirrorRow = new QHBoxLayout();
    mirrorRow->addWidget(new QLabel(QStringLiteral("Mirror Mode"), this));
    mirror_ = new QComboBox(this);
    mirror_->setObjectName(QStringLiteral("Effects: Mirror Mode"));
    for (const relief::MirrorMode m : viewmodel::allMirrorModes()) {
        mirror_->addItem(QString::fromUtf8(viewmodel::mirrorModeName(m)), static_cast<int>(m));
    }
    mirrorRow->addWidget(mirror_, 1);
    outer->addLayout(mirrorRow);
    bake_ = new QPushButton(QStringLiteral("Bake Effects"), this);
    bake_->setObjectName(QStringLiteral("Effects: Bake"));
    bake_->setToolTip(QStringLiteral("Make the clipping and mirror permanent in one merged component"));
    outer->addWidget(bake_);
    connect(apply_, &QPushButton::clicked, this, [this]() { applyClip(); });
    connect(remove_, &QPushButton::clicked, this, [this]() { removeClip(); });
    connect(mirror_, &QComboBox::activated, this, [this](int i) {
        setMirror(static_cast<relief::MirrorMode>(mirror_->itemData(i).toInt()));
    });
    connect(bake_, &QPushButton::clicked, this, [this]() { bake(); });
    refresh();
}

document::LevelId LevelEffectsPanel::level() const {
    const document::Document* d = doc();
    return d != nullptr ? viewmodel::effectsTargetLevel(*d, picked_) : document::LevelId{};
}

void LevelEffectsPanel::refresh() {
    refreshing_ = true;
    const document::Document* d = doc();
    std::optional<viewmodel::LevelEffectsSummary> s;
    if (d != nullptr) {
        s = viewmodel::summarizeEffects(*d, level());
    }
    if (!s) {
        summary_->setText(QStringLiteral("No relief level."));
    } else {
        summary_->setText(qs(s->text()));
        const int idx = mirror_->findData(static_cast<int>(s->mirror));
        mirror_->setCurrentIndex(idx);
    }
    apply_->setEnabled(s.has_value());
    remove_->setEnabled(s && s->clipPolygons > 0);
    bake_->setEnabled(s && (s->clipPolygons > 0 || s->mirror != relief::MirrorMode::None));
    mirror_->setEnabled(s.has_value());
    refreshing_ = false;
}

bool LevelEffectsPanel::applyClip() {
    document::Document* d = doc();
    return d != nullptr && finish(viewmodel::applyLevelClip(*d, level()));
}

bool LevelEffectsPanel::removeClip() {
    document::Document* d = doc();
    return d != nullptr && finish(viewmodel::removeLevelClip(*d, level()));
}

bool LevelEffectsPanel::setMirror(relief::MirrorMode mode) {
    document::Document* d = doc();
    const bool ok = d != nullptr && finish(viewmodel::setLevelMirror(*d, level(), mode));
    refresh();  // a refused choice puts the box back
    return ok;
}

bool LevelEffectsPanel::bake() {
    document::Document* d = doc();
    return d != nullptr && finish(viewmodel::bakeLevelEffects(*d, level()));
}

}  // namespace carvetto::app
