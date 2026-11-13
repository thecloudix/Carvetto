#include "trace_dialog.hpp"

#include <QAction>
#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDialogButtonBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMenu>
#include <QMessageBox>
#include <QMetaObject>
#include <QPainter>
#include <QPainterPath>
#include <QPixmap>
#include <QProgressBar>
#include <QPushButton>
#include <QSpinBox>
#include <QStatusBar>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <span>

#include "canvas_widget.hpp"
#include "carvetto/document/bitmap_store.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "main_window.hpp"
#include "trace_image.hpp"

namespace carvetto::app {

namespace {

constexpr int kPreviewW = 380;
constexpr int kPreviewH = 300;

QString qs(const std::string& s) { return QString::fromStdString(s); }

QPixmap renderPreview(const std::vector<io::TraceLayer>& layers, io::TraceMode mode, double widthMm, double heightMm,
                      geometry::Vec2 origin) {
    QPixmap pm(kPreviewW, kPreviewH);
    pm.fill(QColor(0xee, 0xee, 0xee));
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing, true);
    const double scale = std::min((kPreviewW - 10.0) / widthMm, (kPreviewH - 10.0) / heightMm);
    const double offX = (kPreviewW - widthMm * scale) / 2.0, offY = (kPreviewH - heightMm * scale) / 2.0;
    p.fillRect(QRectF(offX, offY, widthMm * scale, heightMm * scale), Qt::white);
    geometry::Tolerance tol;
    tol.chordal = std::max(0.25 / scale, 1.0e-6);  // quarter of a screen pixel
    for (const io::TraceLayer& layer : layers) {
        QPainterPath path;
        path.setFillRule(Qt::OddEvenFill);
        for (const geometry::Contour& c : layer.contours) {
            const geometry::Polyline pl = c.polygonise(tol);
            if (pl.points.size() < 2) continue;
            QPolygonF poly;
            for (const geometry::Vec2& v : pl.points) {
                poly << QPointF(offX + (v.x - origin.x) * scale, offY + (heightMm - (v.y - origin.y)) * scale);
            }
            path.addPolygon(poly);
            path.closeSubpath();
        }
        p.setPen(Qt::NoPen);
        p.setBrush(mode == io::TraceMode::BlackWhite ? QColor(30, 30, 30) : QColor(layer.r, layer.g, layer.b));
        p.drawPath(path);
    }
    p.setPen(QColor(0x88, 0x88, 0x88));
    p.setBrush(Qt::NoBrush);
    p.drawRect(QRectF(offX, offY, widthMm * scale, heightMm * scale));
    return pm;
}

}  // namespace

TraceBitmapDialog::TraceBitmapDialog(io::RasterImage image, const QString& fileName, document::Units units, QWidget* parent)
    : QDialog(parent), units_(units) {
    setWindowTitle(QStringLiteral("Trace Bitmap"));
    imageWidth_ = image.width;
    imageHeight_ = image.height;
    fullStorage_ = std::move(image);
    full_ = std::make_shared<const io::RasterImage>(std::move(fullStorage_));
    viewmodel::PreviewImage pv = viewmodel::previewImage(*full_);
    previewScale_ = pv.scale;
    small_ = std::make_shared<const io::RasterImage>(std::move(pv.image));

    mode_ = new QComboBox(this);
    mode_->addItem(QStringLiteral("Black and white"), static_cast<int>(io::TraceMode::BlackWhite));
    mode_->addItem(QStringLiteral("Colour"), static_cast<int>(io::TraceMode::Colour));
    threshold_ = new QSpinBox(this);
    threshold_->setRange(0, 255);
    threshold_->setValue(128);
    invert_ = new QCheckBox(QStringLiteral("Trace the light areas"), this);
    colours_ = new QSpinBox(this);
    colours_->setRange(2, 16);
    colours_->setValue(8);
    noise_ = new QSpinBox(this);
    noise_->setRange(0, 1000);
    noise_->setValue(2);
    noise_->setSuffix(QStringLiteral(" px"));
    cornerFit_ = new QSpinBox(this);
    cornerFit_->setRange(0, 100);
    cornerFit_->setValue(50);
    cornerFit_->setSuffix(QStringLiteral(" %  (0 smooth, 100 sharp corners)"));
    sizeBasis_ = new QComboBox(this);
    sizeBasis_->addItem(QStringLiteral("Size of one pixel"), static_cast<int>(viewmodel::TraceSizeBasis::PerPixel));
    sizeBasis_->addItem(QStringLiteral("Result width"), static_cast<int>(viewmodel::TraceSizeBasis::TargetWidth));
    sizeBasis_->addItem(QStringLiteral("Result height"), static_cast<int>(viewmodel::TraceSizeBasis::TargetHeight));
    size_ = new QDoubleSpinBox(this);
    size_->setDecimals(4);
    size_->setRange(0.0, 1.0e6);
    size_->setValue(0.1);
    const std::string_view suffix = document::unitSuffix(units_);
    size_->setSuffix(QStringLiteral(" ") + QString::fromUtf8(suffix.data(), static_cast<int>(suffix.size())));
    sizeInfo_ = new QLabel(this);
    pickButton_ = new QPushButton(QStringLiteral("Pick from 2D View..."), this);
    pickButton_->setToolTip(QStringLiteral("Choose a bitmap placed in the drawing and trace that instead"));
    fade_ = new QSpinBox(this);
    fade_->setRange(0, 100);
    fade_->setValue(60);
    fade_->setSuffix(QStringLiteral(" %"));
    fade_->setToolTip(QStringLiteral("Dims the picked bitmap on the 2D view while this dialog is open"));
    fade_->setEnabled(false);
    preview_ = new QLabel(this);
    preview_->setFixedSize(kPreviewW, kPreviewH);
    preview_->setFrameShape(QFrame::Box);
    status_ = new QLabel(this);
    status_->setWordWrap(true);
    busy_ = new QProgressBar(this);
    busy_->setRange(0, 0);
    busy_->setTextVisible(false);
    busy_->setVisible(false);

    auto* form = new QFormLayout;
    form->addRow(QStringLiteral("Image"), new QLabel(QStringLiteral("%1 (%2 x %3 px)").arg(QFileInfo(fileName).fileName()).arg(imageWidth_).arg(imageHeight_), this));
    form->addRow(QStringLiteral("Mode"), mode_);
    form->addRow(QStringLiteral("Threshold"), threshold_);
    form->addRow(invert_);
    form->addRow(QStringLiteral("Colours"), colours_);
    form->addRow(QStringLiteral("Noise filter"), noise_);
    form->addRow(QStringLiteral("Corner fit"), cornerFit_);
    form->addRow(QStringLiteral("Size by"), sizeBasis_);
    form->addRow(QStringLiteral("Size"), size_);
    form->addRow(sizeInfo_);
    form->addRow(QStringLiteral("Bitmap fading"), fade_);
    form->addRow(pickButton_);
    auto* buttons = new QDialogButtonBox(this);
    traceButton_ = buttons->addButton(QStringLiteral("Trace"), QDialogButtonBox::AcceptRole);
    cancelButton_ = buttons->addButton(QDialogButtonBox::Cancel);
    connect(traceButton_, &QPushButton::clicked, this, [this]() { accept(); });
    connect(cancelButton_, &QPushButton::clicked, this, [this]() { reject(); });
    connect(pickButton_, &QPushButton::clicked, this, [this]() { requestPick(); });
    connect(fade_, qOverload<int>(&QSpinBox::valueChanged), this, [this]() { applyFade(); });
    auto* left = new QVBoxLayout;
    left->addLayout(form);
    left->addStretch(1);
    auto* right = new QVBoxLayout;
    right->addWidget(preview_);
    right->addWidget(status_);
    right->addWidget(busy_);
    auto* top = new QHBoxLayout;
    top->addLayout(left);
    top->addLayout(right);
    auto* layout = new QVBoxLayout(this);
    layout->addLayout(top);
    layout->addWidget(buttons);

    debounce_ = new QTimer(this);
    debounce_->setSingleShot(true);
    debounce_->setInterval(250);
    connect(debounce_, &QTimer::timeout, this, [this]() { refreshPreview(); });
    for (QSpinBox* s : {threshold_, colours_, noise_, cornerFit_}) {
        connect(s, qOverload<int>(&QSpinBox::valueChanged), this, [this]() { settingsChanged(); });
    }
    connect(size_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this]() { settingsChanged(); });
    connect(mode_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]() { settingsChanged(); });
    connect(sizeBasis_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]() { settingsChanged(); });
    connect(invert_, &QCheckBox::toggled, this, [this]() { settingsChanged(); });
    updateEnabling();
    settingsChanged();
}

TraceBitmapDialog::~TraceBitmapDialog() {
    restoreFade();
    closing_ = true;
    runner_.cancel();  // the runner's destructor joins; the core stops within milliseconds
}

viewmodel::TraceForm TraceBitmapDialog::form() const {
    viewmodel::TraceForm f;
    f.mode = static_cast<io::TraceMode>(mode_->currentData().toInt());
    f.threshold = threshold_->value();
    f.invert = invert_->isChecked();
    f.colourCount = colours_->value();
    f.noiseFilter = noise_->value();
    f.cornerFit = cornerFit_->value() / 100.0;
    f.sizeBasis = static_cast<viewmodel::TraceSizeBasis>(sizeBasis_->currentData().toInt());
    f.sizeValue = size_->value();
    f.units = units_;
    f.originMm = origin_;
    return f;
}

void TraceBitmapDialog::setForm(const viewmodel::TraceForm& f) {
    mode_->setCurrentIndex(mode_->findData(static_cast<int>(f.mode)));
    threshold_->setValue(f.threshold);
    invert_->setChecked(f.invert);
    colours_->setValue(f.colourCount);
    noise_->setValue(f.noiseFilter);
    cornerFit_->setValue(static_cast<int>(std::lround(f.cornerFit * 100.0)));
    sizeBasis_->setCurrentIndex(sizeBasis_->findData(static_cast<int>(f.sizeBasis)));
    size_->setValue(f.sizeValue);
}

void TraceBitmapDialog::setSourceFromView(geometry::Vec2 originMm, CanvasWidget* canvas) {
    origin_ = originMm;
    canvas_ = canvas;
    fade_->setEnabled(canvas_ != nullptr && !tracing_);
    applyFade();
    settingsChanged();  // the preview is placed by the origin
}

int TraceBitmapDialog::fadePercent() const { return fade_->value(); }
void TraceBitmapDialog::setFadePercent(int percent) { fade_->setValue(percent); }
bool TraceBitmapDialog::fadeEnabled() const { return fade_->isEnabled(); }

void TraceBitmapDialog::applyFade() {
    if (canvas_ == nullptr) return;
    if (!fadeApplied_) {
        fadeBefore_ = canvas_->bitmapFade();
        fadeApplied_ = true;
    }
    canvas_->setBitmapFade(1.0 - fade_->value() / 100.0);
}

void TraceBitmapDialog::restoreFade() {
    if (canvas_ != nullptr && fadeApplied_) {
        canvas_->setBitmapFade(fadeBefore_);
        fadeApplied_ = false;
    }
}

void TraceBitmapDialog::requestPick() {
    if (tracing_) return;
    closing_ = true;
    runner_.cancel();
    done(kPickRequested);
}

void TraceBitmapDialog::done(int code) {
    restoreFade();
    QDialog::done(code);
}

void TraceBitmapDialog::updateEnabling() {
    fade_->setEnabled(canvas_ != nullptr && !tracing_);
    pickButton_->setEnabled(!tracing_);
    const bool colour = static_cast<io::TraceMode>(mode_->currentData().toInt()) == io::TraceMode::Colour;
    threshold_->setEnabled(!colour && !tracing_);
    invert_->setEnabled(!colour && !tracing_);
    colours_->setEnabled(colour && !tracing_);
    for (QWidget* w : {static_cast<QWidget*>(noise_), static_cast<QWidget*>(cornerFit_), static_cast<QWidget*>(sizeBasis_),
                       static_cast<QWidget*>(size_), static_cast<QWidget*>(mode_)}) {
        w->setEnabled(!tracing_);
    }
    busy_->setVisible(tracing_ || previewRunning_);
}

QString TraceBitmapDialog::problemShown() const {
    return status_->text();
}

void TraceBitmapDialog::settingsChanged() {
    updateEnabling();
    const viewmodel::TraceForm f = form();
    const std::string problem = viewmodel::validateTraceForm(f, imageWidth_, imageHeight_);
    if (!problem.empty()) {
        status_->setText(qs(problem));
        sizeInfo_->clear();
        traceButton_->setEnabled(false);
        debounce_->stop();
        return;
    }
    const double mm = viewmodel::traceMmPerPixel(f, imageWidth_, imageHeight_).value();
    sizeInfo_->setText(QStringLiteral("Result: %1 x %2 %3")
                           .arg(document::toDisplay(mm * imageWidth_, units_), 0, 'f', 3)
                           .arg(document::toDisplay(mm * imageHeight_, units_), 0, 'f', 3)
                           .arg(QString::fromUtf8(document::unitSuffix(units_).data(), static_cast<int>(document::unitSuffix(units_).size()))));
    traceButton_->setEnabled(!tracing_);
    status_->clear();
    if (!tracing_) debounce_->start();
}

bool TraceBitmapDialog::previewRunning() const {
    return previewRunning_ || pendingRestart_ || debounce_->isActive();
}

void TraceBitmapDialog::refreshPreview() {
    if (tracing_) return;
    const viewmodel::TraceForm f = form();
    if (!viewmodel::validateTraceForm(f, imageWidth_, imageHeight_).empty()) return;
    if (runner_.busy()) {
        pendingRestart_ = true;
        runner_.cancel();
        return;
    }
    const io::TraceParams full = viewmodel::toTraceParams(f, imageWidth_, imageHeight_).value();
    previewRunning_ = runner_.start(small_, viewmodel::previewParams(full, previewScale_), [this]() {
        QMetaObject::invokeMethod(this, [this]() { onWorkerDone(); }, Qt::QueuedConnection);
    });
    updateEnabling();
}

void TraceBitmapDialog::onWorkerDone() {
    if (closing_ || !runner_.finished()) return;  // a stale notification: the run now in flight has its own
    std::optional<viewmodel::TraceOutcome> out = runner_.take();
    if (!out) return;
    if (tracing_) {
        tracing_ = false;
        if (out->ok) {
            result_ = std::move(*out);
            resultMode_ = form().mode;
            updateEnabling();
            QDialog::accept();
            return;
        }
        if (!out->cancelled) status_->setText(qs(out->message));
        else status_->setText(QStringLiteral("Trace cancelled."));
        updateEnabling();
        settingsChanged();
        return;
    }
    previewRunning_ = false;
    if (pendingRestart_) {
        pendingRestart_ = false;
        refreshPreview();
        return;
    }
    showPreview(*out);
    updateEnabling();
}

void TraceBitmapDialog::showPreview(const viewmodel::TraceOutcome& outcome) {
    if (!outcome.ok) {
        previewLayers_ = 0;
        preview_->setPixmap(QPixmap());
        status_->setText(qs(outcome.message));
        return;
    }
    previewLayers_ = outcome.layers.size();
    const viewmodel::TraceForm f = form();
    const double mm = viewmodel::traceMmPerPixel(f, imageWidth_, imageHeight_).value();
    preview_->setPixmap(renderPreview(outcome.layers, f.mode, mm * imageWidth_, mm * imageHeight_, f.originMm));
    status_->setText(outcome.layers.empty() ? QStringLiteral("Nothing found with these settings.") : QString());
}

bool TraceBitmapDialog::waitIdle() {
    QElapsedTimer timer;
    timer.start();
    if (debounce_->isActive()) {  // a change is pending: preview now
        debounce_->stop();
        refreshPreview();
    }
    while ((previewRunning_ || pendingRestart_ || tracing_) && timer.elapsed() < 20000) {
        QEventLoop loop;
        QTimer::singleShot(50, &loop, &QEventLoop::quit);  // one bounded wait; worker completion is queued
        loop.exec();
    }
    return !(previewRunning_ || pendingRestart_ || tracing_);
}

void TraceBitmapDialog::startTrace() {
    if (tracing_) return;
    const viewmodel::TraceForm f = form();
    const auto params = viewmodel::toTraceParams(f, imageWidth_, imageHeight_);
    if (!params.ok()) {
        status_->setText(qs(params.error()));
        return;
    }
    debounce_->stop();
    if (runner_.busy()) {  // drop the preview in flight, then trace
        runner_.cancel();
        runner_.take();
        previewRunning_ = false;
        pendingRestart_ = false;
    }
    tracing_ = runner_.start(full_, params.value(), [this]() {
        QMetaObject::invokeMethod(this, [this]() { onWorkerDone(); }, Qt::QueuedConnection);
    });
    status_->setText(QStringLiteral("Tracing..."));
    updateEnabling();
}

void TraceBitmapDialog::cancelTrace() {
    if (tracing_) runner_.cancel();
}

bool TraceBitmapDialog::traceNow() {
    startTrace();
    if (!waitIdle()) {
        runner_.cancel();
        return false;
    }
    return result_.ok;
}

void TraceBitmapDialog::accept() {
    startTrace();
}

void TraceBitmapDialog::reject() {
    if (tracing_) {  // Cancel stops the trace and keeps the dialog open
        runner_.cancel();
        status_->setText(QStringLiteral("Cancelling..."));
        return;
    }
    closing_ = true;
    runner_.cancel();
    QDialog::reject();
}

// ------------------------------------------------------------------ menu

TraceBitmapMenu::TraceBitmapMenu(MainWindow& window, QMenu* fileMenu, QMenu* editMenu) : window_(window) {
    const QString title = QStringLiteral("Trace &Bitmap...");
    fileAction_ = new QAction(title, fileMenu);
    QList<QAction*> actions = fileMenu->actions();
    QAction* before = nullptr;
    if (actions.size() >= 2 && actions[actions.size() - 2]->isSeparator()) before = actions[actions.size() - 2];
    else if (!actions.isEmpty()) before = actions.last();
    fileMenu->insertAction(before, fileAction_);
    editAction_ = editMenu->addAction(title);
    for (QAction* a : {fileAction_, editAction_}) {
        QObject::connect(a, &QAction::triggered, a, [this]() { run(QString()); });
    }
    viewAction_ = editMenu->addAction(QStringLiteral("Trace Bitmap from &Drawing..."));
    QObject::connect(viewAction_, &QAction::triggered, viewAction_, [this]() { runFromView(); });
}

namespace {

/// What one pass of the dialog traces.
struct TraceSource {
    io::RasterImage image;
    QString name;
    std::optional<document::ObjectId> bitmap;
    geometry::Vec2 originMm{0, 0};
    viewmodel::TraceForm form;
    bool haveForm{false};
};

}  // namespace

std::optional<document::ObjectId> TraceBitmapMenu::pickBitmap(QString* error) {
    CanvasWidget* canvas = window_.canvas();
    if (canvas == nullptr) return std::nullopt;
    std::optional<geometry::Vec2> where;
    bool clicked = false;
    QEventLoop loop;
    canvas->beginPick([&](std::optional<geometry::Vec2> w) {
        where = w;
        clicked = w.has_value();
        loop.quit();
    });
    window_.statusBar()->showMessage(QStringLiteral("Click the bitmap to trace (Esc cancels)."));
    loop.exec();
    window_.statusBar()->clearMessage();
    if (!clicked) return std::nullopt;
    const std::optional<document::ObjectId> id = viewmodel::bitmapAtPoint(window_.currentDocument(), *where);
    if (!id && error != nullptr) *error = QStringLiteral("There is no bitmap at that point.");
    return id;
}

bool TraceBitmapMenu::runFromView(QString* error, bool showErrors) {
    QString message;
    const std::optional<document::ObjectId> id = pickBitmap(&message);
    if (!id) {
        if (message.isEmpty()) return false;  // cancelled
        if (error != nullptr) *error = message;
        if (showErrors) QMessageBox::warning(&window_, QStringLiteral("Trace Bitmap"), message);
        return false;
    }
    return traceBitmapObject(*id, error, showErrors);
}

bool TraceBitmapMenu::traceBitmapObject(document::ObjectId first, QString* error, bool showErrors) {
    auto fail = [&](const QString& message) {
        if (error != nullptr) *error = message;
        if (showErrors) QMessageBox::warning(&window_, QStringLiteral("Trace Bitmap"), message);
        return false;
    };
    const document::BitmapDecoder decoder = [](std::span<const std::uint8_t> bytes) -> geometry::ShapeResult<io::RasterImage> {
        DecodedImage d = decodeImageBytes(QByteArray(reinterpret_cast<const char*>(bytes.data()), static_cast<qsizetype>(bytes.size())));
        if (!d.ok) return d.error.toStdString();
        return std::move(d.image);
    };
    const document::Units units = window_.currentDocument().job().displayUnits;
    // Loads the picked object into `src`; an error text when it cannot be traced.
    auto load = [&](document::ObjectId id, TraceSource& src) -> QString {
        const document::Object* obj = window_.currentDocument().findObject(id);
        const document::BitmapObject* bmp = obj != nullptr ? obj->asBitmap() : nullptr;
        if (bmp == nullptr) return QStringLiteral("That object is not a bitmap.");
        auto pixels = document::decodeBitmapObject(window_.currentDocument(), *bmp, decoder);
        if (!pixels.ok()) return qs(pixels.error());
        src.image = std::move(pixels.value());
        src.name = obj->name.empty() ? QStringLiteral("bitmap in the drawing") : qs(obj->name);
        src.bitmap = id;
        src.originMm = bmp->placementMm.min;
        src.form = viewmodel::traceFormForBitmap(*bmp, units, src.form);
        src.haveForm = true;
        return {};
    };
    TraceSource src;
    if (const QString why = load(first, src); !why.isEmpty()) return fail(why);
    for (;;) {
        TraceBitmapDialog dialog(src.image, src.name, units, &window_);
        dialog.setSourceFromView(src.originMm, window_.canvas());
        if (src.haveForm) dialog.setForm(src.form);
        const int rc = dialog.exec();
        if (rc == QDialog::Accepted) return addResult(dialog, error, showErrors);
        if (rc != TraceBitmapDialog::kPickRequested) return false;
        src.form = dialog.form();
        QString message;
        const std::optional<document::ObjectId> id = pickBitmap(&message);
        if (!id) {
            if (!message.isEmpty()) fail(message);
            continue;  // cancelled or missed: back to the dialog with the same bitmap
        }
        TraceSource next;
        next.form = src.form;
        if (const QString why = load(*id, next); !why.isEmpty()) {
            fail(why);
            continue;
        }
        src = std::move(next);
    }
}

bool TraceBitmapMenu::run(const QString& pathIn, QString* error, bool showErrors) {
    auto fail = [&](const QString& message) {
        if (error != nullptr) *error = message;
        if (showErrors) QMessageBox::warning(&window_, QStringLiteral("Trace Bitmap"), message);
        return false;
    };
    QString path = pathIn;
    if (path.isEmpty()) {
        path = QFileDialog::getOpenFileName(&window_, QStringLiteral("Trace Bitmap"), lastFolder_,
                                            QStringLiteral("Images (*.png *.jpg *.jpeg *.bmp)"));
        if (path.isEmpty()) return false;  // cancelled, not an error
        lastFolder_ = QFileInfo(path).absolutePath();
    }
    DecodedImage decoded = decodeImageFile(path);
    if (!decoded.ok) return fail(decoded.error);
    TraceBitmapDialog dialog(std::move(decoded.image), path, window_.currentDocument().job().displayUnits, &window_);
    const int rc = dialog.exec();
    if (rc == TraceBitmapDialog::kPickRequested) return runFromView(error, showErrors);
    if (rc != QDialog::Accepted) return false;
    return addResult(dialog, error, showErrors);
}

bool TraceBitmapMenu::addResult(const TraceBitmapDialog& dialog, QString* error, bool showErrors) {
    const viewmodel::TraceApplyResult applied =
        viewmodel::applyTrace(window_.currentDocument(), dialog.result().layers, dialog.resultMode());
    if (!applied.ok) {
        const QString message = qs(applied.message);
        if (error != nullptr) *error = message;
        if (showErrors) QMessageBox::warning(&window_, QStringLiteral("Trace Bitmap"), message);
        return false;
    }
    window_.documentChanged();
    return true;
}

}  // namespace carvetto::app
