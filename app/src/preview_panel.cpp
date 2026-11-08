#include "preview_panel.hpp"

#include <QAction>
#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QSlider>
#include <QThread>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>

#include "canvas_widget.hpp"
#include "relief_view.hpp"
#include "toolpaths_panel.hpp"

namespace carvetto::app {

namespace {

constexpr int kSliderMax = 10000;
constexpr int kTimerMs = 30;
constexpr int kMaxDiagnosticLines = 300;

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

QString clockText(double seconds) {
    return qs(viewmodel::formatDuration(seconds));
}

const char* diagnosticName(simulate::DiagnosticKind kind) {
    switch (kind) {
        case simulate::DiagnosticKind::RapidInMaterial:
            return "Rapid in material";
        case simulate::DiagnosticKind::BelowTable:
            return "Below the table";
        case simulate::DiagnosticKind::ToolTooDeep:
            return "Tool too deep";
        case simulate::DiagnosticKind::ToolChangeWithoutRetract:
            return "Tool change without retract";
    }
    return "";
}

}  // namespace

PreviewPanel::PreviewPanel(ToolpathsPanel* toolpaths, CanvasWidget* canvas, ReliefView* relief, QWidget* parent)
    : QWidget(parent), toolpaths_(toolpaths), canvas_(canvas), relief_(relief) {
    build();
    canvas_->setPreview(&model_);
    model_.setEnabled(true);
    timer_.setInterval(kTimerMs);
    connect(&timer_, &QTimer::timeout, this, [this]() { tick(); });
    // Selecting a toolpath in the Toolpaths dock highlights it on the canvas.
    connect(toolpaths_->list(), &QTreeWidget::currentItemChanged, this, [this]() {
        model_.setSelected(toolpaths_->currentToolpath());
        updateCanvasState();
        canvas_->update();
    });
    refresh();
}

PreviewPanel::~PreviewPanel() {
    sim_.stop();
}

void PreviewPanel::build() {
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(0, 0, 0, 0);
    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    outer->addWidget(scroll);
    auto* body = new QWidget(scroll);
    scroll->setWidget(body);
    auto* col = new QVBoxLayout(body);

    // --- 2D preview ---
    auto* preview = new QGroupBox(QStringLiteral("Toolpath preview (2D view)"), body);
    auto* pv = new QVBoxLayout(preview);
    show_ = new QCheckBox(QStringLiteral("Show toolpaths on the canvas"), preview);
    show_->setChecked(true);
    selectedOnly_ = new QCheckBox(QStringLiteral("Only the selected toolpath"), preview);
    pv->addWidget(show_);
    pv->addWidget(selectedOnly_);
    for (std::size_t i = 0; i < viewmodel::kMoveClassCount; ++i) {
        const auto cls = static_cast<viewmodel::MoveClass>(i);
        const viewmodel::PreviewColour c = viewmodel::colourOf(cls);
        auto* box = new QCheckBox(QString::fromUtf8(viewmodel::nameOf(cls)), preview);
        box->setChecked(true);
        box->setStyleSheet(QStringLiteral("QCheckBox { color: rgb(%1,%2,%3); font-weight: bold; }")
                               .arg(c.r)
                               .arg(c.g)
                               .arg(c.b));
        classChecks_[i] = box;
        pv->addWidget(box);
        connect(box, &QCheckBox::toggled, this, [this]() {
            updateCanvasState();
            canvas_->update();
        });
    }
    list_ = new QTreeWidget(preview);
    list_->setHeaderLabels({QStringLiteral("Show toolpath")});
    list_->setRootIsDecorated(false);
    list_->setMaximumHeight(130);
    pv->addWidget(list_);
    col->addWidget(preview);

    showAction_ = new QAction(QStringLiteral("Show &Toolpath Preview"), this);
    showAction_->setCheckable(true);
    showAction_->setChecked(true);
    showAction_->setShortcut(QKeySequence(QStringLiteral("F5")));
    connect(showAction_, &QAction::toggled, this, [this](bool on) { setShowPreview(on); });
    connect(show_, &QCheckBox::toggled, this, [this](bool on) { setShowPreview(on); });
    connect(selectedOnly_, &QCheckBox::toggled, this, [this](bool on) {
        model_.setSelectedOnly(on);
        canvas_->update();
    });
    connect(list_, &QTreeWidget::itemChanged, this, [this](QTreeWidgetItem* item, int) {
        if (updatingUi_ || item == nullptr) {
            return;
        }
        model_.setVisible(document::ToolpathId{item->data(0, Qt::UserRole).toULongLong()},
                          item->checkState(0) == Qt::Checked);
        canvas_->update();
    });

    // --- time ---
    auto* timeBox = new QGroupBox(QStringLiteral("Machining time"), body);
    auto* tv = new QVBoxLayout(timeBox);
    time_ = new QLabel(timeBox);
    time_->setWordWrap(true);
    time_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    tv->addWidget(time_);
    auto* rateRow = new QHBoxLayout();
    rateRow->addWidget(new QLabel(QStringLiteral("Rapid rate:"), timeBox));
    rapid_ = new QDoubleSpinBox(timeBox);
    rapid_->setRange(0.0, 100000.0);
    rapid_->setDecimals(0);
    rapid_->setSingleStep(500.0);
    rapid_->setSuffix(QStringLiteral(" mm/min"));
    rapid_->setSpecialValueText(QStringLiteral("not set"));
    rapid_->setToolTip(QStringLiteral("The machine's rapid travel rate. Without it rapid moves are left out of the "
                                      "time estimate (the playback assumes 5000 mm/min)."));
    rateRow->addWidget(rapid_);
    rateRow->addStretch();
    tv->addLayout(rateRow);
    col->addWidget(timeBox);
    connect(rapid_, &QDoubleSpinBox::valueChanged, this, [this]() { applyRapidRate(); });

    // --- simulation ---
    auto* simBox = new QGroupBox(QStringLiteral("Simulation (3D View tab)"), body);
    auto* sv = new QVBoxLayout(simBox);
    auto* form = new QFormLayout();
    quality_ = new QComboBox(simBox);
    quality_->addItem(QStringLiteral("Coarse (0.5 mm)"), static_cast<int>(simulate::Quality::Coarse));
    quality_->addItem(QStringLiteral("Medium (0.25 mm)"), static_cast<int>(simulate::Quality::Medium));
    quality_->addItem(QStringLiteral("Fine (0.125 mm)"), static_cast<int>(simulate::Quality::Fine));
    quality_->setCurrentIndex(1);
    colour_ = new QComboBox(simBox);
    colour_->addItem(QStringLiteral("By depth"));
    colour_->addItem(QStringLiteral("By toolpath"));
    form->addRow(QStringLiteral("Quality:"), quality_);
    form->addRow(QStringLiteral("Colour:"), colour_);
    sv->addLayout(form);
    auto* runRow = new QHBoxLayout();
    simulate_ = new QPushButton(QStringLiteral("Simulate"), simBox);
    stop_ = new QPushButton(QStringLiteral("Stop"), simBox);
    clear_ = new QPushButton(QStringLiteral("Clear"), simBox);
    runRow->addWidget(simulate_);
    runRow->addWidget(stop_);
    runRow->addWidget(clear_);
    sv->addLayout(runRow);
    progress_ = new QProgressBar(simBox);
    progress_->setRange(0, 100);
    progress_->setTextVisible(true);
    sv->addWidget(progress_);
    status_ = new QLabel(simBox);
    status_->setWordWrap(true);
    sv->addWidget(status_);
    notes_ = new QLabel(simBox);
    notes_->setWordWrap(true);
    sv->addWidget(notes_);

    scrubber_ = new QSlider(Qt::Horizontal, simBox);
    scrubber_->setRange(0, kSliderMax);
    sv->addWidget(scrubber_);
    auto* playRow = new QHBoxLayout();
    stepBack_ = new QPushButton(QStringLiteral("|<"), simBox);
    play_ = new QPushButton(QStringLiteral("Play"), simBox);
    stepForward_ = new QPushButton(QStringLiteral(">|"), simBox);
    stepBack_->setToolTip(QStringLiteral("Step back one move"));
    stepForward_->setToolTip(QStringLiteral("Step forward one move"));
    stepBack_->setAutoRepeat(true);
    stepForward_->setAutoRepeat(true);
    speed_ = new QComboBox(simBox);
    for (const double s : viewmodel::PlaybackModel::kSpeeds) {
        speed_->addItem(QStringLiteral("%1x").arg(s), s);
    }
    speed_->setCurrentIndex(3);  // 10x
    speed_->setToolTip(QStringLiteral("Playback speed relative to machine time"));
    playRow->addWidget(stepBack_);
    playRow->addWidget(play_);
    playRow->addWidget(stepForward_);
    playRow->addWidget(speed_);
    sv->addLayout(playRow);
    diagnostics_ = new QListWidget(simBox);
    diagnostics_->setMaximumHeight(110);
    diagnostics_->setToolTip(QStringLiteral("Simulation warnings: rapids through material, cuts below the table, ..."));
    sv->addWidget(diagnostics_);
    col->addWidget(simBox);
    col->addStretch();

    connect(simulate_, &QPushButton::clicked, this, [this]() { startSimulation(); });
    connect(stop_, &QPushButton::clicked, this, [this]() {
        playback_.pause();
        sim_.cancel();
        updateControls();
    });
    connect(clear_, &QPushButton::clicked, this, [this]() { clearSimulation(); });
    connect(play_, &QPushButton::clicked, this, [this]() {
        if (!playbackReady_) {
            return;
        }
        playback_.toggle();
        clock_.restart();
        seekToPlayback();
        updateControls();
    });
    connect(stepBack_, &QPushButton::clicked, this, [this]() {
        playback_.pause();
        playback_.stepBack();
        seekToPlayback();
        updateControls();
    });
    connect(stepForward_, &QPushButton::clicked, this, [this]() {
        playback_.pause();
        playback_.stepForward();
        seekToPlayback();
        updateControls();
    });
    connect(speed_, &QComboBox::currentIndexChanged, this,
            [this](int) { playback_.setSpeed(speed_->currentData().toDouble()); });
    connect(colour_, &QComboBox::currentIndexChanged, this, [this](int) {
        viewmodel::ReliefOptions o;
        o.colourBy = colour_->currentIndex() == 1 ? viewmodel::ReliefOptions::ColourBy::Toolpath
                                                  : viewmodel::ReliefOptions::ColourBy::Depth;
        sim_.setReliefOptions(o);
    });
    connect(scrubber_, &QSlider::valueChanged, this, [this](int v) { onSliderMoved(v); });
    playback_.setSpeed(10.0);
    updateControls();
}

void PreviewPanel::setShowPreview(bool on) {
    model_.setEnabled(on);
    for (QObject* w : {static_cast<QObject*>(show_), static_cast<QObject*>(showAction_)}) {
        w->blockSignals(true);
    }
    show_->setChecked(on);
    showAction_->setChecked(on);
    show_->blockSignals(false);
    showAction_->blockSignals(false);
    updateCanvasState();
    canvas_->update();
}

geometry::Box2 PreviewPanel::stockBox() const {
    const simulate::Stock& s = sim_.setup().stock;
    return geometry::Box2(s.origin, geometry::Vec2{s.origin.x + s.width, s.origin.y + s.height});
}

void PreviewPanel::refresh() {
    model_.sync(toolpaths_->session());
    model_.setSelected(toolpaths_->currentToolpath());
    rebuildToolpathList();
    updateTime();
    if (sim_.active()) {
        const viewmodel::SimulationValidity v = viewmodel::checkSimulationCurrent(sim_.setup(), toolpaths_->session());
        simCurrent_ = v.current;
        simReason_ = v.reason;
        relief_->setBanner(v.current ? QString() : qs("Out of date: " + v.reason + " Press Simulate to update."));
    } else {
        simCurrent_ = false;
        simReason_.clear();
        relief_->setBanner(QString());
    }
    updateCanvasState();
    updateControls();
    canvas_->update();
}

void PreviewPanel::detach() {
    sim_.stop();
    timer_.stop();
    playback_.reset(nullptr);
    playbackReady_ = false;
    simCurrent_ = false;
    simReason_.clear();
    shownSerial_ = 0;
    lastSeek_ = static_cast<std::size_t>(-1);
    model_.clear();
}

void PreviewPanel::documentReplaced() {
    detach();
    setupProblem_.clear();
    diagnostics_->clear();
    notes_->clear();
    model_.clear();
    relief_->setFrame(nullptr, geometry::Box2{});
    relief_->setBanner(QString());
    relief_->setTool(std::nullopt, 0.0);
    relief_->setStatus(QString());
    listSignature_ = "-";  // forces the list to be rebuilt (and emptied)
    refresh();
}

void PreviewPanel::shutdown() {
    timer_.stop();
    playback_.pause();
    sim_.stop();
}

void PreviewPanel::rebuildToolpathList() {
    std::string sig;
    for (const viewmodel::PreviewToolpath& tp : model_.scene().toolpaths) {
        sig += std::to_string(tp.id.value) + '|' + tp.name + '|' + (tp.stale ? 's' : 'c') + (tp.enabled ? 'e' : 'd') +
               (tp.computed ? 'm' : 'n') + (model_.isVisible(tp.id) ? 'v' : 'h') + '\n';
    }
    if (sig == listSignature_) {
        return;
    }
    listSignature_ = sig;
    updatingUi_ = true;
    list_->clear();
    for (const viewmodel::PreviewToolpath& tp : model_.scene().toolpaths) {
        QString text = qs(tp.name);
        if (!tp.computed) {
            text += QStringLiteral(" (not computed)");
        } else if (!tp.enabled) {
            text += QStringLiteral(" (disabled)");
        } else if (tp.stale) {
            text += QStringLiteral(" (needs recalculation)");
        }
        auto* item = new QTreeWidgetItem(list_);
        item->setText(0, text);
        item->setData(0, Qt::UserRole, static_cast<qulonglong>(tp.id.value));
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(0, model_.isVisible(tp.id) ? Qt::Checked : Qt::Unchecked);
    }
    updatingUi_ = false;
}

void PreviewPanel::updateTime() {
    viewmodel::ToolpathSession& session = toolpaths_->session();
    if (!updatingUi_) {
        updatingUi_ = true;
        rapid_->setValue(session.machineSetup().rapidRateMmPerMin);
        updatingUi_ = false;
    }
    const viewmodel::TimeReadout r = viewmodel::makeTimeReadout(session.summary());
    QString text = qs(r.headline);
    if (!r.breakdown.empty()) {
        text += QStringLiteral("\n") + qs(r.breakdown);
    }
    for (const std::string& w : r.warnings) {
        text += QStringLiteral("\n- ") + qs(w);
    }
    if (!r.perToolpath.empty()) {
        text += QStringLiteral("\nPer toolpath:");
        for (const std::string& line : r.perToolpath) {
            text += QStringLiteral("\n  ") + qs(line);
        }
    }
    time_->setText(text);
}

void PreviewPanel::applyRapidRate() {
    if (updatingUi_) {
        return;
    }
    viewmodel::MachineSetup setup = toolpaths_->session().machineSetup();
    setup.rapidRateMmPerMin = rapid_->value();
    if (toolpaths_->session().setMachineSetup(setup)) {
        toolpaths_->refresh();  // the dock's summary line uses the same rates
        refresh();
    }
}

bool PreviewPanel::startSimulation() {
    if (toolpaths_->busy()) {
        status_->setText(QStringLiteral("Wait for the toolpath calculation to finish."));
        return false;
    }
    viewmodel::SimulationRequest request;
    request.quality = static_cast<simulate::Quality>(quality_->currentData().toInt());
    viewmodel::SimulationSetup setup = viewmodel::makeSimulationSetup(toolpaths_->session(), request);
    if (!setup.ok) {
        setupProblem_ = qs(setup.problem);
        status_->setText(setupProblem_);
        return false;
    }
    viewmodel::ReliefOptions options;
    options.colourBy = colour_->currentIndex() == 1 ? viewmodel::ReliefOptions::ColourBy::Toolpath
                                                    : viewmodel::ReliefOptions::ColourBy::Depth;
    setupProblem_.clear();
    setupNotes_ = setup.notes;
    sim_.stop();
    playback_.reset(nullptr);
    playbackReady_ = false;
    shownSerial_ = 0;
    lastSeek_ = static_cast<std::size_t>(-1);
    diagnostics_->clear();
    relief_->setFrame(nullptr, geometry::Box2{});
    if (!sim_.start(std::move(setup), options)) {
        status_->setText(QStringLiteral("The simulation could not start."));
        return false;
    }
    relief_->setEmptyText(QStringLiteral("Preparing the simulation..."));
    QString notes;
    for (const std::string& n : setupNotes_) {
        notes += (notes.isEmpty() ? QString() : QStringLiteral("\n")) + qs(n);
    }
    notes_->setText(notes);
    clock_.restart();
    timer_.start();
    refresh();
    return true;
}

void PreviewPanel::clearSimulation() {
    sim_.stop();
    timer_.stop();
    playback_.reset(nullptr);
    playbackReady_ = false;
    shownSerial_ = 0;
    lastSeek_ = static_cast<std::size_t>(-1);
    diagnostics_->clear();
    notes_->clear();
    status_->clear();
    relief_->setFrame(nullptr, geometry::Box2{});
    relief_->setTool(std::nullopt, 0.0);
    relief_->setStatus(QString());
    relief_->setBanner(QString());
    relief_->setEmptyText(QStringLiteral("Simulation view: run Simulate in the Preview panel to see the cut stock."));
    refresh();
}

void PreviewPanel::scrubTo(std::size_t move) {
    if (!playbackReady_) {
        return;
    }
    playback_.pause();
    playback_.setPosition(move);
    seekToPlayback();
    updateControls();
}

void PreviewPanel::seekToPlayback() {
    if (!sim_.active() || !playbackReady_) {
        return;
    }
    if (playback_.position() != lastSeek_) {
        lastSeek_ = playback_.position();
        sim_.seek(lastSeek_);
    }
    updateCanvasState();
    canvas_->update();
}

void PreviewPanel::onSliderMoved(int value) {
    if (updatingUi_ || !playbackReady_) {
        return;
    }
    playback_.pause();
    playback_.setFraction(static_cast<double>(value) / kSliderMax);
    seekToPlayback();
    updateControls();
}

void PreviewPanel::tick() {
    if (!sim_.active()) {
        timer_.stop();
        return;
    }
    const double dt = static_cast<double>(clock_.restart()) / 1000.0;
    if (!playbackReady_) {
        if (const auto meta = sim_.meta()) {
            playback_.reset(meta->moveEndTimes);
            playback_.setSpeed(speed_->currentData().toDouble());
            playbackReady_ = true;
            lastSeek_ = 0;
        }
    }
    if (playbackReady_ && playback_.playing() && playback_.tick(dt)) {
        seekToPlayback();
    }
    const auto frame = sim_.frame();
    if (frame && frame->serial != shownSerial_) {
        showFrame(frame);
    }
    // A stopped run ends where it got to: the scrubber follows.
    const viewmodel::SimProgress progress = sim_.progress();
    if (playbackReady_ && progress.cancelled && progress.state == viewmodel::SimState::Ready && frame &&
        frame->move != playback_.position()) {
        playback_.pause();
        playback_.setPosition(frame->move);
        lastSeek_ = frame->move;
    }
    updateStatus();
    updateControls();
    updateCanvasState();  // repaints the canvas only when the state it shows changed
}

void PreviewPanel::showFrame(const std::shared_ptr<const viewmodel::SimulationFrame>& frame) {
    shownSerial_ = frame->serial;
    relief_->setFrame(frame, stockBox());
    diagnostics_->clear();
    int shown = 0;
    for (const simulate::SimDiagnostic& d : frame->diagnostics) {
        if (shown++ >= kMaxDiagnosticLines) {
            break;
        }
        diagnostics_->addItem(QStringLiteral("%1 (move %2): %3")
                                  .arg(QString::fromUtf8(diagnosticName(d.kind)))
                                  .arg(d.move + 1)
                                  .arg(qs(d.message)));
    }
    const std::size_t extra = frame->diagnostics.size() > static_cast<std::size_t>(kMaxDiagnosticLines)
                                  ? frame->diagnostics.size() - kMaxDiagnosticLines
                                  : 0;
    if (extra + frame->droppedDiagnostics > 0) {
        diagnostics_->addItem(QStringLiteral("... and %1 more").arg(extra + frame->droppedDiagnostics));
    }
}

void PreviewPanel::updateStatus() {
    const viewmodel::SimProgress p = sim_.progress();
    QString text;
    switch (p.state) {
        case viewmodel::SimState::Idle:
            break;
        case viewmodel::SimState::Preparing:
            text = QStringLiteral("Preparing the simulation...");
            progress_->setRange(0, 0);
            break;
        case viewmodel::SimState::Working:
            progress_->setRange(0, 100);
            progress_->setValue(static_cast<int>(p.fraction() * 100.0));
            text = QStringLiteral("Simulating... move %1 of %2").arg(p.movesDone).arg(p.moveCount);
            break;
        case viewmodel::SimState::Ready:
            progress_->setRange(0, 100);
            progress_->setValue(100);
            text = p.cancelled ? QStringLiteral("Stopped at move %1 of %2").arg(p.movesDone).arg(p.moveCount)
                               : QStringLiteral("Move %1 of %2").arg(p.movesDone).arg(p.moveCount);
            break;
        case viewmodel::SimState::Failed:
            progress_->setRange(0, 100);
            progress_->setValue(0);
            text = QStringLiteral("The simulation failed: ") + qs(sim_.error());
            break;
    }
    if (!simReason_.empty() && sim_.active()) {
        text += QStringLiteral("\nOut of date: ") + qs(simReason_);
    }
    status_->setText(text);
    const auto frame = sim_.frame();
    if (frame && playbackReady_) {
        QString line = QStringLiteral("Move %1 / %2 - %3 / %4").arg(frame->move).arg(playback_.moveCount())
                           .arg(clockText(frame->seconds), clockText(playback_.totalSeconds()));
        if (!frame->diagnostics.empty()) {
            line += QStringLiteral(" - %1 warning(s)").arg(frame->diagnostics.size() + frame->droppedDiagnostics);
        }
        relief_->setStatus(line);
    }
}

void PreviewPanel::updateCanvasState() {
    PreviewPaintState state;
    for (std::size_t i = 0; i < viewmodel::kMoveClassCount; ++i) {
        state.classVisible[i] = classChecks_[i]->isChecked();
    }
    const bool follow = sim_.active() && playbackReady_ && simCurrent_;
    if (follow) {
        state.playback = true;
        state.playedMoves = playback_.position();
        state.toolPosMm = model_.positionAfter(playback_.position());
        // Diameter of the tool of the toolpath the position is in.
        std::size_t first = 0;
        for (const viewmodel::SimulationToolpath& tp : sim_.setup().toolpaths) {
            const std::size_t n = tp.moves ? tp.moves->size() : 0;
            const std::size_t at = playback_.position() == 0 ? 0 : playback_.position() - 1;
            if (at >= first && at < first + n) {
                state.toolDiameterMm = tp.tool.diameter;
                break;
            }
            first += n;
        }
        relief_->setTool(state.toolPosMm, state.toolDiameterMm);
    } else {
        relief_->setTool(std::nullopt, 0.0);
    }
    canvas_->setPreviewState(state);
}

void PreviewPanel::updateControls() {
    const bool active = sim_.active();
    const viewmodel::SimProgress p = sim_.progress();
    const bool working = active && (p.state == viewmodel::SimState::Working || p.state == viewmodel::SimState::Preparing);
    simulate_->setEnabled(!toolpaths_->busy());
    stop_->setEnabled(working);
    clear_->setEnabled(active);
    const bool ready = active && playbackReady_ && p.state != viewmodel::SimState::Failed;
    scrubber_->setEnabled(ready);
    stepBack_->setEnabled(ready && playback_.position() > 0);
    stepForward_->setEnabled(ready && !playback_.atEnd());
    play_->setEnabled(ready);
    play_->setText(playback_.playing() ? QStringLiteral("Pause") : QStringLiteral("Play"));
    speed_->setEnabled(true);
    if (!active) {
        progress_->setRange(0, 100);
        progress_->setValue(0);
    }
    updatingUi_ = true;
    scrubber_->setValue(ready ? static_cast<int>(std::lround(playback_.fraction() * kSliderMax)) : 0);
    updatingUi_ = false;
}

bool PreviewPanel::waitSimulation(int timeoutMs) {
    QElapsedTimer t;
    t.start();
    while (t.elapsed() < timeoutMs) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
        const viewmodel::SimProgress p = sim_.progress();
        if (p.state == viewmodel::SimState::Failed) {
            return false;
        }
        const auto frame = sim_.frame();
        if (p.state == viewmodel::SimState::Ready && frame && playbackReady_ && frame->move == playback_.position() &&
            frame->serial == shownSerial_ && !playback_.playing()) {
            return true;
        }
        QThread::msleep(2);
    }
    return false;
}

}  // namespace carvetto::app
