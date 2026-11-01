#include "main_window.hpp"

#include <QAction>
#include <QByteArray>
#include <QCloseEvent>
#include <QComboBox>
#include <QDockWidget>
#include <QGridLayout>
#include <QKeySequence>
#include <QLabel>
#include <QListWidget>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QSettings>
#include <QStatusBar>
#include <QString>
#include <QTabWidget>
#include <QToolBar>
#include <QVariant>
#include <QWidget>
#include <cstddef>
#include <string>
#include <utility>

#include "canvas_widget.hpp"
#include "carvetto/viewmodel/new_job_form.hpp"
#include "carvetto/viewmodel/units_format.hpp"
#include "new_job_dialog.hpp"
#include "ruler_widget.hpp"

namespace carvetto::app {

namespace {

constexpr int kLayoutVersion = 1;
const char* const kGeometryKey = "mainWindow/geometry";
const char* const kStateKey = "mainWindow/state";

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

QLabel* placeholder(const char* text, QWidget* parent) {
    auto* label = new QLabel(QString::fromUtf8(text), parent);
    label->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    label->setWordWrap(true);
    label->setMargin(8);
    return label;
}

}  // namespace

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), doc_(std::make_unique<document::Document>()) {
    setWindowTitle(QStringLiteral("Carvetto"));
    setObjectName(QStringLiteral("CarvettoMainWindow"));
    setDockOptions(QMainWindow::AnimatedDocks | QMainWindow::AllowTabbedDocks);

    buildCentralArea();
    buildDocks();
    buildMenus();
    buildToolBar();

    cursorLabel_ = new QLabel(this);
    cursorLabel_->setMinimumWidth(220);
    jobLabel_ = new QLabel(this);
    statusBar()->addWidget(cursorLabel_);
    statusBar()->addPermanentWidget(jobLabel_);

    resize(1280, 800);
    canvas_->setDocument(doc_.get());
    documentChanged();
}

MainWindow::~MainWindow() = default;

void MainWindow::buildCentralArea() {
    auto* page2d = new QWidget(this);
    canvas_ = new CanvasWidget(page2d);
    hRuler_ = new RulerWidget(Qt::Horizontal, canvas_, page2d);
    vRuler_ = new RulerWidget(Qt::Vertical, canvas_, page2d);
    auto* corner = new QWidget(page2d);
    corner->setFixedSize(RulerWidget::kThickness, RulerWidget::kThickness);

    auto* grid = new QGridLayout(page2d);
    grid->setContentsMargins(0, 0, 0, 0);
    grid->setSpacing(0);
    grid->addWidget(corner, 0, 0);
    grid->addWidget(hRuler_, 0, 1);
    grid->addWidget(vRuler_, 1, 0);
    grid->addWidget(canvas_, 1, 1);
    grid->setRowStretch(1, 1);
    grid->setColumnStretch(1, 1);

    auto* page3d = placeholder("3D view: not implemented yet.", this);
    page3d->setAlignment(Qt::AlignCenter);

    views_ = new QTabWidget(this);
    views_->setDocumentMode(true);
    views_->addTab(page2d, QStringLiteral("2D View"));
    views_->addTab(page3d, QStringLiteral("3D View"));
    setCentralWidget(views_);

    canvas_->onViewChanged = [this]() {
        hRuler_->update();
        vRuler_->update();
    };
    canvas_->onCursorMoved = [this](geometry::Vec2 mm) {
        cursorLabel_->setText(qs(viewmodel::formatCursorPosition(mm, doc_->job().displayUnits)));
    };
}

void MainWindow::buildDocks() {
    auto makeDock = [this](const char* title, const char* name, QWidget* content) {
        auto* dock = new QDockWidget(QString::fromUtf8(title), this);
        dock->setObjectName(QString::fromUtf8(name));
        dock->setWidget(content);
        dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
        return dock;
    };

    designDock_ =
        makeDock("Design", "designDock", placeholder("Drawing and editing tools will appear here.", this));
    sheetsList_ = new QListWidget(this);
    sheetsDock_ = makeDock("Sheets", "sheetsDock", sheetsList_);
    layersList_ = new QListWidget(this);
    layersDock_ = makeDock("Layers", "layersDock", layersList_);
    componentsDock_ =
        makeDock("Components", "componentsDock", placeholder("3D components will be listed here.", this));
    libraryDock_ = makeDock("Library", "libraryDock", placeholder("Clipart and project library.", this));
    toolpathsDock_ =
        makeDock("Toolpaths", "toolpathsDock", placeholder("Toolpath operations and list.", this));

    addDockWidget(Qt::LeftDockWidgetArea, designDock_);
    addDockWidget(Qt::LeftDockWidgetArea, sheetsDock_);
    addDockWidget(Qt::LeftDockWidgetArea, layersDock_);
    addDockWidget(Qt::LeftDockWidgetArea, componentsDock_);
    addDockWidget(Qt::LeftDockWidgetArea, libraryDock_);
    tabifyDockWidget(designDock_, sheetsDock_);
    tabifyDockWidget(sheetsDock_, layersDock_);
    tabifyDockWidget(layersDock_, componentsDock_);
    tabifyDockWidget(componentsDock_, libraryDock_);
    designDock_->raise();
    addDockWidget(Qt::RightDockWidgetArea, toolpathsDock_);

    connect(sheetsList_, &QListWidget::currentRowChanged, this, [this](int row) { pickSheet(row); });
    connect(layersList_, &QListWidget::currentRowChanged, this, [this](int row) { pickLayer(row); });
}

void MainWindow::buildMenus() {
    QMenu* file = menuBar()->addMenu(QStringLiteral("&File"));
    QAction* newAction = file->addAction(QStringLiteral("&New Job..."));
    newAction->setShortcut(QKeySequence::New);
    connect(newAction, &QAction::triggered, this, [this]() { newJob(); });
    file->addSeparator();
    QAction* quitAction = file->addAction(QStringLiteral("E&xit"));
    quitAction->setShortcut(QKeySequence::Quit);
    connect(quitAction, &QAction::triggered, this, [this]() { close(); });

    QMenu* edit = menuBar()->addMenu(QStringLiteral("&Edit"));
    undoAction_ = edit->addAction(QStringLiteral("&Undo"));
    undoAction_->setShortcut(QKeySequence::Undo);
    connect(undoAction_, &QAction::triggered, this, [this]() { undo(); });
    redoAction_ = edit->addAction(QStringLiteral("&Redo"));
    redoAction_->setShortcut(QKeySequence::Redo);
    connect(redoAction_, &QAction::triggered, this, [this]() { redo(); });

    QMenu* view = menuBar()->addMenu(QStringLiteral("&View"));
    QAction* fit = view->addAction(QStringLiteral("Zoom to &Material"));
    fit->setShortcut(QKeySequence(QStringLiteral("F4")));
    connect(fit, &QAction::triggered, this, [this]() { canvas_->zoomToMaterial(); });
    view->addSeparator();
    QMenu* panels = view->addMenu(QStringLiteral("&Panels"));
    for (QDockWidget* dock :
         {designDock_, sheetsDock_, layersDock_, componentsDock_, libraryDock_, toolpathsDock_}) {
        panels->addAction(dock->toggleViewAction());
    }

    QMenu* help = menuBar()->addMenu(QStringLiteral("&Help"));
    QAction* about = help->addAction(QStringLiteral("&About Carvetto"));
    connect(about, &QAction::triggered, this, [this]() {
        QMessageBox::about(this, QStringLiteral("About Carvetto"),
                           QStringLiteral("Carvetto - open-source CNC design and toolpath software."));
    });
}

void MainWindow::buildToolBar() {
    QToolBar* bar = addToolBar(QStringLiteral("Selectors"));
    bar->setObjectName(QStringLiteral("selectorsToolBar"));
    bar->setMovable(false);
    bar->addWidget(new QLabel(QStringLiteral(" Sheet: "), bar));
    sheetCombo_ = new QComboBox(bar);
    sheetCombo_->setMinimumWidth(140);
    bar->addWidget(sheetCombo_);
    bar->addWidget(new QLabel(QStringLiteral("  Layer: "), bar));
    layerCombo_ = new QComboBox(bar);
    layerCombo_->setMinimumWidth(140);
    bar->addWidget(layerCombo_);

    connect(sheetCombo_, &QComboBox::currentIndexChanged, this, [this](int row) { pickSheet(row); });
    connect(layerCombo_, &QComboBox::currentIndexChanged, this, [this](int row) { pickLayer(row); });
}

void MainWindow::setDocument(std::unique_ptr<document::Document> doc) {
    if (!doc) {
        return;
    }
    doc_ = std::move(doc);
    canvas_->setDocument(doc_.get());
    documentChanged();
}

void MainWindow::documentChanged() {
    refreshSelectors();
    refreshStatus();
    const document::Units units = doc_->job().displayUnits;
    hRuler_->setUnits(units);
    vRuler_->setUnits(units);
    undoAction_->setEnabled(doc_->undoStack().canUndo());
    redoAction_->setEnabled(doc_->undoStack().canRedo());
    undoAction_->setText(doc_->undoStack().canUndo()
                             ? QStringLiteral("&Undo ") + qs(doc_->undoStack().undoText())
                             : QStringLiteral("&Undo"));
    redoAction_->setText(doc_->undoStack().canRedo()
                             ? QStringLiteral("&Redo ") + qs(doc_->undoStack().redoText())
                             : QStringLiteral("&Redo"));
    canvas_->refresh();
}

bool MainWindow::undo() {
    const bool done = doc_->undo();
    documentChanged();
    return done;
}

bool MainWindow::redo() {
    const bool done = doc_->redo();
    documentChanged();
    return done;
}

void MainWindow::refreshSelectors() {
    refreshing_ = true;
    sheetCombo_->clear();
    sheetsList_->clear();
    int activeSheetRow = 0;
    for (std::size_t i = 0; i < doc_->sheets().size(); ++i) {
        const document::Sheet& sheet = doc_->sheets()[i];
        sheetCombo_->addItem(qs(sheet.name));
        sheetsList_->addItem(qs(sheet.name));
        if (sheet.id == doc_->activeSheet()) {
            activeSheetRow = static_cast<int>(i);
        }
    }
    sheetCombo_->setCurrentIndex(activeSheetRow);
    sheetsList_->setCurrentRow(activeSheetRow);

    layerCombo_->clear();
    layersList_->clear();
    int activeLayerRow = 0;
    for (std::size_t i = 0; i < doc_->layers().size(); ++i) {
        const document::Layer& layer = doc_->layers()[i];
        layerCombo_->addItem(qs(layer.name));
        layersList_->addItem(qs(layer.name));
        if (layer.id == doc_->activeLayer()) {
            activeLayerRow = static_cast<int>(i);
        }
    }
    layerCombo_->setCurrentIndex(activeLayerRow);
    layersList_->setCurrentRow(activeLayerRow);
    refreshing_ = false;
}

// Selecting a sheet or layer only syncs the twin selector and repaints; it does not rebuild the
// lists, so no widget is cleared from inside its own change signal.
void MainWindow::pickSheet(int row) {
    if (refreshing_ || row < 0 || static_cast<std::size_t>(row) >= doc_->sheets().size()) {
        return;
    }
    doc_->setActiveSheet(doc_->sheets()[static_cast<std::size_t>(row)].id);
    refreshing_ = true;
    sheetCombo_->setCurrentIndex(row);
    sheetsList_->setCurrentRow(row);
    refreshing_ = false;
    canvas_->zoomToMaterial();
    canvas_->refresh();
}

void MainWindow::pickLayer(int row) {
    if (refreshing_ || row < 0 || static_cast<std::size_t>(row) >= doc_->layers().size()) {
        return;
    }
    doc_->setActiveLayer(doc_->layers()[static_cast<std::size_t>(row)].id);
    refreshing_ = true;
    layerCombo_->setCurrentIndex(row);
    layersList_->setCurrentRow(row);
    refreshing_ = false;
}

void MainWindow::refreshStatus() {
    const document::Job& job = doc_->job();
    const document::Units u = job.displayUnits;
    const std::string text =
        "Job " + viewmodel::formatLength(job.sizeMm.x, u) + " x " + viewmodel::formatLength(job.sizeMm.y, u) +
        " x " + viewmodel::formatLength(job.sizeMm.z, u) + " " + std::string(document::unitSuffix(u));
    jobLabel_->setText(qs(text));
}

void MainWindow::newJob() {
    NewJobDialog dialog(viewmodel::formFromJob(doc_->job()), this);
    if (dialog.exec() != QDialog::Accepted || !dialog.job().has_value()) {
        return;
    }
    setDocument(std::make_unique<document::Document>(*dialog.job()));
}

void MainWindow::saveLayout(QSettings& settings) const {
    settings.setValue(QString::fromUtf8(kGeometryKey), saveGeometry());
    settings.setValue(QString::fromUtf8(kStateKey), saveState(kLayoutVersion));
}

bool MainWindow::restoreLayout(QSettings& settings) {
    const QByteArray geometryBytes = settings.value(QString::fromUtf8(kGeometryKey)).toByteArray();
    const QByteArray stateBytes = settings.value(QString::fromUtf8(kStateKey)).toByteArray();
    if (stateBytes.isEmpty()) {
        return false;
    }
    if (!geometryBytes.isEmpty()) {
        restoreGeometry(geometryBytes);
    }
    return restoreState(stateBytes, kLayoutVersion);
}

void MainWindow::closeEvent(QCloseEvent* event) {
    if (persistLayout_) {
        QSettings settings;
        saveLayout(settings);
    }
    QMainWindow::closeEvent(event);
}

}  // namespace carvetto::app
