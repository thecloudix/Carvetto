#include "main_window.hpp"

#include <QAction>
#include <QActionGroup>
#include <QByteArray>
#include <QCloseEvent>
#include <QComboBox>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QDockWidget>
#include <QGridLayout>
#include <QInputDialog>
#include <QKeySequence>
#include <QLabel>
#include <QListWidget>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QSettings>
#include <QStatusBar>
#include <QStringList>
#include <QString>
#include <QTabWidget>
#include <QToolBar>
#include <QVariant>
#include <QWidget>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "array_copy_menu.hpp"
#include "canvas_widget.hpp"
#include "carvetto/viewmodel/dxf_exchange.hpp"
#include "carvetto/viewmodel/new_job_form.hpp"
#include "carvetto/viewmodel/units_format.hpp"
#include "new_job_dialog.hpp"
#include "node_edit_menu.hpp"
#include "ruler_widget.hpp"
#include "tool_panel.hpp"
#include "vector_ops_menu.hpp"

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

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setWindowTitle(QStringLiteral("Carvetto"));
    setObjectName(QStringLiteral("CarvettoMainWindow"));
    setDockOptions(QMainWindow::AnimatedDocks | QMainWindow::AllowTabbedDocks);

    buildCentralArea();
    buildDocks();
    buildMenus();
    buildToolBar();
    buildToolPalette();

    cursorLabel_ = new QLabel(this);
    cursorLabel_->setMinimumWidth(220);
    jobLabel_ = new QLabel(this);
    statusBar()->addWidget(cursorLabel_);
    statusBar()->addPermanentWidget(jobLabel_);

    resize(1280, 800);
    canvas_->setDocument(&session_.document());
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
        cursorLabel_->setText(qs(viewmodel::formatCursorPosition(mm, session_.document().job().displayUnits)));
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

    toolPanel_ = new ToolPanel(canvas_, this);
    designDock_ = makeDock("Design", "designDock", toolPanel_);
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
    QAction* openAction = file->addAction(QStringLiteral("&Open..."));
    openAction->setShortcut(QKeySequence::Open);
    connect(openAction, &QAction::triggered, this, [this]() { openDialog(); });
    recentMenu_ = file->addMenu(QStringLiteral("Open &Recent"));
    file->addSeparator();
    QAction* saveAction = file->addAction(QStringLiteral("&Save"));
    saveAction->setShortcut(QKeySequence::Save);
    connect(saveAction, &QAction::triggered, this, [this]() { saveDialog(); });
    QAction* saveAsAction = file->addAction(QStringLiteral("Save &As..."));
    saveAsAction->setShortcut(QKeySequence::SaveAs);
    connect(saveAsAction, &QAction::triggered, this, [this]() { saveAsDialog(); });
    file->addSeparator();
    QAction* importAction = file->addAction(QStringLiteral("&Import DXF..."));
    importAction->setShortcut(QKeySequence(QStringLiteral("Ctrl+I")));
    connect(importAction, &QAction::triggered, this, [this]() { importDxfDialog(); });
    QAction* exportSheet = file->addAction(QStringLiteral("&Export Sheet as DXF..."));
    connect(exportSheet, &QAction::triggered, this, [this]() { exportDxfDialog(false); });
    QAction* exportSelection = file->addAction(QStringLiteral("Export &Selection as DXF..."));
    connect(exportSelection, &QAction::triggered, this, [this]() { exportDxfDialog(true); });
    file->addSeparator();
    QAction* quitAction = file->addAction(QStringLiteral("E&xit"));
    quitAction->setShortcut(QKeySequence::Quit);
    connect(quitAction, &QAction::triggered, this, [this]() { close(); });
    rebuildRecentMenu();

    QMenu* edit = menuBar()->addMenu(QStringLiteral("&Edit"));
    undoAction_ = edit->addAction(QStringLiteral("&Undo"));
    undoAction_->setShortcut(QKeySequence::Undo);
    connect(undoAction_, &QAction::triggered, this, [this]() { undo(); });
    redoAction_ = edit->addAction(QStringLiteral("&Redo"));
    redoAction_->setShortcut(QKeySequence::Redo);
    connect(redoAction_, &QAction::triggered, this, [this]() { redo(); });
    edit->addSeparator();
    vectorOps_ = std::make_unique<VectorOpsMenu>(*this, edit);
    copies_ = std::make_unique<CopiesMenu>(*this, edit);

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

void MainWindow::buildToolPalette() {
    struct Entry {
        viewmodel::ToolKind kind;
        const char* text;
        const char* tip;
    };
    const Entry entries[] = {
        {viewmodel::ToolKind::Select, "Select", "Select objects (click, or drag a marquee)"},
        {viewmodel::ToolKind::Rectangle, "Rectangle", "Draw a rectangle (Alt: from centre, Ctrl: square)"},
        {viewmodel::ToolKind::Circle, "Circle", "Draw a circle (Alt: drag the diameter)"},
        {viewmodel::ToolKind::Ellipse, "Ellipse", "Draw an ellipse (Alt: from centre, Ctrl: circle)"},
        {viewmodel::ToolKind::Polygon, "Polygon", "Draw a regular polygon"},
        {viewmodel::ToolKind::Star, "Star", "Draw a star"},
        {viewmodel::ToolKind::Arc, "Arc", "Draw an arc"},
        {viewmodel::ToolKind::Polyline, "Polyline",
         "Draw lines (Shift: 45 degrees, Enter/double-click: finish, Tab: close, Backspace: undo point)"},
        {viewmodel::ToolKind::NodeEdit, "Node Edit",
         "Edit nodes, control points and spans (N). Drag; Shift: add; Ctrl+drag moves a span; right click: menu"},
    };
    QToolBar* bar = addToolBar(QStringLiteral("Tools"));
    bar->setObjectName(QStringLiteral("toolsToolBar"));
    bar->setMovable(false);
    toolGroup_ = new QActionGroup(this);
    toolGroup_->setExclusive(true);
    for (const Entry& e : entries) {
        QAction* action = bar->addAction(QString::fromUtf8(e.text));
        action->setCheckable(true);
        action->setToolTip(QString::fromUtf8(e.tip));
        toolGroup_->addAction(action);
        toolActions_[e.kind] = action;
        if (e.kind == viewmodel::ToolKind::NodeEdit) {
            action->setShortcut(QKeySequence(QStringLiteral("N")));
        }
        const viewmodel::ToolKind kind = e.kind;
        connect(action, &QAction::triggered, this, [this, kind]() {
            canvas_->tools().setTool(kind);
            toolPanel_->rebuild();
            canvas_->update();
        });
    }
    toolActions_[viewmodel::ToolKind::Select]->setChecked(true);

    canvas_->onDocumentEdited = [this]() {
        documentChanged();
        toolPanel_->refreshValues();
    };
    canvas_->onToolStateChanged = [this]() { toolPanel_->refreshValues(); };

    nodeEdit_ = std::make_unique<NodeEditMenu>(*this);
    canvas_->onNodeContextMenu = [this](const QPoint& globalPos, geometry::Vec2 px) {
        nodeEdit_->showContextMenu(globalPos, px);
    };
    canvas_->onNodeProperties = [this]() { nodeEdit_->showProperties(); };
    canvas_->onNodeMessage = [this](const std::string& text) {
        if (!text.empty()) {
            statusBar()->showMessage(qs(text), 6000);
        }
    };
}

void MainWindow::selectTool(viewmodel::ToolKind tool) {
    const auto it = toolActions_.find(tool);
    if (it != toolActions_.end()) {
        it->second->setChecked(true);
        it->second->trigger();
    }
}

QAction* MainWindow::toolAction(viewmodel::ToolKind tool) const {
    const auto it = toolActions_.find(tool);
    return it == toolActions_.end() ? nullptr : it->second;
}

void MainWindow::setDocument(std::unique_ptr<document::Document> doc) {
    if (!doc) {
        return;
    }
    session_.newDocument(std::move(doc));
    canvas_->setDocument(&session_.document());
    toolPanel_->rebuild();
    documentChanged();
}

void MainWindow::documentChanged() {
    setWindowTitle(qs(session_.title("Carvetto")));
    refreshSelectors();
    refreshStatus();
    const document::Units units = session_.document().job().displayUnits;
    hRuler_->setUnits(units);
    vRuler_->setUnits(units);
    undoAction_->setEnabled(session_.document().undoStack().canUndo());
    redoAction_->setEnabled(session_.document().undoStack().canRedo());
    undoAction_->setText(session_.document().undoStack().canUndo()
                             ? QStringLiteral("&Undo ") + qs(session_.document().undoStack().undoText())
                             : QStringLiteral("&Undo"));
    redoAction_->setText(session_.document().undoStack().canRedo()
                             ? QStringLiteral("&Redo ") + qs(session_.document().undoStack().redoText())
                             : QStringLiteral("&Redo"));
    canvas_->refresh();
}

bool MainWindow::undo() {
    const bool done = session_.document().undo();
    documentChanged();
    return done;
}

bool MainWindow::redo() {
    const bool done = session_.document().redo();
    documentChanged();
    return done;
}

void MainWindow::refreshSelectors() {
    refreshing_ = true;
    sheetCombo_->clear();
    sheetsList_->clear();
    int activeSheetRow = 0;
    for (std::size_t i = 0; i < session_.document().sheets().size(); ++i) {
        const document::Sheet& sheet = session_.document().sheets()[i];
        sheetCombo_->addItem(qs(sheet.name));
        sheetsList_->addItem(qs(sheet.name));
        if (sheet.id == session_.document().activeSheet()) {
            activeSheetRow = static_cast<int>(i);
        }
    }
    sheetCombo_->setCurrentIndex(activeSheetRow);
    sheetsList_->setCurrentRow(activeSheetRow);

    layerCombo_->clear();
    layersList_->clear();
    int activeLayerRow = 0;
    for (std::size_t i = 0; i < session_.document().layers().size(); ++i) {
        const document::Layer& layer = session_.document().layers()[i];
        layerCombo_->addItem(qs(layer.name));
        layersList_->addItem(qs(layer.name));
        if (layer.id == session_.document().activeLayer()) {
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
    if (refreshing_ || row < 0 || static_cast<std::size_t>(row) >= session_.document().sheets().size()) {
        return;
    }
    session_.document().setActiveSheet(session_.document().sheets()[static_cast<std::size_t>(row)].id);
    refreshing_ = true;
    sheetCombo_->setCurrentIndex(row);
    sheetsList_->setCurrentRow(row);
    refreshing_ = false;
    canvas_->zoomToMaterial();
    canvas_->refresh();
}

void MainWindow::pickLayer(int row) {
    if (refreshing_ || row < 0 || static_cast<std::size_t>(row) >= session_.document().layers().size()) {
        return;
    }
    session_.document().setActiveLayer(session_.document().layers()[static_cast<std::size_t>(row)].id);
    refreshing_ = true;
    layerCombo_->setCurrentIndex(row);
    layersList_->setCurrentRow(row);
    refreshing_ = false;
}

void MainWindow::refreshStatus() {
    const document::Job& job = session_.document().job();
    const document::Units u = job.displayUnits;
    const std::string text =
        "Job " + viewmodel::formatLength(job.sizeMm.x, u) + " x " + viewmodel::formatLength(job.sizeMm.y, u) +
        " x " + viewmodel::formatLength(job.sizeMm.z, u) + " " + std::string(document::unitSuffix(u));
    jobLabel_->setText(qs(text));
}

void MainWindow::newJob() {
    if (!confirmDiscard()) {
        return;
    }
    NewJobDialog dialog(viewmodel::formFromJob(session_.document().job()), this);
    if (dialog.exec() != QDialog::Accepted || !dialog.job().has_value()) {
        return;
    }
    setDocument(std::make_unique<document::Document>(*dialog.job()));
}

void MainWindow::setPersistLayout(bool persist) {
    persistLayout_ = persist;
    if (persist) {
        QSettings settings;
        std::vector<std::filesystem::path> paths;
        for (const QString& s : settings.value(QStringLiteral("recentFiles")).toStringList()) {
            paths.push_back(viewmodel::pathFromUtf8(s.toStdString()));
        }
        session_.setRecentFiles(paths);
        rebuildRecentMenu();
    }
}

void MainWindow::storeRecent() {
    if (persistLayout_) {
        QStringList list;
        for (const auto& p : session_.recentFiles()) {
            list << qs(viewmodel::pathToUtf8(p));
        }
        QSettings settings;
        settings.setValue(QStringLiteral("recentFiles"), list);
    }
    rebuildRecentMenu();
}

void MainWindow::rebuildRecentMenu() {
    recentMenu_->clear();
    for (const auto& p : session_.recentFiles()) {
        QAction* action = recentMenu_->addAction(qs(viewmodel::pathToUtf8(p)));
        connect(action, &QAction::triggered, this, [this, p]() {
            if (confirmDiscard()) {
                openFile(p);
            }
        });
    }
    recentMenu_->setEnabled(!recentMenu_->isEmpty());
}

void MainWindow::showError(const QString& title) {
    if (!quiet_) {
        QMessageBox::critical(this, title, qs(session_.lastError()));
    }
}

bool MainWindow::openFile(const std::filesystem::path& path) {
    const bool ok = session_.open(path);
    if (ok) {
        canvas_->setDocument(&session_.document());
        documentChanged();
    } else {
        showError(QStringLiteral("Open failed"));
    }
    storeRecent();
    return ok;
}

bool MainWindow::saveFile() {
    if (!session_.hasPath()) {
        return saveAsDialog();
    }
    const bool ok = session_.save();
    if (!ok) {
        showError(QStringLiteral("Save failed"));
    }
    storeRecent();
    documentChanged();
    return ok;
}

bool MainWindow::saveFileAs(const std::filesystem::path& path) {
    const bool ok = session_.saveAs(path);
    if (!ok) {
        showError(QStringLiteral("Save failed"));
    }
    storeRecent();
    documentChanged();
    return ok;
}

bool MainWindow::importDxfFile(const std::filesystem::path& path, std::optional<double> forcedMmPerUnit) {
    const bool ok = session_.importDxf(path, forcedMmPerUnit);
    if (!ok) {
        showError(QStringLiteral("Import failed"));
        return false;
    }
    documentChanged();
    const viewmodel::DxfImportOutcome& report = session_.lastImport();
    if (!quiet_) {
        QString text = QStringLiteral("Imported %1 object(s) and %2 new layer(s).")
                           .arg(static_cast<qulonglong>(report.objectsAdded))
                           .arg(static_cast<qulonglong>(report.layersAdded));
        if (report.warnings.empty()) {
            QMessageBox::information(this, QStringLiteral("DXF import"), text);
        } else {
            QStringList lines;
            for (const std::string& w : report.warnings) {
                lines << qs(w);
            }
            QMessageBox box(QMessageBox::Warning, QStringLiteral("DXF import"),
                            text + QStringLiteral(" There were warnings."), QMessageBox::Ok, this);
            box.setDetailedText(lines.join(QLatin1Char('\n')));
            box.exec();
        }
    }
    return true;
}

bool MainWindow::exportDxfFile(const std::filesystem::path& path, bool selectionOnly) {
    const bool ok = session_.exportDxf(path, selectionOnly);
    if (!ok) {
        showError(QStringLiteral("Export failed"));
    }
    return ok;
}

bool MainWindow::confirmDiscard() {
    if (!session_.isDirty() || quiet_) {
        return true;
    }
    QMessageBox box(QMessageBox::Warning, QStringLiteral("Unsaved changes"),
                    QStringLiteral("Save changes to %1?").arg(qs(session_.displayName())),
                    QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel, this);
    box.setDefaultButton(QMessageBox::Save);
    switch (box.exec()) {
        case QMessageBox::Save:
            return saveFile();
        case QMessageBox::Discard:
            return true;
        default:
            return false;
    }
}

void MainWindow::openDialog() {
    if (!confirmDiscard()) {
        return;
    }
    const QString name = QFileDialog::getOpenFileName(this, QStringLiteral("Open"), QString(),
                                                      QStringLiteral("Carvetto files (*.carv);;All files (*)"));
    if (!name.isEmpty()) {
        openFile(viewmodel::pathFromUtf8(name.toStdString()));
    }
}

bool MainWindow::saveDialog() {
    return saveFile();
}

bool MainWindow::saveAsDialog() {
    const QString name = QFileDialog::getSaveFileName(this, QStringLiteral("Save As"), qs(session_.displayName()),
                                                      QStringLiteral("Carvetto files (*.carv)"));
    if (name.isEmpty()) {
        return false;
    }
    return saveFileAs(viewmodel::withCarvExtension(viewmodel::pathFromUtf8(name.toStdString())));
}

void MainWindow::importDxfDialog() {
    const QString name = QFileDialog::getOpenFileName(this, QStringLiteral("Import DXF"), QString(),
                                                      QStringLiteral("DXF drawings (*.dxf);;All files (*)"));
    if (name.isEmpty()) {
        return;
    }
    const auto& choices = viewmodel::dxfUnitChoices();
    QStringList labels;
    for (const auto& choice : choices) {
        labels << qs(choice.label);
    }
    bool accepted = false;
    const QString picked = QInputDialog::getItem(this, QStringLiteral("DXF units"),
                                                 QStringLiteral("Drawing units:"), labels, 0, false, &accepted);
    if (!accepted) {
        return;
    }
    const int row = static_cast<int>(labels.indexOf(picked));
    const std::optional<double> scale =
        row >= 0 ? choices[static_cast<std::size_t>(row)].mmPerUnit : std::optional<double>{};
    importDxfFile(viewmodel::pathFromUtf8(name.toStdString()), scale);
}

void MainWindow::exportDxfDialog(bool selectionOnly) {
    const QString name = QFileDialog::getSaveFileName(this, QStringLiteral("Export DXF"), QString(),
                                                      QStringLiteral("DXF drawings (*.dxf)"));
    if (name.isEmpty()) {
        return;
    }
    std::filesystem::path path = viewmodel::pathFromUtf8(name.toStdString());
    if (!path.has_extension()) {
        path += ".dxf";
    }
    exportDxfFile(path, selectionOnly);
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
    if (!confirmDiscard()) {
        event->ignore();
        return;
    }
    if (persistLayout_) {
        QSettings settings;
        saveLayout(settings);
    }
    QMainWindow::closeEvent(event);
}

}  // namespace carvetto::app
