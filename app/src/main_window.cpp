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
#include <span>
#include <QStringList>
#include <QString>
#include <QTabWidget>
#include <QToolBar>
#include <QVariant>
#include <QWidget>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "array_copy_menu.hpp"
#include "canvas_widget.hpp"
#include "document_panels.hpp"
#include "carvetto/viewmodel/dxf_exchange.hpp"
#include "carvetto/viewmodel/file_session.hpp"
#include "carvetto/viewmodel/new_job_form.hpp"
#include "carvetto/viewmodel/starter_font.hpp"
#include "carvetto/viewmodel/text_tool.hpp"
#include "carvetto/viewmodel/units_format.hpp"
#include "new_job_dialog.hpp"
#include "node_edit_menu.hpp"
#include "save_toolpaths_dialog.hpp"
#include "preview_panel.hpp"
#include "properties_panel.hpp"
#include "relief_view.hpp"
#include "ruler_widget.hpp"
#include "tool_panel.hpp"
#include "toolpaths_panel.hpp"
#include "transform_menu.hpp"
#include "trace_dialog.hpp"
#include "texture_menu.hpp"
#include "keepout_panel.hpp"
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

    relief_ = new ReliefView(this);

    views_ = new QTabWidget(this);
    views_->setDocumentMode(true);
    views_->addTab(page2d, QStringLiteral("2D View"));
    views_->addTab(relief_, QStringLiteral("3D View"));
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
    const DocumentGetter currentDoc = [this]() { return &session_.document(); };
    sheetsPanel_ = new SheetsPanel(currentDoc, this);
    sheetsDock_ = makeDock("Sheets", "sheetsDock", sheetsPanel_);
    layersPanel_ = new LayersPanel(currentDoc, this);
    layersDock_ = makeDock("Layers", "layersDock", layersPanel_);
    properties_ = new PropertiesPanel(currentDoc, this);
    propertiesDock_ = makeDock("Properties", "propertiesDock", properties_);
    historyPanel_ = new HistoryPanel(currentDoc, this);
    historyDock_ = makeDock("History", "historyDock", historyPanel_);
    for (PanelBase* panel : {static_cast<PanelBase*>(sheetsPanel_), static_cast<PanelBase*>(layersPanel_),
                             static_cast<PanelBase*>(properties_), static_cast<PanelBase*>(historyPanel_)}) {
        panel->onEdited = [this]() { documentChanged(); };
        panel->onMessage = [this](const std::string& text) { statusBar()->showMessage(qs(text), 6000); };
    }
    historyPanel_->historyBlocked = [this]() { return toolpaths_ != nullptr && toolpaths_->busy(); };
    sheetsPanel_->onPick = [this](int row) { pickSheet(row); };
    layersPanel_->onPick = [this](int row) { pickLayer(row); };
    componentsDock_ =
        makeDock("Components", "componentsDock", placeholder("3D components will be listed here.", this));
    libraryDock_ = makeDock("Library", "libraryDock", placeholder("Clipart and project library.", this));
    toolpaths_ = new ToolpathsPanel(&session_.document(), this);
    toolpaths_->onDocumentEdited = [this]() { documentChanged(); };
    toolpaths_->onSelectionChanged = [this]() {
        if (properties_ != nullptr) {
            properties_->refresh();
        }
        if (transformMenu_) {
            transformMenu_->refresh();
        }
        canvas_->refresh();
    };
    toolpaths_->onBusyChanged = [this]() {
        if (undoAction_ != nullptr) {
            documentChanged();  // Undo / Redo are disabled while a calculation runs
        }
    };
    toolpathsDock_ = makeDock("Toolpaths", "toolpathsDock", toolpaths_);
    preview_ = new PreviewPanel(toolpaths_, canvas_, relief_, this);
    previewDock_ = makeDock("Preview", "previewDock", preview_);
    toolpaths_->onSetupChanged = [this]() {
        preview_->refresh();  // the time readout uses the active machine's rates
        if (persistLayout_) {
            QSettings settings;
            settings.setValue(QStringLiteral("activeMachine"), qs(toolpaths_->activeMachineId()));
        }
    };

    keepOut_ = new KeepOutPanel(currentDoc, this);
    keepOutDock_ = makeDock("Keep-out Zones", "keepOutDock", keepOut_);
    keepOut_->onEdited = [this]() { documentChanged(); };
    keepOut_->onMessage = [this](const std::string& text) { statusBar()->showMessage(qs(text), 6000); };
    keepOut_->routingText = [this]() {
        const toolpath::RoutingReport& r = preview_->model().scene().routing;
        std::string text;
        if (r.routed != 0) {
            text += std::to_string(r.routed) + " rapid move(s) routed round the zones.";
        }
        if (r.unroutable != 0) {
            text += (text.empty() ? "" : " ") + std::to_string(r.unroutable) + " rapid move(s) cannot be routed.";
        }
        if (r.tooComplex != 0) {
            text += (text.empty() ? "" : " ") + std::to_string(r.tooComplex) + " rapid move(s) not routed (zones too complex).";
        }
        return text;
    };

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
    addDockWidget(Qt::LeftDockWidgetArea, propertiesDock_);
    addDockWidget(Qt::LeftDockWidgetArea, historyDock_);
    tabifyDockWidget(libraryDock_, propertiesDock_);
    tabifyDockWidget(propertiesDock_, historyDock_);
    addDockWidget(Qt::RightDockWidgetArea, toolpathsDock_);
    addDockWidget(Qt::RightDockWidgetArea, previewDock_);
    tabifyDockWidget(toolpathsDock_, previewDock_);
    addDockWidget(Qt::RightDockWidgetArea, keepOutDock_);
    tabifyDockWidget(previewDock_, keepOutDock_);
    toolpathsDock_->raise();
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
    saveToolpathsAction_ = file->addAction(QStringLiteral("Save &Toolpaths..."));
    connect(saveToolpathsAction_, &QAction::triggered, this, [this]() { saveToolpathsDialog(); });
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
    traceBitmap_ = std::make_unique<TraceBitmapMenu>(*this, file, edit);
    textureMenu_ = std::make_unique<TextureMenu>(*this, edit);
    transformMenu_ = std::make_unique<TransformMenu>(*this);
    keepOut_->buildMenu(edit);
    edit->addSeparator();
    QAction* convertText = edit->addAction(QStringLiteral("Convert &Text to Curves"));
    connect(convertText, &QAction::triggered, this, [this]() {
        const viewmodel::TextEditResult r = canvas_->tools().textEditor().convertSelectionToCurves();
        if (r.committed) {
            documentChanged();
        } else {
            statusBar()->showMessage(qs(r.message), 6000);
        }
    });

    QMenu* view = menuBar()->addMenu(QStringLiteral("&View"));
    QAction* fit = view->addAction(QStringLiteral("Zoom to &Material"));
    fit->setShortcut(QKeySequence(QStringLiteral("F4")));
    connect(fit, &QAction::triggered, this, [this]() { canvas_->zoomToMaterial(); });
    view->addSeparator();
    QMenu* panels = view->addMenu(QStringLiteral("&Panels"));
    for (QDockWidget* dock :
         {designDock_, sheetsDock_, layersDock_, propertiesDock_, historyDock_, componentsDock_, libraryDock_,
          toolpathsDock_, previewDock_, keepOutDock_}) {
        panels->addAction(dock->toggleViewAction());
    }
    view->addAction(preview_->showAction());
    QAction* simView = view->addAction(QStringLiteral("&Simulation View"));
    simView->setCheckable(true);
    simView->setShortcut(QKeySequence(QStringLiteral("F6")));
    connect(simView, &QAction::toggled, this, [this](bool on) { showView(on); });
    connect(views_, &QTabWidget::currentChanged, this, [simView](int index) {
        simView->blockSignals(true);
        simView->setChecked(index == 1);
        simView->blockSignals(false);
    });

    QMenu* setup = menuBar()->addMenu(QStringLiteral("&Setup"));
    jobSetupAction_ = setup->addAction(QStringLiteral("&Job Setup..."));
    jobSetupAction_->setObjectName(QStringLiteral("jobSetupAction"));
    connect(jobSetupAction_, &QAction::triggered, this, [this]() { jobSetupDialog(); });
    toolSetupAction_ = setup->addAction(QStringLiteral("&Tools, Machines and Materials..."));
    toolSetupAction_->setObjectName(QStringLiteral("toolSetupAction"));
    connect(toolSetupAction_, &QAction::triggered, this, [this]() { toolSetupDialog(); });

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
        {viewmodel::ToolKind::Text, "Text",
         "Place text: click (point text), drag a box (text in a box) or click a curve (text on a curve). "
         "Type; Enter: new line, Ctrl+Enter: place, Escape: cancel. Click a text to edit it"},
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
    canvas_->onSelectionChanged = [this]() {
        properties_->refresh();
        if (transformMenu_) {
            transformMenu_->refresh();  // keeps the shortcut actions (M, T, R, H, V, F9) usable
        }
        if (keepOut_ != nullptr) {
            keepOut_->refresh();
        }
    };


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
    std::unique_ptr<document::Document> incoming = std::move(doc);
    replaceDocument([this, &incoming]() {
        session_.newDocument(std::move(incoming));
        return true;
    });
}

bool MainWindow::replaceDocument(const std::function<bool()>& swap) {
    // The one path that swaps the Document (File > New, Open, setDocument). The file session frees the
    // old document inside `swap`; nothing may dereference it from then on. So, in this order:
    //  1. repaints are blocked for the whole window (a progress bar or label changing value repaints
    //     synchronously, and a dirty canvas would be painted with the dead document);
    //  2. every holder is detached: the canvas (and its tool controller) drops its pointer, the
    //     preview stops its simulation and forgets its scene (no widget is touched);
    //  3. the swap runs; 4. every holder is pointed at the current document; 5. only then refresh.
    const bool wasEnabled = updatesEnabled();
    setUpdatesEnabled(false);
    canvas_->setDocument(nullptr);
    preview_->detach();
    const bool ok = swap();
    if (ok) {
        toolpaths_->setDocument(&session_.document());
    }
    preview_->documentReplaced();
    canvas_->setDocument(&session_.document());
    if (ok) {
        toolPanel_->rebuild();
    }
    documentChanged();
    setUpdatesEnabled(wasEnabled);
    return ok;
}

void MainWindow::documentChanged() {
    if (transformMenu_) {
        transformMenu_->refresh();
    }
    setWindowTitle(qs(session_.title("Carvetto")));
    refreshSelectors();
    refreshStatus();
    const document::Units units = session_.document().job().displayUnits;
    hRuler_->setUnits(units);
    vRuler_->setUnits(units);
    const bool calculating = toolpaths_ != nullptr && toolpaths_->busy();
    undoAction_->setEnabled(session_.document().undoStack().canUndo() && !calculating);
    redoAction_->setEnabled(session_.document().undoStack().canRedo() && !calculating);
    undoAction_->setText(session_.document().undoStack().canUndo()
                             ? QStringLiteral("&Undo ") + qs(session_.document().undoStack().undoText())
                             : QStringLiteral("&Undo"));
    redoAction_->setText(session_.document().undoStack().canRedo()
                             ? QStringLiteral("&Redo ") + qs(session_.document().undoStack().redoText())
                             : QStringLiteral("&Redo"));
    canvas_->refresh();
    if (properties_ != nullptr) {
        properties_->refresh();
        historyPanel_->refresh();
    }
    if (toolpaths_ != nullptr) {
        toolpaths_->refresh();
    }
    if (preview_ != nullptr) {
        preview_->refresh();
    }
    if (keepOut_ != nullptr) {
        keepOut_->refresh();  // after the preview: it shows the routing counts
    }
}

void MainWindow::showView(bool simulation) {
    views_->setCurrentIndex(simulation ? 1 : 0);
}

bool MainWindow::simulationViewShown() const {
    return views_->currentIndex() == 1;
}

bool MainWindow::undo() {
    if (toolpaths_ != nullptr && toolpaths_->busy()) {
        return false;  // a toolpath calculation is running: it would be applied to a different state
    }
    const bool done = session_.document().undo();
    documentChanged();
    return done;
}

bool MainWindow::redo() {
    if (toolpaths_ != nullptr && toolpaths_->busy()) {
        return false;
    }
    const bool done = session_.document().redo();
    documentChanged();
    return done;
}

void MainWindow::refreshSelectors() {
    refreshing_ = true;
    sheetCombo_->clear();
    int activeSheetRow = 0;
    for (std::size_t i = 0; i < session_.document().sheets().size(); ++i) {
        const document::Sheet& sheet = session_.document().sheets()[i];
        sheetCombo_->addItem(qs(sheet.name));
        if (sheet.id == session_.document().activeSheet()) {
            activeSheetRow = static_cast<int>(i);
        }
    }
    sheetCombo_->setCurrentIndex(activeSheetRow);

    layerCombo_->clear();
    int activeLayerRow = 0;
    for (std::size_t i = 0; i < session_.document().layers().size(); ++i) {
        const document::Layer& layer = session_.document().layers()[i];
        layerCombo_->addItem(qs(layer.name));
        if (layer.id == session_.document().activeLayer()) {
            activeLayerRow = static_cast<int>(i);
        }
    }
    layerCombo_->setCurrentIndex(activeLayerRow);
    sheetsPanel_->refresh();
    layersPanel_->refresh();
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
    sheetsPanel_->setCurrentRow(row);
    refreshing_ = false;
    canvas_->zoomToMaterial();
    canvas_->refresh();
    toolpaths_->invalidate();  // the active sheet decides which vectors a toolpath machines
    properties_->refresh();
}

void MainWindow::pickLayer(int row) {
    if (refreshing_ || row < 0 || static_cast<std::size_t>(row) >= session_.document().layers().size()) {
        return;
    }
    session_.document().setActiveLayer(session_.document().layers()[static_cast<std::size_t>(row)].id);
    refreshing_ = true;
    layerCombo_->setCurrentIndex(row);
    layersPanel_->setCurrentRow(row);
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
        loadUserSetup(settings.value(QStringLiteral("activeMachine")).toString());
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

void MainWindow::saveToolpathsDialog() {
    const std::filesystem::path path = session_.path();
    const std::filesystem::path folder = path.empty() ? viewmodel::pathFromUtf8(QDir::homePath().toStdString())
                                                      : path.parent_path();
    const std::string job = path.empty() ? std::string("toolpaths") : viewmodel::pathToUtf8(path.stem());
    // Enabled toolpaths are posted; uncalculated ones are named in a warning, out-of-date ones need a tick.
    viewmodel::SaveSource source = viewmodel::makeSaveSource(toolpaths_->session());
    if (source.records.empty()) {
        if (!quiet_) {
            QMessageBox::information(this, QStringLiteral("Save Toolpaths"),
                                     source.notComputed.empty()
                                         ? QStringLiteral("There are no enabled toolpaths to save.")
                                         : QStringLiteral("The enabled toolpaths have not been calculated yet: "
                                                          "recalculate them first."));
        }
        return;
    }
    std::vector<tooling::Machine> machines;
    for (const auto& entry : toolpaths_->session().toolDatabase().machines()) {
        machines.push_back(entry.second);
    }
    SaveToolpathsDialog dialog(std::move(source), job, machines, folder, this);
    dialog.exec();
}

void MainWindow::showError(const QString& title) {
    if (!quiet_) {
        QMessageBox::critical(this, title, qs(session_.lastError()));
    }
}

bool MainWindow::openFile(const std::filesystem::path& path) {
    const bool ok = replaceDocument([this, &path]() { return session_.open(path); });
    if (!ok) {
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
    // DXF text has no usable font names (no system fonts): it is set in the Text tool's font, the
    // starter font until the user picks another one.
    const viewmodel::TextEditor& textEditor = canvas_->tools().textEditor();
    const auto starter = viewmodel::starterFontBytes();
    const std::span<const std::uint8_t> textFont = textEditor.hasFont()
                                                       ? std::span<const std::uint8_t>(textEditor.fontBytes())
                                                       : starter;
    const bool ok = session_.importDxf(path, forcedMmPerUnit, textFont);
    if (!ok) {
        showError(QStringLiteral("Import failed"));
        return false;
    }
    documentChanged();
    const viewmodel::DxfImportOutcome& report = session_.lastImport();
    if (!quiet_) {
        QString text = QStringLiteral("Imported %1 object(s), %2 text(s) and %3 new layer(s).")
                           .arg(static_cast<qulonglong>(report.objectsAdded))
                           .arg(static_cast<qulonglong>(report.textAdded))
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
    if (preview_ != nullptr) {
        preview_->shutdown();
    }
    if (toolpaths_ != nullptr) {
        toolpaths_->showClosing();
    }
    QMainWindow::closeEvent(event);
}

}  // namespace carvetto::app
