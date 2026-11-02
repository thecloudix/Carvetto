#pragma once

#include <QMainWindow>
#include <memory>

#include "carvetto/document/document.hpp"

class QAction;
class QCloseEvent;
class QComboBox;
class QDockWidget;
class QLabel;
class QListWidget;
class QSettings;
class QTabWidget;

namespace carvetto::app {

class CanvasWidget;
class RulerWidget;

/// Main window: 2D/3D view tabs with rulers around the 2D canvas, sheet and layer selectors,
/// docked panels (Design, Sheets, Layers, Components, Library on the left; Toolpaths on the
/// right), File/Edit/View menus and a status bar with the cursor position in display units.
class MainWindow : public QMainWindow {
   public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    document::Document& currentDocument() { return *doc_; }
    const document::Document& currentDocument() const { return *doc_; }
    /// Replaces the open document (the view re-fits to its material).
    void setDocument(std::unique_ptr<document::Document> doc);
    /// Refreshes every view after the document changed.
    void documentChanged();

    /// Undoes / redoes one step and refreshes. Return false when there was nothing to do.
    bool undo();
    bool redo();

    CanvasWidget* canvas() const { return canvas_; }
    QDockWidget* toolpathsDock() const { return toolpathsDock_; }
    QDockWidget* layersDock() const { return layersDock_; }

    /// Writes window geometry and dock layout to `settings`.
    void saveLayout(QSettings& settings) const;
    /// Restores what `saveLayout` wrote. Returns false when nothing (valid) was stored.
    bool restoreLayout(QSettings& settings);
    /// When true, closing the window saves the layout to the application's default settings.
    void setPersistLayout(bool persist) { persistLayout_ = persist; }

   protected:
    void closeEvent(QCloseEvent* event) override;

   private:
    void buildMenus();
    void buildToolBar();
    void buildDocks();
    void buildCentralArea();
    void newJob();
    void refreshSelectors();
    void pickSheet(int row);
    void pickLayer(int row);
    void refreshStatus();

    std::unique_ptr<document::Document> doc_;

    QTabWidget* views_{nullptr};
    CanvasWidget* canvas_{nullptr};
    RulerWidget* hRuler_{nullptr};
    RulerWidget* vRuler_{nullptr};

    QComboBox* sheetCombo_{nullptr};
    QComboBox* layerCombo_{nullptr};
    QListWidget* sheetsList_{nullptr};
    QListWidget* layersList_{nullptr};

    QDockWidget* designDock_{nullptr};
    QDockWidget* sheetsDock_{nullptr};
    QDockWidget* layersDock_{nullptr};
    QDockWidget* componentsDock_{nullptr};
    QDockWidget* libraryDock_{nullptr};
    QDockWidget* toolpathsDock_{nullptr};

    QAction* undoAction_{nullptr};
    QAction* redoAction_{nullptr};
    QLabel* cursorLabel_{nullptr};
    QLabel* jobLabel_{nullptr};

    bool persistLayout_{false};
    bool refreshing_{false};
};

}  // namespace carvetto::app
