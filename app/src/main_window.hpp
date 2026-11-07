#pragma once

#include <QMainWindow>
#include <memory>

#include <filesystem>
#include <optional>

#include "carvetto/document/document.hpp"
#include "carvetto/viewmodel/file_session.hpp"

class QAction;
class QMenu;
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
class CopiesMenu;

/// Main window: 2D/3D view tabs with rulers around the 2D canvas, sheet and layer selectors,
/// docked panels (Design, Sheets, Layers, Components, Library on the left; Toolpaths on the
/// right), File/Edit/View menus and a status bar with the cursor position in display units.
class MainWindow : public QMainWindow {
   public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    document::Document& currentDocument() { return session_.document(); }
    const document::Document& currentDocument() const { return session_.document(); }
    /// The file session behind the File menu (path, dirty state, recent files).
    viewmodel::FileSession& session() { return session_; }
    /// Replaces the open document with a new untitled one (the view re-fits to its material).
    /// No unsaved-changes prompt: callers ask `confirmDiscard()` first.
    void setDocument(std::unique_ptr<document::Document> doc);
    /// Refreshes every view after the document changed.
    void documentChanged();

    /// Undoes / redoes one step and refreshes. Return false when there was nothing to do.
    bool undo();
    bool redo();
    /// The Edit > Copies submenu (array, circular and along-vector copies).
    CopiesMenu* copies() const { return copies_.get(); }

    CanvasWidget* canvas() const { return canvas_; }
    QDockWidget* toolpathsDock() const { return toolpathsDock_; }
    QDockWidget* layersDock() const { return layersDock_; }

    /// Writes window geometry and dock layout to `settings`.
    void saveLayout(QSettings& settings) const;
    /// Restores what `saveLayout` wrote. Returns false when nothing (valid) was stored.
    bool restoreLayout(QSettings& settings);
    /// When true, closing the window saves the layout to the application's default settings, and
    /// the recent-files list is read from and written to them.
    void setPersistLayout(bool persist);

    /// File operations without dialogs for the file name (the menu actions call them after the
    /// file dialog; the self-test calls them directly). They show an error box on failure unless
    /// `setQuiet(true)`; the text stays available in `session().lastError()`.
    bool openFile(const std::filesystem::path& path);
    bool saveFile();
    bool saveFileAs(const std::filesystem::path& path);
    bool importDxfFile(const std::filesystem::path& path, std::optional<double> forcedMmPerUnit = std::nullopt);
    bool exportDxfFile(const std::filesystem::path& path, bool selectionOnly);
    /// True when it is fine to drop the open document: it is clean, or the user chose Save (and
    /// it saved) or Discard in the prompt. False on Cancel or a failed save. Always true when
    /// quiet (no prompt can be shown).
    bool confirmDiscard();
    /// Suppresses message boxes (headless self-test); the file operations still report by result.
    void setQuiet(bool quiet) { quiet_ = quiet; }

   protected:
    void closeEvent(QCloseEvent* event) override;

   private:
    void buildMenus();
    void buildToolBar();
    void buildDocks();
    void buildCentralArea();
    void newJob();
    void openDialog();
    bool saveDialog();
    bool saveAsDialog();
    void importDxfDialog();
    void exportDxfDialog(bool selectionOnly);
    void rebuildRecentMenu();
    void storeRecent();
    void showError(const QString& title);
    void refreshSelectors();
    void pickSheet(int row);
    void pickLayer(int row);
    void refreshStatus();

    viewmodel::FileSession session_;
    QMenu* recentMenu_{nullptr};
    std::unique_ptr<CopiesMenu> copies_;

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
    bool quiet_{false};
};

}  // namespace carvetto::app
