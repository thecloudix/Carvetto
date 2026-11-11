#pragma once

#include <QMainWindow>
#include <map>
#include <memory>

#include <filesystem>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/tooling/machine.hpp"
#include "carvetto/viewmodel/file_session.hpp"
#include "carvetto/viewmodel/tools.hpp"

class QAction;
class QActionGroup;
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
class HistoryPanel;
class LayersPanel;
class PropertiesPanel;
class SheetsPanel;
class RulerWidget;
class CopiesMenu;
class DimensionMenu;
class NodeEditMenu;
class PreviewPanel;
class KeepOutPanel;
class ReliefView;
class ToolPanel;
class ToolpathsPanel;
class TransformMenu;
class VectorOpsMenu;
class TextureMenu;

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
    VectorOpsMenu* vectorOps() const { return vectorOps_.get(); }
    /// Edit > Create Vector Texture (T-0254).
    TextureMenu* textureMenu() const { return textureMenu_.get(); }
    /// The node-editing menus and Properties dialog of the NodeEdit tool.
    NodeEditMenu* nodeEdit() const { return nodeEdit_.get(); }
    TransformMenu* transformMenu() const { return transformMenu_.get(); }
    /// Edit > Dimensions (T-0255).
    DimensionMenu* dimensions() const { return dimensions_.get(); }
    /// Keep-out zones dock and Edit submenu (T-0256).
    KeepOutPanel* keepOutPanel() const { return keepOut_; }
    QDockWidget* keepOutDock() const { return keepOutDock_; }
    QDockWidget* toolpathsDock() const { return toolpathsDock_; }
    /// File > Save Toolpaths... (T-0224): posts the enabled, calculated toolpaths.
    QAction* saveToolpathsAction() const { return saveToolpathsAction_; }
    /// Opens the Save Toolpaths dialog (modal) for `toolpaths()->session().enabledToolpaths()`, with
    /// the machines of the tool database.
    void saveToolpathsDialog();
    ToolpathsPanel* toolpaths() const { return toolpaths_; }
    /// Setup > Job Setup... (edits the open job, one undo step) and Setup > Tools, Machines and
    /// Materials... (T-0235). The dialogs are modal; the self-test drives the dialog classes directly.
    QAction* jobSetupAction() const { return jobSetupAction_; }
    QAction* toolSetupAction() const { return toolSetupAction_; }
    void jobSetupDialog();
    void toolSetupDialog();
    /// The Preview dock (toolpath preview, time readout, simulation) and the simulation view in the 3D tab.
    PreviewPanel* preview() const { return preview_; }
    QDockWidget* previewDock() const { return previewDock_; }
    ReliefView* reliefView() const { return relief_; }
    /// Switches the central tabs: the 2D design view or the simulation ("3D") view.
    void showView(bool simulation);
    bool simulationViewShown() const;
    QAction* undoAction() const { return undoAction_; }
    QAction* redoAction() const { return redoAction_; }
    QDockWidget* layersDock() const { return layersDock_; }
    /// Properties, Layers, Sheets and History panels (T-0210).
    PropertiesPanel* propertiesPanel() const { return properties_; }
    LayersPanel* layersPanel() const { return layersPanel_; }
    SheetsPanel* sheetsPanel() const { return sheetsPanel_; }
    HistoryPanel* historyPanel() const { return historyPanel_; }
    QDockWidget* propertiesDock() const { return propertiesDock_; }
    QDockWidget* historyDock() const { return historyDock_; }
    ToolPanel* toolPanel() const { return toolPanel_; }

    /// Activates a drawing / selection tool (same as clicking its toolbar button).
    void selectTool(viewmodel::ToolKind tool);
    /// The toolbar action of a tool (checkable, exclusive).
    QAction* toolAction(viewmodel::ToolKind tool) const;

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
    /// Imports an image as a bitmap object (T-0275): embedded in the document, one undo step, selected.
    bool importBitmapFile(const std::filesystem::path& path);
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
    bool replaceDocument(const std::function<bool()>& swap);
    void buildMenus();
    void buildToolBar();
    void buildToolPalette();
    void buildDocks();
    void buildCentralArea();
    void newJob();
    void openDialog();
    bool saveDialog();
    bool saveAsDialog();
    void importDxfDialog();
    void importBitmapDialog();
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
    QAction* saveToolpathsAction_{nullptr};
    QAction* jobSetupAction_{nullptr};
    QAction* toolSetupAction_{nullptr};
    std::unique_ptr<CopiesMenu> copies_;
    std::unique_ptr<VectorOpsMenu> vectorOps_;
    std::unique_ptr<TextureMenu> textureMenu_;
    std::unique_ptr<NodeEditMenu> nodeEdit_;
    std::unique_ptr<TransformMenu> transformMenu_;
    std::unique_ptr<DimensionMenu> dimensions_;

    QTabWidget* views_{nullptr};
    CanvasWidget* canvas_{nullptr};
    RulerWidget* hRuler_{nullptr};
    RulerWidget* vRuler_{nullptr};

    QComboBox* sheetCombo_{nullptr};
    QComboBox* layerCombo_{nullptr};
    PropertiesPanel* properties_{nullptr};
    LayersPanel* layersPanel_{nullptr};
    SheetsPanel* sheetsPanel_{nullptr};
    HistoryPanel* historyPanel_{nullptr};

    QDockWidget* designDock_{nullptr};
    QDockWidget* sheetsDock_{nullptr};
    QDockWidget* layersDock_{nullptr};
    QDockWidget* componentsDock_{nullptr};
    QDockWidget* libraryDock_{nullptr};
    QDockWidget* toolpathsDock_{nullptr};
    QDockWidget* previewDock_{nullptr};
    QDockWidget* keepOutDock_{nullptr};
    KeepOutPanel* keepOut_{nullptr};
    QDockWidget* propertiesDock_{nullptr};
    QDockWidget* historyDock_{nullptr};

    QActionGroup* toolGroup_{nullptr};
    std::map<viewmodel::ToolKind, QAction*> toolActions_;
    ToolPanel* toolPanel_{nullptr};
    ToolpathsPanel* toolpaths_{nullptr};
    PreviewPanel* preview_{nullptr};
    ReliefView* relief_{nullptr};

    QAction* undoAction_{nullptr};
    QAction* redoAction_{nullptr};
    QLabel* cursorLabel_{nullptr};
    QLabel* jobLabel_{nullptr};

    /// Reads the user's tool setup from the config folder (T-0235); warns when the file is refused.
    void loadUserSetup(const QString& activeMachine);
    bool persistLayout_{false};
    bool refreshing_{false};
    bool quiet_{false};
};

}  // namespace carvetto::app
