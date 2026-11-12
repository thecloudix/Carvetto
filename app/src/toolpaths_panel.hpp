#pragma once

#include <QPointer>
#include <QString>
#include <QWidget>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/viewmodel/toolpath_runner.hpp"
#include "carvetto/viewmodel/toolpath_session.hpp"

class QAction;
class QLabel;
class QListWidget;
class QProgressBar;
class QPushButton;
class QToolBar;
class QTreeWidget;
class QTreeWidgetItem;

namespace carvetto::app {

class ToolpathFormDialog;

/// The Toolpaths dock (T-0221, ADR-0031): the toolpath list (name with enable box, operation, tool,
/// state), a toolbar (add Profile / Pocket / Drill / Engrave / V-Carve, edit, duplicate, delete,
/// move up / down, recalculate, recalculate all), the operation forms, a diagnostics list and the
/// job summary. All logic is `viewmodel::ToolpathSession`; this class is widgets and threading.
///
/// Calculation runs OFF the UI thread (`viewmodel::ToolpathRunner`): the job is a copy of the
/// vectors and tools, the panel keeps no reference into the document across the gap, and the
/// result is applied back on the UI thread through `ToolpathSession::apply`, which refuses a result
/// whose inputs changed meanwhile (stale / refused / cancelled outcomes are shown inline). While a
/// calculation or its apply is running the panel is "busy": its actions are disabled (except
/// Cancel) and `onBusyChanged` lets the window disable Undo / Redo. Replacing the document
/// (`setDocument`) cancels the run and discards its result.
class ToolpathsPanel : public QWidget {
   public:
    explicit ToolpathsPanel(document::Document* doc, QWidget* parent = nullptr);
    ~ToolpathsPanel() override;

    /// Rebinds to another document (the old one may already be destroyed: nothing of it is
    /// touched). Cancels a running calculation and discards its result; closes an open form.
    void setDocument(document::Document* doc);
    /// Shows "Closing..." while the window waits for the worker to stop.
    void showClosing();
    /// Re-reads the list, summary and diagnostics after the document changed.
    void refresh();
    /// Forgets cached staleness after a change made without a command (active sheet), then refreshes.
    void invalidate();

    viewmodel::ToolpathSession& session() { return session_; }
    const viewmodel::ToolpathSession& session() const { return session_; }

    /// True from the start of a calculation until its result has been applied or discarded.
    bool busy() const { return busy_; }
    /// Runs the event loop until the panel is idle (self-test). False on timeout.
    bool waitIdle(int timeoutMs);
    /// The last inline message (outcome of the latest operation).
    QString statusText() const;

    // Callbacks (set by the main window).
    std::function<void()> onDocumentEdited;    ///< after toolpaths were applied or edited
    std::function<void()> onSelectionChanged;  ///< after the list linked the document selection
    std::function<void()> onBusyChanged;
    std::function<void()> onSetupChanged;      ///< after the tool database / active machine changed

    // --- Tool database and machine (T-0235) ---------------------------------------------------------
    /// The tool database the dock offers (the user's setup, or the built-in starter tools).
    const tooling::ToolDatabase& toolDatabase() const { return db_; }
    /// The machine whose rapid rate and acceleration feed the time estimate ("" = the first one).
    const std::string& activeMachineId() const { return activeMachineId_; }
    /// Replaces the tool database and the active machine; toolpaths whose tool changed or vanished
    /// show it in their state. Saves to the setup file when one is set.
    void applySetup(tooling::ToolDatabase db, const std::string& activeMachineId);
    /// Uses `path` as the user's setup file: loads it now (the built-in tools when it is missing or
    /// refused; the refusal text is returned, empty when fine) and saves there on `applySetup`.
    std::string useSetupFile(const std::filesystem::path& path, const std::string& activeMachineId);
    /// Text of the last failed save of the setup file (empty when fine).
    const std::string& setupSaveError() const { return setupSaveError_; }

    // --- Actions (also reachable by name for the self-test) ---------------------------------------
    QAction* addAction(document::ToolpathOperation operation) const;
    QAction* editAction() const { return edit_; }
    QAction* duplicateAction() const { return duplicate_; }
    QAction* deleteAction() const { return delete_; }
    QAction* moveUpAction() const { return up_; }
    QAction* moveDownAction() const { return down_; }
    QAction* recalculateAction() const { return recalculate_; }
    QAction* recalculateAllAction() const { return recalculateAll_; }
    QAction* cancelAction() const { return cancel_; }

    QTreeWidget* list() const { return list_; }
    QListWidget* diagnostics() const { return diagnostics_; }
    QLabel* summaryLabel() const { return summary_; }
    /// The open form dialog (non-modal inside the window), or null.
    ToolpathFormDialog* formDialog() const;
    /// Selects the row of toolpath `id` (as a click would: links the document selection).
    void selectToolpath(document::ToolpathId id);
    document::ToolpathId currentToolpath() const;

    /// Opens the form to add a toolpath of `operation` / to edit toolpath `id`.
    void openAddForm(document::ToolpathOperation operation);
    void openEditForm(document::ToolpathId id);

   private:
    enum class Kind { Add, Edit, Recalculate, RecalculateAll };
    struct Pending {
        Kind kind{Kind::Add};
        std::uint64_t generation{0};
        std::string label;
        /// Toolpaths that could not even be prepared (recalculate all): shown with the result.
        std::vector<viewmodel::ToolpathMessage> notes;
    };

    void build();
    void rebuildList();
    void rebuildDiagnostics();
    void rebuildSummary();
    void updateActions();
    void setBusy(bool busy, const QString& text);
    void setStatus(const QString& text, bool error);
    void linkSelection(document::ToolpathId id);
    void openForm(viewmodel::ToolpathForm form, bool editing, document::ToolpathId id);
    void start(std::vector<viewmodel::ToolpathJob> jobs, Kind kind, const std::string& label,
               std::vector<viewmodel::ToolpathMessage> notes = {});
    void finishRun();
    void doDuplicate();
    void doDelete();
    void doMove(int delta);
    void doRecalculate();
    void doRecalculateAll();
    void edited();

    viewmodel::ToolpathSession session_;
    document::Document* doc_{nullptr};
    tooling::ToolDatabase db_;
    std::string activeMachineId_;
    std::filesystem::path setupFile_;
    std::string setupSaveError_;
    std::uint64_t generation_{1};
    Pending pending_;
    bool busy_{false};
    bool refreshing_{false};
    QString status_;
    bool statusError_{false};
    /// Messages of calculations that failed (their records are unchanged), by target toolpath.
    std::map<document::ToolpathId, std::vector<viewmodel::ToolpathMessage>> failed_;
    std::vector<viewmodel::ToolpathMessage> lastMessages_;

    QToolBar* bar_{nullptr};
    QTreeWidget* list_{nullptr};
    QListWidget* diagnostics_{nullptr};
    QLabel* summary_{nullptr};
    QLabel* statusLabel_{nullptr};
    QLabel* busyLabel_{nullptr};
    QProgressBar* progress_{nullptr};
    QPushButton* cancelButton_{nullptr};
    std::map<int, QAction*> addActions_;
    QAction* edit_{nullptr};
    QAction* duplicate_{nullptr};
    QAction* delete_{nullptr};
    QAction* up_{nullptr};
    QAction* down_{nullptr};
    QAction* recalculate_{nullptr};
    QAction* recalculateAll_{nullptr};
    QAction* cancel_{nullptr};
    QPointer<ToolpathFormDialog> dialog_;
    /// Calculation-box expressions of the form being calculated (T-0257), bound to the toolpath(s)
    /// when the result is applied.
    std::map<std::string, std::string> pendingExpressions_;
    // Declared last: destroyed first, so a still-running worker is cancelled and joined before the
    // widgets its completion callback posts to go away.
    viewmodel::ToolpathRunner runner_;
};

class MainWindow;

/// Drives the Toolpaths dock of `window` through its real widgets (self-test, offscreen): add a
/// Profile toolpath on two selected rectangles, edit its depth, move a rectangle (stale), recalculate,
/// link the selection, refusals, and the document replaced / edited / window closed mid-calculation.
/// Returns an empty string on success, else the first failure. Replaces the window's document.
std::string toolpathsSelfTest(MainWindow& window);

/// Photo V-carve end to end (T-0275): import a PNG, see it on the canvas, add a Photo V-Carve through
/// the form, edit it, stale after moving the bitmap, save and reload. Replaces the window's document.
std::string photoSelfTest(MainWindow& window);

}  // namespace carvetto::app
